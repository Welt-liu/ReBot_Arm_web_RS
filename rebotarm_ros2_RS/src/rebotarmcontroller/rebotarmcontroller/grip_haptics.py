"""Unilateral virtual wall and grasp-state feedback, independent of ROS/UART."""
import json
import math
from pathlib import Path

SCHEMA = json.loads(Path(__file__).with_name('haptic_settings.json').read_text())
DEFAULTS = {item['key']: item['default'] for item in SCHEMA}


def finite(value):
    return isinstance(value, (float, int)) and not isinstance(value, bool) and math.isfinite(value)


def validate_settings(values, previous=None):
    if not isinstance(values, dict) or set(values) - set(DEFAULTS):
        raise ValueError('Unknown haptic settings')
    result = {**DEFAULTS, **(previous or {}), **values}
    for item in SCHEMA:
        value = result[item['key']]
        if not finite(value) or not item['min'] <= value <= item['max']:
            raise ValueError(f"{item['key']} must be {item['min']}..{item['max']}")
        if item['step'] == 1 and value != int(value):
            raise ValueError(f"{item['key']} must be an integer")
    if result['contact_off_n'] >= result['contact_on_n']:
        raise ValueError('Contact release threshold must be lower than contact threshold')
    if result['force_deadband_n'] >= result['full_scale_n']:
        raise ValueError('Force deadband must be lower than force reference')
    return result


def validate_contacts(contacts):
    if not isinstance(contacts, list) or len(contacts) > 16:
        raise ValueError('Invalid per-object contact data')
    seen = set()
    for contact in contacts:
        if not isinstance(contact, dict):
            raise ValueError('Invalid contact record')
        ident = contact.get('id')
        if not isinstance(ident, str) or not ident or len(ident) > 64 or ident in seen:
            raise ValueError('Invalid/duplicate grasp object')
        seen.add(ident)
        for key in ('left_n', 'right_n'):
            if not finite(contact.get(key)) or not 0 <= contact[key] <= 1e6:
                raise ValueError('Invalid finger force')
        position = contact.get('relative_mm')
        if not isinstance(position, list) or len(position) != 3 or not all(finite(v) and abs(v) < 1e6 for v in position):
            raise ValueError('Invalid object position relative to gripper')
    return contacts


class GripHaptics:
    def __init__(self, settings=None):
        self.settings = validate_settings(settings or {})
        self.reset()

    def reset(self):
        self.state = 'free'
        self.active = {}
        self.candidate = self.candidate_at = None
        self.held = self.anchor = self.slip_at = None
        self.slip_until = 0.0
        self.slip_started = None
        self.last_target = self.last_sample_at = None
        self.velocity = 0.0
        self.target_raw = self.output = 0.0
        self.last_output_at = None
        self.left_n = self.right_n = self.wall_mm = 0.0
        self.force_term = self.spring_term = self.damping_term = 0.0
        self.object_id = None
        self.wall_position_mm = self.wall_object_id = None

    def configure(self, settings):
        self.settings = validate_settings(settings, self.settings)
        self.output = min(self.output, self.settings['max_raw'])

    def update(self, *, force_n, contacts, actual_mm, target_mm, now):
        s = self.settings
        self.force_term = s['force_gain'] * 100 * min(1.0, max(0.0, force_n - s['force_deadband_n']) /
                                                     (s['full_scale_n'] - s['force_deadband_n']))
        self.spring_term = self.damping_term = self.wall_mm = 0.0
        if actual_mm is not None and (not finite(actual_mm) or not -10 <= actual_mm <= 60):
            raise ValueError('Invalid simulated gripper opening')
        if target_mm is not None and (not finite(target_mm) or not 0 <= target_mm <= 50):
            raise ValueError('Invalid target opening')
        if target_mm is not None and self.last_target is not None:
            dt = now - self.last_sample_at
            if 0 < dt < 0.3:
                speed = max(-100.0, min(100.0, (target_mm - self.last_target) / dt))
                self.velocity += min(1.0, dt / (0.04 + dt)) * (speed - self.velocity)
        self.last_target, self.last_sample_at = target_mm, now
        opening = self.velocity > 0.5

        # Compatibility with older browser clients. Current clients always
        # provide per-finger metadata and use the complete state machine.
        if contacts is None:
            self.state = 'touch' if force_n > s['force_deadband_n'] else 'free'
            self.target_raw = min(s['max_raw'], self.force_term)
            self.wall_position_mm = self.wall_object_id = None
            return
        contacts = validate_contacts(contacts)
        self.left_n = sum(c['left_n'] for c in contacts)
        self.right_n = sum(c['right_n'] for c in contacts)
        flags = {}
        for c in contacts:
            old = self.active.get(c['id'], (False, False))
            flags[c['id']] = tuple(c[key] >= (s['contact_off_n'] if old[i] else s['contact_on_n'])
                                    for i, key in enumerate(('left_n', 'right_n')))
        self.active = flags
        candidates = [c for c in contacts if all(flags[c['id']])]
        paired = max(candidates, key=lambda c: min(c['left_n'], c['right_n']), default=None)
        ident = paired['id'] if paired else None
        if ident != self.candidate:
            self.candidate, self.candidate_at = ident, now if ident else None
        any_contact = any(any(v) for v in flags.values())
        stable = paired is not None and now - self.candidate_at >= s['grip_confirm_ms'] / 1000
        self.state = 'touch' if any_contact else 'free'
        self.object_id = ident
        if opening:
            self.held = self.anchor = self.slip_at = None
            self.slip_until = 0.0
            if any_contact:
                self.state = 'opening'
        elif self.held is not None:
            held = next((c for c in contacts if c['id'] == self.held), None)
            displaced = held is None or math.dist(held['relative_mm'], self.anchor) > s['slip_distance_mm']
            lost = ident != self.held
            if lost or displaced:
                if self.slip_at is None:
                    self.slip_at = now
                if now - self.slip_at >= s['slip_confirm_ms'] / 1000:
                    self.slip_started = now
                    self.slip_until = now + s['slip_hold_ms'] / 1000
                    self.held = self.anchor = self.slip_at = None
                    self.candidate_at = now if ident else None
                    stable = False
            else:
                self.slip_at = None
                self.state = 'gripping'
        if not opening and now < self.slip_until:
            self.state = 'slip'
        elif not opening and stable and self.slip_at is None:
            self.state = 'gripping'
            if self.held != ident:
                self.held, self.anchor = ident, list(paired['relative_mm'])
        touching = [c for c in contacts if any(flags[c['id']])]
        wall_contact = paired or max(touching, key=lambda c: c['left_n'] + c['right_n'], default=None)
        if self.state in ('free', 'slip') or wall_contact is None:
            self.wall_position_mm = self.wall_object_id = None
        elif wall_contact['id'] != self.wall_object_id or self.wall_position_mm is None:
            # Freeze the first-contact boundary, including through grip confirmation.
            self.wall_object_id = wall_contact['id']
            self.wall_position_mm = actual_mm
        if self.state in ('touch', 'gripping') and self.wall_position_mm is not None and target_mm is not None:
            self.wall_mm = max(0.0, self.wall_position_mm - target_mm - s['wall_deadband_mm'])
            self.spring_term = self.wall_mm * s['stiffness']
            self.damping_term = max(0.0, -self.velocity - 0.5) * s['damping']
        target = s['wall_hold_raw'] + self.force_term + self.spring_term + self.damping_term
        if self.state == 'free':
            target = 0.0
        elif self.state == 'touch':
            target = min(target, s['touch_cap_raw'])
        elif self.state == 'opening':
            target = 0.0  # Old saved opening_fraction settings cannot retain force.
        elif self.state == 'slip':
            # Optional gentle cue; default amplitude 0 avoids artificial motion.
            target = s['slip_pulse_raw'] if now - self.slip_started < 0.08 else 0.0
        self.target_raw = min(s['max_raw'], max(0.0, target))

    def advance(self, now):
        dt = 0.02 if self.last_output_at is None else max(0.0, min(0.1, now - self.last_output_at))
        self.last_output_at = now
        if self.target_raw == 0:
            self.output = 0.0
        else:
            tau = self.settings['smoothing_ms'] / 1000
            desired = self.output + (self.target_raw - self.output) * (dt / (tau + dt) if tau else 1)
            limit = self.settings['slew_raw_s'] * dt
            self.output += max(-limit, min(limit, desired - self.output))
        self.output = min(self.settings['max_raw'], max(0.0, self.output))
        return round(self.output)

    def telemetry(self):
        return dict(grip_state=self.state, object_id=self.object_id, left_force_n=self.left_n,
                    right_force_n=self.right_n, wall_error_mm=self.wall_mm,
                    force_term=self.force_term, spring_term=self.spring_term,
                    damping_term=self.damping_term, closing_speed_mm_s=max(0, -self.velocity))


class PositionWall:
    """Local unilateral gate in encoder coordinates, independent of browser rate.

    direction * angle increases while opening. Reverse displacement releases the
    drive immediately, and latches it off until a deliberate closing movement.
    The selected boundary never follows a hand pushing farther into the wall.
    """
    def __init__(self):
        self.reset()

    def reset(self):
        self.key = self.angle_deg = self.direction = None
        self.low = self.high = None
        self.released = self.active = False

    def select(self, key, angle_deg, direction):
        if key is None:
            self.reset()
        elif key != self.key or direction != self.direction:
            self.reset()
            self.key, self.direction = key, direction
            # Use the identical quantized target as the hardware position loop.
            self.angle_deg = direction * math.ceil(direction * angle_deg * 4096 / 360 - 1e-9) * 360 / 4096

    def command(self, angle_deg, raw, *, release_deg, rearm_deg):
        self.active = False
        if self.key is None:
            return 0
        position = self.direction * angle_deg
        boundary = self.direction * self.angle_deg
        self.low = position if self.low is None else min(self.low, position)
        if not self.released and position - self.low >= release_deg:
            self.released, self.high = True, position
        if self.released:
            self.high = max(self.high, position)
            if self.high - position >= rearm_deg and position <= boundary:
                self.released = False
                self.low = position
            else:
                return 0
        # Never servo back toward the wall from its freely opening side.
        self.active = raw > 0 and position <= boundary
        return raw if self.active else 0
