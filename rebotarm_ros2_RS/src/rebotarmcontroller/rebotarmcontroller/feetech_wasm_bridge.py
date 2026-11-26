"""Local browser MuJoCo -> HLS3604M bridge. No ROS imports or services."""
from __future__ import annotations

import argparse
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from pathlib import Path
import secrets
import sys
import threading
import time
from urllib.parse import urlsplit

from .feetech_feedback import FeetechForceOutput, MAX_RAW
from .grip_haptics import GripHaptics, PositionWall, validate_settings, validate_contacts

FORCE_GAIN = 3.5
GRIPPER_MAX_MM = 50.0
GRIPPER_MM_PER_DEG = 0.5
HARD_WALL_FORCE_N = 20.0
TRIGGER_RETURN_RAW = 30


def force_to_raw(force_n, full_scale_n=20.0, deadband_n=0.5):
    values = (force_n, full_scale_n, deadband_n)
    if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in values):
        raise ValueError('Force and settings must be finite numbers')
    if force_n < 0 or not 0 <= deadband_n < full_scale_n <= 1000:
        raise ValueError('Require force >= 0 and 0 <= deadband < full_scale <= 1000 N')
    base_raw = 100 * min(1.0, max(0.0, force_n - deadband_n) / (full_scale_n - deadband_n))
    return round(min(MAX_RAW, FORCE_GAIN * base_raw))


def angle_to_gripper_mm(angle_deg, zero_deg, direction=1, scale=GRIPPER_MM_PER_DEG):
    if not all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)
               for v in (angle_deg, zero_deg)) or isinstance(direction, bool) or direction not in (-1, 1):
        raise ValueError('Invalid servo angle or gripper direction')
    return min(GRIPPER_MAX_MM, max(0.0, (angle_deg - zero_deg) * direction * scale))


def serial_ports():
    """List devices without opening them; prefer stable Linux USB aliases."""
    from serial.tools.list_ports import comports
    aliases = {}
    for alias in sorted(Path('/dev/serial/by-id').glob('*')):
        if alias.exists():
            aliases.setdefault(str(alias.resolve()), str(alias))
    devices = {}
    for info in comports():
        physical = str(Path(info.device).resolve())
        device = aliases.get(physical, info.device)
        description = info.description or 'Serial port'
        devices.setdefault(physical, dict(device=device, label=f'{device} · {description}'))
    return sorted(devices.values(), key=lambda item: item['device'])


class WasmFeedback:
    """Serialized UART access; the independent watchdog releases stale leases."""
    def __init__(self, output=None, *, timeout=0.3, output_factory=None, ports_provider=None):
        self.output, self.timeout = output, timeout
        self.output_factory = output_factory
        self.ports_provider = ports_provider or serial_ports
        self.port = ''
        self.managed = output_factory is not None
        self.haptics = GripHaptics()
        self.wall = PositionWall()
        self.open_limit = PositionWall()
        self.close_limit = PositionWall()
        self.force_wall = PositionWall()
        self.rich_sample = self.testing = False
        self.return_active = False
        self.return_direction = 0
        self.lock = threading.RLock()
        self.token = None
        self.sequence = -1
        self.last_sample = None
        self.desired_raw = self.raw = 0
        self.force_n = 0.0
        self.angle_deg = self.angle_at = self.zero_angle_deg = None
        self.open_reference_deg = None
        self.angle_control = True
        self.gripper_direction = 1
        self.peak_raw, self.peak_force_n = 0, 0.0
        self.fault = ''
        self.last_angle_error = None
        self.reason = 'waiting for browser connection'
        self.last_health_at = 0.0
        self.closed = False
        self.stop_event = threading.Event()
        self.thread = None

    def ports(self):
        return {'ports': self.ports_provider()}

    def _close_output(self):
        output, self.output = self.output, None
        self.raw = 0
        self.angle_deg = self.angle_at = self.zero_angle_deg = None
        self.open_reference_deg = None
        self.port = ''
        if output is not None:
            output.close()

    def _read_angle(self):
        try:
            angle = self.output.read_position_deg()
            if isinstance(angle, bool) or not isinstance(angle, (int, float)) or not math.isfinite(angle):
                raise RuntimeError('Invalid servo angle feedback')
            return angle
        except Exception as exc:
            diagnostics = getattr(exc, 'diagnostics', None)
            if not isinstance(diagnostics, dict):
                try:
                    diagnostics = self.output.read_error_diagnostics()
                except Exception as diagnostic_error:
                    diagnostics = dict(status_available=False, diagnostic_error=str(diagnostic_error))
            if not isinstance(diagnostics, dict):
                diagnostics = dict(status_available=False)
            self.last_angle_error = {**diagnostics, 'original_error': diagnostics.get('original_error', str(exc)),
                                     'at_ms': round(time.time() * 1000), 'port': self.port}
            print('HLS angle read failure: ' + json.dumps(self.last_angle_error, ensure_ascii=False),
                  file=sys.stderr, flush=True)
            raise

    def connect(self, settings=None, port=None):
        with self.lock:
            if self.closed:
                raise RuntimeError("Bridge stopped")
            if self.fault and not self.managed:
                raise RuntimeError(self.fault)
            if self.token and self.last_sample is not None and time.monotonic() - self.last_sample < self.timeout:
                raise RuntimeError('Another browser is connected; disconnect it first')
            validated = validate_settings({} if settings is None else settings, self.haptics.settings)
            if self.managed:
                if not isinstance(port, str) or not port:
                    raise ValueError('Select a serial port before connecting')
                if port not in {item['device'] for item in self.ports_provider()}:
                    raise ValueError('Selected serial port is unavailable; refresh the port list')
                self._release('changing serial port')
                self._close_output()
                try:
                    self.output = self.output_factory(port)
                    self.port = port
                    angle = self._read_angle()
                    self.output.check_health()
                    self.angle_deg, self.angle_at = angle, time.monotonic()
                    self.port, self.fault = port, ''
                except Exception as exc:
                    self.fault = str(exc)
                    try:
                        self._close_output()
                    except Exception:
                        pass
                    raise RuntimeError(f'Cannot connect to selected servo port: {exc}') from exc
            self.haptics.configure(validated)
            self.haptics.reset()
            self.wall.reset()
            self.open_limit.reset()
            self.close_limit.reset()
            self.force_wall.reset()
            self.rich_sample = self.testing = False
            self.token = secrets.token_urlsafe(24)
            self.sequence = -1
            self.open_reference_deg = self.angle_deg if self.angle_at is not None \
                and time.monotonic() - self.angle_at < self.timeout else None
            self._update_zero()
            self.last_sample = time.monotonic()
            self.desired_raw, self.force_n = 0, 0.0
            self.peak_raw, self.peak_force_n = 0, 0.0
            self.reason = 'connected; waiting for simulation contacts'
            return {'token': self.token, **self.status()}

    def _require_token(self, token):
        if not self.token or token != self.token:
            raise ValueError('Browser connection expired; reconnect feedback')

    def _update_zero(self, direction=None, scale=None):
        """Keep the calibrated encoder angle at 50 mm across mapping changes."""
        direction = self.gripper_direction if direction is None else direction
        scale = self.haptics.settings['angle_scale'] if scale is None else scale
        self.zero_angle_deg = None if self.open_reference_deg is None else \
            self.open_reference_deg - direction * GRIPPER_MAX_MM / scale

    def sample(self, payload):
        with self.lock:
            if self.fault:
                raise RuntimeError(self.fault)
            self._require_token(payload.get('token'))
            if self.last_sample is None or time.monotonic() - self.last_sample >= self.timeout:
                self._release('browser sample timed out')
                raise ValueError('Browser sample timed out; reconnect feedback')
            seq, at = payload.get('seq'), payload.get('at_ms')
            if isinstance(seq, bool) or not isinstance(seq, int) or seq <= self.sequence:
                self._release('invalid sample sequence')
                raise ValueError('Sample sequence must increase')
            if isinstance(at, bool) or not isinstance(at, (int, float)) or not math.isfinite(at):
                self._release('invalid sample timestamp')
                raise ValueError('Invalid sample timestamp')
            age = time.time() - at / 1000
            if not -0.1 <= age < self.timeout:
                self._release('stale browser sample')
                raise ValueError('Stale browser sample; reconnect feedback')
            try:
                control = payload.get('angle_control', False)
                direction = payload.get('gripper_direction', 1)
                if not isinstance(control, bool) or isinstance(direction, bool) or direction not in (-1, 1):
                    raise ValueError('Invalid angle-control settings')
                settings = payload.get('settings', {})
                validate_settings(settings, self.haptics.settings)
                # Accept older clients while the current panel sends the shared schema.
                if 'full_scale_n' in payload:
                    settings = {**settings, 'full_scale_n': payload['full_scale_n']}
                if 'deadband_n' in payload:
                    settings = {**settings, 'force_deadband_n': payload['deadband_n']}
                validated = validate_settings(settings, self.haptics.settings)
                force_to_raw(payload.get('force_n'), validated['full_scale_n'], validated['force_deadband_n'])
                contacts = payload.get('grip_contacts')
                if contacts is not None:
                    validate_contacts(contacts)
                if direction != self.gripper_direction or control != self.angle_control or validated['angle_scale'] != self.haptics.settings['angle_scale']:
                    self.haptics.reset()
                    self._stop_wall()
                self.haptics.configure(validated)
                self._update_zero(direction, validated['angle_scale'])
                target = payload.get('target_width_mm')
                if control and self.zero_angle_deg is not None and self.angle_at is not None:
                    target = angle_to_gripper_mm(self.angle_deg, self.zero_angle_deg, direction, validated['angle_scale'])
                force = sum(c['left_n'] + c['right_n'] for c in contacts) if contacts is not None else payload['force_n']
                self.haptics.update(force_n=force, contacts=contacts,
                                    actual_mm=payload.get('actual_width_mm'), target_mm=target, now=time.monotonic())
                raw = round(self.haptics.target_raw)
                self.rich_sample = contacts is not None
                testing = payload.get('test') is True
                if testing != self.testing:
                    self._stop_wall()
                self.testing = testing
                if self.testing:
                    raw = min(10, validated['max_raw'])
            except ValueError:
                self._release('invalid force sample')
                raise
            self.sequence, self.last_sample = seq, time.monotonic() - max(0.0, age)
            self.angle_control, self.gripper_direction = control, direction
            self.desired_raw, self.force_n = raw, force
            self.peak_force_n = max(self.peak_force_n, self.force_n)
            self.reason = 'test output 10' if payload.get('test') is True else 'following WASM gripper contacts'
            return self.status()

    def _stop_wall(self):
        self.wall.reset()
        self.open_limit.reset()
        self.close_limit.reset()
        self.force_wall.reset()
        self.return_active = False
        self.return_direction = 0
        self.haptics.output = 0.0
        self.raw = 0
        if self.output is not None:
            try:
                self.output.release()
            except Exception as exc:
                # A failed zeroing command must never allow a later sample to
                # re-enable the same session after an uncertain UART write.
                self.fault = str(exc)
                self.token, self.last_sample = None, None
                self.desired_raw = 0
                raise

    def _release(self, reason):
        self.haptics.reset()
        self.rich_sample = self.testing = False
        self.token, self.last_sample = None, None
        self.desired_raw, self.force_n, self.reason = 0, 0.0, reason
        try:
            self._stop_wall()
        except Exception as exc:
            self.fault = str(exc)

    def disconnect(self, token):
        with self.lock:
            self._require_token(token)
            self._release('disconnected; force released')
            if self.managed:
                self._close_output()
            return self.status()

    def calibrate_gripper(self, token):
        with self.lock:
            self._require_token(token)
            if self.fault or self.angle_at is None or time.monotonic() - self.angle_at >= self.timeout:
                raise RuntimeError(self.fault or 'Servo angle is stale; reconnect feedback')
            self.open_reference_deg = self.angle_deg
            self._update_zero()
            self.haptics.reset()
            self._stop_wall()
            self.desired_raw = 0
            return self.status()

    def status(self):
        with self.lock:
            age = time.monotonic() - self.last_sample if self.last_sample is not None else None
            angle_age = time.monotonic() - self.angle_at if self.angle_at is not None else None
            angle_valid = not self.fault and angle_age is not None and 0 <= angle_age < self.timeout
            width = angle_to_gripper_mm(self.angle_deg, self.zero_angle_deg, self.gripper_direction, self.haptics.settings['angle_scale']) \
                if angle_valid and self.zero_angle_deg is not None else None
            displayed_wall = self.force_wall if self.force_wall.key is not None else self.wall
            if self.open_limit.active or displayed_wall.angle_deg is None:
                displayed_wall = self.open_limit
            if self.close_limit.active:
                displayed_wall = self.close_limit
            return dict(connected=bool(self.token) and age is not None and age < self.timeout,
                        raw=self.raw, target_raw=self.desired_raw, force_n=self.force_n,
                        fault=self.fault, reason=self.reason, sample_age_s=age, source='wasm',
                        angle_error=self.last_angle_error,
                        port=self.port, serial_open=self.output is not None, port_selection_required=self.managed,
                        peak_raw=self.peak_raw, peak_force_n=self.peak_force_n,
                        force_gain=self.haptics.settings['force_gain'], max_raw=self.haptics.settings['max_raw'],
                        drive_mode='speed_return' if self.return_active and self.return_direction else 'position_wall',
                        output_direction=self.return_direction if self.return_active else self.gripper_direction,
                        target_speed_raw=(self.return_direction if self.return_active and self.return_direction else 1) *
                                         int(self.haptics.settings['resistance_speed']) if self.raw else 0,
                        wall_angle_deg=displayed_wall.angle_deg, wall_active=displayed_wall.active,
                        wall_released=displayed_wall.released,
                        wall_source='opening_limit' if displayed_wall is self.open_limit else
                                    'closing_limit' if displayed_wall is self.close_limit else
                                    'force_threshold' if displayed_wall is self.force_wall else 'contact',
                        hard_wall=self.force_wall.active, hard_wall_trigger_n=HARD_WALL_FORCE_N,
                        trigger_return_active=self.return_active,
                        trigger_return_direction=self.return_direction,
                        trigger_return_raw=min(TRIGGER_RETURN_RAW, int(self.haptics.settings['max_raw'])),
                        trigger_return_target_mm=GRIPPER_MAX_MM,
                        open_reference_deg=self.open_reference_deg, calibration_width_mm=GRIPPER_MAX_MM,
                        settings=self.haptics.settings.copy(),
                        **self.haptics.telemetry(),
                        angle_deg=self.angle_deg if angle_valid else None,
                        zero_angle_deg=self.zero_angle_deg, angle_age_s=angle_age,
                        angle_control=self.angle_control, gripper_direction=self.gripper_direction,
                        gripper_mm_per_deg=self.haptics.settings['angle_scale'],
                        gripper_mm=width)

    def tick(self):
        with self.lock:
            now = time.monotonic()
            if self.last_sample is not None and now - self.last_sample >= self.timeout:
                self._release('browser sample timed out; force released')
            try:
                if self.output is None:
                    return
                if self.managed and not self.token:
                    self._close_output()
                    return
                if self.fault:
                    self.output.release()
                    self.raw = 0
                    return
                angle = self._read_angle()
                if now - self.last_health_at >= 0.1:
                    self.output.check_health()
                    self.last_health_at = now
                self.angle_deg, self.angle_at = angle, time.monotonic()
                if self.last_sample is not None and self.angle_at - self.last_sample >= self.timeout:
                    self._release('browser sample timed out during serial read; force released')
                    return
                if self.token and self.zero_angle_deg is None:
                    self.open_reference_deg = angle
                    self._update_zero()
                magnitude = self.haptics.advance(now) if self.rich_sample and not self.testing else self.desired_raw
                magnitude = min(int(self.haptics.settings['max_raw']), magnitude)
                key = '__test__' if self.testing else self.haptics.wall_object_id if self.rich_sample \
                    else '__force__' if self.desired_raw > 0 else None
                target = angle
                if not self.testing and self.angle_control and self.zero_angle_deg is not None \
                        and self.haptics.wall_position_mm is not None:
                    target = self.zero_angle_deg + self.gripper_direction * \
                        self.haptics.wall_position_mm / self.haptics.settings['angle_scale']
                    # A delayed first contact cannot cause a large powered return.
                    target = angle + self.gripper_direction * min(2.0, max(0.0,
                        (target - angle) * self.gripper_direction)) if self.wall.key != key else target
                self.wall.select(key, target, self.gripper_direction)
                magnitude = self.wall.command(angle, magnitude,
                    release_deg=self.haptics.settings['release_angle_deg'],
                    rearm_deg=self.haptics.settings['rearm_angle_deg'])
                active_wall = self.wall
                contact_key = self.haptics.wall_object_id if self.rich_sample else '__force__'
                force_key = ('hard_wall', contact_key)
                if (self.testing or not self.token or contact_key is None
                        or self.force_n <= self.haptics.settings['force_deadband_n']
                        or self.haptics.state in ('free', 'slip')):
                    self.force_wall.reset()
                elif self.force_wall.key != force_key:
                    self.force_wall.reset()
                    if self.force_n > HARD_WALL_FORCE_N and self.haptics.state != 'opening':
                        # Capture the handle angle at threshold crossing, not the
                        # first light touch. No confirmation or force ramp delay.
                        self.force_wall.select(force_key, angle, self.gripper_direction)
                if self.force_wall.key is not None:
                    magnitude = self.force_wall.command(angle,
                        0 if self.haptics.state == 'opening' else int(self.haptics.settings['max_raw']),
                        release_deg=self.haptics.settings['release_angle_deg'],
                        rearm_deg=self.haptics.settings['rearm_angle_deg'])
                    active_wall = self.force_wall
                    # Keep this boundary until opening/contact loss so a small
                    # force dip below 20 N cannot make the wall chatter.
                # The 50 mm stop blocks further OPENING, opposite to an object
                # contact wall. Keep its release latch independent of contacts.
                self.open_limit.select('__50mm__' if self.token and not self.testing else None,
                                       self.open_reference_deg, -self.gripper_direction)
                limit_raw = self.open_limit.command(angle, int(self.haptics.settings['max_raw']),
                    release_deg=self.haptics.settings['release_angle_deg'],
                    rearm_deg=self.haptics.settings['rearm_angle_deg'])
                if limit_raw:
                    magnitude, active_wall = limit_raw, self.open_limit
                # The mapped zero is the other travel stop: resist closing,
                # and release freely towards opening, independently of contacts.
                self.close_limit.select('__0mm__' if self.token and not self.testing else None,
                                        self.zero_angle_deg, self.gripper_direction)
                close_raw = self.close_limit.command(angle, int(self.haptics.settings['max_raw']),
                    release_deg=self.haptics.settings['release_angle_deg'],
                    rearm_deg=self.haptics.settings['rearm_angle_deg'])
                if close_raw:
                    magnitude, active_wall = close_raw, self.close_limit
                width = None if self.zero_angle_deg is None else \
                    (angle - self.zero_angle_deg) * self.gripper_direction * self.haptics.settings['angle_scale']
                # One encoder count covers quantization at the mapped zero.
                # At the edge retain trigger preload; deeper closing still
                # engages the hard stop. A force-threshold wall takes priority.
                zero_edge_mm = 360.0 / 4096.0 * self.haptics.settings['angle_scale']
                if (close_raw and width is not None and abs(width) <= zero_edge_mm
                        and not self.force_wall.active):
                    magnitude = 0
                target_deg = active_wall.angle_deg
                self.return_active = False
                self.return_direction = 0
                if (not magnitude and self.token and not self.testing
                        and self.zero_angle_deg is not None and self.open_reference_deg is not None):
                    # Every healthy released wall falls back to trigger return,
                    # including outside the mapped range. Drive towards the
                    # actual 50 mm reference, rather than a clipped width.
                    magnitude = min(TRIGGER_RETURN_RAW, int(self.haptics.settings['max_raw']))
                    target_deg = self.open_reference_deg
                    self.return_active = magnitude > 0
                    delta_deg = self.open_reference_deg - angle
                    if abs(delta_deg) > 360.0 / 4096.0:
                        self.return_direction = 1 if delta_deg > 0 else -1
                    # At 50 mm, retain current-limited position holding instead
                    # of continuing to drive through the endpoint.
                if not magnitude:
                    # Reverse release bypasses all output smoothing/slew limits.
                    self.haptics.output = 0.0
                    if self.raw:
                        self.output.release()
                else:
                    if self.return_active and self.return_direction:
                        self.output.set_resistance(magnitude, direction=self.return_direction,
                                                   speed_raw=int(self.haptics.settings['resistance_speed']))
                    else:
                        self.output.set_wall(magnitude, target_deg=target_deg,
                                             speed_raw=int(self.haptics.settings['resistance_speed']))
                self.raw = magnitude
                self.peak_raw = max(self.peak_raw, self.raw)
            except Exception as exc:
                self.fault = str(exc)
                self._release('servo fault; force output inhibited')
                self.raw = 0
                try:
                    if self.managed:
                        self._close_output()
                    elif self.output is not None:
                        self.output.release()
                except Exception:
                    pass

    def start(self):
        def run():
            while not self.stop_event.wait(0.01):
                self.tick()
        self.thread = threading.Thread(target=run, name='feetech-watchdog', daemon=True)
        self.thread.start()

    def close(self):
        self.stop_event.set()
        if self.thread:
            self.thread.join(timeout=2)
        with self.lock:
            if not self.closed:
                self.closed = True
                self._release('bridge stopped')
                self._close_output()


class MockOutput:
    def read_position_deg(self): return 0.0
    def set_raw(self, raw): pass
    def set_resistance(self, raw, *, direction=1, speed_raw=2): pass
    def set_wall(self, raw, *, target_deg, speed_raw=2): pass
    def release(self): pass
    def check_health(self): pass
    def close(self): pass


class BridgeHandler(SimpleHTTPRequestHandler):
    def log_message(self, *_args): pass

    def _local_request(self):
        host = urlsplit('http://' + self.headers.get('Host', '')).hostname
        origin = self.headers.get('Origin')
        return host in ('127.0.0.1', 'localhost', '::1') and (
            not origin or urlsplit(origin).hostname in ('127.0.0.1', 'localhost', '::1'))

    def end_headers(self):
        self.send_header('Cache-Control', 'no-store' if self.path.startswith('/feetech/') else 'no-cache')
        super().end_headers()

    def _json(self, status, value):
        body = json.dumps(value, allow_nan=False).encode()
        self.send_response(status)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if not self._local_request():
            return self._json(403, {'error': 'Use a localhost browser address'})
        if self.path == '/feetech/ports':
            return self._json(200, self.server.feedback.ports())
        if self.path == '/feetech/status':
            return self._json(200, self.server.feedback.status())
        super().do_GET()

    def do_POST(self):
        if not self._local_request():
            return self._json(403, {'error': 'Only local browser pages may control feedback'})
        try:
            if self.headers.get('Content-Type', '').split(';')[0] != 'application/json':
                raise ValueError('Content-Type must be application/json')
            length = int(self.headers.get('Content-Length', '0'))
            if not 0 < length <= 4096:
                raise ValueError('Invalid request size')
            payload = json.loads(self.rfile.read(length))
            if not isinstance(payload, dict):
                raise ValueError('Expected a JSON object')
            feedback = self.server.feedback
            if self.path == '/feetech/connect':
                result = feedback.connect(payload.get('settings'), payload.get('port'))
            elif self.path == '/feetech/sample':
                result = feedback.sample(payload)
            elif self.path == '/feetech/disconnect':
                result = feedback.disconnect(payload.get('token'))
            elif self.path == '/feetech/calibrate':
                result = feedback.calibrate_gripper(payload.get('token'))
            else:
                return self._json(404, {'error': 'Unknown feedback endpoint'})
            self._json(200, result)
        except (ValueError, RuntimeError, OSError) as exc:
            self._json(400, {'error': str(exc), 'angle_error': self.server.feedback.last_angle_error})


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', default='', help='Legacy option; select the port manually in the web panel')
    parser.add_argument('--servo-id', type=int, default=1)
    parser.add_argument('--baudrate', type=int, default=1_000_000)
    parser.add_argument('--http-port', type=int, default=8766)
    parser.add_argument('--web-root', type=Path, required=True)
    parser.add_argument('--mock', action='store_true', help='Test with no physical servo')
    options = parser.parse_args(args)
    feedback = server = None
    try:
        if not (options.web_root / 'index.html').is_file():
            raise ValueError('Web build missing: run npm --prefix web_mujoco run build')
        FeetechForceOutput(None, options.servo_id)  # Validate ID without opening a device.
        if options.mock:
            feedback = WasmFeedback(output_factory=lambda _port: MockOutput(), ports_provider=lambda: [
                {'device': 'mock://hls3604m', 'label': 'MOCK · HLS3604M (no physical servo)'}])
        else:
            feedback = WasmFeedback(output_factory=lambda port: FeetechForceOutput.open(
                port, options.servo_id, options.baudrate, mode=0))
        server = ThreadingHTTPServer(('127.0.0.1', options.http_port),
                                    partial(BridgeHandler, directory=str(options.web_root)))
        server.feedback = feedback
        feedback.start()
        print(f'WASM force feedback ready: http://127.0.0.1:{options.http_port}', flush=True)
        print('Open the page, select a serial port, then click Connect. No port is opened until selected. '
              + ('MOCK: no physical servo.' if options.mock else f'Force gain: {FORCE_GAIN:g}x; raw current range: 0..{MAX_RAW}.'), flush=True)
        server.serve_forever(poll_interval=0.1)
    except KeyboardInterrupt:
        pass
    except (OSError, ValueError, RuntimeError) as exc:
        print(f'WASM feedback failed: {exc}', file=sys.stderr)
        return 1
    finally:
        if server:
            server.server_close()
        if feedback:
            feedback.close()


if __name__ == '__main__':
    sys.exit(main())
