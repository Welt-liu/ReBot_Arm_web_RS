"""HLS3604M force feedback, with no ROS dependency.

HLS position/speed modes keep WORD44 as a nonnegative current limit.
This application caps the raw command at 150, without percentage scaling.
"""
from __future__ import annotations

import math
import time

TORQUE_ENABLE = 40
OPERATING_MODE = 33
GOAL_CURRENT = 44
LOCK = 55
PRESENT_STATUS = 65
RESPONSE_LEVEL = 8
GOAL_SPEED = 46
GOAL_POSITION = 42
MAX_RESISTANCE_SPEED = 10
MAX_RAW = 150
LEGACY_MAX_RAW = 100


class ServoAngleReadError(RuntimeError):
    """Preserve the original angle failure and subsequent read-only diagnostics."""
    def __init__(self, message, diagnostics):
        super().__init__(message)
        self.diagnostics = diagnostics


def describe_servo_status(status):
    names = [(1, "input voltage fault"), (2, "encoder fault"),
             (4, "overtemperature"), (8, "overcurrent"), (32, "overload")]
    parts = [name for bit, name in names if status & bit]
    unknown = status & ~sum(bit for bit, _ in names)
    if unknown:
        parts.append(f"unknown bits 0x{unknown:02x}")
    return ", ".join(parts) if parts else "OK"


def read_voltage_diagnostics(packet, servo_id):
    # A valid read response may carry a persistent alarm bit. Diagnostic reads
    # preserve that telemetry instead of recursively treating it as transport
    # failure. No register is written here.
    readings = {}
    for address, name in ((62, "voltage_v"), (15, "min_voltage_v"), (14, "max_voltage_v")):
        try:
            value, communication, error = packet.read1ByteTxRx(servo_id, address)
            readings[name] = value / 10.0 if communication == 0 else None
        except Exception:
            readings[name] = None
    return readings


class FeedbackSignal:
    """Map measured follower motor torque (Nm) to a bounded current command."""

    def __init__(self, deadband=0.05, full_scale=1.0, timeout=0.3):
        if not all(math.isfinite(x) for x in (deadband, full_scale, timeout)):
            raise ValueError("feedback settings must be finite")
        if not 0 <= deadband < full_scale or not 0 < timeout <= 1.0:
            raise ValueError("require 0 <= deadband < full_scale and 0 < timeout <= 1s")
        self.deadband, self.full_scale, self.timeout = deadband, full_scale, timeout
        self.raw, self.received_at, self.stamp_ns = 0, None, -1

    def invalidate(self):
        self.raw, self.received_at = 0, None

    def update(self, torque, status, stamp_ns, now_ns, *, now=None):
        # Reject queued/replayed frames using the ROS source timestamp. Use a
        # monotonic receive deadline as well, so paused ROS clocks still expire.
        age = (now_ns - stamp_ns) / 1e9
        if stamp_ns == self.stamp_ns and math.isfinite(torque) and status == 0:
            return  # A cached publication must not renew the receive deadline.
        if (not math.isfinite(torque) or status != 0 or stamp_ns < self.stamp_ns
                or not -0.05 <= age < self.timeout):
            self.invalidate()
            return
        self.stamp_ns = stamp_ns
        magnitude = max(0.0, abs(torque) - self.deadband)
        self.raw = round(LEGACY_MAX_RAW * min(1.0, magnitude / (self.full_scale - self.deadband)))
        self.received_at = (time.monotonic() if now is None else now) - max(0.0, age)

    def value(self, *, active, now=None):
        now = time.monotonic() if now is None else now
        if not active or self.received_at is None or not 0 <= now - self.received_at < self.timeout:
            return 0
        return self.raw


class FeetechForceOutput:
    """HLS position wall (mode 0) or legacy speed resistance (mode 1).

    WORD42 selects the wall position; WORD44 is always 0..150 in both modes.
    Never encode direction in the current limit or modify motor phase/EEPROM.
    """

    def __init__(self, packet, servo_id, close_port=lambda: None, *, mode=1):
        if isinstance(servo_id, bool) or not isinstance(servo_id, int) or not 0 <= servo_id <= 253:
            raise ValueError("servo_id must be 0..253 (broadcast is forbidden)")
        self.packet, self.servo_id, self.close_port = packet, servo_id, close_port
        if isinstance(mode, bool) or mode not in (0, 1):
            raise ValueError('Feedback requires position mode 0 or speed mode 1')
        self.mode = mode
        self.position_limits = None
        self.last_position = None
        self.last_raw = None
        self.last_direction, self.last_speed = None, 0
        self.previous_mode = None
        self.closed = False
        self.expect_write_reply = True
        self.touched = False

    @classmethod
    def open(cls, port, servo_id, baudrate=1_000_000, *, mode=1):
        cls(None, servo_id, mode=mode)  # Validate before touching the bus.
        from scservo_sdk import PortHandler
        from scservo_sdk.hls import hls
        handler = PortHandler(port)
        output = None
        try:
            if not handler.openPort() or not handler.setBaudRate(baudrate):
                raise RuntimeError(f"cannot open Feetech port {port} at {baudrate}")
            handler.ser.exclusive = True
            output = cls(hls(handler), servo_id, handler.closePort, mode=mode)
            output.initialize()
            return output
        except Exception:
            if output is not None and output.touched:
                try:
                    output.close()
                except Exception:
                    pass
            elif handler.is_open:
                handler.closePort()
            raise

    def _check(self, result):
        communication, error = result
        if communication == -6:
            raise RuntimeError("No servo response (communication=-6). Check servo ID, baudrate, "
                               "power and half-duplex TTL wiring; use feetech_probe to scan.")
        if communication != 0:
            raise RuntimeError(f"Feetech communication failed: communication={communication}, servo={error}")
        if error:
            details = describe_servo_status(error)
            if error & 1:
                readings = read_voltage_diagnostics(self.packet, self.servo_id)
                voltage, low, high = (readings[name] for name in
                                      ("voltage_v", "min_voltage_v", "max_voltage_v"))
                if voltage is not None:
                    details += f"; measured={voltage:.1f}V"
                if low is not None and high is not None:
                    details += f"; configured voltage limits={low:.1f}..{high:.1f}V"
                    if voltage is not None and voltage > high:
                        details += "; OVERVOLTAGE: disconnect power and verify the servo supply"
                    elif voltage is not None and voltage < low:
                        details += "; UNDERVOLTAGE: verify the servo supply and wiring"
            raise RuntimeError(f"HLS servo fault (servo={error}): {details}; force output inhibited")

    def _read_byte(self, address):
        value, communication, error = self.packet.read1ByteTxRx(self.servo_id, address)
        self._check((communication, error))
        return value

    def _write_byte(self, address, value):
        self.touched = True
        if self.expect_write_reply:
            self._check(self.packet.write1ByteTxRx(self.servo_id, address, value))
        else:
            result = self.packet.write1ByteTxOnly(self.servo_id, address, value)
            self._check((result, 0))
            if self._read_byte(address) != value:
                raise RuntimeError(f"HLS write verification failed at register {address}")

    @staticmethod
    def _validate_raw(raw):
        if isinstance(raw, bool) or not isinstance(raw, int) or not 0 <= raw <= MAX_RAW:
            raise ValueError(f"feedback raw register value must be an integer in 0..{MAX_RAW}")

    def _write_word(self, address, value):
        if address == GOAL_CURRENT:
            self._validate_raw(value)  # Guard the final UART boundary too.
        if isinstance(value, bool) or not isinstance(value, int) or not 0 <= value <= 0xffff:
            raise ValueError('Invalid HLS word value')
        self.touched = True
        if self.expect_write_reply:
            self._check(self.packet.write2ByteTxRx(self.servo_id, address, value))
        else:
            self._check((self.packet.write2ByteTxOnly(self.servo_id, address, value), 0))
            actual, communication, error = self.packet.read2ByteTxRx(self.servo_id, address)
            self._check((communication, error))
            if actual != value:
                raise RuntimeError(f"HLS write verification failed at register {address}")

    def _write_current(self, raw):
        self._write_word(GOAL_CURRENT, raw)

    def _write_speed(self, speed, direction=1):
        if (isinstance(speed, bool) or not isinstance(speed, int) or not 0 <= speed <= MAX_RESISTANCE_SPEED
                or isinstance(direction, bool) or direction not in (-1, 1)):
            raise ValueError('Invalid resistance speed/direction')
        # HLS WriteSpe/WriteSpec encode direction in SPEED only, never in current.
        self._write_word(GOAL_SPEED, speed | (0x8000 if speed and direction < 0 else 0))

    def initialize(self):
        # Identify the read/ack path before writing anything to this bus.
        mode = self._read_byte(OPERATING_MODE)
        if mode not in (0, 1, 2, 3):
            raise RuntimeError(f"unexpected HLS operating mode: {mode}")
        response_level = self._read_byte(RESPONSE_LEVEL)
        if response_level not in (0, 1):
            raise RuntimeError(f"unexpected HLS response level: {response_level}")
        self.expect_write_reply = response_level == 1
        if self.mode == 0:
            limits = []
            for address in (9, 11):
                value, communication, error = self.packet.read2ByteTxRx(self.servo_id, address)
                self._check((communication, error))
                limits.append(value)
            low, high = limits
            if not (low == high == 0 or 0 <= low < high <= 4095):
                raise RuntimeError('Invalid HLS position limits; wall output inhibited')
            self.position_limits = (-32767, 32767) if low == high == 0 else (low, high)
        self.release()
        self.previous_mode = mode
        # Lock=1 makes changes temporary. Never unlock EEPROM or save mode.
        self._write_byte(LOCK, 1)
        self._write_byte(OPERATING_MODE, self.mode)
        if self._read_byte(OPERATING_MODE) != self.mode:
            raise RuntimeError("HLS feedback mode could not be verified")

    def check_health(self):
        status = self._read_byte(PRESENT_STATUS)
        if status:
            self._check((0, status))
        if self._read_byte(OPERATING_MODE) != self.mode:
            raise RuntimeError("HLS left feedback mode; reconnect feedback")

    def read_error_diagnostics(self):
        """Read status first, accepting valid payloads even with alarm replies.

        Stop on a transport failure: a disconnected bus must not accumulate
        telemetry timeouts and delay release. No writes or recursive _check.
        """
        result = dict(status_available=False, status_raw=None, status_flags=None,
                      status_text='unavailable', reads={})
        for address, name, scale in ((65, 'status', 1), (40, 'torque_enable', 1),
                                     (33, 'mode', 1), (62, 'voltage_v', 0.1),
                                     (63, 'temperature_c', 1)):
            try:
                value, communication, error = self.packet.read1ByteTxRx(self.servo_id, address)
                communication = communication if isinstance(communication, int) and not isinstance(communication, bool) else None
                error = error if isinstance(error, int) and not isinstance(error, bool) and 0 <= error <= 255 else None
                valid = (communication == 0 and isinstance(value, int) and not isinstance(value, bool)
                         and 0 <= value <= 255 and error is not None)
                result['reads'][name] = dict(communication=communication, reply_error=error,
                                             value=value if valid else None)
                if not valid:
                    break
                if name == 'status':
                    result.update(status_available=True, status_raw=value,
                                  status_flags=value | error,
                                  status_text=describe_servo_status(value | error))
                else:
                    result[name] = round(value * scale, 1) if scale != 1 else value
            except Exception as exc:
                result['reads'][name] = dict(error=str(exc))
                break
        return result

    def read_position_deg(self):
        communication = error = None
        try:
            position, communication, error = self.packet.ReadPos(self.servo_id)
            self._check((communication, error))
            if isinstance(position, bool) or not isinstance(position, (int, float)) or not math.isfinite(position):
                raise RuntimeError('Invalid HLS position feedback')
        except Exception as exc:
            diagnostics = self.read_error_diagnostics()
            diagnostics.update(original_error=str(exc), angle_communication=communication,
                               angle_reply_error=error)
            status = f"status65={diagnostics['status_raw']}, {diagnostics['status_text']}" \
                if diagnostics['status_available'] else 'status65 unreadable (communication failure)'
            raise ServoAngleReadError(f'Angle read failed: {exc}; {status}', diagnostics) from exc
        # The SDK already decodes the BIT15 direction flag, including multi-turn positions.
        return position * 360.0 / 4096.0

    def set_raw(self, raw):
        self.set_resistance(raw)

    def _select_mode(self, mode):
        """Switch between position walls and directed return with output off."""
        if self.mode == mode:
            return
        self.release()
        self._write_byte(OPERATING_MODE, mode)
        if self._read_byte(OPERATING_MODE) != mode:
            raise RuntimeError('HLS feedback mode switch could not be verified')
        self.mode = mode

    def set_resistance(self, raw, *, direction=1, speed_raw=2):
        self._validate_raw(raw)
        if (isinstance(direction, bool) or direction not in (-1, 1) or isinstance(speed_raw, bool)
                or not isinstance(speed_raw, int) or not 1 <= speed_raw <= MAX_RESISTANCE_SPEED):
            raise ValueError('Invalid resistance speed/direction')
        if self.closed:
            raise RuntimeError("Feetech port is closed")
        if raw == self.last_raw and (raw == 0 or (direction, speed_raw) == (self.last_direction, self.last_speed)):
            return
        try:
            if raw == 0:
                self.release()
                return
            self._select_mode(1)
            changed = (direction, speed_raw) != (self.last_direction, self.last_speed)
            if changed or not self.last_raw:
                if self.last_raw != 0:
                    self.release()
                # Current is zero before changing speed, even on auto-enabling firmware.
                self._write_speed(speed_raw, direction)
                self._write_current(raw)
                self._write_byte(TORQUE_ENABLE, 1)
            else:
                self._write_current(raw)
            self.last_raw, self.last_direction, self.last_speed = raw, direction, speed_raw
        except Exception:
            self.last_raw = None
            try:
                self.release()
            except Exception:
                pass
            raise

    def set_wall(self, raw, *, target_deg, speed_raw=2):
        """Hold a fixed boundary in position mode; the caller gates the free side.

        Position sign belongs to WORD42 only. Never infer a current direction.
        Validate factory limits rather than silently wrapping/clamping a wall.
        """
        self._validate_raw(raw)
        if (isinstance(target_deg, bool) or not isinstance(target_deg, (int, float))
                or not math.isfinite(target_deg) or abs(target_deg) > 32767 * 360 / 4096):
            raise ValueError('Invalid wall angle')
        if isinstance(speed_raw, bool) or not isinstance(speed_raw, int) or not 1 <= speed_raw <= MAX_RESISTANCE_SPEED:
            raise ValueError('Invalid wall return speed')
        position = round(target_deg * 4096.0 / 360.0)
        if not -32767 <= position <= 32767:
            raise ValueError('Wall angle exceeds HLS position range')
        if self.position_limits is None:
            raise RuntimeError('Initialize position mode before setting a wall')
        if not self.position_limits[0] <= position <= self.position_limits[1]:
            raise ValueError('Wall angle exceeds configured servo travel')
        if self.closed:
            raise RuntimeError('Feetech port is closed')
        if raw == 0:
            if self.last_raw != 0:
                self.release()
            return
        if (raw, position, speed_raw) == (self.last_raw, self.last_position, self.last_speed):
            return
        try:
            self._select_mode(0)
            if position != self.last_position or speed_raw != self.last_speed or not self.last_raw:
                # No stale target or current may be active while retargeting.
                if self.last_raw != 0:
                    self.release()
                self._write_word(GOAL_POSITION, abs(position) | (0x8000 if position < 0 else 0))
                self._write_speed(speed_raw)
                self._write_current(raw)
                self._write_byte(TORQUE_ENABLE, 1)
            else:
                self._write_current(raw)
            self.last_raw, self.last_position, self.last_speed = raw, position, speed_raw
        except Exception:
            self.last_raw = None
            try:
                self.release()
            except Exception:
                pass
            raise

    def release(self):
        self.last_raw = None
        self.last_direction, self.last_speed = None, 0
        self.last_position = None
        try:
            self._write_byte(TORQUE_ENABLE, 0)
        finally:
            try:
                self._write_current(0)
            finally:
                self._write_speed(0)
        self.last_raw = 0

    def close(self):
        if self.closed:
            return
        try:
            self.release()
            if self.previous_mode is not None:
                self._write_byte(OPERATING_MODE, self.previous_mode)
        finally:
            self.closed = True
            self.close_port()
