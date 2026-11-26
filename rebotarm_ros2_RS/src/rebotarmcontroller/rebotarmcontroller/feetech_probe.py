"""Read-only HLS discovery. Never enables torque or writes a register."""
from __future__ import annotations

import argparse
import sys
import time

from .feetech_feedback import describe_servo_status, read_voltage_diagnostics

# The pinned Python SDK does not accept 76800 in PortHandler.getCFlagBaud.
BAUDRATES = (1_000_000, 115_200, 500_000, 250_000, 128_000, 57_600, 38_400)


def parse_ids(value):
    try:
        result = set()
        for item in value.split(','):
            bounds = item.split('-')
            if len(bounds) == 1:
                result.add(int(item))
            elif len(bounds) == 2:
                start, end = map(int, bounds)
                if start > end:
                    raise ValueError()
                result.update(range(start, end + 1))
            else:
                raise ValueError()
        if not result or min(result) < 0 or max(result) > 253:
            raise ValueError()
        return sorted(result)
    except ValueError:
        raise argparse.ArgumentTypeError('IDs must be 0..253, e.g. 1,6 or 0-253') from None


def probe_servos(packet, ids):
    for servo_id in ids:
        model, communication, status = packet.read2ByteTxRx(servo_id, 3)
        if communication != 0:
            continue
        fields = dict(servo_id=servo_id, model=model, status=status)
        for address, name in ((8, 'response_level'), (33, 'mode'), (40, 'enabled'),
                              (65, 'register_status'), (63, 'temperature_c')):
            value, result, error = packet.read1ByteTxRx(servo_id, address)
            fields[name] = value if result == 0 else None
        fields['status_text'] = describe_servo_status(status | (fields['register_status'] or 0))
        fields.update(read_voltage_diagnostics(packet, servo_id))
        yield fields


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True)
    parser.add_argument('--ids', type=parse_ids, default=parse_ids('0-253'))
    parser.add_argument('--baudrates', nargs='+', type=int, choices=BAUDRATES, default=BAUDRATES)
    options = parser.parse_args(args)
    from scservo_sdk import PortHandler
    from scservo_sdk.hls import hls
    port = PortHandler(options.port)
    try:
        for baudrate in options.baudrates:
            if not port.setBaudRate(baudrate):
                raise RuntimeError(f'Cannot open port at {baudrate} baud')
            port.ser.exclusive = True
            time.sleep(0.2)
            print(f'Read-only scan: baud={baudrate}, IDs={options.ids[0]}..{options.ids[-1]}', flush=True)
            found = False
            for device in probe_servos(hls(port), options.ids):
                found = True
                print(f'FOUND baudrate={baudrate} {device}', flush=True)
                print(f'Use: -p servo_id:={device["servo_id"]} -p baudrate:={baudrate}', flush=True)
            if found:
                return 0
        print('No servo answered. Check servo power, common GND, half-duplex TTL wiring, '
              'and whether another program owns this serial port.', file=sys.stderr)
        return 1
    except (OSError, RuntimeError) as exc:
        print(f'Probe failed: {exc}', file=sys.stderr)
        return 1
    finally:
        if port.is_open:
            port.closePort()


if __name__ == '__main__':
    sys.exit(main())
