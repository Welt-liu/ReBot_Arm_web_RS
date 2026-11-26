"""Run actual official HLS packet encoding against an in-memory servo bus.

Run with .deps/FTServo_Python on PYTHONPATH after setup_rs_feedback.sh.
"""
import pytest

HlsPacket = pytest.importorskip('scservo_sdk.hls').hls

from rebotarmcontroller.feetech_feedback import FeetechForceOutput


class ServoBus:
    def __init__(self, response_level=1, alarm=0):
        self.is_using = False
        self.registers = bytearray(256)
        self.registers[8] = response_level
        self.registers[14], self.registers[15], self.registers[62] = 100, 40, 60
        self.registers[65] = alarm
        self.alarm = alarm
        self.rx = bytearray()
        self.packets = []

    def clearPort(self):
        self.rx.clear()

    def setPacketTimeout(self, _length):
        pass

    def isPacketTimeout(self):
        return not self.rx

    def writePort(self, packet):
        data = bytes(packet)
        self.packets.append(data)
        assert data[:2] == b'\xff\xff'
        assert sum(data[2:]) & 255 == 255
        instruction, address = data[4:6]
        payload = b''
        if instruction == 3:
            self.registers[address:address + len(data[6:-1])] = data[6:-1]
            if not self.registers[8]:
                return len(data)
        elif instruction == 2:
            payload = self.registers[address:address + data[6]]
        reply = bytes([255, 255, data[2], len(payload) + 2, self.alarm]) + payload
        self.rx.extend(reply + bytes([~sum(reply[2:]) & 255]))
        return len(data)

    def readPort(self, length):
        result = self.rx[:length]
        del self.rx[:length]
        return result


@pytest.mark.parametrize('response_level', [0, 1])
@pytest.mark.parametrize('raw', [100, 150])
def test_official_sdk_encodes_hls_current_word_and_releases(response_level, raw):
    bus = ServoBus(response_level)
    output = FeetechForceOutput(HlsPacket(bus), 1)
    output.initialize()
    output.set_raw(raw)
    assert bus.registers[33] == 1
    assert bus.registers[40] == 1
    assert bus.registers[44:46] == bytes([raw, 0])
    assert bus.registers[48:50] == b'\x00\x00'  # STS limit was never used.
    assert any(p[4:8] == bytes([3, 44, raw, 0]) for p in bus.packets)
    output.check_health()
    output.close()
    assert bus.registers[33] == 0
    assert bus.registers[40] == 0
    assert bus.registers[44:46] == b'\x00\x00'


def test_probe_only_sends_read_instructions():
    from rebotarmcontroller.feetech_probe import probe_servos
    bus = ServoBus()
    devices = list(probe_servos(HlsPacket(bus), [1]))
    assert devices[0]['servo_id'] == 1
    assert devices[0]['response_level'] == 1
    assert all(packet[4] == 2 for packet in bus.packets)
    assert bus.registers[33] == 0 and bus.registers[40] == 0


def test_probe_retains_voltage_readings_when_reply_contains_alarm():
    from rebotarmcontroller.feetech_probe import probe_servos
    bus = ServoBus(alarm=1)
    bus.registers[62] = 121
    device = list(probe_servos(HlsPacket(bus), [1]))[0]
    assert device['voltage_v'] == 12.1
    assert device['max_voltage_v'] == 10.0
    assert device['status_text'] == 'input voltage fault'
    assert all(packet[4] == 2 for packet in bus.packets)


@pytest.mark.parametrize('raw,degrees', [(1024, 90), (0x8400, -90)])
def test_actual_sdk_angle_decode_includes_direction(raw, degrees):
    bus = ServoBus()
    bus.registers[56:58] = raw.to_bytes(2, 'little')
    output = FeetechForceOutput(HlsPacket(bus), 1)
    assert output.read_position_deg() == degrees
    assert all(packet[4] == 2 for packet in bus.packets)


@pytest.mark.parametrize('response_level', [0, 1])
def test_reversing_changes_speed_only_and_releases_before_switching(response_level):
    bus = ServoBus(response_level)
    output = FeetechForceOutput(HlsPacket(bus), 1)
    output.initialize()
    output.set_resistance(150, direction=1, speed_raw=2)
    assert bus.registers[33] == 1
    assert bus.registers[44:46] == b'\x96\x00'
    assert bus.registers[46:48] == b'\x02\x00'
    before = len(bus.packets)
    output.set_resistance(150, direction=-1, speed_raw=2)
    writes = [(p[5], int.from_bytes(p[6:-1], 'little')) for p in bus.packets[before:] if p[4] == 3]
    assert writes == [(40, 0), (44, 0), (46, 0), (46, 0x8002), (44, 150), (40, 1)]
    assert bus.registers[44:46] == b'\x96\x00'
    assert bus.registers[46:48] == b'\x02\x80'
    for packet in bus.packets:
        if packet[4:6] == bytes([3, 44]):
            assert 0 <= int.from_bytes(packet[6:-1], 'little') <= 150
    output.close()
    assert bus.registers[40] == 0
    assert bus.registers[44:48] == b'\x00' * 4


@pytest.mark.parametrize('direction', [-1, 1])
@pytest.mark.parametrize('raw', [1, 100, 150])
@pytest.mark.parametrize('speed', [1, 10])
def test_both_directions_keep_actual_current_word_bounded(direction, raw, speed):
    bus = ServoBus()
    output = FeetechForceOutput(HlsPacket(bus), 1)
    output.initialize()
    output.set_resistance(raw, direction=direction, speed_raw=speed)
    assert int.from_bytes(bus.registers[44:46], 'little') == raw
    assert int.from_bytes(bus.registers[46:48], 'little') == speed | (0x8000 if direction < 0 else 0)
    output.close()


@pytest.mark.parametrize('raw', [-150, -1, 151, 0x8096, 0xffff, True, 1.5])
def test_encoded_or_negative_current_never_reaches_bus(raw):
    bus = ServoBus()
    output = FeetechForceOutput(HlsPacket(bus), 1)
    for write in (output.set_raw, output.set_resistance, output._write_current,
                  lambda value: output._write_word(44, value)):
        with pytest.raises(ValueError):
            write(raw)
    assert bus.packets == []


@pytest.mark.parametrize('options', [dict(speed_raw=0), dict(speed_raw=11), dict(speed_raw=True),
                                     dict(direction=0), dict(direction=True)])
def test_bad_speed_settings_rejected_before_bus_access(options):
    bus = ServoBus()
    output = FeetechForceOutput(HlsPacket(bus), 1)
    with pytest.raises(ValueError):
        output.set_resistance(100, **options)
    assert bus.packets == []


def test_wasm_bridge_reverse_and_live_speed_keep_real_wire_current_bounded(monkeypatch):
    from rebotarmcontroller import feetech_wasm_bridge as bridge
    now = [10.0]
    monkeypatch.setattr(bridge.time, 'monotonic', lambda: now[0])
    monkeypatch.setattr(bridge.time, 'time', lambda: 1000.0)
    bus = ServoBus()
    output = FeetechForceOutput(HlsPacket(bus), 1, mode=0)
    output.initialize()
    controller = bridge.WasmFeedback(output)
    token = controller.connect()['token']
    for seq, (direction, speed) in enumerate([(1, 2), (-1, 2), (-1, 10), (1, 1)]):
        now[0] += .02
        controller.sample(dict(token=token, seq=seq, at_ms=1000000, force_n=20,
                               gripper_direction=direction, settings={'resistance_speed': speed}))
        controller.tick()
        assert not controller.status()['fault']
        assert bus.registers[33] == 0
        assert bus.registers[42:44] == b'\x00\x00'
        assert controller.status()['raw'] == 150
        assert int.from_bytes(bus.registers[44:46], 'little') == 150
        assert int.from_bytes(bus.registers[46:48], 'little') == speed
    controller.disconnect(token)
    controller.tick()
    assert bus.registers[40] == 0
    assert bus.registers[44:48] == b'\x00' * 4
    assert all(0 <= int.from_bytes(p[6:-1], 'little') <= 150
               for p in bus.packets if p[4:6] == bytes([3, 44]))
    controller.close()


@pytest.mark.parametrize('response_level', [0, 1])
@pytest.mark.parametrize('angle,encoded', [(90, 1024), (-90, 0x8400)])
def test_position_wall_writes_fixed_target_and_nonnegative_current(response_level, angle, encoded):
    bus = ServoBus(response_level)
    bus.registers[33] = 1
    output = FeetechForceOutput(HlsPacket(bus), 1, mode=0)
    output.initialize()
    assert bus.registers[33] == 0
    output.set_wall(150, target_deg=angle)
    assert int.from_bytes(bus.registers[42:44], 'little') == encoded
    assert int.from_bytes(bus.registers[44:46], 'little') == 150
    assert int.from_bytes(bus.registers[46:48], 'little') == 2
    before = len(bus.packets)
    output.set_wall(100, target_deg=angle)
    writes = [p for p in bus.packets[before:] if p[4] == 3]
    assert len(writes) == 1 and writes[0][5:8] == bytes([44, 100, 0])
    before = len(bus.packets)
    output.set_wall(100, target_deg=-angle)
    writes = [(p[5], int.from_bytes(p[6:-1], 'little')) for p in bus.packets[before:] if p[4] == 3]
    assert writes[:3] == [(40, 0), (44, 0), (46, 0)]
    assert writes[-1] == (40, 1)
    output.close()
    assert bus.registers[33] == 1  # Restore previous mode, with output disabled.
    assert bus.registers[40] == 0
    assert bus.registers[44:48] == b'\x00' * 4
    assert all(0 <= int.from_bytes(p[6:-1], 'little') <= 150
               for p in bus.packets if p[4:6] == bytes([3, 44]))


@pytest.mark.parametrize('angle', [-1, 361, float('nan'), float('inf'), True, 4000])
def test_position_wall_rejects_invalid_or_outside_factory_travel_before_write(angle):
    bus = ServoBus()
    bus.registers[11:13] = (4095).to_bytes(2, 'little')
    output = FeetechForceOutput(HlsPacket(bus), 1, mode=0)
    output.initialize()
    before = len(bus.packets)
    with pytest.raises(ValueError):
        output.set_wall(150, target_deg=angle)
    assert len(bus.packets) == before


@pytest.mark.parametrize('address,value', [(42, 1024), (46, 2), (44, 100), (40, 1)])
def test_position_wall_failure_at_each_enable_step_releases(address, value):
    bus = ServoBus()
    packet = HlsPacket(bus)
    output = FeetechForceOutput(packet, 1, mode=0)
    output.initialize()
    original = packet.write2ByteTxRx if address != 40 else packet.write1ByteTxRx
    def failed_write(sid, register, data):
        return (-6, 0) if (register, data) == (address, value) else original(sid, register, data)
    if address == 40:
        packet.write1ByteTxRx = failed_write
    else:
        packet.write2ByteTxRx = failed_write
    with pytest.raises(RuntimeError):
        output.set_wall(100, target_deg=90)
    assert bus.registers[40] == 0
    assert bus.registers[44:48] == b'\x00' * 4


@pytest.mark.parametrize('direction', [-1, 1])
def test_trigger_return_increases_mapped_width_and_switches_back_to_wall(monkeypatch, direction):
    from rebotarmcontroller import feetech_wasm_bridge as bridge
    now = [10.0]
    monkeypatch.setattr(bridge.time, 'monotonic', lambda: now[0])
    monkeypatch.setattr(bridge.time, 'time', lambda: 1000.0)
    bus = ServoBus()
    bus.registers[56:58] = (2048).to_bytes(2, 'little')  # 180° is 50 mm.
    output = FeetechForceOutput(HlsPacket(bus), 1, mode=0)
    output.initialize()
    controller = bridge.WasmFeedback(output)
    token = controller.connect()['token']
    controller.tick()
    # Move 22.5° away from 50 mm, in either mapping orientation.
    bus.registers[56:58] = (2048 - direction * 256).to_bytes(2, 'little')
    controller.sample(dict(token=token, seq=0, at_ms=1000000, force_n=0,
        grip_contacts=[], actual_width_mm=38.75, angle_control=True, gripper_direction=direction))
    now[0] += .02
    controller.tick()
    status = controller.status()
    assert not status['fault']
    assert status['trigger_return_active']
    assert status['raw'] == 30
    assert bus.registers[33] == 1
    speed = int.from_bytes(bus.registers[46:48], 'little')
    signed_speed = -(speed & 0x7fff) if speed & 0x8000 else speed
    assert direction * signed_speed > 0  # Returned width must INCREASE.
    assert int.from_bytes(bus.registers[44:46], 'little') == 30
    # At 50 mm the preload ends and the original position wall takes over.
    bus.registers[56:58] = (2048).to_bytes(2, 'little')
    now[0] += .02
    controller.tick()
    assert not controller.status()['trigger_return_active']
    assert controller.status()['wall_source'] == 'opening_limit'
    assert bus.registers[33] == 0
    assert int.from_bytes(bus.registers[42:44], 'little') == 2048
    assert int.from_bytes(bus.registers[44:46], 'little') == 150
    for packet in bus.packets:
        if packet[4:6] == bytes([3, 33]):
            assert packet[6] in (0, 1)
        if packet[4:6] == bytes([3, 44]):
            assert 0 <= int.from_bytes(packet[6:-1], 'little') <= 150
    controller.close()


@pytest.mark.parametrize('direction', [-1, 1])
def test_zero_wall_keeps_thirty_preload_but_deeper_closing_uses_hard_stop(monkeypatch, direction):
    from rebotarmcontroller import feetech_wasm_bridge as bridge
    now = [10.0]
    monkeypatch.setattr(bridge.time, 'monotonic', lambda: now[0])
    monkeypatch.setattr(bridge.time, 'time', lambda: 1000.0)
    bus = ServoBus()
    bus.registers[56:58] = (2048).to_bytes(2, 'little')
    output = FeetechForceOutput(HlsPacket(bus), 1, mode=0)
    output.initialize()
    controller = bridge.WasmFeedback(output)
    token = controller.connect()['token']
    controller.tick()
    controller.sample(dict(token=token, seq=0, at_ms=1000000, force_n=0,
        grip_contacts=[], actual_width_mm=0, angle_control=True, gripper_direction=direction))
    zero_count = round(2048 - direction * 100 * 4096 / 360)
    bus.registers[56:58] = zero_count.to_bytes(2, 'little')
    now[0] += .02
    controller.tick()
    assert not controller.status()['fault']
    assert controller.status()['trigger_return_active']
    assert int.from_bytes(bus.registers[44:46], 'little') == 30
    assert bus.registers[33] == 1
    speed = int.from_bytes(bus.registers[46:48], 'little')
    signed_speed = -(speed & 0x7fff) if speed & 0x8000 else speed
    assert direction * signed_speed > 0
    bus.registers[56:58] = (zero_count - direction * 2).to_bytes(2, 'little')
    now[0] += .02
    controller.tick()
    assert not controller.status()['fault']
    assert not controller.status()['trigger_return_active']
    assert controller.status()['wall_source'] == 'closing_limit'
    assert int.from_bytes(bus.registers[44:46], 'little') == 150
    assert bus.registers[33] == 0
    controller.close()


@pytest.mark.parametrize('direction', [-1, 1])
@pytest.mark.parametrize('offset_count', [0, 256, -1280])
def test_released_wall_returns_thirty_even_outside_range_and_holds_at_fifty(monkeypatch, direction, offset_count):
    from rebotarmcontroller import feetech_wasm_bridge as bridge
    now = [10.0]
    monkeypatch.setattr(bridge.time, 'monotonic', lambda: now[0])
    monkeypatch.setattr(bridge.time, 'time', lambda: 1000.0)
    bus = ServoBus()
    bus.registers[56:58] = (2048).to_bytes(2, 'little')
    output = FeetechForceOutput(HlsPacket(bus), 1, mode=0)
    output.initialize()
    controller = bridge.WasmFeedback(output)
    token = controller.connect()['token']
    controller.tick()
    controller.sample(dict(token=token, seq=0, at_ms=1000000, force_n=0,
        grip_contacts=[], actual_width_mm=50, angle_control=True, gripper_direction=direction))
    controller.tick()
    position = 2048 + direction * offset_count
    angle = position * 360 / 4096
    bus.registers[56:58] = position.to_bytes(2, 'little')
    # Model each endpoint's normal release latch, before rearming its wall.
    controller.open_limit.released = controller.close_limit.released = True
    controller.open_limit.high = -direction * angle
    controller.close_limit.high = direction * angle
    now[0] += .02
    controller.tick()
    status = controller.status()
    assert not status['fault']
    assert status['wall_released'] and status['trigger_return_active']
    assert status['raw'] == 30
    if offset_count:
        assert bus.registers[33] == 1
        speed = int.from_bytes(bus.registers[46:48], 'little')
        signed_speed = -(speed & 0x7fff) if speed & 0x8000 else speed
        assert (180 - angle) * signed_speed > 0
    else:
        assert bus.registers[33] == 0
        assert int.from_bytes(bus.registers[42:44], 'little') == 2048
        assert status['trigger_return_direction'] == 0
    controller.close()
