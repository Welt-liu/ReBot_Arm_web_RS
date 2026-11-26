import math
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from rebotarmcontroller import feetech_wasm_bridge as bridge
from rebotarmcontroller.feetech_feedback import ServoAngleReadError


@pytest.mark.parametrize('force,raw', [(0, 0), (0.5, 0), (2.45, 35), (4.4, 70),
                                      (7, 117), (8.86, 150), (10.25, 150), (20, 150), (1e308, 150)])
def test_contact_force_maps_to_bounded_raw(force, raw):
    assert bridge.force_to_raw(force) == raw


@pytest.mark.parametrize('force,scale', [(math.nan, 20), (math.inf, 20), (-1, 20),
                                       (True, 20), (1, 0.5), (1, 1001)])
def test_invalid_contact_force_settings_are_rejected(force, scale):
    with pytest.raises(ValueError):
        bridge.force_to_raw(force, scale)


@pytest.fixture
def state(monkeypatch):
    clock = SimpleNamespace(mono=10, wall=1000)
    monkeypatch.setattr(bridge.time, 'monotonic', lambda: clock.mono)
    monkeypatch.setattr(bridge.time, 'time', lambda: clock.wall)
    output = Mock()
    output.read_position_deg.return_value = 0.0
    controller = bridge.WasmFeedback(output)
    token = controller.connect()['token']
    return controller, output, clock, token


def payload(token, clock, **updates):
    result = dict(token=token, seq=0, at_ms=clock.wall * 1000, force_n=20)
    result.update(updates)
    return result


def test_wasm_contact_reaches_servo_without_ros(state):
    controller, output, clock, token = state
    controller.sample(payload(token, clock))
    controller.tick()
    output.set_wall.assert_called_with(150, target_deg=0, speed_raw=2)
    assert controller.status()['source'] == 'wasm'
    assert controller.status()['force_gain'] == 3.5
    assert controller.status()['gripper_mm_per_deg'] == 0.5
    assert controller.status()['max_raw'] == 150
    controller.sample(payload(token, clock, seq=1, force_n=0))
    controller.tick()
    output.release.assert_called()
    assert controller.status()['raw'] == 0


def test_browser_stall_releases_force_and_expires_session(state):
    controller, output, clock, token = state
    controller.sample(payload(token, clock))
    controller.tick()
    clock.mono += 0.31
    controller.tick()
    output.release.assert_called()
    assert controller.status()['raw'] == 0
    assert not controller.status()['connected']
    with pytest.raises(ValueError):
        controller.sample(payload(token, clock, seq=1))


@pytest.mark.parametrize('updates', [dict(seq=-1), dict(at_ms=999000),
                                   dict(at_ms=math.nan), dict(force_n=math.nan)])
def test_invalid_or_stale_browser_data_releases(state, updates):
    controller, output, clock, token = state
    with pytest.raises(ValueError):
        controller.sample(payload(token, clock, **updates))
    controller.tick()
    output.release.assert_called()
    assert controller.status()['raw'] == 0
    assert not controller.status()['connected']


def test_second_browser_cannot_take_control(state):
    controller, output, clock, token = state
    with pytest.raises(RuntimeError, match='Another browser'):
        controller.connect()
    with pytest.raises(ValueError):
        controller.disconnect('wrong token')
    assert controller.status()['connected']


def test_explicit_test_is_fixed_at_ten_and_release_uses_contact_force(state):
    controller, output, clock, token = state
    controller.sample(payload(token, clock, force_n=0, test=True))
    controller.tick()
    output.set_wall.assert_called_with(10, target_deg=0, speed_raw=2)
    controller.sample(payload(token, clock, seq=1, force_n=0, test=False))
    controller.tick()
    output.release.assert_called()
    assert controller.status()['raw'] == 0


def test_servo_fault_latches_and_releases(state):
    controller, output, clock, token = state
    output.check_health.side_effect = RuntimeError('overvoltage')
    controller.sample(payload(token, clock))
    controller.tick()
    output.set_wall.assert_not_called()
    output.release.assert_called()
    assert controller.status()['fault'] == 'overvoltage'
    with pytest.raises(RuntimeError):
        controller.connect()


def test_disconnect_and_close_do_not_resume_force(state):
    controller, output, clock, token = state
    controller.sample(payload(token, clock))
    controller.disconnect(token)
    controller.tick()
    output.release.assert_called()
    assert controller.status()['raw'] == 0
    controller.close()
    controller.close()
    output.close.assert_called_once()


@pytest.mark.parametrize('angle,zero,direction,width', [
    (30, 0, 1, 15), (-42, -62, 1, 10), (-20, 0, 1, 0),
    (80, 0, 1, 40), (120, 0, 1, 50), (-20, 0, -1, 10), (380, 360, 1, 10)])
def test_servo_angle_maps_one_degree_to_half_mm(angle, zero, direction, width):
    assert bridge.angle_to_gripper_mm(angle, zero, direction) == width


def test_angle_is_returned_with_software_zero_and_recalibration(state):
    controller, output, clock, token = state
    output.read_position_deg.return_value = -62
    controller.tick()
    assert controller.status()['zero_angle_deg'] == -62
    output.read_position_deg.return_value = -42
    controller.tick()
    assert controller.status()['angle_deg'] == -42
    assert controller.status()['gripper_mm'] == 10
    controller.calibrate_gripper(token)
    assert controller.status()['zero_angle_deg'] == -42
    assert controller.status()['gripper_mm'] == 0


def test_angle_disable_reverse_and_stale_telemetry(state):
    controller, output, clock, token = state
    controller.tick()
    output.read_position_deg.return_value = -30
    controller.tick()
    controller.sample(payload(token, clock, force_n=0, angle_control=True, gripper_direction=-1))
    assert controller.status()['gripper_mm'] == 15
    controller.sample(payload(token, clock, seq=1, force_n=0, angle_control=False))
    assert not controller.status()['angle_control']
    clock.mono += 0.31
    assert controller.status()['angle_deg'] is None
    assert controller.status()['gripper_mm'] is None
    with pytest.raises(RuntimeError):
        controller.calibrate_gripper(token)


def test_bad_angle_feedback_releases_both_directions_of_control(state):
    controller, output, clock, token = state
    output.read_position_deg.return_value = math.nan
    controller.tick()
    assert controller.status()['fault']
    assert controller.status()['gripper_mm'] is None
    output.release.assert_called()
    output.set_wall.assert_not_called()


def test_rich_contacts_live_settings_reverse_and_stall_release(state):
    controller, output, clock, token = state
    controller.tick()
    rich = dict(grip_contacts=[dict(id='red', left_n=3, right_n=3, relative_mm=[0, 0, 0])],
                actual_width_mm=20, target_width_mm=19, gripper_direction=-1, angle_control=False,
                settings=dict(stiffness=10, max_raw=80, force_gain=2))
    controller.sample(payload(token, clock, **rich))
    controller.tick()
    assert controller.status()['grip_state'] == 'touch'
    assert controller.status()['settings']['stiffness'] == 10
    assert 0 < controller.status()['raw'] <= 12
    output.set_wall.assert_called_with(controller.status()['raw'], target_deg=0, speed_raw=2)
    clock.mono += .12
    controller.sample(payload(token, clock, seq=1, **rich))
    controller.tick()
    assert controller.status()['grip_state'] == 'gripping'
    assert controller.status()['spring_term'] == pytest.approx(8)
    rich['settings']['max_raw'] = 5
    clock.mono += .02
    controller.sample(payload(token, clock, seq=2, **rich))
    controller.tick()
    assert controller.status()['raw'] <= 5
    clock.mono += .31
    controller.tick()
    output.release.assert_called()
    assert controller.status()['raw'] == 0
    assert controller.status()['grip_state'] == 'free'
    assert controller.status()['target_speed_raw'] == 0


@pytest.mark.parametrize('extra', [dict(settings={'max_raw': 151}), dict(settings=[]),
    dict(grip_contacts=[dict(id='red', left_n=1, right_n=1, relative_mm=[float('nan'), 0, 0])]),
    dict(actual_width_mm=float('nan'))])
def test_bad_live_setting_or_metadata_releases(state, extra):
    controller, output, clock, token = state
    with pytest.raises(ValueError):
        controller.sample(payload(token, clock, **extra))
    controller.tick()
    output.release.assert_called()
    assert controller.status()['raw'] == 0
    assert not controller.status()['connected']


def test_live_angle_scale_and_settings_on_connect(state):
    controller, output, clock, token = state
    controller.disconnect(token)
    controller.connect({'angle_scale': 1, 'force_gain': 2})
    controller.tick()
    output.read_position_deg.return_value = 20
    controller.tick()
    assert controller.status()['gripper_mm'] == 20
    assert controller.status()['force_gain'] == 2


@pytest.fixture
def selectable(monkeypatch):
    clock = SimpleNamespace(mono=10, wall=1000)
    monkeypatch.setattr(bridge.time, 'monotonic', lambda: clock.mono)
    monkeypatch.setattr(bridge.time, 'time', lambda: clock.wall)
    outputs = []
    def factory(port):
        output = Mock()
        output.read_position_deg.return_value = 15.0
        outputs.append((port, output))
        return output
    devices = [dict(device='/dev/serial/by-id/servo-a', label='Servo A'),
               dict(device='/dev/serial/by-id/servo-b', label='Servo B')]
    controller = bridge.WasmFeedback(output_factory=factory, ports_provider=lambda: devices.copy())
    return controller, outputs, devices, clock


def test_bridge_waits_for_explicit_port_and_listing_never_opens_it(selectable):
    controller, outputs, devices, clock = selectable
    controller.tick()
    assert controller.ports()['ports'] == devices
    assert not controller.status()['serial_open']
    assert outputs == []
    for port in (None, '', '/tmp/not-a-serial-device'):
        with pytest.raises(ValueError):
            controller.connect(port=port)
    assert outputs == []


def test_selected_port_only_opens_then_disconnect_releases_and_allows_switch(selectable):
    controller, outputs, devices, clock = selectable
    result = controller.connect(port=devices[0]['device'])
    first = outputs[0][1]
    assert outputs[0][0] == devices[0]['device']
    assert result['port'] == devices[0]['device']
    assert result['gripper_mm'] == 0
    controller.sample(payload(result['token'], clock))
    controller.tick()
    first.set_wall.assert_called_with(150, target_deg=controller.status()['wall_angle_deg'], speed_raw=2)
    with pytest.raises(RuntimeError, match='Another browser'):
        controller.connect(port=devices[1]['device'])
    with pytest.raises(ValueError):
        controller.disconnect('wrong-token')
    first.close.assert_not_called()
    assert len(outputs) == 1
    controller.disconnect(result['token'])
    first.close.assert_called_once()
    assert controller.status()['raw'] == 0
    assert not controller.status()['serial_open']
    controller.connect(port=devices[1]['device'])
    assert outputs[-1][0] == devices[1]['device']
    controller.close()
    outputs[-1][1].close.assert_called_once()


def test_timeout_closes_device_and_unplugged_port_cannot_reconnect(selectable):
    controller, outputs, devices, clock = selectable
    result = controller.connect(port=devices[0]['device'])
    clock.mono += .31
    controller.tick()
    assert not controller.status()['connected']
    assert not controller.status()['serial_open']
    outputs[0][1].close.assert_called_once()
    devices.clear()
    assert controller.ports()['ports'] == []
    with pytest.raises(ValueError, match='unavailable'):
        controller.connect(port=result['port'])
    assert len(outputs) == 1


def test_fault_closes_device_and_explicit_reconnect_can_retry(selectable):
    controller, outputs, devices, clock = selectable
    controller.connect(port=devices[0]['device'])
    outputs[0][1].check_health.side_effect = RuntimeError('USB disconnected')
    controller.tick()
    assert controller.status()['fault'] == 'USB disconnected'
    assert not controller.status()['serial_open']
    outputs[0][1].close.assert_called_once()
    result = controller.connect(port=devices[1]['device'])
    assert result['connected'] and not result['fault']
    controller.close()


def test_open_failure_stays_offline_and_invalid_settings_never_open(selectable):
    controller, outputs, devices, clock = selectable
    with pytest.raises(ValueError):
        controller.connect({'max_raw': 999}, port=devices[0]['device'])
    assert outputs == []
    bad = Mock()
    bad.read_position_deg.return_value = float('nan')
    controller.output_factory = lambda port: bad
    with pytest.raises(RuntimeError, match='Invalid servo angle'):
        controller.connect(port=devices[0]['device'])
    bad.close.assert_called_once()
    assert not controller.status()['serial_open']
    assert not controller.status()['connected']
    assert controller.status()['raw'] == 0


def test_serial_port_list_deduplicates_devices_and_prefers_stable_alias(tmp_path, monkeypatch):
    from serial.tools import list_ports
    device = tmp_path / 'ttyACM0'
    device.touch()
    alias = tmp_path / 'usb-servo-if00'
    alias.symlink_to(device)
    original_glob = bridge.Path.glob
    def glob(path, pattern):
        return iter([alias]) if str(path) == '/dev/serial/by-id' else original_glob(path, pattern)
    monkeypatch.setattr(bridge.Path, 'glob', glob)
    monkeypatch.setattr(list_ports, 'comports', lambda: [
        SimpleNamespace(device=str(device), description='USB servo'),
        SimpleNamespace(device=str(alias), description='USB servo')])
    assert bridge.serial_ports() == [dict(device=str(alias), label=f'{alias} · USB servo')]


@pytest.mark.parametrize('direction', [-1, 1])
def test_fixed_wall_local_reverse_release_without_browser_sample_and_rearm(state, direction):
    controller, output, clock, token = state
    controller.tick()  # Zero at current position.
    output.read_position_deg.return_value = direction * 40
    controller.tick()
    rich = dict(angle_control=True, gripper_direction=direction,
                grip_contacts=[dict(id='red', left_n=3, right_n=3, relative_mm=[0, 0, 0])],
                actual_width_mm=20, settings={'smoothing_ms': 200, 'slew_raw_s': 50})
    controller.sample(payload(token, clock, **rich))
    clock.mono += .02
    controller.tick()
    wall = controller.status()['wall_angle_deg']
    assert abs(wall - direction * 40) < 360 / 4096
    clock.mono += .11
    rich['actual_width_mm'] = 19  # Object compression cannot move the boundary.
    controller.sample(payload(token, clock, seq=1, **rich))
    controller.tick()
    assert controller.status()['wall_angle_deg'] == wall
    assert controller.status()['grip_state'] == 'gripping'
    output.read_position_deg.return_value = direction * 39
    clock.mono += .01
    controller.tick()
    assert controller.status()['raw'] > 0
    output.set_wall.reset_mock()
    output.read_position_deg.return_value = direction * 39.5
    clock.mono += .01
    controller.tick()  # No new contact/browser sample is needed for release.
    assert controller.status()['raw'] == 0
    assert controller.status()['wall_released']
    output.release.assert_called()
    output.set_wall.assert_not_called()
    clock.mono += .01
    controller.tick()
    assert controller.status()['raw'] == 0
    output.set_wall.assert_not_called()
    output.read_position_deg.return_value = direction * 39
    clock.mono += .02
    controller.tick()
    assert controller.status()['raw'] > 0
    assert controller.status()['wall_angle_deg'] == wall
    controller.sample(payload(token, clock, seq=2, angle_control=True, gripper_direction=direction,
                              grip_contacts=[], actual_width_mm=19, force_n=0))
    controller.tick()
    assert controller.status()['raw'] == 0
    assert controller.status()['wall_angle_deg'] is None


def test_recalibration_releases_power_before_next_tick(state):
    controller, output, clock, token = state
    controller.sample(payload(token, clock))
    controller.tick()
    assert controller.status()['raw'] > 0
    output.release.reset_mock()
    controller.calibrate_gripper(token)
    output.release.assert_called_once()
    assert controller.status()['raw'] == 0
    assert controller.status()['wall_angle_deg'] is None


def test_stalled_serial_read_cannot_enable_output_using_expired_contact(state):
    controller, output, clock, token = state
    controller.sample(payload(token, clock))
    def stalled_read():
        clock.mono += .31
        return 0
    output.read_position_deg.side_effect = stalled_read
    controller.tick()
    output.set_wall.assert_not_called()
    assert not controller.status()['connected']
    assert controller.status()['raw'] == 0


def test_first_contact_limits_powered_return_to_two_degrees(state):
    controller, output, clock, token = state
    controller.tick()
    controller.sample(payload(token, clock, angle_control=True, actual_width_mm=40,
        grip_contacts=[dict(id='red', left_n=3, right_n=3, relative_mm=[0, 0, 0])]))
    clock.mono += .02
    controller.tick()
    assert 0 <= controller.status()['wall_angle_deg'] <= 2 + 360 / 4096


def test_failed_reconfiguration_release_expires_session_and_cannot_resume(state):
    controller, output, clock, token = state
    controller.sample(payload(token, clock))
    controller.tick()
    output.release.side_effect = RuntimeError('UART release failed')
    with pytest.raises(RuntimeError, match='UART release failed'):
        controller.sample(payload(token, clock, seq=1, gripper_direction=-1))
    assert controller.status()['fault'] == 'UART release failed'
    assert not controller.status()['connected']
    output.release.side_effect = None
    output.set_wall.reset_mock()
    controller.tick()
    output.set_wall.assert_not_called()
    with pytest.raises(RuntimeError):
        controller.sample(payload(token, clock, seq=2))


def test_angle_diagnostic_survives_force_release_and_port_close(state):
    controller, output, clock, token = state
    diagnostic = dict(status_available=True, status_raw=8, status_flags=8,
                      status_text='overcurrent', torque_enable=0, original_error='angle timeout')
    output.read_position_deg.side_effect = ServoAngleReadError('angle timeout; overcurrent', diagnostic)
    controller.tick()
    report = controller.status()['angle_error']
    assert report['status_flags'] == 8 and report['original_error'] == 'angle timeout'
    assert report['at_ms'] == 1000000
    assert not controller.status()['connected'] and controller.status()['raw'] == 0
    output.read_error_diagnostics.assert_not_called()  # Low-level capture is reused.
    output.release.assert_called()
    controller.close()
    assert controller.status()['angle_error'] == report


def test_angle_diagnostic_fallback_for_invalid_angle_does_not_mask_original_failure(state):
    controller, output, clock, token = state
    output.read_position_deg.return_value = math.nan
    output.read_error_diagnostics.side_effect = RuntimeError('status read timed out')
    controller.tick()
    report = controller.status()['angle_error']
    assert not report['status_available']
    assert report['diagnostic_error'] == 'status read timed out'
    assert report['original_error'] == 'Invalid servo angle feedback'
    assert 'Invalid servo angle feedback' in controller.status()['fault']
    output.release.assert_called()


def test_angle_diagnostic_is_retained_after_successful_reconnection(selectable):
    controller, outputs, devices, clock = selectable
    original_factory = controller.output_factory
    def failing_factory(port):
        output = original_factory(port)
        output.read_position_deg.side_effect = ServoAngleReadError('angle timeout',
            dict(status_available=False, original_error='angle timeout'))
        return output
    controller.output_factory = failing_factory
    with pytest.raises(RuntimeError, match='angle timeout'):
        controller.connect(port=devices[0]['device'])
    report = controller.status()['angle_error']
    assert report['port'] == devices[0]['device']
    assert not controller.status()['serial_open']
    controller.output_factory = original_factory
    result = controller.connect(port=devices[1]['device'])
    assert result['connected'] and not result['fault']
    assert result['angle_error'] == report
    controller.close()
