"""Verify HLS force commands and failure release without connected motors."""
from unittest.mock import Mock

import pytest

from rebotarmcontroller.feetech_feedback import FeedbackSignal, FeetechForceOutput, ServoAngleReadError


@pytest.mark.parametrize('torque, expected', [(0, 0), (0.05, 0), (0.525, 50),
                                            (1, 100), (-0.525, 50), (1e308, 100)])
def test_torque_mapping_is_raw_0_to_100(torque, expected):
    signal = FeedbackSignal()
    signal.update(torque, 0, 1_000_000_000, 1_000_000_000, now=10)
    assert signal.value(active=True, now=10.1) == expected
    assert signal.value(active=False, now=10.1) == 0
    assert signal.value(active=True, now=10.31) == 0


@pytest.mark.parametrize('torque,status,stamp', [(float('nan'), 0, 1.1e9),
    (float('inf'), 0, 1.1e9), (0.8, 4, 1.1e9), (0.8, 0, 0.9e9),
    (0.8, 0, 0.5e9), (0.8, 0, 2e9)])
def test_invalid_faulted_replayed_stale_and_future_data_release(torque, status, stamp):
    signal = FeedbackSignal()
    signal.update(0.8, 0, 1_000_000_000, 1_000_000_000, now=10)
    signal.update(torque, status, int(stamp), 1_100_000_000, now=10.1)
    assert signal.value(active=True, now=10.1) == 0


def test_cache_replays_do_not_renew_deadline_and_source_age_counts():
    signal = FeedbackSignal()
    signal.update(1, 0, 1_000_000_000, 1_200_000_000, now=10)
    signal.update(1, 0, 1_000_000_000, 1_250_000_000, now=10.05)
    assert signal.value(active=True, now=10.05) == 100
    assert signal.value(active=True, now=10.11) == 0


@pytest.mark.parametrize('settings', [dict(deadband=-1), dict(full_scale=0.05),
    dict(full_scale=float('inf')), dict(timeout=0), dict(timeout=2)])
def test_invalid_mapping_settings(settings):
    with pytest.raises(ValueError):
        FeedbackSignal(**settings)


def make_output():
    packet = Mock()
    packet.write1ByteTxRx.return_value = (0, 0)
    packet.write2ByteTxRx.return_value = (0, 0)
    packet.read1ByteTxRx.side_effect = lambda sid, address: (
        0 if address == 65 else 1 if address == 8 else 1, 0, 0)
    return FeetechForceOutput(packet, 1), packet


def test_speed_mode_is_temporary_and_current_limit_is_separate():
    output, packet = make_output()
    output.initialize()
    assert packet.write1ByteTxRx.call_args_list[0].args == (1, 40, 0)
    assert packet.write1ByteTxRx.call_args_list[1].args == (1, 55, 1)
    assert packet.write1ByteTxRx.call_args_list[2].args == (1, 33, 1)
    packet.reset_mock()
    output.set_raw(100)
    assert [c.args for c in packet.write2ByteTxRx.call_args_list] == [(1, 46, 2), (1, 44, 100)]
    packet.write1ByteTxRx.assert_called_once_with(1, 40, 1)
    output.set_raw(100)
    assert packet.write2ByteTxRx.call_count == 2
    output.set_raw(0)
    assert (1, 44, 0) in [c.args for c in packet.write2ByteTxRx.call_args_list]
    assert packet.write2ByteTxRx.call_args.args == (1, 46, 0)
    assert packet.write1ByteTxRx.call_args.args == (1, 40, 0)


@pytest.mark.parametrize('raw', [-1, 151, 1000, 1.5, True, float('nan')])
def test_invalid_raw_command_never_reaches_bus(raw):
    output, packet = make_output()
    with pytest.raises(ValueError):
        output.set_raw(raw)
    assert not packet.mock_calls


def test_raw_150_reaches_servo_without_extra_scaling():
    output, packet = make_output()
    output.initialize()
    packet.reset_mock()
    output.set_raw(150)
    assert [c.args for c in packet.write2ByteTxRx.call_args_list if c.args[1] == 44] == [(1, 44, 150)]


@pytest.mark.parametrize('servo_id', [-1, 254, 255, True, 1.5])
def test_invalid_or_broadcast_id_rejected(servo_id):
    with pytest.raises(ValueError):
        FeetechForceOutput(Mock(), servo_id)


@pytest.mark.parametrize('failure', [(-6, 0), (0, 4)])
def test_failed_write_releases_torque_and_zeroes_current(failure):
    output, packet = make_output()
    output.initialize()
    packet.reset_mock()
    packet.write2ByteTxRx.side_effect = lambda sid, address, value: failure if (address, value) == (44, 80) else (0, 0)
    with pytest.raises(RuntimeError):
        output.set_raw(80)
    packet.write1ByteTxRx.assert_called_with(1, 40, 0)
    assert (1, 44, 0) in [c.args for c in packet.write2ByteTxRx.call_args_list]
    assert packet.write2ByteTxRx.call_args.args == (1, 46, 0)


def test_release_still_zeroes_current_if_disable_fails():
    output, packet = make_output()
    packet.write1ByteTxRx.return_value = (-6, 0)
    with pytest.raises(RuntimeError):
        output.release()
    assert [c.args for c in packet.write2ByteTxRx.call_args_list] == [(1, 44, 0), (1, 46, 0)]
    assert output.last_raw is None


def test_close_restores_mode_without_reenabling_and_closes_once():
    output, packet = make_output()
    output.previous_mode = 0
    output.close_port = Mock()
    output.close()
    assert packet.write1ByteTxRx.call_args.args == (1, 33, 0)
    output.close()
    output.close_port.assert_called_once()


def test_health_detects_servo_fault():
    output, packet = make_output()
    packet.read1ByteTxRx.side_effect = None
    packet.read1ByteTxRx.return_value = (4, 0, 0)
    with pytest.raises(RuntimeError, match='servo fault'):
        output.check_health()


def test_unresponsive_servo_is_detected_before_any_writes():
    output, packet = make_output()
    packet.read1ByteTxRx.side_effect = None
    packet.read1ByteTxRx.return_value = (0, -6, 0)
    with pytest.raises(RuntimeError, match='No servo response'):
        output.initialize()
    packet.write1ByteTxRx.assert_not_called()
    packet.write2ByteTxRx.assert_not_called()
    assert not output.touched


@pytest.mark.parametrize('voltage,label', [(121, 'OVERVOLTAGE'), (35, 'UNDERVOLTAGE')])
def test_voltage_alarm_reports_telemetry_and_blocks_initialization(voltage, label):
    output, packet = make_output()
    values = {33: 0, 62: voltage, 15: 40, 14: 100}
    packet.read1ByteTxRx.side_effect = lambda sid, address: (values[address], 0, 1)
    with pytest.raises(RuntimeError, match=label) as failure:
        output.initialize()
    assert f'measured={voltage / 10:.1f}V' in str(failure.value)
    assert '4.0..10.0V' in str(failure.value)
    packet.write1ByteTxRx.assert_not_called()
    packet.write2ByteTxRx.assert_not_called()


def test_angle_diagnostic_reads_status_first_and_keeps_alarm_payloads_without_writes():
    output, packet = make_output()
    packet.ReadPos.return_value = (0, -6, 0)
    values = {65: 8, 40: 0, 33: 1, 62: 60, 63: 72}
    packet.read1ByteTxRx.side_effect = lambda sid, address: (values[address], 0, 8)
    with pytest.raises(ServoAngleReadError) as failure:
        output.read_position_deg()
    diagnostic = failure.value.diagnostics
    assert diagnostic['status_available']
    assert diagnostic['status_raw'] == diagnostic['status_flags'] == 8
    assert diagnostic['status_text'] == 'overcurrent'
    assert diagnostic['angle_communication'] == -6
    assert diagnostic['torque_enable'] == 0
    assert diagnostic['voltage_v'] == 6.0
    assert diagnostic['temperature_c'] == 72
    assert packet.read1ByteTxRx.call_args_list[0].args == (1, 65)
    packet.write1ByteTxRx.assert_not_called()
    packet.write2ByteTxRx.assert_not_called()


def test_angle_diagnostic_status_timeout_stays_unknown_and_skips_extra_reads():
    output, packet = make_output()
    packet.ReadPos.return_value = (0, -6, 0)
    packet.read1ByteTxRx.side_effect = None
    packet.read1ByteTxRx.return_value = (0, -6, 0)
    with pytest.raises(ServoAngleReadError, match='status65 unreadable') as failure:
        output.read_position_deg()
    diagnostic = failure.value.diagnostics
    assert not diagnostic['status_available']
    assert diagnostic['status_raw'] is None and diagnostic['status_flags'] is None
    assert diagnostic['reads']['status']['communication'] == -6
    packet.read1ByteTxRx.assert_called_once_with(1, 65)
    packet.write1ByteTxRx.assert_not_called()


@pytest.mark.parametrize('position', [float('nan'), float('inf'), True])
def test_angle_diagnostic_invalid_position_also_reads_error_register(position):
    output, packet = make_output()
    packet.ReadPos.return_value = (position, 0, 0)
    packet.read1ByteTxRx.side_effect = lambda sid, address: (0, 0, 0)
    with pytest.raises(ServoAngleReadError) as failure:
        output.read_position_deg()
    assert failure.value.diagnostics['status_available']
    assert failure.value.diagnostics['status_flags'] == 0
    assert failure.value.diagnostics['original_error'] == 'Invalid HLS position feedback'
