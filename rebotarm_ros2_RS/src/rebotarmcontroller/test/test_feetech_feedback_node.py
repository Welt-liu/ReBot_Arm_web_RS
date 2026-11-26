"""Exercise node policy without requiring ROS discovery or a physical servo."""
import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace
from unittest.mock import Mock, patch

from rebotarmcontroller.feetech_feedback import FeedbackSignal


def load_host():
    path = Path(__file__).parents[1] / 'rebotarmcontroller' / 'feetech_feedback_node.py'
    spec = importlib.util.spec_from_file_location('rebotarmcontroller._feedback_test_host', path)
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, {
        'rclpy': SimpleNamespace(), 'rclpy.node': SimpleNamespace(Node=object),
        'rclpy.clock': SimpleNamespace(Clock=object, ClockType=object),
        'rclpy.qos': SimpleNamespace(qos_profile_sensor_data=object()),
        'rebotarm_msgs.msg': SimpleNamespace(JointMotorState=object),
        'std_msgs.msg': SimpleNamespace(UInt16=SimpleNamespace),
        'std_srvs.srv': SimpleNamespace(SetBool=object),
    }):
        spec.loader.exec_module(module)
    return module


host = load_host()


def node_with_signal():
    node = host.FeetechFeedbackNode.__new__(host.FeetechFeedbackNode)
    node.signal = FeedbackSignal()
    node.signal.update(1, 0, 1_000_000_000, 1_000_000_000, now=10)
    node.enabled, node.fault, node.last_health_at = True, '', 10
    node.output, node.raw_pub = Mock(), Mock()
    node.get_logger = Mock()
    return node


def test_node_forwards_current_and_zeroes_on_stale_data_without_leader():
    node = node_with_signal()
    with patch.object(host.time, 'monotonic', return_value=10.05):
        node.tick()
    node.output.set_raw.assert_called_with(100)
    with patch.object(host.time, 'monotonic', return_value=10.4):
        node.tick()
    node.output.set_raw.assert_called_with(0)
    assert node.raw_pub.publish.call_args.args[0].data == 0


def test_disable_releases_immediately_and_reenable_requires_new_measurement():
    node = node_with_signal()
    response = node.on_enable(SimpleNamespace(data=False), SimpleNamespace())
    assert response.success and not node.enabled
    node.output.release.assert_called_once()
    response = node.on_enable(SimpleNamespace(data=True), SimpleNamespace())
    assert response.success and node.enabled
    assert node.signal.value(active=True, now=10.1) == 0


def test_uart_failure_latches_and_never_reenables_automatically():
    node = node_with_signal()
    node.output.set_raw.side_effect = RuntimeError('UART disconnected')
    with patch.object(host.time, 'monotonic', return_value=10.05):
        node.tick()
    assert node.fault == 'UART disconnected'
    assert not node.enabled
    node.output.reset_mock()
    with patch.object(host.time, 'monotonic', return_value=10.06):
        node.tick()
    node.output.release.assert_called_once()
    node.output.set_raw.assert_not_called()
    response = node.on_enable(SimpleNamespace(data=True), SimpleNamespace())
    assert not response.success and not node.enabled


def test_health_error_prevents_current_write():
    node = node_with_signal()
    node.output.check_health.side_effect = RuntimeError('servo overheated')
    with patch.object(host.time, 'monotonic', return_value=10.2):
        node.tick()
    assert node.fault == 'servo overheated'
    node.output.set_raw.assert_not_called()
