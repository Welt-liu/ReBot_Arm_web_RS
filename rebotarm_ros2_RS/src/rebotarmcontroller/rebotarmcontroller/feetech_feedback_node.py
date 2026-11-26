"""Independent UART node: follower gripper torque -> HLS3604M speed + current-limit feedback."""
from __future__ import annotations

import time
import sys

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import qos_profile_sensor_data
from rebotarm_msgs.msg import JointMotorState
from std_msgs.msg import UInt16
from std_srvs.srv import SetBool

from .feetech_feedback import FeedbackSignal, FeetechForceOutput


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


class FeetechFeedbackNode(Node):
    def __init__(self):
        super().__init__("feetech_gripper_feedback")
        defaults = dict(port="", servo_id=1, baudrate=1_000_000,
                        arm_namespace="rebot_arm_rs", enabled=False,
                        deadband_nm=0.05,
                        full_scale_nm=1.0, timeout_s=0.3)
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        params = {key: self.get_parameter(key).value for key in defaults}
        if not params["port"]:
            raise ValueError("set port to the dedicated HLS3604M USB serial adapter")
        self.signal = FeedbackSignal(params["deadband_nm"], params["full_scale_nm"], params["timeout_s"])
        self.enabled = params["enabled"]
        self.last_health_at = 0.0
        self.fault = ""
        ns = "/" + params["arm_namespace"].strip("/")
        self.raw_pub = self.create_publisher(UInt16, ns + "/gripper/feedback_raw", 1)
        self.create_subscription(JointMotorState, ns + "/gripper/state", self.on_gripper, qos_profile_sensor_data)
        self.create_service(SetBool, ns + "/gripper/feedback_enable", self.on_enable)
        self.output = FeetechForceOutput.open(params["port"], params["servo_id"], params["baudrate"])
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(0.02, self.tick, clock=self.steady_clock)
        self.get_logger().info("HLS3604M gripper feedback ready: raw register range 0..100")

    def on_gripper(self, msg):
        self.signal.update(msg.torque, msg.status_code, stamp_ns(msg.header.stamp),
                           self.get_clock().now().nanoseconds)

    def on_enable(self, request, response):
        self.enabled = False
        self.signal.invalidate()  # Re-enabling requires a newly received measurement.
        try:
            self.output.release()
            if request.data and self.fault:
                raise RuntimeError("restart feedback node after checking the UART fault: " + self.fault)
            self.enabled = bool(request.data)
            response.success, response.message = True, "Feedback enabled" if self.enabled else "Feedback released"
        except Exception as exc:
            self.fault = str(exc)
            response.success, response.message = False, self.fault
        return response

    def tick(self):
        now = time.monotonic()
        active = self.enabled and not self.fault
        raw = self.signal.value(active=active, now=now)
        try:
            if self.fault:
                self.output.release()  # Retry release, never auto-resume after UART failure.
            else:
                if now - self.last_health_at >= 0.1:
                    self.output.check_health()
                    self.last_health_at = now
                self.output.set_raw(raw)
        except Exception as exc:
            if not self.fault:
                self.get_logger().error(str(exc))
            self.fault, self.enabled = str(exc), False
            raw = 0
            try:
                self.output.release()
            except Exception:
                pass  # The next steady-clock tick retries release.
        message = UInt16()
        message.data = raw
        self.raw_pub.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = FeetechFeedbackNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except (OSError, ValueError, RuntimeError) as exc:
        print(f"Feetech feedback failed: {exc}", file=sys.stderr)
        return 1
    finally:
        if node is not None:
            try:
                node.output.close()
            finally:
                node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
