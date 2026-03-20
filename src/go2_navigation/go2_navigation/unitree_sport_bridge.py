import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from unitree_api.msg import Request

from go2_navigation.unitree_api_helpers import SportRequestBuilder


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


class UnitreeSportBridge(Node):
    """Bridge standard Twist commands into Unitree Sport API requests."""

    def __init__(self) -> None:
        super().__init__("unitree_sport_bridge")

        cmd_vel_topic = self.declare_parameter("cmd_vel_topic", "/cmd_vel").value
        sport_request_topic = self.declare_parameter("sport_request_topic", "/api/sport/request").value
        control_rate_hz = float(self.declare_parameter("control_rate_hz", 20.0).value)
        command_timeout = float(self.declare_parameter("command_timeout", 0.3).value)
        linear_deadband = float(self.declare_parameter("linear_deadband", 0.02).value)
        angular_deadband = float(self.declare_parameter("angular_deadband", 0.02).value)
        self._min_linear_speed = float(self.declare_parameter("min_linear_speed", 0.0).value)
        self._max_linear_x = float(self.declare_parameter("max_linear_x", 0.6).value)
        self._max_linear_y = float(self.declare_parameter("max_linear_y", 0.4).value)
        self._min_angular_speed = float(self.declare_parameter("min_angular_speed", 0.0).value)
        self._max_angular_z = float(self.declare_parameter("max_angular_z", 1.2).value)

        self._command_timeout_ns = int(command_timeout * 1e9)
        self._linear_deadband = abs(linear_deadband)
        self._angular_deadband = abs(angular_deadband)
        self._builder = SportRequestBuilder()
        self._last_twist = Twist()
        self._last_command_time = None
        self._motion_active = False

        self._request_pub = self.create_publisher(Request, sport_request_topic, 10)
        self._cmd_sub = self.create_subscription(Twist, cmd_vel_topic, self._cmd_callback, 10)
        self._timer = self.create_timer(1.0 / max(control_rate_hz, 1.0), self._on_timer)

        self.get_logger().info(
            f"Bridge ready: {cmd_vel_topic} -> {sport_request_topic} at {control_rate_hz:.1f} Hz"
        )

    def _cmd_callback(self, msg: Twist) -> None:
        self._last_twist = msg
        self._last_command_time = self.get_clock().now()

    def _is_zero_command(self, msg: Twist) -> bool:
        return (
            abs(msg.linear.x) < self._linear_deadband
            and abs(msg.linear.y) < self._linear_deadband
            and abs(msg.angular.z) < self._angular_deadband
        )

    def _publish_stop(self) -> None:
        self._request_pub.publish(self._builder.build_stop_request())
        self._motion_active = False

    def _publish_move(self, msg: Twist) -> None:
        vx = clamp(msg.linear.x, -self._max_linear_x, self._max_linear_x)
        vy = clamp(msg.linear.y, -self._max_linear_y, self._max_linear_y)
        wz = clamp(msg.angular.z, -self._max_angular_z, self._max_angular_z)

        linear_speed = math.hypot(vx, vy)
        if 1e-6 < linear_speed < self._min_linear_speed:
            scale = self._min_linear_speed / linear_speed
            vx *= scale
            vy *= scale

        if 1e-6 < abs(wz) < self._min_angular_speed:
            wz = math.copysign(self._min_angular_speed, wz)

        self._request_pub.publish(self._builder.build_move_request(vx, vy, wz))
        self._motion_active = math.hypot(vx, vy) > 0.0 or abs(wz) > 0.0

    def _on_timer(self) -> None:
        if self._last_command_time is None:
            return

        age_ns = (self.get_clock().now() - self._last_command_time).nanoseconds
        if age_ns > self._command_timeout_ns:
            if self._motion_active:
                self._publish_stop()
                self.get_logger().info("cmd_vel timeout, sent StopMove")
            return

        if self._is_zero_command(self._last_twist):
            if self._motion_active:
                self._publish_stop()
            return

        self._publish_move(self._last_twist)


def main() -> None:
    rclpy.init()
    node = UnitreeSportBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node._publish_stop()
        node.destroy_node()
        rclpy.shutdown()
