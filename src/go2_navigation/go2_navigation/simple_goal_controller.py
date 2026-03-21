import math
from dataclasses import dataclass
from typing import Optional

import rclpy
from geometry_msgs.msg import Pose, PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


@dataclass
class GoalState:
    pose: Pose
    frame_id: str


class SimpleGoalController(Node):
    """Closed-loop point-to-point controller using odometry only."""

    def __init__(self) -> None:
        super().__init__("go2_simple_goal_controller")

        odom_topic = self.declare_parameter("odom_topic", "/utlidar/robot_odom").value
        goal_topic = self.declare_parameter("goal_topic", "/goal_pose").value
        cmd_vel_topic = self.declare_parameter("cmd_vel_topic", "/cmd_vel").value
        control_rate_hz = float(self.declare_parameter("control_rate_hz", 20.0).value)

        self._stale_odometry_timeout_ns = int(
            float(self.declare_parameter("stale_odometry_timeout", 0.5).value) * 1e9
        )
        self._position_tolerance = float(self.declare_parameter("position_tolerance", 0.10).value)
        self._yaw_tolerance = float(self.declare_parameter("yaw_tolerance", 0.12).value)
        self._rotate_in_place_threshold = float(
            self.declare_parameter("rotate_in_place_threshold", 1.0).value
        )
        self._k_forward = float(self.declare_parameter("k_forward", 0.8).value)
        self._k_lateral = float(self.declare_parameter("k_lateral", 0.8).value)
        self._k_heading = float(self.declare_parameter("k_heading", 1.4).value)
        self._k_final_yaw = float(self.declare_parameter("k_final_yaw", 1.6).value)
        self._min_linear_speed = float(self.declare_parameter("min_linear_speed", 0.0).value)
        self._max_linear_x = float(self.declare_parameter("max_linear_x", 0.4).value)
        self._max_linear_y = float(self.declare_parameter("max_linear_y", 0.25).value)
        self._max_linear_speed = float(self.declare_parameter("max_linear_speed", 0.45).value)
        self._min_angular_speed = float(self.declare_parameter("min_angular_speed", 0.0).value)
        self._max_angular_z = float(self.declare_parameter("max_angular_z", 0.8).value)
        self._allow_reverse = bool(self.declare_parameter("allow_reverse", False).value)
        self._allow_lateral_motion = bool(self.declare_parameter("allow_lateral_motion", True).value)
        self._align_final_yaw = bool(self.declare_parameter("align_final_yaw", True).value)
        self._reverse_mode_switch_margin = float(
            self.declare_parameter("reverse_mode_switch_margin", 0.20).value
        )
        self._max_linear_accel = float(self.declare_parameter("max_linear_accel", 0.8).value)
        self._max_angular_accel = float(self.declare_parameter("max_angular_accel", 1.5).value)

        self._odom_frame = "odom"
        self._latest_pose: Optional[Pose] = None
        self._latest_pose_time = None
        self._goal: Optional[GoalState] = None
        self._prefer_reverse = False
        self._last_command = Twist()
        self._last_control_time = None

        self._cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self._odom_sub = self.create_subscription(Odometry, odom_topic, self._odom_callback, 50)
        self._goal_sub = self.create_subscription(PoseStamped, goal_topic, self._goal_callback, 10)
        self._timer = self.create_timer(1.0 / max(control_rate_hz, 1.0), self._on_timer)

        self.get_logger().info(
            f"Simple goal controller ready: {odom_topic} + {goal_topic} -> {cmd_vel_topic}"
        )

    def _odom_callback(self, msg: Odometry) -> None:
        self._latest_pose = msg.pose.pose
        self._latest_pose_time = self.get_clock().now()
        if msg.header.frame_id:
            self._odom_frame = msg.header.frame_id

    def _goal_callback(self, msg: PoseStamped) -> None:
        goal_frame = msg.header.frame_id or self._odom_frame
        if goal_frame != self._odom_frame:
            self.get_logger().error(
                f"Rejected goal in frame '{goal_frame}', expected '{self._odom_frame}'."
            )
            return

        self._goal = GoalState(pose=msg.pose, frame_id=goal_frame)
        self._prefer_reverse = False
        self.get_logger().info(
            f"Received goal x={msg.pose.position.x:.3f}, y={msg.pose.position.y:.3f}, z={msg.pose.position.z:.3f}"
        )

    def _publish_zero(self) -> None:
        zero = Twist()
        self._cmd_pub.publish(zero)
        self._last_command = zero

    def _apply_min_linear_speed(self, x_value: float, y_value: float) -> tuple[float, float]:
        linear_speed = math.hypot(x_value, y_value)
        if linear_speed < 1e-6 or linear_speed >= self._min_linear_speed:
            return x_value, y_value
        scale = self._min_linear_speed / linear_speed
        return x_value * scale, y_value * scale

    def _apply_min_angular_speed(self, value: float) -> float:
        if abs(value) < 1e-6 or self._min_angular_speed <= 0.0:
            return value
        return math.copysign(max(abs(value), self._min_angular_speed), value)

    def _odometry_is_stale(self) -> bool:
        if self._latest_pose_time is None:
            return True
        return (self.get_clock().now() - self._latest_pose_time).nanoseconds > self._stale_odometry_timeout_ns

    def _goal_yaw(self) -> float:
        assert self._goal is not None
        q = self._goal.pose.orientation
        return yaw_from_quaternion(q.x, q.y, q.z, q.w)

    def _current_yaw(self) -> float:
        assert self._latest_pose is not None
        q = self._latest_pose.orientation
        return yaw_from_quaternion(q.x, q.y, q.z, q.w)

    def _select_motion_heading(self, heading_to_goal: float, yaw: float) -> tuple[float, bool]:
        forward_heading_error = normalize_angle(heading_to_goal - yaw)
        if not self._allow_reverse:
            self._prefer_reverse = False
            return forward_heading_error, False

        reverse_heading = normalize_angle(heading_to_goal + math.pi)
        reverse_heading_error = normalize_angle(reverse_heading - yaw)
        if abs(reverse_heading_error) + self._reverse_mode_switch_margin < abs(forward_heading_error):
            self._prefer_reverse = True
        elif abs(forward_heading_error) + self._reverse_mode_switch_margin < abs(reverse_heading_error):
            self._prefer_reverse = False

        if self._prefer_reverse:
            return reverse_heading_error, True
        return forward_heading_error, False

    def _rate_limit(self, previous: float, target: float, max_delta: float) -> float:
        if max_delta <= 0.0:
            return target
        return clamp(target, previous - max_delta, previous + max_delta)

    def _rate_limit_command(self, command: Twist) -> Twist:
        now = self.get_clock().now()
        if self._last_control_time is None:
            self._last_control_time = now
            self._last_command = command
            return command

        dt = max((now - self._last_control_time).nanoseconds / 1e9, 0.0)
        self._last_control_time = now
        if dt <= 1e-6:
            self._last_command = command
            return command

        limited = Twist()
        linear_delta = self._max_linear_accel * dt
        angular_delta = self._max_angular_accel * dt
        limited.linear.x = self._rate_limit(self._last_command.linear.x, command.linear.x, linear_delta)
        limited.linear.y = self._rate_limit(self._last_command.linear.y, command.linear.y, linear_delta)
        limited.angular.z = self._rate_limit(
            self._last_command.angular.z, command.angular.z, angular_delta
        )
        self._last_command = limited
        return limited

    def _publish_command(self, command: Twist) -> None:
        limited = self._rate_limit_command(command)
        self._cmd_pub.publish(limited)

    def _on_timer(self) -> None:
        if self._goal is None:
            return

        if self._latest_pose is None or self._odometry_is_stale():
            self._publish_zero()
            return

        assert self._latest_pose is not None

        dx = self._goal.pose.position.x - self._latest_pose.position.x
        dy = self._goal.pose.position.y - self._latest_pose.position.y
        distance = math.hypot(dx, dy)

        yaw = self._current_yaw()
        goal_yaw = self._goal_yaw()
        heading_to_goal = math.atan2(dy, dx)
        heading_error, using_reverse = self._select_motion_heading(heading_to_goal, yaw)
        final_yaw_error = normalize_angle(goal_yaw - yaw)

        command = Twist()

        if distance <= self._position_tolerance:
            if not self._align_final_yaw:
                self._publish_zero()
                self.get_logger().info("Goal reached")
                self._goal = None
                return
            if abs(final_yaw_error) <= self._yaw_tolerance:
                self._publish_zero()
                self.get_logger().info("Goal reached")
                self._goal = None
                return

            command.angular.z = clamp(
                self._k_final_yaw * final_yaw_error,
                -self._max_angular_z,
                self._max_angular_z,
            )
            command.angular.z = self._apply_min_angular_speed(command.angular.z)
            self._publish_command(command)
            return

        if abs(heading_error) > self._rotate_in_place_threshold and not self._allow_reverse:
            command.angular.z = clamp(
                self._k_heading * heading_error,
                -self._max_angular_z,
                self._max_angular_z,
            )
            command.angular.z = self._apply_min_angular_speed(command.angular.z)
            self._publish_command(command)
            return

        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        forward_error = cos_yaw * dx + sin_yaw * dy
        lateral_error = -sin_yaw * dx + cos_yaw * dy

        min_linear_x = -self._max_linear_x if self._allow_reverse else 0.0
        if not self._allow_lateral_motion:
            lateral_error = 0.0

        target_linear_x = self._k_forward * forward_error
        if using_reverse and target_linear_x > 0.0:
            target_linear_x = 0.0
        if not using_reverse and target_linear_x < 0.0:
            target_linear_x = 0.0
        command.linear.x = clamp(target_linear_x, min_linear_x, self._max_linear_x)
        command.linear.y = clamp(
            self._k_lateral * lateral_error,
            -self._max_linear_y,
            self._max_linear_y,
        )
        command.angular.z = clamp(
            self._k_heading * heading_error,
            -self._max_angular_z,
            self._max_angular_z,
        )
        command.angular.z = self._apply_min_angular_speed(command.angular.z)

        linear_speed = math.hypot(command.linear.x, command.linear.y)
        if linear_speed > self._max_linear_speed and linear_speed > 1e-6:
            scale = self._max_linear_speed / linear_speed
            command.linear.x *= scale
            command.linear.y *= scale
        command.linear.x, command.linear.y = self._apply_min_linear_speed(
            command.linear.x, command.linear.y
        )

        self._publish_command(command)


def main() -> None:
    rclpy.init()
    node = SimpleGoalController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._publish_zero()
        node.destroy_node()
        rclpy.shutdown()
