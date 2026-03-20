import math
from dataclasses import dataclass

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


@dataclass
class Clearance:
    front: float = math.inf
    left: float = math.inf
    right: float = math.inf


class DynamicObstacleFilter(Node):
    """Fuse range and point-cloud data to stop, slow, or bias around obstacles."""

    def __init__(self) -> None:
        super().__init__("go2_dynamic_obstacle_filter")

        input_cmd_topic = self.declare_parameter("input_cmd_topic", "/cmd_vel_nav").value
        output_cmd_topic = self.declare_parameter("output_cmd_topic", "/cmd_vel").value
        range_info_topic = self.declare_parameter("range_info_topic", "/utlidar/range_info").value
        cloud_topic = self.declare_parameter("cloud_topic", "/utlidar/cloud_deskewed").value
        odom_topic = self.declare_parameter("odom_topic", "/utlidar/robot_odom").value
        status_topic = self.declare_parameter(
            "status_topic", "/go2_navigation/obstacle_filter_status"
        ).value
        fused_range_topic = self.declare_parameter(
            "fused_range_topic", "/go2_navigation/fused_range_info"
        ).value
        control_rate_hz = float(self.declare_parameter("control_rate_hz", 20.0).value)

        self._cmd_timeout_ns = int(float(self.declare_parameter("cmd_timeout", 0.4).value) * 1e9)
        self._sensor_timeout_ns = int(
            float(self.declare_parameter("sensor_timeout", 0.4).value) * 1e9
        )
        self._max_sensor_distance = float(self.declare_parameter("max_sensor_distance", 5.0).value)
        self._fail_safe_stop_on_stale = bool(
            self.declare_parameter("fail_safe_stop_on_stale", True).value
        )
        self._use_range_info = bool(self.declare_parameter("use_range_info", True).value)
        self._use_cloud = bool(self.declare_parameter("use_cloud", True).value)

        self._front_stop_distance = float(self.declare_parameter("front_stop_distance", 0.40).value)
        self._front_avoid_distance = float(
            self.declare_parameter("front_avoid_distance", 0.75).value
        )
        self._front_slow_distance = float(self.declare_parameter("front_slow_distance", 1.20).value)
        self._side_stop_distance = float(self.declare_parameter("side_stop_distance", 0.18).value)
        self._side_slow_distance = float(self.declare_parameter("side_slow_distance", 0.32).value)
        self._minimum_slowdown_ratio = float(
            self.declare_parameter("minimum_slowdown_ratio", 0.20).value
        )

        self._avoid_forward_speed = float(self.declare_parameter("avoid_forward_speed", 0.10).value)
        self._avoid_lateral_speed = float(self.declare_parameter("avoid_lateral_speed", 0.18).value)
        self._avoid_turn_speed = float(self.declare_parameter("avoid_turn_speed", 0.35).value)
        self._side_repulsion_distance = float(
            self.declare_parameter("side_repulsion_distance", 0.45).value
        )
        self._lateral_repulsion_gain = float(
            self.declare_parameter("lateral_repulsion_gain", 1.0).value
        )
        self._yaw_repulsion_gain = float(self.declare_parameter("yaw_repulsion_gain", 1.0).value)

        self._max_linear_x = float(self.declare_parameter("max_linear_x", 0.40).value)
        self._max_linear_y = float(self.declare_parameter("max_linear_y", 0.25).value)
        self._max_angular_z = float(self.declare_parameter("max_angular_z", 0.80).value)
        self._linear_deadband = float(self.declare_parameter("linear_deadband", 0.02).value)
        self._angular_deadband = float(self.declare_parameter("angular_deadband", 0.02).value)

        self._cloud_min_rel_z = float(self.declare_parameter("cloud_min_rel_z", -0.10).value)
        self._cloud_max_rel_z = float(self.declare_parameter("cloud_max_rel_z", 0.65).value)
        self._cloud_forward_limit = float(self.declare_parameter("cloud_forward_limit", 2.50).value)
        self._cloud_rear_limit = float(self.declare_parameter("cloud_rear_limit", -0.20).value)
        self._cloud_side_limit = float(self.declare_parameter("cloud_side_limit", 1.20).value)
        self._front_half_width = float(self.declare_parameter("front_half_width", 0.35).value)
        self._side_forward_min = float(self.declare_parameter("side_forward_min", -0.10).value)
        self._side_forward_max = float(self.declare_parameter("side_forward_max", 0.80).value)
        self._side_inner_offset = float(self.declare_parameter("side_inner_offset", 0.05).value)
        self._side_outer_offset = float(self.declare_parameter("side_outer_offset", 0.65).value)
        self._clearance_release_alpha = float(
            self.declare_parameter("clearance_release_alpha", 0.18).value
        )
        self._avoid_direction_lock_time = float(
            self.declare_parameter("avoid_direction_lock_time", 0.8).value
        )
        self._side_selection_margin = float(
            self.declare_parameter("side_selection_margin", 0.08).value
        )

        self._latest_cmd = Twist()
        self._latest_cmd_time = None
        self._motion_active = False
        self._range_clearance = Clearance()
        self._range_time = None
        self._cloud_clearance = Clearance()
        self._cloud_time = None
        self._odom_time = None
        self._robot_x = 0.0
        self._robot_y = 0.0
        self._robot_z = 0.0
        self._robot_yaw = 0.0
        self._last_state = ""
        self._filtered_clearance = Clearance()
        self._avoid_direction_sign = 0.0
        self._avoid_direction_lock_until_ns = 0

        self._cmd_pub = self.create_publisher(Twist, output_cmd_topic, 10)
        self._status_pub = self.create_publisher(String, status_topic, 10)
        self._fused_range_pub = self.create_publisher(PointStamped, fused_range_topic, 10)

        self.create_subscription(Twist, input_cmd_topic, self._cmd_callback, 10)
        self.create_subscription(Odometry, odom_topic, self._odom_callback, 50)
        self.create_subscription(PointStamped, range_info_topic, self._range_callback, 20)
        self.create_subscription(PointCloud2, cloud_topic, self._cloud_callback, 10)
        self._timer = self.create_timer(1.0 / max(control_rate_hz, 1.0), self._on_timer)

        self.get_logger().info(
            f"Dynamic obstacle filter ready: {input_cmd_topic} -> {output_cmd_topic}"
        )

    def _cmd_callback(self, msg: Twist) -> None:
        self._latest_cmd = msg
        self._latest_cmd_time = self.get_clock().now()

    def _odom_callback(self, msg: Odometry) -> None:
        self._odom_time = self.get_clock().now()
        self._robot_x = msg.pose.pose.position.x
        self._robot_y = msg.pose.pose.position.y
        self._robot_z = msg.pose.pose.position.z
        q = msg.pose.pose.orientation
        self._robot_yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)

    def _sanitize_distance(self, value: float) -> float:
        if not math.isfinite(value) or value <= 0.0:
            return math.inf
        return min(value, self._max_sensor_distance)

    def _range_callback(self, msg: PointStamped) -> None:
        self._range_time = self.get_clock().now()
        self._range_clearance = Clearance(
            front=self._sanitize_distance(msg.point.x),
            left=self._sanitize_distance(msg.point.y),
            right=self._sanitize_distance(msg.point.z),
        )

    def _cloud_callback(self, msg: PointCloud2) -> None:
        if not self._use_cloud or self._odom_time is None:
            return

        points = np.asarray(
            point_cloud2.read_points_numpy(msg, field_names=["x", "y", "z"], skip_nans=True)
        )
        if points.size == 0:
            self._cloud_time = self.get_clock().now()
            self._cloud_clearance = Clearance()
            return
        if points.ndim == 1:
            points = points.reshape(1, -1)

        dx = points[:, 0] - self._robot_x
        dy = points[:, 1] - self._robot_y
        dz = points[:, 2] - self._robot_z

        cos_yaw = math.cos(self._robot_yaw)
        sin_yaw = math.sin(self._robot_yaw)
        forward = cos_yaw * dx + sin_yaw * dy
        lateral = -sin_yaw * dx + cos_yaw * dy

        base_mask = (
            (forward > self._cloud_rear_limit)
            & (forward < self._cloud_forward_limit)
            & (np.abs(lateral) < self._cloud_side_limit)
            & (dz > self._cloud_min_rel_z)
            & (dz < self._cloud_max_rel_z)
        )

        front_mask = base_mask & (forward > 0.0) & (np.abs(lateral) < self._front_half_width)
        left_mask = (
            base_mask
            & (forward > self._side_forward_min)
            & (forward < self._side_forward_max)
            & (lateral > self._side_inner_offset)
            & (lateral < self._side_outer_offset)
        )
        right_mask = (
            base_mask
            & (forward > self._side_forward_min)
            & (forward < self._side_forward_max)
            & (lateral < -self._side_inner_offset)
            & (lateral > -self._side_outer_offset)
        )

        front = float(np.min(forward[front_mask])) if np.any(front_mask) else math.inf
        left = float(np.min(lateral[left_mask])) if np.any(left_mask) else math.inf
        right = float(np.min(-lateral[right_mask])) if np.any(right_mask) else math.inf

        self._cloud_time = self.get_clock().now()
        self._cloud_clearance = Clearance(front=front, left=left, right=right)

    def _copy_twist(self, msg: Twist) -> Twist:
        out = Twist()
        out.linear.x = msg.linear.x
        out.linear.y = msg.linear.y
        out.linear.z = msg.linear.z
        out.angular.x = msg.angular.x
        out.angular.y = msg.angular.y
        out.angular.z = msg.angular.z
        return out

    def _publish_state(self, state: str) -> None:
        if state == self._last_state:
            return
        self._last_state = state
        status = String()
        status.data = state
        self._status_pub.publish(status)
        self.get_logger().info(f"Obstacle filter state: {state}")

    def _publish_zero(self, state: str) -> None:
        self._cmd_pub.publish(Twist())
        self._motion_active = False
        self._publish_state(state)

    def _timer_stale(self, stamp) -> bool:
        if stamp is None:
            return True
        return (self.get_clock().now() - stamp).nanoseconds > self._sensor_timeout_ns

    def _combine_clearance(self) -> Clearance:
        sources = []
        if self._use_range_info and not self._timer_stale(self._range_time):
            sources.append(self._range_clearance)
        if self._use_cloud and not self._timer_stale(self._cloud_time):
            sources.append(self._cloud_clearance)
        if not sources:
            return Clearance()
        return Clearance(
            front=min(s.front for s in sources),
            left=min(s.left for s in sources),
            right=min(s.right for s in sources),
        )

    def _smooth_distance(self, previous: float, current: float) -> float:
        if not math.isfinite(previous):
            return current
        if not math.isfinite(current):
            return previous
        if current <= previous:
            return current
        return previous + self._clearance_release_alpha * (current - previous)

    def _smooth_clearance(self, clearance: Clearance) -> Clearance:
        self._filtered_clearance = Clearance(
            front=self._smooth_distance(self._filtered_clearance.front, clearance.front),
            left=self._smooth_distance(self._filtered_clearance.left, clearance.left),
            right=self._smooth_distance(self._filtered_clearance.right, clearance.right),
        )
        return self._filtered_clearance

    def _choose_avoid_direction(self, left: float, right: float) -> float:
        now_ns = self.get_clock().now().nanoseconds
        if self._avoid_direction_sign != 0.0 and now_ns < self._avoid_direction_lock_until_ns:
            return self._avoid_direction_sign

        if abs(left - right) < self._side_selection_margin and self._avoid_direction_sign != 0.0:
            sign = self._avoid_direction_sign
        else:
            sign = 1.0 if left >= right else -1.0

        self._avoid_direction_sign = sign
        self._avoid_direction_lock_until_ns = now_ns + int(self._avoid_direction_lock_time * 1e9)
        return sign

    def _publish_fused_clearance(self, clearance: Clearance) -> None:
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.point.x = clearance.front if math.isfinite(clearance.front) else -1.0
        msg.point.y = clearance.left if math.isfinite(clearance.left) else -1.0
        msg.point.z = clearance.right if math.isfinite(clearance.right) else -1.0
        self._fused_range_pub.publish(msg)

    def _slowdown_factor(self, distance: float, stop_distance: float, slow_distance: float) -> float:
        if not math.isfinite(distance):
            return 1.0
        if distance <= stop_distance:
            return 0.0
        if distance >= slow_distance:
            return 1.0
        span = max(slow_distance - stop_distance, 1e-6)
        ratio = (distance - stop_distance) / span
        return self._minimum_slowdown_ratio + ratio * (1.0 - self._minimum_slowdown_ratio)

    def _on_timer(self) -> None:
        if self._latest_cmd_time is None:
            return

        if (self.get_clock().now() - self._latest_cmd_time).nanoseconds > self._cmd_timeout_ns:
            if self._motion_active:
                self._publish_zero("cmd_timeout")
            return

        if self._odom_time is None or self._timer_stale(self._odom_time):
            if self._fail_safe_stop_on_stale:
                self._publish_zero("stale_odom")
            return

        clearance = self._smooth_clearance(self._combine_clearance())
        self._publish_fused_clearance(clearance)

        if (
            self._fail_safe_stop_on_stale
            and self._use_range_info
            and self._use_cloud
            and self._timer_stale(self._range_time)
            and self._timer_stale(self._cloud_time)
        ):
            self._publish_zero("stale_sensors")
            return

        cmd = self._copy_twist(self._latest_cmd)
        state = "clear"
        front = clearance.front
        left = clearance.left
        right = clearance.right

        if left < self._side_stop_distance and cmd.linear.y > self._linear_deadband:
            cmd.linear.y = 0.0
        if right < self._side_stop_distance and cmd.linear.y < -self._linear_deadband:
            cmd.linear.y = 0.0
        if left < self._side_stop_distance and cmd.angular.z > self._angular_deadband:
            cmd.angular.z = 0.0
        if right < self._side_stop_distance and cmd.angular.z < -self._angular_deadband:
            cmd.angular.z = 0.0

        front_factor = self._slowdown_factor(front, self._front_stop_distance, self._front_slow_distance)
        side_factor = self._slowdown_factor(min(left, right), self._side_stop_distance, self._side_slow_distance)
        linear_factor = min(front_factor, side_factor)
        if linear_factor < 1.0:
            cmd.linear.x *= linear_factor
            cmd.linear.y *= linear_factor
            state = "slowdown"

        if front < self._front_stop_distance and self._latest_cmd.linear.x > self._linear_deadband:
            side_sign = self._choose_avoid_direction(left, right)
            side_open = max(left, right) > self._side_slow_distance
            if side_open:
                cmd.linear.x = min(cmd.linear.x, self._avoid_forward_speed)
                cmd.linear.y += side_sign * self._avoid_lateral_speed
                cmd.angular.z += side_sign * self._avoid_turn_speed
                state = "avoid"
            else:
                cmd.linear.x = 0.0
                cmd.linear.y = 0.0
                cmd.angular.z = 0.0
                state = "stop"
        elif front < self._front_avoid_distance and self._latest_cmd.linear.x > self._linear_deadband:
            closeness = (
                (self._front_avoid_distance - front)
                / max(self._front_avoid_distance - self._front_stop_distance, 1e-6)
                if math.isfinite(front)
                else 0.0
            )
            side_sign = self._choose_avoid_direction(left, right)
            cmd.linear.x = min(cmd.linear.x, self._avoid_forward_speed + 0.08)
            cmd.linear.y += side_sign * self._avoid_lateral_speed * closeness
            cmd.angular.z += side_sign * self._avoid_turn_speed * closeness
            state = "avoid"

        if left < self._side_repulsion_distance:
            closeness = (self._side_repulsion_distance - left) / max(self._side_repulsion_distance, 1e-6)
            cmd.linear.y -= self._avoid_lateral_speed * self._lateral_repulsion_gain * closeness
            cmd.angular.z -= self._avoid_turn_speed * self._yaw_repulsion_gain * closeness
            if state != "stop":
                state = "avoid"
        if right < self._side_repulsion_distance:
            closeness = (self._side_repulsion_distance - right) / max(self._side_repulsion_distance, 1e-6)
            cmd.linear.y += self._avoid_lateral_speed * self._lateral_repulsion_gain * closeness
            cmd.angular.z += self._avoid_turn_speed * self._yaw_repulsion_gain * closeness
            if state != "stop":
                state = "avoid"

        cmd.linear.x = clamp(cmd.linear.x, -self._max_linear_x, self._max_linear_x)
        cmd.linear.y = clamp(cmd.linear.y, -self._max_linear_y, self._max_linear_y)
        cmd.angular.z = clamp(cmd.angular.z, -self._max_angular_z, self._max_angular_z)

        if state == "stop":
            self._publish_zero(state)
            return

        self._cmd_pub.publish(cmd)
        self._motion_active = (
            abs(cmd.linear.x) > self._linear_deadband
            or abs(cmd.linear.y) > self._linear_deadband
            or abs(cmd.angular.z) > self._angular_deadband
        )
        self._publish_state(state)


def main() -> None:
    rclpy.init()
    node = DynamicObstacleFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
