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
from unitree_go.msg import HeightMap


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


@dataclass
class Clearance:
    front: float = math.inf
    rear: float = math.inf
    left: float = math.inf
    right: float = math.inf


@dataclass
class TerrainAssessment:
    max_step: float = 0.0
    max_drop: float = 0.0
    max_slope_deg: float = 0.0
    front_score: float = 0.0
    left_score: float = 0.0
    right_score: float = 0.0


class DynamicObstacleFilter(Node):
    """Fuse range and point-cloud data to stop, slow, or bias around obstacles."""

    def __init__(self) -> None:
        super().__init__("go2_dynamic_obstacle_filter")

        input_cmd_topic = self.declare_parameter("input_cmd_topic", "/cmd_vel_nav").value
        output_cmd_topic = self.declare_parameter("output_cmd_topic", "/cmd_vel").value
        range_info_topic = self.declare_parameter("range_info_topic", "/utlidar/range_info").value
        cloud_topic = self.declare_parameter("cloud_topic", "/utlidar/cloud_deskewed").value
        height_map_topic = self.declare_parameter(
            "height_map_topic", "/utlidar/height_map_array"
        ).value
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
        self._use_height_map = bool(self.declare_parameter("use_height_map", True).value)

        self._front_stop_distance = float(self.declare_parameter("front_stop_distance", 0.40).value)
        self._front_avoid_distance = float(
            self.declare_parameter("front_avoid_distance", 0.75).value
        )
        self._front_slow_distance = float(self.declare_parameter("front_slow_distance", 1.20).value)
        self._rear_stop_distance = float(self.declare_parameter("rear_stop_distance", 0.22).value)
        self._rear_slow_distance = float(self.declare_parameter("rear_slow_distance", 0.55).value)
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
        self._rear_half_width = float(self.declare_parameter("rear_half_width", 0.30).value)
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

        self._terrain_hard_step_limit = float(
            self.declare_parameter("terrain_hard_step_limit", 0.16).value
        )
        self._terrain_soft_step_limit = float(
            self.declare_parameter("terrain_soft_step_limit", 0.10).value
        )
        self._terrain_hard_drop_limit = float(
            self.declare_parameter("terrain_hard_drop_limit", 0.16).value
        )
        self._terrain_soft_drop_limit = float(
            self.declare_parameter("terrain_soft_drop_limit", 0.10).value
        )
        self._terrain_hard_slope_deg = float(
            self.declare_parameter("terrain_hard_slope_deg", 40.0).value
        )
        self._terrain_soft_slope_deg = float(
            self.declare_parameter("terrain_soft_slope_deg", 30.0).value
        )
        self._terrain_forward_near = float(
            self.declare_parameter("terrain_forward_near", 0.30).value
        )
        self._terrain_forward_far = float(
            self.declare_parameter("terrain_forward_far", 1.00).value
        )
        self._terrain_half_width = float(self.declare_parameter("terrain_half_width", 0.25).value)
        self._terrain_side_forward_far = float(
            self.declare_parameter("terrain_side_forward_far", 0.80).value
        )
        self._terrain_side_inner_offset = float(
            self.declare_parameter("terrain_side_inner_offset", 0.20).value
        )
        self._terrain_side_outer_offset = float(
            self.declare_parameter("terrain_side_outer_offset", 0.50).value
        )
        self._terrain_base_near = float(self.declare_parameter("terrain_base_near", -0.20).value)
        self._terrain_base_far = float(self.declare_parameter("terrain_base_far", 0.20).value)
        self._terrain_base_half_width = float(
            self.declare_parameter("terrain_base_half_width", 0.20).value
        )
        self._terrain_bias_gain = float(self.declare_parameter("terrain_bias_gain", 1.0).value)
        self._terrain_bias_margin = float(
            self.declare_parameter("terrain_bias_margin", 0.12).value
        )
        self._terrain_score_clip = float(self.declare_parameter("terrain_score_clip", 3.0).value)
        self._terrain_stop_hold_time = float(
            self.declare_parameter("terrain_stop_hold_time", 0.5).value
        )

        self._latest_cmd = Twist()
        self._latest_cmd_time = None
        self._motion_active = False
        self._range_clearance = Clearance()
        self._range_time = None
        self._cloud_clearance = Clearance()
        self._cloud_time = None
        self._terrain_time = None
        self._terrain = TerrainAssessment()
        self._odom_time = None
        self._robot_x = 0.0
        self._robot_y = 0.0
        self._robot_z = 0.0
        self._robot_yaw = 0.0
        self._last_state = ""
        self._filtered_clearance = Clearance()
        self._avoid_direction_sign = 0.0
        self._avoid_direction_lock_until_ns = 0
        self._terrain_stop_hold_until_ns = 0

        self._cmd_pub = self.create_publisher(Twist, output_cmd_topic, 10)
        self._status_pub = self.create_publisher(String, status_topic, 10)
        self._fused_range_pub = self.create_publisher(PointStamped, fused_range_topic, 10)

        self.create_subscription(Twist, input_cmd_topic, self._cmd_callback, 10)
        self.create_subscription(Odometry, odom_topic, self._odom_callback, 50)
        self.create_subscription(PointStamped, range_info_topic, self._range_callback, 20)
        self.create_subscription(PointCloud2, cloud_topic, self._cloud_callback, 10)
        self.create_subscription(HeightMap, height_map_topic, self._height_map_callback, 10)
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
        rear_mask = base_mask & (forward < 0.0) & (np.abs(lateral) < self._rear_half_width)
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
        rear = float(np.min(-forward[rear_mask])) if np.any(rear_mask) else math.inf
        left = float(np.min(lateral[left_mask])) if np.any(left_mask) else math.inf
        right = float(np.min(-lateral[right_mask])) if np.any(right_mask) else math.inf

        self._cloud_time = self.get_clock().now()
        self._cloud_clearance = Clearance(front=front, rear=rear, left=left, right=right)

    def _copy_twist(self, msg: Twist) -> Twist:
        out = Twist()
        out.linear.x = msg.linear.x
        out.linear.y = msg.linear.y
        out.linear.z = msg.linear.z
        out.angular.x = msg.angular.x
        out.angular.y = msg.angular.y
        out.angular.z = msg.angular.z
        return out

    def _normalize_hazard(self, value: float, soft_limit: float, hard_limit: float) -> float:
        if value <= soft_limit:
            return 0.0
        span = max(hard_limit - soft_limit, 1e-6)
        return clamp((value - soft_limit) / span, 0.0, self._terrain_score_clip)

    def _terrain_score(self, step: float, drop: float, slope_deg: float) -> float:
        return max(
            self._normalize_hazard(
                step, self._terrain_soft_step_limit, self._terrain_hard_step_limit
            ),
            self._normalize_hazard(
                drop, self._terrain_soft_drop_limit, self._terrain_hard_drop_limit
            ),
            self._normalize_hazard(
                slope_deg, self._terrain_soft_slope_deg, self._terrain_hard_slope_deg
            ),
        )

    def _window_terrain(
        self,
        rel_x: np.ndarray,
        rel_y: np.ndarray,
        height: np.ndarray,
        slope_deg: np.ndarray,
        baseline_height: float,
        mask: np.ndarray,
    ) -> tuple[float, float, float, float]:
        if not np.any(mask):
            return 0.0, 0.0, math.inf, self._terrain_score_clip

        window_height = height[mask]
        finite_height = window_height[np.isfinite(window_height)]
        if finite_height.size == 0:
            return 0.0, 0.0, math.inf, self._terrain_score_clip

        local_step = max(0.0, float(np.max(finite_height) - baseline_height))
        local_drop = max(0.0, float(baseline_height - np.min(finite_height)))

        local_slope_samples = slope_deg[mask]
        local_slope_samples = local_slope_samples[np.isfinite(local_slope_samples)]
        local_slope_deg = (
            float(np.percentile(local_slope_samples, 90))
            if local_slope_samples.size > 0
            else math.inf
        )
        return (
            local_step,
            local_drop,
            local_slope_deg,
            self._terrain_score(local_step, local_drop, local_slope_deg),
        )

    def _height_map_callback(self, msg: HeightMap) -> None:
        if not self._use_height_map or self._odom_time is None:
            return

        width = int(msg.width)
        height_cells = int(msg.height)
        expected_size = width * height_cells
        if width <= 1 or height_cells <= 1 or len(msg.data) != expected_size:
            self.get_logger().warn("Received invalid height map dimensions or data length")
            return

        resolution = float(msg.resolution)
        if resolution <= 0.0:
            self.get_logger().warn("Received non-positive height map resolution")
            return

        grid = np.asarray(msg.data, dtype=np.float32).reshape((height_cells, width))
        xs = float(msg.origin[0]) + np.arange(width, dtype=np.float32) * resolution
        ys = float(msg.origin[1]) + np.arange(height_cells, dtype=np.float32) * resolution
        world_x, world_y = np.meshgrid(xs, ys, indexing="xy")

        dx = world_x - self._robot_x
        dy = world_y - self._robot_y
        cos_yaw = math.cos(self._robot_yaw)
        sin_yaw = math.sin(self._robot_yaw)
        rel_x = cos_yaw * dx + sin_yaw * dy
        rel_y = -sin_yaw * dx + cos_yaw * dy

        base_mask = (
            (rel_x >= self._terrain_base_near)
            & (rel_x <= self._terrain_base_far)
            & (np.abs(rel_y) <= self._terrain_base_half_width)
        )
        base_samples = grid[base_mask]
        base_samples = base_samples[np.isfinite(base_samples)]
        if base_samples.size == 0:
            return
        baseline_height = float(np.median(base_samples))

        grad_y, grad_x = np.gradient(grid.astype(np.float64), resolution, resolution)
        slope_deg = np.rad2deg(np.arctan(np.hypot(grad_x, grad_y)))

        front_mask = (
            (rel_x >= self._terrain_forward_near)
            & (rel_x <= self._terrain_forward_far)
            & (np.abs(rel_y) <= self._terrain_half_width)
        )
        left_mask = (
            (rel_x >= self._terrain_forward_near)
            & (rel_x <= self._terrain_side_forward_far)
            & (rel_y >= self._terrain_side_inner_offset)
            & (rel_y <= self._terrain_side_outer_offset)
        )
        right_mask = (
            (rel_x >= self._terrain_forward_near)
            & (rel_x <= self._terrain_side_forward_far)
            & (rel_y <= -self._terrain_side_inner_offset)
            & (rel_y >= -self._terrain_side_outer_offset)
        )

        front_step, front_drop, front_slope_deg, front_score = self._window_terrain(
            rel_x, rel_y, grid, slope_deg, baseline_height, front_mask
        )
        left_step, left_drop, left_slope_deg, left_score = self._window_terrain(
            rel_x, rel_y, grid, slope_deg, baseline_height, left_mask
        )
        right_step, right_drop, right_slope_deg, right_score = self._window_terrain(
            rel_x, rel_y, grid, slope_deg, baseline_height, right_mask
        )

        self._terrain_time = self.get_clock().now()
        self._terrain = TerrainAssessment(
            max_step=front_step,
            max_drop=front_drop,
            max_slope_deg=front_slope_deg,
            front_score=front_score,
            left_score=left_score,
            right_score=right_score,
        )

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
            rear=min(s.rear for s in sources),
            left=min(s.left for s in sources),
            right=min(s.right for s in sources),
        )

    def _enabled_sensor_stale(self) -> bool:
        sensor_times = []
        if self._use_range_info:
            sensor_times.append(self._range_time)
        if self._use_cloud:
            sensor_times.append(self._cloud_time)
        if self._use_height_map:
            sensor_times.append(self._terrain_time)
        if not sensor_times:
            return False
        return all(self._timer_stale(stamp) for stamp in sensor_times)

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
            rear=self._smooth_distance(self._filtered_clearance.rear, clearance.rear),
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

    def _terrain_stop_active(self, now_ns: int) -> bool:
        return now_ns < self._terrain_stop_hold_until_ns

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

        if self._fail_safe_stop_on_stale and self._enabled_sensor_stale():
            self._publish_zero("stale_sensors")
            return

        cmd = self._copy_twist(self._latest_cmd)
        state = "clear"
        front = clearance.front
        rear = clearance.rear
        left = clearance.left
        right = clearance.right
        now_ns = self.get_clock().now().nanoseconds
        moving_forward = self._latest_cmd.linear.x > self._linear_deadband
        moving_reverse = self._latest_cmd.linear.x < -self._linear_deadband

        if left < self._side_stop_distance and cmd.linear.y > self._linear_deadband:
            cmd.linear.y = 0.0
        if right < self._side_stop_distance and cmd.linear.y < -self._linear_deadband:
            cmd.linear.y = 0.0
        if left < self._side_stop_distance and cmd.angular.z > self._angular_deadband:
            cmd.angular.z = 0.0
        if right < self._side_stop_distance and cmd.angular.z < -self._angular_deadband:
            cmd.angular.z = 0.0

        front_factor = self._slowdown_factor(front, self._front_stop_distance, self._front_slow_distance)
        rear_factor = self._slowdown_factor(rear, self._rear_stop_distance, self._rear_slow_distance)
        side_factor = self._slowdown_factor(min(left, right), self._side_stop_distance, self._side_slow_distance)
        longitudinal_factor = front_factor if moving_forward else rear_factor if moving_reverse else 1.0
        linear_factor = min(longitudinal_factor, side_factor)
        if linear_factor < 1.0:
            cmd.linear.x *= linear_factor
            cmd.linear.y *= linear_factor
            state = "slowdown"

        terrain = self._terrain
        terrain_fresh = self._use_height_map and not self._timer_stale(self._terrain_time)
        if terrain_fresh:
            terrain_hard_stop = (
                terrain.max_step >= self._terrain_hard_step_limit
                or terrain.max_drop >= self._terrain_hard_drop_limit
                or terrain.max_slope_deg >= self._terrain_hard_slope_deg
            )
            if terrain_hard_stop and moving_forward:
                self._terrain_stop_hold_until_ns = now_ns + int(self._terrain_stop_hold_time * 1e9)

        if self._terrain_stop_active(now_ns) and moving_forward:
            self._publish_zero("terrain_stop")
            return

        if terrain_fresh and not self._terrain_stop_active(now_ns):
                terrain_factor = 1.0 - clamp(terrain.front_score, 0.0, 1.0)
                terrain_factor = max(self._minimum_slowdown_ratio, terrain_factor)
                if terrain.front_score > 0.0:
                    cmd.linear.x *= terrain_factor
                    cmd.linear.y *= terrain_factor
                    if state == "clear":
                        state = "terrain_slowdown"

                score_delta = terrain.left_score - terrain.right_score
                if abs(score_delta) > self._terrain_bias_margin and state != "terrain_stop":
                    bias_sign = -1.0 if score_delta > 0.0 else 1.0
                    bias_scale = clamp(abs(score_delta), 0.0, 1.0) * self._terrain_bias_gain
                    cmd.linear.y += bias_sign * self._avoid_lateral_speed * bias_scale
                    cmd.angular.z += bias_sign * self._avoid_turn_speed * bias_scale
                    if state in {"clear", "terrain_slowdown"}:
                        state = "terrain_bias"

        if rear < self._rear_stop_distance and moving_reverse:
            cmd.linear.x = 0.0
            cmd.linear.y = 0.0
            cmd.angular.z = 0.0
            state = "reverse_stop"
        elif front < self._front_stop_distance and moving_forward:
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
        elif front < self._front_avoid_distance and moving_forward:
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

        if state in {"stop", "terrain_stop", "reverse_stop"}:
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
