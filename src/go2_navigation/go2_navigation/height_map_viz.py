from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, List, Sequence, Tuple

import rclpy
from builtin_interfaces.msg import Time
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from unitree_go.msg import HeightMap

Point3 = Tuple[float, float, float]

DEFAULT_INPUT_TOPIC = "/utlidar/height_map_array"
DEFAULT_OUTPUT_TOPIC = "/utlidar/height_map_viz"
DEFAULT_SENTINEL = 1e8

RELIABLE_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)


@dataclass(frozen=True)
class HeightMapView:
    frame_id: str
    stamp_sec: float
    resolution: float
    width: int
    height: int
    origin: Tuple[float, float]
    data: Sequence[float]


def convert_height_map_to_points(
    height_map: HeightMapView,
    sentinel_threshold: float = DEFAULT_SENTINEL,
) -> List[Point3]:
    points: List[Point3] = []
    width = int(height_map.width)
    height = int(height_map.height)
    resolution = float(height_map.resolution)
    origin_x, origin_y = float(height_map.origin[0]), float(height_map.origin[1])

    for iy in range(height):
        row_offset = width * iy
        y = iy * resolution + origin_y
        for ix in range(width):
            cell_value = float(height_map.data[row_offset + ix])
            if abs(cell_value) >= sentinel_threshold:
                continue
            x = ix * resolution + origin_x
            points.append((x, y, cell_value))

    return points


def stamp_from_seconds(stamp_sec: float) -> Time:
    stamp = Time()
    stamp.sec = int(stamp_sec)
    stamp.nanosec = int(round((stamp_sec - stamp.sec) * 1e9))
    if stamp.nanosec >= 1_000_000_000:
        stamp.sec += 1
        stamp.nanosec -= 1_000_000_000
    return stamp


def height_map_message_to_view(message: HeightMap) -> HeightMapView:
    return HeightMapView(
        frame_id=str(message.frame_id),
        stamp_sec=float(message.stamp),
        resolution=float(message.resolution),
        width=int(message.width),
        height=int(message.height),
        origin=(float(message.origin[0]), float(message.origin[1])),
        data=list(message.data),
    )


def points_to_pointcloud2(
    frame_id: str,
    stamp_sec: float,
    points: Iterable[Point3],
) -> PointCloud2:
    header = Header()
    header.frame_id = frame_id
    header.stamp = stamp_from_seconds(stamp_sec)
    return point_cloud2.create_cloud_xyz32(header, list(points))


class HeightMapVizNode(Node):
    def __init__(self) -> None:
        super().__init__("height_map_viz")

        self.declare_parameter("input_topic", DEFAULT_INPUT_TOPIC)
        self.declare_parameter("output_topic", DEFAULT_OUTPUT_TOPIC)
        self.declare_parameter("sentinel_threshold", DEFAULT_SENTINEL)

        input_topic = self.get_parameter("input_topic").get_parameter_value().string_value
        output_topic = self.get_parameter("output_topic").get_parameter_value().string_value
        self._sentinel_threshold = (
            self.get_parameter("sentinel_threshold").get_parameter_value().double_value
        )

        self._publisher = self.create_publisher(PointCloud2, output_topic, RELIABLE_QOS)
        self._subscription = self.create_subscription(
            HeightMap,
            input_topic,
            self._on_height_map,
            RELIABLE_QOS,
        )

        self.get_logger().info(
            f"Converting {input_topic} -> {output_topic} "
            f"(sentinel_threshold={self._sentinel_threshold})"
        )

    def _on_height_map(self, message: HeightMap) -> None:
        view = height_map_message_to_view(message)
        points = convert_height_map_to_points(view, self._sentinel_threshold)
        cloud = points_to_pointcloud2(view.frame_id, view.stamp_sec, points)
        self._publisher.publish(cloud)


def main(args: Sequence[str] | None = None) -> None:
    rclpy.init(args=args)
    node = HeightMapVizNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
