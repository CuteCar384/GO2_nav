from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import List, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


PCD_TYPE_MAP = {
    ("F", 4): np.float32,
    ("F", 8): np.float64,
    ("I", 1): np.int8,
    ("I", 2): np.int16,
    ("I", 4): np.int32,
    ("I", 8): np.int64,
    ("U", 1): np.uint8,
    ("U", 2): np.uint16,
    ("U", 4): np.uint32,
    ("U", 8): np.uint64,
}

POINT_FIELD_TYPE_MAP = {
    np.dtype(np.float32): PointField.FLOAT32,
    np.dtype(np.float64): PointField.FLOAT64,
    np.dtype(np.int8): PointField.INT8,
    np.dtype(np.uint8): PointField.UINT8,
    np.dtype(np.int16): PointField.INT16,
    np.dtype(np.uint16): PointField.UINT16,
    np.dtype(np.int32): PointField.INT32,
    np.dtype(np.uint32): PointField.UINT32,
}


@dataclass
class ParsedPcd:
    structured_points: np.ndarray
    fields: List[PointField]
    point_count: int


def parse_pcd(path: str) -> ParsedPcd:
    header = {}
    data_type = None
    with Path(path).open("rb") as handle:
        while True:
            line = handle.readline()
            if not line:
                raise ValueError("Unexpected end of file while reading PCD header")
            decoded = line.decode("ascii").strip()
            if not decoded or decoded.startswith("#"):
                continue
            key, _, value = decoded.partition(" ")
            key = key.upper()
            header[key] = value
            if key == "DATA":
                data_type = value.lower()
                break

        if data_type != "binary":
            raise ValueError(f"Only binary PCD is supported, got DATA {data_type!r}")

        fields = header["FIELDS"].split()
        sizes = [int(v) for v in header["SIZE"].split()]
        types = header["TYPE"].split()
        counts = [int(v) for v in header.get("COUNT", " ".join(["1"] * len(fields))).split()]
        point_count = int(header.get("POINTS", header["WIDTH"]))

        dtype_fields: List[Tuple[str, np.dtype]] = []
        point_fields: List[PointField] = []
        offset = 0

        for field_name, size, field_type, count in zip(fields, sizes, types, counts):
            base_dtype = np.dtype(PCD_TYPE_MAP[(field_type, size)])
            field_dtype = (base_dtype, count) if count > 1 else base_dtype
            dtype_fields.append((field_name, field_dtype))

            ros_datatype = POINT_FIELD_TYPE_MAP.get(base_dtype)
            if ros_datatype is None:
                raise ValueError(f"Unsupported PointField dtype {base_dtype} for field {field_name}")

            point_fields.append(
                PointField(
                    name=field_name,
                    offset=offset,
                    datatype=ros_datatype,
                    count=count,
                )
            )
            offset += size * count

        structured_dtype = np.dtype(dtype_fields)
        raw = handle.read()
        structured_points = np.frombuffer(raw, dtype=structured_dtype, count=point_count)

    return ParsedPcd(
        structured_points=structured_points,
        fields=point_fields,
        point_count=point_count,
    )


class PcdMapPublisher(Node):
    """Publish a saved PCD map as PointCloud2 with a stable frame and topic."""

    def __init__(self) -> None:
        super().__init__("go2_saved_map_publisher")

        pcd_path = self.declare_parameter("pcd_path", "/home/huang/xxx/output/go2_built_map.pcd").value
        cloud_topic = self.declare_parameter("cloud_topic", "/go2_saved_map").value
        frame_id = self.declare_parameter("frame_id", "odom").value
        publish_period = float(self.declare_parameter("publish_period", 1.0).value)

        self._frame_id = frame_id
        self._pcd = parse_pcd(pcd_path)
        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = ReliabilityPolicy.RELIABLE

        self._pub = self.create_publisher(PointCloud2, cloud_topic, qos)
        self._timer = self.create_timer(max(publish_period, 0.1), self._publish_map)

        self.get_logger().info(
            f"Publishing saved map {pcd_path} with {self._pcd.point_count} points on {cloud_topic} in frame {frame_id}"
        )
        self._publish_map()

    def _publish_map(self) -> None:
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self._frame_id
        msg = point_cloud2.create_cloud(
            header=header,
            fields=self._pcd.fields,
            points=self._pcd.structured_points.tolist(),
        )
        self._pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = PcdMapPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
