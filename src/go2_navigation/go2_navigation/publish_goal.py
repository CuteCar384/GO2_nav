import argparse
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


def quaternion_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class GoalPublisher(Node):
    def __init__(self, topic: str, frame_id: str, x: float, y: float, z: float, yaw: float) -> None:
        super().__init__("go2_goal_publisher")
        self._publisher = self.create_publisher(PoseStamped, topic, 10)
        self._message = PoseStamped()
        self._message.header.frame_id = frame_id
        self._message.pose.position.x = x
        self._message.pose.position.y = y
        self._message.pose.position.z = z
        qx, qy, qz, qw = quaternion_from_yaw(yaw)
        self._message.pose.orientation.x = qx
        self._message.pose.orientation.y = qy
        self._message.pose.orientation.z = qz
        self._message.pose.orientation.w = qw
        self._publish_count = 0
        self._timer = self.create_timer(0.1, self._on_timer)

    def _on_timer(self) -> None:
        self._message.header.stamp = self.get_clock().now().to_msg()
        self._publisher.publish(self._message)
        self._publish_count += 1
        if self._publish_count >= 5:
            raise SystemExit


def main() -> None:
    parser = argparse.ArgumentParser(description="Publish a one-shot 2D goal for the Go2 simple goal controller.")
    parser.add_argument("--topic", default="/go2_navigation/goal")
    parser.add_argument("--frame-id", default="odom")
    parser.add_argument("--x", type=float, required=True)
    parser.add_argument("--y", type=float, required=True)
    parser.add_argument("--z", type=float, default=0.0)
    parser.add_argument("--yaw", type=float, default=0.0)
    args = parser.parse_args()

    rclpy.init()
    node = GoalPublisher(args.topic, args.frame_id, args.x, args.y, args.z, args.yaw)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
