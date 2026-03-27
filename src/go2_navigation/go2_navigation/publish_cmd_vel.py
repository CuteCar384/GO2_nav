import argparse
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class CmdVelPublisher(Node):
    def __init__(
        self,
        topic: str,
        linear_x: float,
        linear_y: float,
        angular_z: float,
        duration: float,
        rate_hz: float,
    ) -> None:
        super().__init__("go2_cmd_vel_publisher")
        self._publisher = self.create_publisher(Twist, topic, 10)
        self._message = Twist()
        self._message.linear.x = linear_x
        self._message.linear.y = linear_y
        self._message.angular.z = angular_z
        self._duration = max(duration, 0.0)
        self._period = 1.0 / max(rate_hz, 1.0)

    def run(self) -> None:
        self.get_logger().info(
            "Publishing cmd_vel on %s for %.2fs: vx=%.3f vy=%.3f wz=%.3f"
            % (
                self._publisher.topic_name,
                self._duration,
                self._message.linear.x,
                self._message.linear.y,
                self._message.angular.z,
            )
        )

        end_time = time.monotonic() + self._duration
        while time.monotonic() < end_time and rclpy.ok():
            self._publisher.publish(self._message)
            time.sleep(self._period)

        self._publisher.publish(Twist())
        self.get_logger().info("Published trailing zero Twist.")


def main() -> None:
    parser = argparse.ArgumentParser(description="Publish a fixed Twist for a short duration.")
    parser.add_argument("--topic", default="/cmd_vel")
    parser.add_argument("--x", type=float, default=0.3)
    parser.add_argument("--y", type=float, default=0.0)
    parser.add_argument("--yaw", type=float, default=0.0)
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument("--rate", type=float, default=20.0)
    args = parser.parse_args()

    rclpy.init()
    node = CmdVelPublisher(args.topic, args.x, args.y, args.yaw, args.duration, args.rate)
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
