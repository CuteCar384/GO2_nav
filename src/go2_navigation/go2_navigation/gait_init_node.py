"""Initialize GO2 gait mode on startup."""
import json

import rclpy
from rclpy.node import Node
from unitree_api.msg import Request


API_ID_CLASSIC_WALK = 2049


class GaitInitNode(Node):
    """Send classic walk gait mode request on startup."""

    def __init__(self) -> None:
        super().__init__("gait_init_node")

        self.declare_parameter("sport_request_topic", "/api/sport/request")
        sport_request_topic = self.get_parameter("sport_request_topic").value

        self._request_pub = self.create_publisher(Request, sport_request_topic, 10)

        # Wait briefly for the API to be ready, then send classic walk enable
        self._timer = self.create_timer(0.5, self._on_timer)
        self._request_sent = False
        self.get_logger().info("Gait init node started, will send ClassicWalk(True) request")

    def _on_timer(self) -> None:
        if self._request_sent:
            return

        request = Request()
        request.header.identity.id = 1
        request.header.identity.api_id = API_ID_CLASSIC_WALK
        request.header.policy.priority = 0
        request.header.policy.noreply = True
        request.parameter = json.dumps({"data": True})

        self._request_pub.publish(request)
        self._request_sent = True
        self.get_logger().info("Sent ClassicWalk(True) to set classic gait mode")

        # Shutdown after sending
        self.get_logger().info("Gait init complete, shutting down node")
        raise KeyboardInterrupt


def main() -> None:
    rclpy.init()
    node = GaitInitNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
