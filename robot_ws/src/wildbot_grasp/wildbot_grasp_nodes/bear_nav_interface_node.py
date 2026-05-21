import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger


class BearNavInterfaceNode(Node):
    """Topic-only bridge reserved for future Navigation integration.

    Future Nav2/task code should subscribe to /bear_grasp/nav/request and publish
    /bear_grasp/nav/status. This node is a lightweight helper for testing and for
    documenting the contract.
    """

    def __init__(self):
        super().__init__("bear_nav_interface_node")
        self.declare_parameter("request_topic", "/bear_grasp/nav/request")
        self.declare_parameter("status_topic", "/bear_grasp/nav/status")
        self.request_topic = str(self.get_parameter("request_topic").value)
        self.status_topic = str(self.get_parameter("status_topic").value)
        self.latest_request = None
        self.latest_status = {
            "capture_area_reached": False,
            "home_reached": False,
            "state": "idle",
        }
        self.create_subscription(String, self.request_topic, self._on_request, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.create_service(Trigger, "~/mark_capture_reached", self._mark_capture_reached)
        self.create_service(Trigger, "~/mark_home_reached", self._mark_home_reached)
        self.create_service(Trigger, "~/reset", self._reset)
        self.create_timer(1.0, self._publish_status)
        self.get_logger().info(
            f"Bear nav interface ready: subscribe {self.request_topic}, publish {self.status_topic}"
        )

    def _on_request(self, msg: String):
        try:
            self.latest_request = json.loads(msg.data)
        except json.JSONDecodeError:
            self.latest_request = {"raw": msg.data}
        command = self.latest_request.get("command") if isinstance(self.latest_request, dict) else None
        if command == "go_to_capture_area":
            self.latest_status.update(state="going_to_capture_area", capture_area_reached=False)
        elif command == "return_home":
            self.latest_status.update(state="returning_home", home_reached=False)
        else:
            self.latest_status.update(state="request_received")
        self.get_logger().info(f"nav request received: {msg.data}")
        self._publish_status()

    def _mark_capture_reached(self, _request, response):
        self.latest_status.update(state="capture_area_reached", capture_area_reached=True)
        self._publish_status()
        response.success = True
        response.message = "capture area marked reached"
        return response

    def _mark_home_reached(self, _request, response):
        self.latest_status.update(state="home_reached", home_reached=True)
        self._publish_status()
        response.success = True
        response.message = "home marked reached"
        return response

    def _reset(self, _request, response):
        self.latest_status.update(state="idle", capture_area_reached=False, home_reached=False)
        self._publish_status()
        response.success = True
        response.message = "nav interface reset"
        return response

    def _publish_status(self):
        msg = String()
        payload = dict(self.latest_status)
        payload["stamp_monotonic"] = time.monotonic()
        if self.latest_request is not None:
            payload["latest_request"] = self.latest_request
        msg.data = json.dumps(payload)
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = BearNavInterfaceNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
