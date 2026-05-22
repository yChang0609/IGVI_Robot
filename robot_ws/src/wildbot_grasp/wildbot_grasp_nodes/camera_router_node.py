import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class CameraRouterNode(Node):
    def __init__(self):
        super().__init__('camera_router_node')

        # Parameters
        self.declare_parameter('rgb_raw_topic', '/rgb/image_raw')
        self.declare_parameter('depth_raw_topic', '/depth_to_rgb/image_raw')
        self.declare_parameter('rgb_filtered_topic', '/rgb/image_filtered')
        self.declare_parameter('depth_filtered_topic', '/depth_to_rgb/image_filtered')
        self.declare_parameter('rgb_slam_topic', '/rgb/image_slam')
        self.declare_parameter('depth_slam_topic', '/depth_to_rgb/image_slam')
        self.declare_parameter('check_rate_hz', 1.0)

        self.filtered_rgb_topic = self.get_parameter('rgb_filtered_topic').value
        self.use_filtered = False

        # Subscribers
        self.create_subscription(
            Image,
            self.get_parameter('rgb_raw_topic').value,
            self.rgb_raw_callback,
            10
        )
        self.create_subscription(
            Image,
            self.get_parameter('depth_raw_topic').value,
            self.depth_raw_callback,
            10
        )
        self.create_subscription(
            Image,
            self.filtered_rgb_topic,
            self.rgb_filtered_callback,
            10
        )
        self.create_subscription(
            Image,
            self.get_parameter('depth_filtered_topic').value,
            self.depth_filtered_callback,
            10
        )

        # Publishers
        self.rgb_slam_pub = self.create_publisher(
            Image,
            self.get_parameter('rgb_slam_topic').value,
            10
        )
        self.depth_slam_pub = self.create_publisher(
            Image,
            self.get_parameter('depth_slam_topic').value,
            10
        )

        # Background check timer
        rate = float(self.get_parameter('check_rate_hz').value)
        self.create_timer(1.0 / rate, self.check_graph_timer)

        self.get_logger().info(
            f"Camera Router Node initialized. Listening on raw and filtered feeds. "
            f"Routing to: {self.get_parameter('rgb_slam_topic').value}"
        )

    def check_graph_timer(self):
        # Count the active publishers on the filtered RGB topic
        try:
            num_publishers = self.count_publishers(self.filtered_rgb_topic)
            active = num_publishers > 0
        except Exception as e:
            self.get_logger().warning(f"Error checking publishers for {self.filtered_rgb_topic}: {e}")
            active = False

        if active != self.use_filtered:
            self.use_filtered = active
            state_str = "FILTERED (Masking active)" if active else "RAW (Direct camera feed)"
            self.get_logger().info(f"Router input switched to: {state_str}")

    def rgb_raw_callback(self, msg: Image):
        if not self.use_filtered:
            # Fast zero-copy forwarding of the ROS message pointer
            self.rgb_slam_pub.publish(msg)

    def depth_raw_callback(self, msg: Image):
        if not self.use_filtered:
            self.depth_slam_pub.publish(msg)

    def rgb_filtered_callback(self, msg: Image):
        if self.use_filtered:
            self.rgb_slam_pub.publish(msg)

    def depth_filtered_callback(self, msg: Image):
        if self.use_filtered:
            self.depth_slam_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CameraRouterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
