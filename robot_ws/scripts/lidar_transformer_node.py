import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class LidarRePublisher(Node):
    """A ROS2 node that republishes Lidar scan messages with an updated timestamp.

    This node subscribes to a Lidar scan topic (`/scan_tmp`), updates the timestamp of the 
    received messages, and republishes them to another topic (`/scan`).

    Attributes:
        subscription (Subscription): The subscription to the `/scan_tmp` topic.
        publisher_ (Publisher): The publisher to the `/scan` topic.
    """

    def __init__(self):
        """Initialize the LidarRePublisher node.

        The constructor sets up a subscription to the `/scan_tmp` topic and initializes 
        a publisher to the `/scan` topic. The callback function `listener_callback` is triggered 
        upon receiving a message from the `/scan_tmp` topic.
        """
        super().__init__('lidar_republisher')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan_tmp',
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(LaserScan, '/scan', 10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        """Callback function that processes incoming Lidar messages.

        This function is called when a message is received from the `/scan_tmp` topic.
        It updates the message's timestamp and republishes the message to the `/scan` topic.

        Args:
            msg (LaserScan): The Lidar scan message received from the subscription.
        """
        # Update the timestamp
        msg.header.stamp = self.get_clock().now().to_msg()

        # Publish the message to /scan
        self.publisher_.publish(msg)
        self.get_logger().info('Republishing lidar data with updated timestamp')


def main(args=None):
    """Entry point for the LidarRePublisher node.

    Initializes the ROS2 Python client library, creates the LidarRePublisher node, 
    and keeps it spinning to process messages until the node is shut down.
    
    Args:
        args (list, optional): Command-line arguments. Defaults to None.
    """
    rclpy.init(args=args)
    lidar_republisher = LidarRePublisher()
    rclpy.spin(lidar_republisher)

    # Destroy the node
    lidar_republisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
