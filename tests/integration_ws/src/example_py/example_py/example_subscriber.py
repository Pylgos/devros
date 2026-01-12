"""Example ROS 2 subscriber for integration testing."""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ExampleSubscriber(Node):
    """A simple subscriber node for testing."""

    def __init__(self):
        super().__init__('example_subscriber')
        self.subscription = self.create_subscription(
            String,
            'example_topic',
            self.listener_callback,
            10)
        self.get_logger().info('Example subscriber initialized')

    def listener_callback(self, msg):
        self.get_logger().info(f'Received: {msg.data}')


def main(args=None):
    """Entry point for the subscriber node."""
    rclpy.init(args=args)
    node = ExampleSubscriber()
    
    # For testing, just log that we started
    node.get_logger().info('Subscriber ready')
    
    # Don't spin, just exit for testing
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    main()
