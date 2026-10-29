
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


class TestNode(Node):
    def __init__(self):
        super().__init__('exo_test_node')

        self.publisher = self.create_publisher(
            Int32, '/esp_test_tx', 10
        )

        self.subscription = self.create_subscription(
            Int32, '/esp_test_rx', self.receive_callback, 10
        )

        self.counter = 0
        self.timer = self.create_timer(1.0, self.send_callback)

        self.get_logger().info('Test node started')

    def send_callback(self):
        msg = Int32()
        msg.data = self.counter
        self.publisher.publish(msg)
        self.get_logger().info(f'Sent to ESP32: {msg.data}')
        self.counter += 1

    def receive_callback(self, msg):
        self.get_logger().info(f'Received from ESP32: {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    node = TestNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
