import rclpy
from rclpy.node import Node
from exo_interfaces.srv import Test


class TestClient(Node):
    def __init__(self):
        super().__init__('test_client')
        self.client = self.create_client(Test, 'test')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for ESP32...')

        request = Test.Request()
        request.number = 5

        self.future = self.client.call_async(request)
        self.future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        response = future.result()
        self.get_logger().info(
            f'ESP32 returned: {response.result}'
        )


def main():
    rclpy.init()
    node = TestClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()