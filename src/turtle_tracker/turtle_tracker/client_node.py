import rclpy
from rclpy.node import Node
from turtle_tracker_interfaces.srv import TurtleInfo

class InfoClient(Node):

    def __init__(self):
        super().__init__('info_client')
        self.cli = self.create_client(TurtleInfo, 'turtle_info')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = TurtleInfo.Request()
        self.create_timer(1.0, self.send_request)

    def send_request(self):
        future = self.cli.call_async(self.req)
        future.add_done_callback(self.callback)

    def callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(
                f'\n--- Turtle Info ---\n'
                f'Explorer: ({response.explorer_x:.2f}, {response.explorer_y:.2f}), Theta: {response.explorer_theta:.2f}\n'
                f'Turtle1:  ({response.turtle1_x:.2f}, {response.turtle1_y:.2f}), Theta: {response.turtle1_theta:.2f}\n'
                f'Distance: {response.distance:.2f}\n'
                f'-------------------'
            )
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = InfoClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
