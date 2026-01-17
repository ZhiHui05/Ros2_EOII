import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose  # Importamos el tipo de mensaje del topic

class TurtlePoseSubscriber(Node):
    def __init__(self):
        super().__init__('turtle_pose_subscriber')
        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.listener_callback,
            10
        )
        self.subscription  # evita warning de variable no usada

    def listener_callback(self, msg):
        self.get_logger().info(
            f"Turtle position -> X: {msg.x:.2f}, Y: {msg.y:.2f}, Theta: {msg.theta:.2f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = TurtlePoseSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
