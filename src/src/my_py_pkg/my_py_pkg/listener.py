#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
class Listener(Node):
    def __init__(self):
        super().__init__('listener_py')
        self.suscriptor = self.create_subscription(String,'chatter',self.listener_callback,10)
    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = Listener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()