import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from turtle_tracker_interfaces.srv import TurtleInfo

import math


class ExplorerNode(Node):

    def __init__(self):
        super().__init__('explorer_node')

        # ===== E1: Spawn de la tortuga =====
        self.spawn_client = self.create_client(Spawn, '/spawn')
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando al servicio /spawn...')

        request = Spawn.Request()
        request.x = 2.0
        request.y = 2.0
        request.theta = 0.0
        request.name = 'explorer'
        self.spawn_client.call_async(request)

        # ===== Variables de estado =====
        self.turtle1_pose = None
        self.explorer_pose = None

        # ===== Subscripciones =====
        self.create_subscription(Pose, '/turtle1/pose', self.turtle1_callback, 10)
        self.create_subscription(Pose, '/explorer/pose', self.explorer_callback, 10)

        # ===== Publicador de velocidad =====
        self.cmd_pub = self.create_publisher(Twist, '/explorer/cmd_vel', 10)

        # ===== Timer de control =====
        self.timer = self.create_timer(0.1, self.control_loop)

        # Ganancias proporcionales
        self.k_linear = 1.5
        self.k_angular = 4.0

        # Servicio turtle_info
        self.srv = self.create_service(
            TurtleInfo,
            'turtle_info',
            self.turtle_info_callback
        )

        # Guardar última velocidad
        self.last_cmd = Twist()


    def turtle1_callback(self, msg):
        self.turtle1_pose = msg

    def explorer_callback(self, msg):
        self.explorer_pose = msg

    def control_loop(self):
        if self.turtle1_pose is None or self.explorer_pose is None:
            return

        # Diferencias de posición
        dx = self.turtle1_pose.x - self.explorer_pose.x
        dy = self.turtle1_pose.y - self.explorer_pose.y

        # Distancia euclidiana
        distance = math.sqrt(dx**2 + dy**2)

        # Ángulo hacia turtle1
        target_angle = math.atan2(dy, dx)

        # Error angular
        angle_error = target_angle - self.explorer_pose.theta

        # Normalizar ángulo a [-pi, pi]
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

        # Control proporcional
        cmd = Twist()
        cmd.linear.x = self.k_linear * distance
        cmd.angular.z = self.k_angular * angle_error
        
        self.last_cmd = cmd
        self.cmd_pub.publish(cmd)
    
    def turtle_info_callback(self, request, response):
        if self.turtle1_pose is None or self.explorer_pose is None:
            return response

        # Posiciones turtle1
        response.turtle1_x = self.turtle1_pose.x
        response.turtle1_y = self.turtle1_pose.y
        response.turtle1_theta = self.turtle1_pose.theta

        # Posiciones explorer
        response.explorer_x = self.explorer_pose.x
        response.explorer_y = self.explorer_pose.y
        response.explorer_theta = self.explorer_pose.theta

        # Velocidades actuales
        response.explorer_linear_velocity = self.last_cmd.linear.x
        response.explorer_angular_velocity = self.last_cmd.angular.z

        # Distancia euclidiana
        dx = self.turtle1_pose.x - self.explorer_pose.x
        dy = self.turtle1_pose.y - self.explorer_pose.y
        response.distance = math.sqrt(dx**2 + dy**2)

        return response




def main(args=None):
    rclpy.init(args=args)
    node = ExplorerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
