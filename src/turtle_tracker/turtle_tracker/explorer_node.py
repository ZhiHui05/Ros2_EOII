import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from turtlesim.srv import Spawn
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from turtle_tracker_interfaces.action import TurtleInfo

import math
import asyncio


class ExplorerNode(Node):

    def __init__(self):
        super().__init__('explorer_node')

        # ===== Parámetros E5 (posición inicial) =====
        self.declare_parameter('spawn_x', 2.0)
        self.declare_parameter('spawn_y', 2.0)

        x = self.get_parameter('spawn_x').value
        y = self.get_parameter('spawn_y').value

        if not (0.0 <= x <= 11.0 and 0.0 <= y <= 11.0):
            self.get_logger().error('Posición fuera de los límites de TurtleSim')
            x, y = 2.0, 2.0

        # ===== Spawn explorer usando /spawn =====
        self.spawn_client = self.create_client(Spawn, '/spawn')
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando /spawn...')

        req = Spawn.Request()
        req.x = x
        req.y = y
        req.theta = 0.0
        req.name = 'explorer'
        self.spawn_client.call_async(req)

    # ===== Estado de poses =====
        self.turtle1_pose = None
        self.explorer_pose = None

    # ===== Callback group (evitar bloqueo entre callbacks) =====
        self.callback_group = ReentrantCallbackGroup()

        # ===== Subscripciones a las poses de las tortugas =====
        self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.turtle1_callback,
            10,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            Pose,
            '/explorer/pose',
            self.explorer_callback,
            10,
            callback_group=self.callback_group,
        )

    # ===== Publisher para comandar a explorer =====
        self.cmd_pub = self.create_publisher(Twist, '/explorer/cmd_vel', 10)

    # ===== Control proporcional =====
        self.k_linear = 1.5
        self.k_angular = 4.0
        self.last_cmd = Twist()

    # Timer de control a 10 Hz
        self.create_timer(0.1, self.control_loop, callback_group=self.callback_group)

        # ===== Action Server (turtle_info) =====
        self.action_server = ActionServer(
            self,
            TurtleInfo,
            'turtle_info',
            self.execute_callback,
            callback_group=self.callback_group,
        )

    def turtle1_callback(self, msg):
        # Actualiza la pose de la tortuga principal
        self.turtle1_pose = msg

    def explorer_callback(self, msg):
        # Actualiza la pose de la tortuga perseguidora
        self.explorer_pose = msg

    def control_loop(self):
        # Control proporcional para seguir a turtle1
        if self.turtle1_pose is None or self.explorer_pose is None:
            return

        dx = self.turtle1_pose.x - self.explorer_pose.x
        dy = self.turtle1_pose.y - self.explorer_pose.y
        distance = math.sqrt(dx**2 + dy**2)

        target_angle = math.atan2(dy, dx)
        angle_error = target_angle - self.explorer_pose.theta
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

        cmd = Twist()
        cmd.linear.x = self.k_linear * distance
        cmd.angular.z = self.k_angular * angle_error

        self.last_cmd = cmd
        self.cmd_pub.publish(cmd)

    async def execute_callback(self, goal_handle):
        # Acción: enviar feedback periódico hasta que explorer alcance a turtle1
        self.get_logger().info('Action turtle_info aceptada')

        feedback = TurtleInfo.Feedback()

        while rclpy.ok():
            if self.turtle1_pose is None or self.explorer_pose is None:
                await asyncio.sleep(0.5)
                continue

            dx = self.turtle1_pose.x - self.explorer_pose.x
            dy = self.turtle1_pose.y - self.explorer_pose.y
            distance = math.sqrt(dx**2 + dy**2)

            # Feedback con toda la info requerida
            feedback.turtle1_x = self.turtle1_pose.x
            feedback.turtle1_y = self.turtle1_pose.y
            feedback.turtle1_theta = self.turtle1_pose.theta

            feedback.explorer_x = self.explorer_pose.x
            feedback.explorer_y = self.explorer_pose.y
            feedback.explorer_theta = self.explorer_pose.theta

            feedback.explorer_linear_velocity = self.last_cmd.linear.x
            feedback.explorer_angular_velocity = self.last_cmd.angular.z
            feedback.turtle1_linear_velocity = self.turtle1_pose.linear_velocity
            feedback.turtle1_angular_velocity = self.turtle1_pose.angular_velocity
            feedback.distance = distance

            goal_handle.publish_feedback(feedback)

            # Condición de finalización: explorer alcanza a turtle1
            if distance < 0.2:
                stop = Twist()
                self.cmd_pub.publish(stop)
                goal_handle.succeed()

                result = TurtleInfo.Result()
                # Rellenar resultado final con la información solicitada
                result.turtle1_x = float(self.turtle1_pose.x)
                result.turtle1_y = float(self.turtle1_pose.y)
                result.turtle1_theta = float(self.turtle1_pose.theta)

                result.explorer_x = float(self.explorer_pose.x)
                result.explorer_y = float(self.explorer_pose.y)
                result.explorer_theta = float(self.explorer_pose.theta)

                result.explorer_linear_velocity = float(self.last_cmd.linear.x)
                result.explorer_angular_velocity = float(self.last_cmd.angular.z)
                result.turtle1_linear_velocity = float(self.turtle1_pose.linear_velocity)
                result.turtle1_angular_velocity = float(self.turtle1_pose.angular_velocity)
                result.distance = float(distance)

                self.get_logger().info('Explorer ha alcanzado turtle1')
                return result

            await asyncio.sleep(1.0)


def main(args=None):
    rclpy.init(args=args)
    node = ExplorerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
