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
import time

class ExplorerActionNode(Node):
    """
    Nodo "Ejecutor" (Explorer Action Node).
    Funcionalidades principales:
    1. Igual que el ServiceNode, spawnea y controla a 'explorer' para perseguir a 'turtle1'.
    2. En vez de un servicio simple, ofrece un ACTION SERVER ('turtle_info').
    3. La Acción permite reportar Feedback continuo mientras explorer persigue a turtle1,
       y devuelve un Resultado final cuando la atrapa (distancia < 0.5).
    """

    def __init__(self):
        super().__init__('explorer_action_node')

        # ===== Parámetros E5 (posición inicial) =====
        self.declare_parameter('spawn_x', 2.0)
        self.declare_parameter('spawn_y', 2.0)

        x = self.get_parameter('spawn_x').value
        y = self.get_parameter('spawn_y').value

        if not (0.0 <= x <= 11.0 and 0.0 <= y <= 11.0):
            self.get_logger().error('Posición fuera de los límites de TurtleSim')
            x, y = 2.0, 2.0

        # ===== E1: Spawn explorer usando /spawn =====
        self.spawn_client = self.create_client(Spawn, '/spawn')
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando /spawn...')

        req = Spawn.Request()
        req.x = x
        req.y = y
        req.theta = 0.0
        req.name = 'explorer'
        self.spawn_client.call_async(req)
        self.get_logger().info(f'Tortuga explorer creada en ({x}, {y})')

        # ===== Estado de poses =====
        self.turtle1_pose = None
        self.explorer_pose = None

        # ===== Callback group (evitar bloqueo entre callbacks) =====
        self.callback_group = ReentrantCallbackGroup()

        # ===== E2: Subscripciones a las poses de las tortugas =====
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

        # ===== E2: Publisher para comandar a explorer =====
        self.cmd_pub = self.create_publisher(Twist, '/explorer/cmd_vel', 10)

        # ===== E2: Control proporcional =====
        self.k_linear = 1.5
        self.k_angular = 4.0
        self.last_cmd = Twist()

        # E2: Timer de control a 10 Hz
        self.create_timer(0.1, self.control_loop, callback_group=self.callback_group)

        # ===== E6: Action Server (turtle_info) =====
        # Crea el servidor de acción.
        # execute_callback: Función principal que se ejecuta cuando se acepta una meta.
        self.action_server = ActionServer(
            self,
            TurtleInfo,
            'turtle_info',
            self.execute_callback,
            callback_group=self.callback_group,
        )
        self.get_logger().info('Action Server turtle_info (E6) disponible')

    def turtle1_callback(self, msg):
        # Actualiza la pose de la tortuga principal
        self.turtle1_pose = msg

    def explorer_callback(self, msg):
        # Actualiza la pose de la tortuga perseguidora
        self.explorer_pose = msg

    def control_loop(self):
        # Lógica de Control (P-Controller) - Igual que en el nodo de servicio
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

    def execute_callback(self, goal_handle):
        """
        E6: Callback PRINCIPAL de la Acción.
        Se ejecuta en un hilo separado cuando llega una petición (Goal).
        Bucle: Monitorea la persecución, envía Feedback periódico y verifica éxito.
        """
        self.get_logger().info('Action turtle_info aceptada (E6)')

        feedback = TurtleInfo.Feedback()
        
        # Bucle principal de la acción: se ejecuta mientras ROS esté activo
        while rclpy.ok():
            if self.turtle1_pose is None or self.explorer_pose is None:
                time.sleep(0.5)
                continue

            dx = self.turtle1_pose.x - self.explorer_pose.x
            dy = self.turtle1_pose.y - self.explorer_pose.y
            distance = math.sqrt(dx**2 + dy**2)

            # Rellenar Feedback con toda la información actual
            # Esto se envía al cliente *mientras* la acción sigue ejecutándose
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
            
            # E6: Condición de FINALIZACIÓN (Éxito)
            # Si explorer está muy cerca (catch), termina la acción.
            if distance < 0.5:
                # Detener explorer
                stop = Twist()
                self.cmd_pub.publish(stop)
                
                # Marcar la acción como Exitosa
                goal_handle.succeed()

                # Construir y devolver el Resultado Final
                result = TurtleInfo.Result()
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

                self.get_logger().info('Explorer ha alcanzado turtle1. Action finalizada (E6).')
                return result

            time.sleep(1.0)


def main(args=None):
    rclpy.init(args=args)
    node = ExplorerActionNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()


if __name__ == '__main__':
    main()