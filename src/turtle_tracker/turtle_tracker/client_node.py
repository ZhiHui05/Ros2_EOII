import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from turtle_tracker_interfaces.action import TurtleInfo
from turtlesim.msg import Pose
import math


class InfoClient(Node):

    def __init__(self):
        super().__init__('info_client')

        # Action Client
        self.client = ActionClient(self, TurtleInfo, 'turtle_info')

        # Variables de estado
        self.turtle1_pose = None
        self.explorer_pose = None
        self.goal_active = False

        # Suscripciones para monitorizar la distancia
        self.create_subscription(Pose, '/turtle1/pose', self.turtle1_callback, 10)
        self.create_subscription(Pose, '/explorer/pose', self.explorer_callback, 10)

        # Timer de control (loop de gestión)
        self.create_timer(0.5, self.control_timer_callback)
        
        self.get_logger().info('Cliente de Info (E6) iniciado. Esperando movimiento para activar...')

    def turtle1_callback(self, msg):
        self.turtle1_pose = msg

    def explorer_callback(self, msg):
        self.explorer_pose = msg

    def control_timer_callback(self):
        # 1. Verificar si tenemos datos de posición
        if self.turtle1_pose is None or self.explorer_pose is None:
            return

        # 2. Calcular distancia
        dx = self.turtle1_pose.x - self.explorer_pose.x
        dy = self.turtle1_pose.y - self.explorer_pose.y
        distance = math.sqrt(dx**2 + dy**2)

        # 3. Lógica de activación/reactivación
        # Si NO hay una acción activa Y la distancia es grande (> 0.55),
        # significa que debemos empezar a reportar.
        if not self.goal_active and distance > 0.55:
            self.get_logger().info(f'Movimiento detectado (Distancia: {distance:.2f}). Reactivando informacion (E3)...')
            self.send_goal()

    def send_goal(self):
        # Esperar servidor si no está listo (chequeo rápido)
        if not self.client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warning('Action Server no disponible...')
            return

        goal_msg = TurtleInfo.Goal()
        self.goal_active = True 
        
        self._future = self.client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rechazada por el servidor.')
            self.goal_active = False
            return

        self.get_logger().info('Goal aceptada.')
        self._result_future = goal_handle.get_result_async()
        self._result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        f = feedback_msg.feedback
        self.get_logger().info(
            f'\n--- Turtle Info (E3/Feedback) ---\n'
            f'Explorer: ({f.explorer_x:.2f}, {f.explorer_y:.2f})\n'
            f'Turtle1:  ({f.turtle1_x:.2f}, {f.turtle1_y:.2f})\n'
            f'Distancia: {f.distance:.2f}\n'
            f'-----------------------------'
        )

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Accion finalizada. Distancia final: {result.distance:.2f}')
        self.goal_active = False


def main(args=None):
    rclpy.init(args=args)
    node = InfoClient()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
