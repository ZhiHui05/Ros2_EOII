import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from turtle_tracker_interfaces.action import TurtleInfo


class InfoClient(Node):

    def __init__(self):
        super().__init__('info_client')

        # ActionClient para solicitar la info periódica
        self.client = ActionClient(self, TurtleInfo, 'turtle_info')

        self.get_logger().info('Esperando Action Server...')
        # Esperar al servidor con timeout y manejar ausencia
        while rclpy.ok():
            available = self.client.wait_for_server(timeout_sec=1.0)
            if available:
                break
            self.get_logger().warning('Action Server no disponible, reintentando...')

        if not rclpy.ok():
            self.get_logger().error('Interrumpido antes de conectar con el Action Server')
            return

        try:
            self.send_goal()
        except Exception as e:
            self.get_logger().error(f'Error al enviar el goal: {e}')

    def send_goal(self):
        # Enviar goal vacío (la información llega por feedback)
        goal_msg = TurtleInfo.Goal()
        self._future = self.client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        # Respuesta del servidor: goal aceptado o rechazado
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rechazada')
            return

        self.get_logger().info('Goal aceptada')
        self._result_future = goal_handle.get_result_async()
        self._result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        # Feedback periódico con la info de ambas tortugas
        f = feedback_msg.feedback
        self.get_logger().info(
            f'\n--- Turtle Info (Feedback) ---\n'
            f'Explorer: ({f.explorer_x:.2f}, {f.explorer_y:.2f})\n'
            f'Turtle1:  ({f.turtle1_x:.2f}, {f.turtle1_y:.2f})\n'
            f'Vels explorer (lin/ang): ({f.explorer_linear_velocity:.2f}, {f.explorer_angular_velocity:.2f})\n'
            f'Vels turtle1  (lin/ang): ({f.turtle1_linear_velocity:.2f}, {f.turtle1_angular_velocity:.2f})\n'
            f'Distance: {f.distance:.2f}\n'
            f'-----------------------------'
        )

    def result_callback(self, future):
        # Resultado final cuando explorer alcanza a turtle1
        self.get_logger().info('Seguimiento finalizado con éxito')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = InfoClient()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
