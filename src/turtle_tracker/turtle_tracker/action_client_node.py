import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from turtle_tracker_interfaces.action import TurtleInfo
from turtlesim.msg import Pose
import math


class ActionClientNode(Node):
    """
    Nodo "Gestor" (Action Client Node).
    Funcionalidades principales:
    1. Actúa como CLIENTE de la acción 'turtle_info'.
    2. Monitorea pasivamente la distancia entre tortugas.
    3. Cuando detecta movimiento (se separan > 0.55m), dispara la Acción (send_goal).
    4. Recibe Feedback continuo mientras dura la persecución y el Resultado final cuando termina.
    """

    def __init__(self):
        super().__init__('action_client_node')

        # E6: Action Client
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
        
        self.get_logger().info('Cliente de Action (E6) iniciado.  Esperando movimiento para activar.. .')

    def turtle1_callback(self, msg):
        self.turtle1_pose = msg

    def explorer_callback(self, msg):
        self.explorer_pose = msg

    def control_timer_callback(self):
        """
        Loop de monitoreo (se ejecuta cada 0.5s).
        Decide cuándo activar la acción basándose en la distancia.
        """
        # Verificar si tenemos datos de posición
        if self.turtle1_pose is None or self.explorer_pose is None:
            return

        # Calcular distancia
        dx = self.turtle1_pose.x - self.explorer_pose.x
        dy = self.turtle1_pose.y - self.explorer_pose.y
        distance = math.sqrt(dx**2 + dy**2)

        # Lógica de activación automática:
        # Si NO estamos persiguiendo ya (goal_active=False) Y se han separado mucho (>0.55)
        if not self.goal_active and distance > 0.55:
            self.get_logger().info(f'Movimiento detectado (Distancia: {distance:.2f}). Activando Action (E6)...')
            self.send_goal()

    def send_goal(self):
        """Envía la petición (Goal) al Action Server para iniciar la tarea."""
        # Esperar servidor si no está listo
        if not self.client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warning('Action Server no disponible.. .')
            return

        goal_msg = TurtleInfo.Goal()
        # Marcamos que ya hay una meta activa para no enviarla muchas veces
        self.goal_active = True 
        
        self._future = self.client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback # Definimos callback para feedback
        )
        self._future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Callback ejecutado cuando el servidor acepta (o rechaza) la meta inicial."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rechazada por el servidor.')
            self.goal_active = False
            return

        self.get_logger().info('Goal aceptada por Action Server (E6).')
        
        # Configurar callback para obtener el RESULTADO final (cuando termine)
        self._result_future = goal_handle.get_result_async()
        self._result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        """
        E6: Recibir y mostrar feedback periódico.
        Se ejecuta cada vez que el servidor publica feedback (mientras persigue).
        """
        f = feedback_msg.feedback
        self.get_logger().info(
            f'\n╔════════════════════════════════════════════╗\n'
            f'║   TURTLE INFO (Action E6 - Feedback)       ║\n'
            f'╠════════════════════════════════════════════╣\n'
            f'║ Turtle1:   ({f.turtle1_x:6.2f}, {f.turtle1_y:6.2f})           ║\n'
            f'║ Explorer:  ({f.explorer_x:6.2f}, {f.explorer_y:6.2f})           ║\n'
            f'║ Distancia: {f.distance:6.2f}                       ║\n'
            f'╚════════════════════════════════════════════╝'
        )

    def result_callback(self, future):
        """E6: Recibir resultado final cuando explorer alcanza turtle1"""
        result = future.result().result
        self.get_logger().info(
            f'\n╔════════════════════════════════════════════╗\n'
            f'║   ACTION COMPLETADA (E6)                   ║\n'
            f'╠══���═════════════════════════════════════════╣\n'
            f'║ Explorer alcanzó a Turtle1                 ║\n'
            f'║ Distancia final: {result.distance:6.2f}               ║\n'
            f'╚════════════════════════════════════════════╝'
        )
        self.goal_active = False


def main(args=None):
    rclpy.init(args=args)
    node = ActionClientNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()