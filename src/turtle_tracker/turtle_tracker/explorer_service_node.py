import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from turtlesim.srv import Spawn
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from turtle_tracker_interfaces.srv import TurtleInfo

import math

class ExplorerServiceNode(Node):
    """
    Nodo "Cerebro" (Explorer Service Node).
    Funcionalidades principales:
    1. Spawnea (crea) la tortuga 'explorer' en una posición inicial configurable.
    2. Hace que 'explorer' persiga a 'turtle1' usando un controlador Proporcional (P).
    3. Ofrece un SERVICIO ('turtle_info') que devuelve el estado actual de la persecución (posiciones y distancias).
    """

    def __init__(self):
        super().__init__('explorer_service_node')

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
            self.get_logger().info('Esperando /spawn.. .')

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
        # Llama a la funcion de control 10 veces por segundo para calcular movimiento
        self.create_timer(0.1, self.control_loop, callback_group=self.callback_group)

        # ===== E3: Service Server (turtle_info) =====
        # Crea el SERVIDOR del servicio. Cuando alguien llame a 'turtle_info', se ejecuta self.service_callback
        self.service_server = self.create_service(
            TurtleInfo,
            'turtle_info',
            self.service_callback,
            callback_group=self.callback_group,
        )
        self.get_logger().info('Servicio turtle_info (E3) disponible')

    def turtle1_callback(self, msg):
        # Callback que se ejecuta cada vez que llega un mensaje de posición de turtle1
        # Actualiza la pose de la tortuga principal
        self.turtle1_pose = msg

    def explorer_callback(self, msg):
        # Callback que se ejecuta cada vez que llega un mensaje de posición de explorer
        # Actualiza la pose de la tortuga perseguidora
        self.explorer_pose = msg

    def control_loop(self):
        # Lógica de Control (P-Controller)
        # Se ejecuta periódicamente (10Hz) para mover a explorer hacia turtle1
        
        # Si aun no tenemos las posiciones de ambas tortugas, no hacemos nada
        # E2: Control proporcional para seguir a turtle1
        if self.turtle1_pose is None or self.explorer_pose is None:
            return

        # 1. Calcular error de posición (distancia)
        dx = self.turtle1_pose.x - self.explorer_pose.x
        dy = self.turtle1_pose.y - self.explorer_pose.y
        distance = math.sqrt(dx**2 + dy**2)

        # 2. Calcular error de orientación (ángulo hacia el objetivo)
        target_angle = math.atan2(dy, dx)
        angle_error = target_angle - self.explorer_pose.theta
        # Normalizar el ángulo entre -pi y pi
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

        # 3. Aplicar ganancias (K) para obtener comandos de velocidad
        cmd = Twist()
        cmd.linear.x = self.k_linear * distance        # Velocidad lineal proporcional a la distancia
        cmd.angular.z = self.k_angular * angle_error   # Velocidad angular proporcional al error de ángulo

        self.last_cmd = cmd
        self.cmd_pub.publish(cmd)

    def service_callback(self, request, response):
        """
        E3: Callback del Service Server.
        Se ejecuta SOLO cuando un Cliente (Node) llama al servicio.
        Recopila toda la información actual y la devuelve en la respuesta.
        """
        if self.turtle1_pose is None or self.explorer_pose is None:
            self.get_logger().warning('Poses no disponibles aún')
            return response
        
        dx = self.turtle1_pose.x - self.explorer_pose.x
        dy = self.turtle1_pose.y - self.explorer_pose.y
        distance = math.sqrt(dx**2 + dy**2)
        
        # Rellenar response con toda la información
        response.turtle1_x = self.turtle1_pose.x
        response.turtle1_y = self.turtle1_pose.y
        response.turtle1_theta = self.turtle1_pose.theta
        
        response.explorer_x = self.explorer_pose.x
        response.explorer_y = self.explorer_pose.y
        response.explorer_theta = self.explorer_pose.theta
        
        response.explorer_linear_velocity = self.last_cmd.linear.x
        response.explorer_angular_velocity = self.last_cmd.angular.z
        response.turtle1_linear_velocity = self.turtle1_pose.linear_velocity
        response.turtle1_angular_velocity = self.turtle1_pose.angular_velocity
        
        response.distance = distance
        
        self.get_logger().info(f'Servicio invocado - Distancia: {distance:.2f}')
        
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ExplorerServiceNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()


if __name__ == '__main__': 
    main()