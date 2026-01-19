import rclpy
from rclpy.node import Node
from turtle_tracker_interfaces.srv import TurtleInfo

class ServiceClientNode(Node):
    """
    Nodo "Observador" (Service Client Node).
    Funcionalidades principales:
    1. Actúa como CLIENTE del servicio 'turtle_info'.
    2. Usa un Timer para consultar (Polling) el estado cada 1 segundo.
    3. Muestra la información recibida de forma estructurada en la consola.
    """
    
    def __init__(self):
        super().__init__('service_client_node')
        
        # E4: Cliente del servicio turtle_info
        self.client = self.create_client(TurtleInfo, 'turtle_info')
        
        # Esperar a que el servicio esté disponible
        self.get_logger().info('Esperando servicio turtle_info...')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Servicio no disponible, esperando.. .')
        
        # E4: Timer para llamar cada 1 segundo
        # Configura un bucle para ejecutar self.call_service cada 1.0 segundos
        self.timer = self.create_timer(1.0, self.call_service)
        
        self.get_logger().info('Cliente de servicio (E4) iniciado - consultando cada 1 segundo')
    
    def call_service(self):
        """
        E4: Invocar el servicio turtle_info cada segundo.
        Envía una petición vacía (o con datos si fuera necesario) al servidor.
        """
        request = TurtleInfo.Request()
        
        # Llamada asíncrona: no bloquea el nodo mientras espera respuesta
        future = self.client.call_async(request)
        # Define qué función ejecutar cuando llegue la respuesta
        future.add_done_callback(self.response_callback)
    
    def response_callback(self, future):
        """
        E4: Callback que maneja la respuesta del servicio una vez recibida.
        Extrae los datos y los imprime.
        """
        try:
            response = future.result()
            
            # Mostrar información de forma clara y estructurada
            self.get_logger().info(
                f'\n╔════════════════════════════════════════════╗\n'
                f'║   TURTLE INFO (Servicio E3)                ║\n'
                f'╠════════════════════════════════════════════╣\n'
                f'║ Turtle1:                                   ║\n'
                f'║   Posición:     ({response.turtle1_x:6.2f}, {response.turtle1_y:6.2f})   ║\n'
                f'║   Orientación: {response.turtle1_theta:6.2f} rad              ║\n'
                f'║   Vel.  lineal: {response.turtle1_linear_velocity:6.2f}                 ║\n'
                f'║   Vel. angular:{response.turtle1_angular_velocity:6.2f}                 ║\n'
                f'╠════════════════════════════════════════════╣\n'
                f'║ Explorer:                                   ║\n'
                f'║   Posición:    ({response.explorer_x:6.2f}, {response.explorer_y:6.2f})   ║\n'
                f'║   Orientación:  {response.explorer_theta:6.2f} rad              ║\n'
                f'║   Vel. lineal: {response.explorer_linear_velocity:6.2f}                 ║\n'
                f'║   Vel. angular:{response.explorer_angular_velocity:6.2f}                 ║\n'
                f'╠════════════════════════════════════════════╣\n'
                f'║ Distancia: {response.distance:6.2f}                       ║\n'
                f'╚════════════════════════════════════════════╝'
            )
            
        except Exception as e:
            # E4: Manejo de errores si el servicio no está disponible
            self.get_logger().error(f'Error al llamar al servicio: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ServiceClientNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__': 
    main()