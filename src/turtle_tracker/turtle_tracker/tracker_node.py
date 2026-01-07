import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from turtlesim.srv import Spawn
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
#from turtle_tracker_interfaces.srv import TurtleInfo as TurtleInfoSrv
from turtle_tracker_interfaces.action import TurtleInfo as TurtleInfoAction

class TurtleTracker(Node):

    def __init__(self):
        super().__init__('turtle_tracker')
        
        # Parameters
        self.declare_parameter('explorer_x', 2.0)
        self.declare_parameter('explorer_y', 2.0)
        
        self.explorer_x = self.get_parameter('explorer_x').value
        self.explorer_y = self.get_parameter('explorer_y').value

        # State
        self.turtle1_pose = None
        self.explorer_pose = None
        self.spawned = False

        # Spawner
        self.spawn_client = self.create_client(Spawn, '/spawn')
        # Use a timer to check for service availability instead of blocking
        self.spawn_timer = self.create_timer(1.0, self.try_spawn_explorer)

        # Subscribers
        self.create_subscription(Pose, '/turtle1/pose', self.turtle1_pose_callback, 10)
        self.create_subscription(Pose, '/explorer/pose', self.explorer_pose_callback, 10)

        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/explorer/cmd_vel', 10)

        # Control Loop
        self.create_timer(0.1, self.control_loop)

        # Service Server (E3)
        self.srv = self.create_service(TurtleInfoSrv, 'turtle_info', self.turtle_info_callback)

        # Action Server (E6)
        self._action_server = ActionServer(
            self,
            TurtleInfoAction,
            'turtle_info_action',
            self.execute_callback
        )

    def try_spawn_explorer(self):
        if self.spawned:
            self.spawn_timer.cancel()
            return

        if not self.spawn_client.service_is_ready():
            self.get_logger().info('Spawn service not available, waiting...')
            return

        self.spawn_explorer()
        self.spawn_timer.cancel()

    def spawn_explorer(self):
        req = Spawn.Request()
        req.x = self.explorer_x
        req.y = self.explorer_y
        req.theta = 0.0
        req.name = 'explorer'
        future = self.spawn_client.call_async(req)
        future.add_done_callback(self.spawn_callback)

    def spawn_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Spawned turtle: {response.name}')
            self.spawned = True
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

    def turtle1_pose_callback(self, msg):
        self.turtle1_pose = msg

    def explorer_pose_callback(self, msg):
        self.explorer_pose = msg

    def control_loop(self):
        if not self.spawned or self.turtle1_pose is None or self.explorer_pose is None:
            return

        # Proportional Control
        dx = self.turtle1_pose.x - self.explorer_pose.x
        dy = self.turtle1_pose.y - self.explorer_pose.y
        distance = math.sqrt(dx**2 + dy**2)
        angle_to_target = math.atan2(dy, dx)
        
        msg = Twist()
        
        # Linear velocity proportional to distance
        # Kp_linear = 1.0
        msg.linear.x = 1.0 * distance

        # Angular velocity proportional to angle error
        angle_error = angle_to_target - self.explorer_pose.theta
        # Normalize angle
        while angle_error > math.pi: angle_error -= 2*math.pi
        while angle_error < -math.pi: angle_error += 2*math.pi
        
        # Kp_angular = 4.0
        msg.angular.z = 4.0 * angle_error

        self.cmd_vel_pub.publish(msg)

    def get_info_data(self):
        if self.turtle1_pose is None or self.explorer_pose is None:
            return None
        
        dx = self.turtle1_pose.x - self.explorer_pose.x
        dy = self.turtle1_pose.y - self.explorer_pose.y
        dist = math.sqrt(dx**2 + dy**2)

        return {
            'explorer_x': self.explorer_pose.x,
            'explorer_y': self.explorer_pose.y,
            'explorer_theta': self.explorer_pose.theta,
            'explorer_linear_velocity': self.explorer_pose.linear_velocity,
            'explorer_angular_velocity': self.explorer_pose.angular_velocity,
            'turtle1_x': self.turtle1_pose.x,
            'turtle1_y': self.turtle1_pose.y,
            'turtle1_theta': self.turtle1_pose.theta,
            'turtle1_linear_velocity': self.turtle1_pose.linear_velocity,
            'turtle1_angular_velocity': self.turtle1_pose.angular_velocity,
            'distance': dist
        }

    def turtle_info_callback(self, request, response):
        data = self.get_info_data()
        if data:
            for key, value in data.items():
                setattr(response, key, value)
        return response

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        
        feedback_msg = TurtleInfoAction.Feedback()
        
        while rclpy.ok():
            data = self.get_info_data()
            if data:
                for key, value in data.items():
                    setattr(feedback_msg, key, value)
                
                goal_handle.publish_feedback(feedback_msg)
                
                # Check if caught (distance < threshold, e.g., 0.5)
                if data['distance'] < 0.5:
                    goal_handle.succeed()
                    result = TurtleInfoAction.Result()
                    result.success = True
                    return result
            
            # Rate limit
            import time
            time.sleep(1.0)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
