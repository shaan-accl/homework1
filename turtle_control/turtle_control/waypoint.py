from enum import Enum, auto
from turtle_interfaces.srv import Waypoints
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from std_srvs.srv import Empty
from turtlesim.srv import Spawn, Kill, TeleportAbsolute
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from math import sqrt
import time, math

class State(Enum):
    MOVING = auto(),
    STOPPED = auto()
    
class WayPoint(Node):
    
    def __init__(self):
        super().__init__('waypoint')
        
        self.state = State.STOPPED
        self.turtle_pose = Pose()
        
        self.declare_parameter('frequency', 100, 
                               ParameterDescriptor(description="The frequency of the logger"))
        self.toggle = self.create_service(Empty, 'toggle', self.toggle_callback)
        self.load = self.create_service(Waypoints, 'load', self.load_callback)
        
        self.teleport_cli = self.create_client(TeleportAbsolute, "turtle1/teleport_absolute")
        while not self.teleport_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.teleport_turtle_request = TeleportAbsolute.Request()
        
        self.spawn_cli = self.create_client(Spawn, 'spawn')
        while not self.spawn_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.spawn_turtle_request = Spawn.Request()
        
        self.kill_cli = self.create_client(Kill, 'kill')
        while not self.kill_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.kill_turtle_request = Kill.Request()
        
        self.clear_cli = self.create_client(Empty, 'clear')
        while not self.clear_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.clear_request = Empty.Request()
        
        self.turtle_pose_sub = self.create_subscription(Pose, 'turtle1/pose', self.turtle_pose_sub_callback, 10)
        self.turtle_pose_sub          # prevent unused variable warning
        self.turtle_pose_pub = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        
        # timer_period = 100
        
        timer_period = self.get_parameter('frequency').get_parameter_value().integer_value
        timer_period = 1/timer_period
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # new_param = rclpy.parameter.Parameter(
        #     'frequency',
        #     rclpy.Parameter.Type.INTEGER,
        #     100
        # )
        # self.set_parameters([new_param])
        
        #PID angle controller gain values
        self.kp_angle = 0.6 
        self.ki_angle = 0.0
        self.kd_angle = 0.0
        self.previous_angle_error = 0
        self.integral_angle = 0

        self.kp_linear = 0.2
        self.max_linear_velocity = 1.0 
        self.max_angular_velocity = 5.0
        
        #Waypoints
        self.newx = []
        self.newy = []
        self.time_period = 0.01
        
    def load_callback(self, request, response):
        self.state = State.STOPPED
        
        self.get_logger().info('Clearing the background.')
        self.clear_future = self.clear_cli.call_async(self.clear_request)
        time.sleep(0.1)
        
        self.get_logger().info('Killing turtle1!')
        self.kill_turtle_request.name = "turtle1"
        self.kill_future = self.kill_cli.call_async(self.kill_turtle_request)
        time.sleep(0.1)
        
        self.get_logger().info('Spawning turtle on %f,%f' % (request.x[0], request.y[0]))
        self.spawn_turtle_request.x = request.x[0]
        self.spawn_turtle_request.y = request.y[0]
        self.spawn_turtle_request.theta = 0.0
        self.spawn_turtle_request.name = "turtle1"
        self.spawn_future = self.spawn_cli.call_async(self.spawn_turtle_request)
        # if self.spawn_future.done():
        
        prev_x = 0
        prev_y = 0
        dist = 0
        for i in range(len(request.x)):
            if(i!=0):
                dist += sqrt((request.x[i] - prev_x)**2 + (request.y[i] - prev_y)**2)
            
            prev_x = request.x[i]
            prev_y = request.y[i]
            self.newx.append(request.x[i])
            self.newy.append(request.y[i])
            
            # self.get_logger().info('Placing X on %f,%f' % (request.x[i], request.y[i]))
            # self.teleport_turtle_request.x = request.x[i]
            # self.teleport_turtle_request.y = request.y[i]
            # self.teleport_cli.call_async(self.teleport_turtle_request)
                
        response.dist = dist
        self.get_logger().info('Total distance traversed = %f' % response.dist)
        
        return response
        
    def toggle_callback(self, request, response):
        if self.state == State.MOVING:
            self.get_logger().info('Stopping')
            self.state = State.STOPPED
            
        else:
            self.get_logger().info('Starting to move')
            self.state = State.MOVING
            
        return response
    
    def timer_callback(self):
        if self.state == State.MOVING:
            self.get_logger().debug('Issuing Command!')
            if(len(self.newx)>0):
                newpos_x = self.newx
                newpos_y = self.newy
                distance = math.sqrt((self.turtle1_x - newpos_x[1]) ** 2 + (self.turtle1_y - newpos_y[1]) ** 2)
                self.get_logger().info('Heading towards: (%f, %f)' %(newpos_x[1], newpos_y[1]))
                distance_threshold = 0.1
                angle_tolerance = 0.1
                distance_tolerance = 0.15

                if distance > distance_threshold:
                    target_angle = math.atan2(newpos_y[1]-self.turtle1_y, newpos_x[1]-self.turtle1_x)
                    angle_error = (target_angle - self.turtle1_theta)

                    # self.get_logger().info('Angle error: %f' %(angle_error))

                    angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error)) #normalize angle_error

                    # Set the angular velocity
                    if(abs(angle_error)>angle_tolerance):
                        # PID for angular velocity
                        self.integral_angle += angle_error * self.time_period
                        derivative_angle = (angle_error - self.previous_angle_error) / self.time_period
                        angle_control = (self.kp_angle * angle_error + self.ki_angle * self.integral_angle + self.kd_angle * derivative_angle)
                        self.previous_angle_error = angle_error 
                        # angle_control = self.kp_angle * angle_error
                        # Scaled down angle_control as we approach turtle1
                        # angle_control *= (1 / (1+distance))

                        # Limit angular velocity
                        # angle_control = min(max(angle_control, self.max_angular_velocity), -self.max_angular_velocity)
                    else:
                        angle_control = 0.0

                    # Proportional control for linear velocity
                    if(distance > distance_tolerance):
                        linear_control = self.kp_linear * distance
                    else: 
                        linear_control = 0.0  # Stop if very close
                        newpos_x.append(newpos_x.pop(0)) # New target is the next node
                        newpos_y.append(newpos_y.pop(0)) # New target is the next node

                else: 
                    linear_control = 0.0
                    angle_control = 0.0
                    self.get_logger().error('Point is too close!')
                twist = Twist()
                twist.linear.x = linear_control
                twist.angular.z = angle_control
                self.turtle_pose_pub.publish(twist)
                self.get_logger().info('Publishing Command Velocity: Linear %f and angular %f' % (linear_control, angle_control))
            else:
                self.get_logger().error('No waypoints loaded. Load them with the "load" service.')
            
    def turtle_pose_sub_callback(self, msg):
        self.turtle1_x = msg.x
        self.turtle1_y = msg.y
        self.turtle1_theta = msg.theta
        
def main(args=None):
    rclpy.init(args=args)
    wpnode = WayPoint()
    rclpy.spin(wpnode)
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()