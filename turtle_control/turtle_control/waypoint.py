from enum import Enum, auto
from turtle_interfaces.srv import Waypoints
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from std_srvs.srv import Empty
from turtlesim.srv import Spawn, Kill, TeleportAbsolute
from math import sqrt

class State(Enum):
    MOVING = auto(),
    STOPPED = auto()
    
class WayPoint(Node):
    
    def __init__(self):
        super().__init__('waypoint')
        
        self.state = State.STOPPED
        self.declare_parameter('frequency', 100, 
                               ParameterDescriptor(description="The frequency of the logger"))
        self.toggle = self.create_service(Empty, 'toggle', self.toggle_callback)
        self.load = self.create_service(Waypoints, 'load', self.load_callback)
        self.kill = self.create_client(Kill, "kill")
        self.spawn = self.create_client(Spawn, "spawn")
        self.teleport_cli = self.create_client(TeleportAbsolute, "turtle1/teleport_absolute")
        
        while not self.teleport_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        self.teleport_turtle_request = TeleportAbsolute.Request()
        
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
        
    def load_callback(self, request, response):
        self.state = State.STOPPED
        prev_x = 0
        prev_y = 0
        dist = 0
        for i in range(len(request.x)):    
            if(i!=0):
                dist += sqrt((request.x[i] - prev_x)**2 + (request.y[i] - prev_y)**2)
            
            prev_x = request.x[i]
            prev_y = request.y[i]
            
            self.get_logger().info('Placing X on %f,%f' % (request.x[i], request.y[i]))
            self.teleport_turtle_request.x = request.x[i]
            self.teleport_turtle_request.y = request.y[i]
            self.teleport_cli.call_async(self.teleport_turtle_request)
                
        response.dist = dist
        self.get_logger().info('Total distance traversed = %f' % response.dist)
        
        return response
        
    def toggle_callback(self, request, response):
        if self.state == State.MOVING:
            self.get_logger().info('Stopping')
            self.state = State.STOPPED
            
        else:
            self.state = State.MOVING
            
        return response
    
    def timer_callback(self):
        if self.state == State.MOVING:
            self.get_logger().debug('Issuing Command!')
        
def main(args=None):
    rclpy.init(args=args)
    wpnode = WayPoint()
    rclpy.spin(wpnode)
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()