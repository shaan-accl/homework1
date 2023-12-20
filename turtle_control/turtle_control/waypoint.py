import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor

class WayPoint(Node):
    
    def __init__(self):
        super().__init__('waypoint')
        
        self.declare_parameter('frequency', 100, 
                               ParameterDescriptor(description="The frequency of the logger"))
        
        timer_period = 100
        
        # timer_period = self.get_parameter('frequency').get_parameter_value().integer_value
        timer_period = 1/timer_period
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # new_param = rclpy.parameter.Parameter(
        #     'frequency',
        #     rclpy.Parameter.Type.INTEGER,
        #     100
        # )
        # self.set_parameters([new_param])
        
    def timer_callback(self):
        
        self.get_logger.debug('Issuing Command!')
        
    def main(args=None):
        rclpy.init(args=args)
        wpnode = WayPoint()
        rclpy.spin(wpnode)
        rclpy.shutdown()

if __name__ == '__main__':
    main()