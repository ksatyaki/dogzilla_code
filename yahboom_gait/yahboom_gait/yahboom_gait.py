import rclpy
from rclpy.node import Node
import DOGZILLALib as dog
from geometry_msgs.msg import Twist
import time

class XgoGait(rclpy.node.Node):
    def __init__(self):
        super().__init__('cmd_vel_gait_sub')
        self.subscription = self.create_subscription(Twist,'cmd_vel',self.cmdvel_callback,1)
        self.dogControl = dog.DOGZILLA()
        self.vel_x_=0.0
        self.vel_y_=0.0
        self.angular_z_=0.0
        self.MINVALUE = 0.05
        self.MAXVALUE = 100.0
        self.rate=10
        self.gait = "trot"
        self.declare_parameter('gait', self.gait)
        self.gait_type = self.get_parameter('gait').get_parameter_value().string_value
        self.get_logger().info('gait_type: %s!' % self.gait_type)
        self.mark = 0
        self.declare_parameter('marking', self.mark)
        self.mark_type = self.get_parameter('marking').get_parameter_value().integer_value
        self.get_logger().info('mark_type: %s!' % self.mark_type)
        self.dogControl.gait_type(self.gait_type)
        self.dogControl.mark_time(self.mark_type)
        time.sleep(3)
        self.dogControl.mark_time(0)
        
        
        
    def cmdvel_callback(self, msg):
        
        self.vel_x_=msg.linear.x
        self.vel_y_=msg.linear.y
        self.angular_z_=msg.angular.z
        if abs(self.vel_x_)<self.MINVALUE and abs(self.vel_y_)<self.MINVALUE and abs(self.angular_z_)<self.MINVALUE:
            self.dogControl.stop()
            return
        #print for debug
        #self.get_logger().info('I heard: "%f"' % msg.linear.x)
        #限幅
        if self.vel_x_ > self.MAXVALUE: self.vel_x_ = self.MAXVALUE
        if self.vel_y_ > self.MAXVALUE: self.vel_y_ = self.MAXVALUE
        if self.vel_x_ < -self.MAXVALUE: self.vel_x_ = -self.MAXVALUE
        if self.vel_y_ < -self.MAXVALUE: self.vel_y_ = -self.MAXVALUE
        if self.angular_z_ > self.MAXVALUE: self.angular_z_ = self.MAXVALUE
        if self.angular_z_ < -self.MAXVALUE: self.angular_z_ = -self.MAXVALUE
        #end 限幅
        #set dog motion
        #установить движение собаки 
        self.dogControl.move('x',self.vel_x_*self.rate)
        self.dogControl.move('y',self.vel_y_*self.rate)
        self.dogControl.turn(self.angular_z_*self.rate)
def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = XgoGait()
    rclpy.spin(minimal_subscriber)
    
if __name__ == '__main__':
    main()    
    
    
    
        
      
