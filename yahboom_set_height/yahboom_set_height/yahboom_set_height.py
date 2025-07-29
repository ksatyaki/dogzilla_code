import rclpy
from rclpy.node import Node
import DOGZILLALib as dog
from geometry_msgs.msg import Twist
import time


class XgoSetHeight(rclpy.node.Node):
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
        self.declare_parameter('xGoHeight', 115)
        self.declare_parameter('attitude_p', 0)
        self.declare_parameter('move_x', 25)
        self.declare_parameter('move_y', 18)
        
        self.xGoHeight = self.get_parameter('xGoHeight').get_parameter_value().integer_value
        self.attitude_p = self.get_parameter('attitude_p').get_parameter_value().integer_value
        self.move_y = self.get_parameter('move_y').get_parameter_value().integer_value
        self.move_x = self.get_parameter('move_x').get_parameter_value().integer_value
        
        self.get_logger().info('xGoHeight_value: %s!' % self.xGoHeight)
        self.get_logger().info('attitude_p_value: %s!' % self.attitude_p)
        self.get_logger().info('move_x_value: %s!' % self.move_x)
        self.get_logger().info('move_y_value: %s!' % self.move_y)
        
        self.xGoHeght_value = 115 if int(self.xGoHeight) > 115 else 75 if int(self.xGoHeight) < 75 else int(self.xGoHeight)
        print(self.xGoHeght_value)
        print(self.attitude_p)
        print(self.move_x)
        print(self.move_y)
        
        self.dogControl.translation("z", self.xGoHeght_value)
        time.sleep(0.2)
        self.dogControl.attitude("p", self.attitude_p)
        time.sleep(0.2)
        self.dogControl.move_x(self.move_x)
        time.sleep(0.2)
        self.dogControl.move_y(self.move_y)
        
        
        
        
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
    minimal_subscriber = XgoSetHeight()
    rclpy.spin(minimal_subscriber)
    
if __name__ == '__main__':
    main()    
    
    
    
        
      

