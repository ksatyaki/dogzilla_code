import rclpy   #python for ros2
from rclpy.node import Node   #import
from std_msgs.msg import String   #String message
from geometry_msgs.msg import Twist
import DOGZILLALib as dog
class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('cmd_vel_sub')
        self.subscription = self.create_subscription(Twist,'cmd_vel',self.cmdvel_callback,1)
        self.dogControl = dog.DOGZILLA()
        # ctrl velocity
        self.vel_x_=0.0
        self.vel_y_=0.0
        self.angular_z_=0.0
        # some enviroment value
        self.MINVALUE = 0.05
        self.MAXVALUE = 100.0
        self.rate=40
        
    def cmdvel_callback(self,msg):
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
        #end set dog motion
def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    # Уничтожить узел явно
    # (необязательно, иначе это будет сделано автоматически
    # когда сборщик мусора уничтожает объект node)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()	
