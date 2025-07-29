import time
from image_color_lab.msg import StringStamped
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import redis


class MinimalService(Node):

    def __init__(self):
        super().__init__('ColorIdentify_sub')
        self.obj_status = "none"
        self.color = self.create_subscription(StringStamped, '/obj_msg', self.color_status_callback, 10)
        self.r = redis.Redis(host='localhost', port=9090, db=0)

    def color_status_callback(self, msg):
        msg_str = json.loads(msg.data)
        if msg_str["center_x"]==320:
            self.obj_status = 0
            self.r.set('color_status', 'no')
        else:
            # print("有红色块")
            self.obj_status = 1
            self.r.set('color_status', 'ok')

def main(args=None):
    rclpy.init(args=args)

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
