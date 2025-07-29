import time

from yahboom_color_identify_interfaces.srv import ColorIdentify
from image_color_lab.msg import StringStamped
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class MinimalService(Node):

    def __init__(self):
        super().__init__('ColorIdentify_service')
        self.srv = self.create_service(ColorIdentify, 'yahboomColorIdentify', self.color_status_server_callback)       # CHANGE
        self.color = None
        self.obj_status = "none"
        self.color = self.create_subscription(StringStamped, '/obj_msg', self.color_status_callback, 10)

    def color_status_server_callback(self, request, response):
        map_name = request.if_identify
        # response.response = self.obj_status
        print("obj_status")
        if self.obj_status:
            response.response = "ok"
        else:
            response.response = "no"
        return response

    def color_status_callback(self, msg):
        msg_str = json.loads(msg.data)
        if msg_str["center_x"]==320:
            self.obj_status = 0
        else:
            # print("有红色块")
            self.obj_status = 1

def main(args=None):
    rclpy.init(args=args)

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
