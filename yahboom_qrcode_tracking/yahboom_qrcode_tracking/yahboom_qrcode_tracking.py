import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
import base64
import cv2
from cv_bridge import CvBridge
import math
import io
import numpy as np
from rclpy.qos import qos_profile_sensor_data
import json
from std_msgs.msg import String
import DOGZILLALib
import time
from yahboom_attitude_record_interfaces.srv._attuitude_record import AttuitudeRecord
from geometry_msgs.msg import Twist
from message_filters import ApproximateTimeSynchronizer, Subscriber
#from message_filters import TimeSynchronizer, Subscriber
from image_color_lab.msg import StringStamped
from rclpy.duration import Duration
import sys


class ObjSubscriber(Node):
    def __init__(self):
        super().__init__("qrcode_subscriber")
        self.dog = DOGZILLALib.DOGZILLA()
        print("dddddddddddddddddddd1")
        self.image_sub = Subscriber(self, CompressedImage, '/image_raw/compressed')
        self.info_sub = Subscriber(self, StringStamped, '/obj_msg')
        print("dddddddddddddddddddd2")
        print(qos_profile_sensor_data)
        self.tss = ApproximateTimeSynchronizer([self.image_sub, self.info_sub], queue_size=10, slop=2.0)
       # self.ts = TimeSynchronizer([self.image_sub, self.info_sub], queue_size=5)
        self.tss.registerCallback(self.callback)
        # self.subscription = self.create_subscription(String, "/obj_msg", self.process_image, 5)
        self.cli = self.create_client(AttuitudeRecord, 'yahboomSetAttiude')  # CHANGE
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.send_twist)
        self.req = AttuitudeRecord.Request()
        self.obj_pitch = 0
        self.obj_yaw = 0
        self.last_pitch = 0
        self.last_yaw = 0
        self.x = 0.0
        print("dddddddddddddddddddd")

    def send_service_request(self):
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        request = AttuitudeRecord.Request()
        request.pitch = self.obj_pitch
        request.yaw = self.obj_yaw
        future = self.cli.call_async(request)
        future.add_done_callback(self.handle_service_response)

    def handle_service_response(self, future):
        try:
            response = future.result()
            self.get_logger().info('Received response: %s' % response.res)
        except Exception as e:
            self.get_logger().error('Service call failed: %s' % e)


    def callback(self, image, info):
        print("image::",image.header)

        print("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%")
        print("info::",info.data)
        print("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%")
        labset = json.loads(info.data)
        try:
            if 10000 < int(labset["area"]) < 15000:
                self.x = 0.0
            elif int(labset["area"]) < 10000:
                self.x = 0.5  
            elif int(labset["area"]) > 15000:
                self.x = -0.5  
            else:
                self.x = 0.0
            
        except:
            self.x = 0.0
            pass



    def send_twist(self):
        msg = Twist()
        msg.linear.x = self.x
        msg.angular.z = 0.0
        self.publisher.publish(msg)



    def process_image(self, msg):
        print("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%")
        print(msg)
        print("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%")
        labset = json.loads(msg.data)
        now_pitch = self.dog.read_pitch()
        now_yaw = self.dog.read_yaw()
        # time.sleep(0.5)
        try:

            self.obj_pitch = labset["vertical_movement"]
            self.obj_yaw = labset["horizontal_movement"]
            #self.send_service_request()
            # perform OpenCV operations on cv_image here
            if -22 < self.last_pitch < 22:

                last_pitch_set = self.last_pitch + labset["vertical_movement"]
                self.dog.attitude("p", last_pitch_set)
                now_pitch_is_suc = self.dog.read_pitch()
                if now_pitch_is_suc != now_pitch:
                    self.last_pitch = last_pitch_set
            else:
                self.last_pitch = 0
            if -16 < self.last_yaw < 16:
                last_yaw_set = self.last_yaw + labset["horizontal_movement"]
                self.dog.attitude("y", last_yaw_set)
                now_yaw_is_suc = self.dog.read_yaw()
                if now_pitch_is_suc != now_pitch:
                    self.last_yaw = last_yaw_set
            else:
                self.last_yaw = 0

                #        if labset["center_y"] > labset["img_h"]/2:
            #            print("da::",int(now_pitch))
            #            if int(now_pitch) < int(22):
            #                print("change::", int(now_pitch) + 2)
            #                set_value = int(now_pitch) + 2
            #                self.dog.attitude("p", int(now_pitch) + 1)
            #            else:
            #                self.dog.attitude("p", int(15))
            if (labset["img_h"] / 2 - 15) < labset["center_y"] < (labset["img_h"] / 2 + 15):
                print("已经到达预定点")
                pass
        except:
            self.obj_pitch = 0
            self.obj_yaw = 0
            pass
    #        else:
    #            print("xiao::",(now_pitch))
    #            if int(now_pitch) > -22 and int(now_pitch) < 22:
    #                self.dog.attitude("p", int(now_pitch) - 1)
    #            else:
    #                self.dog.attitude("p", int(0))


def main(args=None):
    rclpy.init(args=args)
    node = ObjSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()
