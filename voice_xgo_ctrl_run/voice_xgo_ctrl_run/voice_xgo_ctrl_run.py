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
from geometry_msgs.msg import Twist
from image_color_lab.msg import StringStamped
from rclpy.duration import Duration
import sys
from Speech_Lib import Speech


class VoiceCtrl(Node):
    def __init__(self):
        super().__init__("voice_ctrl_publisher")
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.send_twist)
        self.x = 0.0
        self.z = 0.0
        self.spe = Speech()

    def send_twist(self):
        command_result = self.spe.speech_read()
        if command_result==4:
        #前进
            self.x = 0.5
        if command_result==2:
        #停止
            self.x = 0.0
            self.z = 0.0
        if command_result==5:
        #后退
            self.x = -0.5
        if command_result==6:
        #左转
            self.x = 0.0
            self.z = 0.2
        if command_result==7:
            self.x = 0.0
            self.z = -0.2
        msg = Twist()
        msg.linear.x = self.x
        msg.angular.z = self.z
        self.publisher.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    node = VoiceCtrl()
    rclpy.spin(node)
    rclpy.shutdown()
