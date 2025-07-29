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


class VoiceCtrlColorLab(Node):
    def __init__(self):
        super().__init__("voice_ctrl_color_publisher")
        self.publisher = self.create_publisher(String, 'lab_set', 10)
        self.timer = self.create_timer(0.1, self.send_color_lab)
        self.spe = Speech()
        self.strmsg = String()

    def send_color_lab(self):
        command_result = self.spe.speech_read()
        if command_result==90:
        #黄色
           lab_dict = {"l":96, "a": 55, "b":188, "l_max": 252 , "a_max": 141, "b_max": 255}
           self.strmsg.data = json.dumps(lab_dict)
           self.publisher.publish(self.strmsg)
        if command_result==87:
        #红色
            lab_dict = {"l":0, "a": 155, "b":21, "l_max": 255 , "a_max": 255, "b_max": 255}
            self.strmsg.data = json.dumps(lab_dict)
            self.publisher.publish(self.strmsg)
        if command_result==88:
        #绿色
            lab_dict = {"l":26, "a": 7, "b":170, "l_max": 143 , "a_max": 110, "b_max": 255}
            self.strmsg.data = json.dumps(lab_dict)
            self.publisher.publish(self.strmsg)
        if command_result==89:
        #蓝色
            lab_dict = {"l":0, "a": 0, "b":0, "l_max": 255 , "a_max": 255, "b_max": 102}
            self.strmsg.data = json.dumps(lab_dict)
            self.publisher.publish(self.strmsg)
        

def main(args=None):
    rclpy.init(args=args)
    node = VoiceCtrlColorLab()
    rclpy.spin(node)
    rclpy.shutdown()
