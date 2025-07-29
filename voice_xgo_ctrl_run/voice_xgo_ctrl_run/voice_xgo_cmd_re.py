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
        super().__init__("voice_xgo_cmd_re")
        self.spe = Speech()
        self.send_cmd_re()

    def send_cmd_re(self):
        while True:		
            command_result = self.spe.speech_read()
            
            print(command_result)
            if command_result==19:
                self.spe.void_write(command_result)  
            if command_result==20:
                 self.spe.void_write(command_result)
            if command_result==21:
                 self.spe.void_write(command_result)
            if command_result==87:
                 self.spe.void_write(69)
            if command_result==88:
                 self.spe.void_write(67)
            if command_result==89:
                  self.spe.void_write(68)
            if command_result==90:
                  self.spe.void_write(66)
            if command_result==73:
                  self.spe.void_write(73)
            if command_result==26:
                  self.spe.void_write(26)
            time.sleep(0.5)
        
def main(args=None):
    rclpy.init(args=args)
    node = VoiceCtrl()
    rclpy.spin(node)
    rclpy.shutdown()