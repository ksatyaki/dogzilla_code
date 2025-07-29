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
import DOGZILLALib as dog
import time
from geometry_msgs.msg import Twist, PoseStamped
from image_color_lab.msg import StringStamped
from rclpy.duration import Duration
import sys
from Speech_Lib import Speech


class VoiceCtrlAction(Node):
    def __init__(self):
        super().__init__("voice_ctrl_publisher")
        self.publisher = self.create_publisher(PoseStamped, '/voice_action', 10)
        self.timer = self.create_timer(0.1, self.send_pose)
        self.x = 0.0
        self.z = 0.0
        self.goal_pose = PoseStamped()
        self.dogControl = dog.DOGZILLA()
        self.spe = Speech()

    def send_pose(self):
        command_result = self.spe.speech_read()


        if command_result==95:
        #动作1趴下
            self.dogControl.action(1)
        if command_result==96:
        #动作2站起
            self.dogControl.action(2)
        if command_result==97:
        #动作3匍匐前进
            self.dogControl.action(3)
        if command_result==98:
        #动作3转圈
            self.dogControl.action(4)
        if command_result==99:
        #动作4原地踏步
            self.dogControl.action(5)
        if command_result==100:
        #动作20伸懒腰
            self.dogControl.action(20)
        if command_result==101:
        #动作21俯卧撑
            self.dogControl.action(21)
        if command_result==102:
        #动作22扭屁股
            self.dogControl.action(22)
        if command_result==103:
        #动作23
            self.dogControl.action(23)
        if command_result==104:
        #动作24摇头
            self.dogControl.action(24)
            
        
        



def main(args=None):
    rclpy.init(args=args)
    node = VoiceCtrlAction()
    rclpy.spin(node)
    rclpy.shutdown()
