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
from geometry_msgs.msg import Twist, PoseStamped
from image_color_lab.msg import StringStamped
from rclpy.duration import Duration
import sys
from Speech_Lib import Speech


class VoiceCtrlSendGoal(Node):
    def __init__(self):
        super().__init__("voice_ctrl_publisher")
        self.publisher = self.create_publisher(PoseStamped, '/voice_command', 10)
        self.timer = self.create_timer(0.1, self.send_pose)
        self.x = 0.0
        self.z = 0.0
        self.goal_pose = PoseStamped()
        self.spe = Speech()

    def send_pose(self):
        command_result = self.spe.speech_read()
        
        if command_result==19:
        #导航去1点
            self.goal_pose.header.frame_id = "map"
            self.goal_pose.pose.position.x = 1.0
            self.goal_pose.pose.position.y = 0.0
            self.goal_pose.pose.orientation.w = 1.0
            self.publisher.publish(self.goal_pose)
        if command_result==20:
        #导航去2点
            self.goal_pose.header.frame_id = "map"
            self.goal_pose.pose.position.x = 1.5
            self.goal_pose.pose.position.y = 0.0
            self.goal_pose.pose.orientation.w = 1.0
            self.publisher.publish(self.goal_pose)
        if command_result==21:
        #导航去3点
            self.goal_pose.header.frame_id = "map"
            self.goal_pose.pose.position.x = 1.0
            self.goal_pose.pose.position.y = 0.5
            self.goal_pose.pose.orientation.w = 1.0
            self.publisher.publish(self.goal_pose)
        if command_result==32:
        #导航去4点
            self.goal_pose.header.frame_id = "map"
            self.goal_pose.pose.position.x = 1.0
            self.goal_pose.pose.position.y = 1.5
            self.goal_pose.pose.orientation.w = 1.0
            self.publisher.publish(self.goal_pose)
        if command_result==33:
        #导航去0点
            self.goal_pose.header.frame_id = "map"
            self.goal_pose.pose.position.x = 0.0
            self.goal_pose.pose.position.y = 0.0
            self.goal_pose.pose.orientation.w = 1.0
            self.publisher.publish(self.goal_pose)
        
        



def main(args=None):
    rclpy.init(args=args)
    node = VoiceCtrlSendGoal()
    rclpy.spin(node)
    rclpy.shutdown()
