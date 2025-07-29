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
from std_msgs.msg import String
import threading
from yahboom_color_identify_interfaces.srv import ColorIdentify
import redis


class VoiceCtrlSendGoal(Node):
    def __init__(self):
        super().__init__("voice_ctrl_publisher")
        self.publisher = self.create_publisher(PoseStamped, '/voice_command', 10)
        self.publisher1 = self.create_publisher(String, 'lab_set', 10)
        self.sub = self.create_subscription(String, '/goal_status',self.nav_status_callback, 10)
        self.cli = self.create_client(ColorIdentify, 'yahboomColorIdentify')
        self.r = redis.Redis(host='localhost', port=6379, db=0)
        while not self.cli.wait_for_service(timeout_sec=10.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = ColorIdentify.Request()
        self.timer = self.create_timer(0.1, self.send_pose)
        self.x = 0.0
        self.z = 0.0
        self.goal_pose = PoseStamped()
        self.spe = Speech()
        self.pose0 = PoseStamped()
        self.pose0.pose.position.x = 0.0
        self.pose0.pose.position.y = 0.0
        self.pose0.pose.orientation.w = 1.0;
        self.pose1 = PoseStamped()
        self.pose1.pose.position.x = 1.0
        self.pose1.pose.position.y = 0.0
        self.pose1.pose.orientation.w = 1.0;
        self.pose2 = PoseStamped()
        self.pose2.pose.position.x = 1.0
        self.pose2.pose.position.y = 0.5
        self.pose2.pose.orientation.w = 1.0;
        self.pose3 = PoseStamped()
        self.pose3.pose.position.x = 1.0
        self.pose3.pose.position.y = 1.0
        self.pose3.pose.orientation.w = 1.0;
        self.count = 0
        self.dogControl = dog.DOGZILLA()
        self.dogControl.attitude("p", 0)
        self.strmsg = String()
        self.client_futures = []
        self.obj_status = 0
        

    def send_pose(self):
        command_result = self.spe.speech_read()
        
        if command_result==19:
        #导航去1点
            self.dogControl.attitude("p", 0)
            self.goal_pose.header.frame_id = "map"
            self.goal_pose.pose.position.x = 1.0
            self.goal_pose.pose.position.y = 0.0
            self.goal_pose.pose.orientation.w = 1.0
            self.publisher.publish(self.goal_pose)
            self.count = 1

    def spin(self):
        while rclpy.ok():
            rclpy.spin_once(self)
            incomplete_futures = []
            for f in self.client_futures:
                if f.done():
                    res = f.result()
                    self.get_logger().info("received service result: {}".format(res))
                    print("received service result: {}".format(res))
                else:
                    incomplete_futures.append(f)
            self.client_futures = incomplete_futures

    def nav_status_callback(self, msg):
        print(msg)
        global obj_status
        if msg.data == "SUCCEEDED":
            if self.count == 1:
                print("1点完成",self.count)
                self.dogControl.attitude("p", 15)
                time.sleep(5)
                lab_dict = {"l":0, "a": 155, "b":21, "l_max": 255 , "a_max": 255, "b_max": 255}
                self.strmsg.data = json.dumps(lab_dict)
                self.publisher1.publish(self.strmsg)
                time.sleep(5)
                self.req.if_identify = "mapssss"
                self.client_futures.append(self.cli.call_async(self.req))
                cs = self.r.get('color_status')
                self.get_logger().info("redis获取消息：%s"%cs)
                print(cs)
                if cs==b'ok':
                    self.obj_status = 1
                else:
                    self.obj_status = 0
                lab_dict = {"l":0, "a": 0, "b":0, "l_max": 0 , "a_max": 0, "b_max": 0}
                self.strmsg.data = json.dumps(lab_dict)
                self.publisher1.publish(self.strmsg)
                time.sleep(2)
                self.dogControl.attitude("p", 0)
                time.sleep(2)
                print("当前颜色状态：", self.obj_status)
                if self.obj_status:
                    self.count = 0
                    self.dogControl.attitude("p", 0)
                    time.sleep(2)
                    self.get_logger().info("1号位有红色木块：返回原点")
                    print("1号为有红色木块：返回原点")
                    self.publisher.publish(self.pose0)
                    return
                else:
                    self.count = 2
                    self.get_logger().info("没有红色木块：去2点查看")
                    print("准备去2号点：恢复姿态")
#                    time.sleep(10)
                    print("没有红色木块：去2点")
                    self.publisher.publish(self.pose2)
                    print("2点数据发送")
                    return
                    
            if self.count == 2:
                print("2点完成",self.count)
                self.dogControl.attitude("p", 15)
                time.sleep(5)
                lab_dict = {"l":0, "a": 155, "b":21, "l_max": 255 , "a_max": 255, "b_max": 255}
                self.strmsg.data = json.dumps(lab_dict)
                self.publisher1.publish(self.strmsg)
                time.sleep(5)
                
                self.req.if_identify = "mapssss"
                self.client_futures.append(self.cli.call_async(self.req))
                cs = self.r.get('color_status')
                print(cs)
                if cs== b'ok':
                    self.obj_status = 1
                else:
                    self.obj_status = 0
                lab_dict = {"l":0, "a": 0, "b":0, "l_max": 0 , "a_max": 0, "b_max": 0}
                self.strmsg.data = json.dumps(lab_dict)
                self.publisher1.publish(self.strmsg)
                time.sleep(2)
                self.dogControl.attitude("p", 0)
                time.sleep(2)
                print("当前颜色状态：", self.obj_status)
                if self.obj_status:
                    self.count = 0
                    self.dogControl.attitude("p", 0)
                    time.sleep(2)
                    self.get_logger().info("2号位有红色木块：返回原点")
                    print("2号为有红色木块：返回原点")
                    self.publisher.publish(self.pose0)
                    return
                else:
                    self.count = 3
                    self.get_logger().info("没有红色木块：去3点查看")
                    print("准备去3号点：恢复姿态")
                    print("没有红色木块：去3点")
                    self.publisher.publish(self.pose3)
                    print("3点数据发送")
                    return
                    
            if self.count == 3:
                self.dogControl.attitude("p", 15)
                print("3点完成",self.count)
                time.sleep(5)
                lab_dict = {"l":0, "a": 155, "b":21, "l_max": 255 , "a_max": 255, "b_max": 255}
                self.strmsg.data = json.dumps(lab_dict)
                self.publisher1.publish(self.strmsg)
                time.sleep(5)
                self.req.if_identify = "mapssss"
                self.client_futures.append(self.cli.call_async(self.req))
                
                cs = self.r.get('color_status')
                print(cs)
                if cs==b'ok':
                    self.obj_status = 1
                else:
                    self.obj_status = 0
                lab_dict = {"l":0, "a": 0, "b":0, "l_max": 0 , "a_max": 0, "b_max": 0}
                self.strmsg.data = json.dumps(lab_dict)
                self.publisher1.publish(self.strmsg)
                time.sleep(2)
                self.dogControl.attitude("p", 0)
                time.sleep(2)
                print("当前颜色状态：", self.obj_status)
                if self.obj_status:
                    self.count = 0
                    self.dogControl.attitude("p", 0)
                    time.sleep(2)
                    self.get_logger().info("3号位有红色木块：返回原点")
                    print("3号为有红色木块：返回原点")
                    self.publisher.publish(self.pose0)
                    return
                else:
                    self.count = 0
                    self.get_logger().info("没有红色木块：返回原点")
                    print("没有红色木块：返回")
                    print("准备去0号点：恢复姿态")
#                    time.sleep(10)
                    self.publisher.publish(self.pose0)
                    print("0点数据发送")
                    return
                       
        
        
def main(args=None):
    rclpy.init(args=args)
    node = VoiceCtrlSendGoal()
#    t = threading.Thread(target=subscriber_thread, args=(node,))
#    t.start()
    node.spin()
#    rclpy.spin(node)
    rclpy.shutdown()
