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
        super().__init__("image_subscriber")
        self.dog = DOGZILLALib.DOGZILLA()
        print("dddddddddddddddddddd1")
        self.image_sub = Subscriber(self, CompressedImage, '/image_contours')
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
        self.obj_pitch = 5
        self.obj_yaw = 0
        self.last_pitch = 0
        self.last_yaw = 0
#        self.dog.attitude("p", 5)
#        self.dog.attitude("y", 0)
        self.x = 0.0
        self.count = 0
#        self.dog.translation("z", 85)
        self.pid_kp_p = 1
        self.pid_kp_y = 1
        self.pid_ki_p = 0.1
        self.pid_ki_y = 0.1
        self.pid_kd_p = 0.2
        self.pid_kd_y = 0.2
        self.pid_error_p = 0
        self.pid_error_y = 0
        self.pid_last_error_p = 0
        self.pid_last_error_y = 0
        self.pid_error_p_list = [0]*10
        

        
        
    

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


    def pid(self, center_x, center_y):
        self.pid_error_p = 240 - center_y
        
        self.pid_error_y = 320 - center_x
        if len(self.pid_error_p_list)==10:
            self.count = 0
        self.pid_error_p_list[self.count] = self.pid_error_p
        self.count += 1
        
        pid_p_p_out = self.pid_kp_p*self.pid_error_p
        pid_y_p_out = self.pid_kp_y*self.pid_error_y
        pid_p_i_out = self.pid_ki_p*self.pid_error_p
        pid_y_i_out = self.pid_ki_y*self.pid_error_y
        pid_p_d_out = self.pid_kd_p*(self.pid_error_p - self.pid_last_error_p)
        pid_y_d_out = self.pid_kd_p*(self.pid_error_y - self.pid_last_error_y)
        pid_p_out = pid_p_p_out + pid_p_i_out + pid_p_d_out
        pid_y_out = pid_y_p_out + pid_y_i_out + pid_y_d_out
        self.pid_last_error_p = self.pid_error_p
        self.pid_last_error_y = self.pid_error_y

        print("pid_error_p：", self.pid_error_p, "pid_error_y", self.pid_error_y)
        return pid_p_out, pid_y_out
        
        
        


    def callback(self, image, info):
        print("image::",image.header)
        print("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%")
        print("info::",info.data)
        print("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%")
        labset = json.loads(info.data)
        now_pitch = self.dog.read_pitch()
        now_yaw = self.dog.read_yaw()
        # time.sleep(0.5)
        try:
		  
            self.obj_pitch = labset["vertical_movement"]
            self.obj_yaw = labset["horizontal_movement"]
            center_x = labset["center_x"] 
            center_y = labset["center_y"] 
            area = labset["area"]
            
#            if int(area) > 500:
#                p_out, y_out = self.pid(center_x, center_y)
#                p = (-int(p_out)/16) + (-int(p_out)/16 + self.last_pitch)/2
#                if  self.obj_pitch > 0:
#                    
#                if p > 22:
#                    p = 22
#                if p < -22:
#                    p = -22
#                print("############################")
#                print("set_pitch::", p)
#                print("now_pitch::", (-int(p_out)/16 + self.last_pitch)/2)
#                print("############################")
#                if 
#                print("p_out:", p_out,"p:",int(p_out)/16, "y_out:", y_out,'y:', int(y_out)/5)
#                self.dog.attitude('p', -int(p_out)/10)
#                self.last_pitch = p
#                #self.dog.attitude(['y','p'],[p_outx/16, y_out/5])
##                time.sleep(0.5)
#                print("有识别到")
#                image_h = labset["img_h"]
#                image_w = labset["img_w"]
#                value_x = int(center_x) - int(image_w)/2
#                value_y = int(center_y) - int(image_h)/2
#                if value_x > 110:
#                    value_x = 110
#                elif value_x < -110:
#                    value_x = -110
#                if value_y > 150:
#                    value_y = 150
#                elif value_y < -150:
#                    value_y = -150
#                print("value_x:",value_x, "value_y:",value_y)
##                self.dog.attitude(['y','p'],[-value_x/10, value_y/10])
##                time.sleep(0.5)
#            else:
#                self.dog.attitude(['y','p'],[0, 0])
#                self.pid_last_error_p = 0
#                self.pid_last_error_y = 0

			
            
            

            
            #self.send_service_request()
            # perform OpenCV operations on cv_image here
            
            if int(area) > 500:
#                time.sleep(0.15)
                self.count = 1 
                self.x = 0.0
                if (labset["img_h"] / 2 - 15) < labset["center_y"] < (labset["img_h"] / 2 + 15) and (labset["img_w"] / 2 - 45) < labset["center_x"] < (labset["img_w"] / 2 + 45):
                    print("已经到达预定点")
                    pass
                else:
                    if -22 < self.last_pitch < 22:
				
                        last_pitch_set = self.last_pitch + labset["vertical_movement"]
                        self.dog.attitude("p", last_pitch_set)
#                        time.sleep(0.15)
                        now_pitch_is_suc = self.dog.read_pitch()

                        if now_pitch_is_suc != now_pitch:
                            self.last_pitch = last_pitch_set
                    else:
                        if self.last_pitch > 0:
                            self.last_pitch = 21
                        if self.last_pitch < 0:
                            self.last_pitch = -21
                    if -16 < self.last_yaw < 16:
                        last_yaw_set = self.last_yaw + labset["horizontal_movement"]
                        self.dog.attitude("y", last_yaw_set)
                        now_yaw_is_suc = self.dog.read_yaw()
#                        time.sleep(0.15)
                        if now_pitch_is_suc != now_pitch:
                            self.last_yaw = last_yaw_set
                    else:
                        if self.last_yaw > 0: 
                            self.last_yaw = 15
                        else:
                            self.last_yaw = -15
                        

                #        if labset["center_y"] > labset["img_h"]/2:
                #            print("da::",int(now_pitch))
                #            if int(now_pitch) < int(22):
                #                print("change::", int(now_pitch) + 2)
                #                set_value = int(now_pitch) + 2
                #                self.dog.attitude("p", int(now_pitch) + 1)
                #            else:
                #                self.dog.attitude("p", int(15))
                
            else:
               
                #self.dog.attitude("p", 0)
                #self.dog.attitude("y", 0)
                if self.count:
                    self.dog.attitude("p", 5)
                    time.sleep(0.2)
                    self.dog.attitude("y", 0)
                    self.count = 0
                self.obj_pitch = 5
                self.obj_yaw = 0
                self.x = 0.4

        except:
            self.obj_pitch = 0
            self.obj_yaw = -5
            self.x = 0.0
            pass
    #        else:
    #            print("xiao::",(now_pitch))
    #            if int(now_pitch) > -22 and int(now_pitch) < 22:
    #                self.dog.attitude("p", int(now_pitch) - 1)
    #            else:
    #                self.dog.attitude("p", int(0))


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



def main(args=None):
    rclpy.init(args=args)
    node = ObjSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()
