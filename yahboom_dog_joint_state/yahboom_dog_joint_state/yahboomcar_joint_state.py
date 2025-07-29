from sensor_msgs.msg import JointState
import time
from math import pi
import rclpy
from rclpy.node import Node
import DOGZILLALib as dog
from sensor_msgs.msg import Imu
import numpy as np
import math



#control = dog.DOGZILLA()

"""
"lf_hip_joint", 左边肩部关节
"lf_lower_leg_joint",  左边前腿底部关节
"lf_upper_leg_joint", 左边前腿上部关节


"lh_lower_leg_joint", 左边后腿底部关节
"lh_upper_leg_joint", 左边后腿上部电机
"lh_hip_joint", 左边臀部关节

"rf_hip_joint", 右边肩部电机
"rf_lower_leg_joint", 右边前腿底部关节
"rf_upper_leg_joint", 右边前腿上部关节

"rh_hip_joint", 右边臀部关节
"rh_lower_leg_joint", 右边后腿底部关节
"rh_upper_leg_joint" 右边后腿上部关节
"""


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('yahboomcar_joint_state')
        self.dogControl = dog.DOGZILLA()
        self.imu_msg = Imu()
        self.imu_msg.header.frame_id = 'imu_link'

        self.publisher_ = self.create_publisher(JointState, '/joint_states', 5)
        self.imu_pub = self.create_publisher(Imu, 'imu/data_raw_self', 10)

        self.timer_period = 0.05  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.i = 0
        self.last_state = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.last_imu = []
        self.last_time = time.time()

    def timer_callback(self):
        try:
            
            msg = JointState()
            t = self.get_clock().now()
            msg.header.stamp = t.to_msg()
            msg.name = ["lf_upper_leg_joint", "lf_lower_leg_link",  "lf_hip_joint",
                        "lh_upper_leg_joint", "lh_lower_leg_joint",  "lh_hip_joint",
                        "rf_upper_leg_joint", "rf_lower_leg_joint",  "rf_hip_joint",
                        "rh_upper_leg_joint", "rh_lower_leg_joint",   "rh_hip_joint"]

            angle = self.dogControl.read_motor()
            print(angle)
            msg.position = [angle[0] * pi / 180, 0 - angle[1] * pi / 180, angle[2] * pi / 180,
                            angle[3] * pi / 180, 0 - angle[4] * pi / 180, angle[5] * pi / 180,
                            0 - angle[6] * pi / 180, angle[7] * pi / 180, angle[8] * pi / 180,
                            0 - angle[9] * pi / 180, angle[10] * pi / 180, angle[11] * pi / 180]

            # msg.position = [-0.74, 0.3, 0.0,
            #                 0.0, 0.0, 0.0,
            #                 0.0, 0.0, 0.0,
            #                 0.0, 0.0, 0.0]
            dt = 1
            
            msg.velocity = [(angle[0] * pi / 180 - self.last_state[0] * pi / 180) / self.timer_period,
                            (angle[1] * pi / 180 - self.last_state[1] * pi / 180) / self.timer_period,
                            (angle[2] * pi / 180 - self.last_state[2] * pi / 180) / self.timer_period,
                            (angle[3] * pi / 180 - self.last_state[3] * pi / 180) / self.timer_period,
                            (angle[4] * pi / 180 - self.last_state[4] * pi / 180) / self.timer_period,
                            (angle[5] * pi / 180 - self.last_state[5] * pi / 180) / self.timer_period,
                            (angle[6] * pi / 180 - self.last_state[6] * pi / 180) / self.timer_period,
                            (angle[7] * pi / 180 - self.last_state[7] * pi / 180) / self.timer_period,
                            (angle[8] * pi / 180 - self.last_state[8] * pi / 180) / self.timer_period,
                            (angle[9] * pi / 180 - self.last_state[9] * pi / 180) / self.timer_period,
                            (angle[10] * pi / 180 - self.last_state[10] * pi / 180) / self.timer_period,
                            (angle[11] * pi / 180 - self.last_state[11] * pi / 180) / self.timer_period,
                            ]

            self.last_state = angle

            msg.effort = [float("nan"), float("nan"), float("nan"), float("nan"), float("nan"), float("nan"),
                          float("nan"),
                          float("nan"), float("nan"), float("nan"), float("nan"), float("nan")]
            time.sleep(0.1)
           
            imu_raw = self.dogControl.read_imu_raw()
            now_time = time.time()
            if imu_raw[0] != 0:
                self.last_imu = imu_raw
            accx,accy,accz,gyrox,gyroy,gyroz,roll,pitch,yaw = self.last_imu[0],self.last_imu[1],self.last_imu[2],self.last_imu[3],self.last_imu[4],self.last_imu[5], self.last_imu[6], self.last_imu[7], self.last_imu[8]
            dt = now_time -self.last_time
            print("&&&&&&&&&&&&&&&&&&&&&&&&&", dt)
            self.last_time = now_time
            #roll, pitch, yaw = self.compute_orientation(accx,accy,accz,gyrox,gyroy,gyroz,dt)
            qua = self.get_quaternion_from_euler(roll,pitch,yaw)
            # 更新IMU消息
            print("##################")
            print(imu_raw)
            self.imu_msg.header.stamp = self.get_clock().now().to_msg()
            self.imu_msg.linear_acceleration.x = accx
            self.imu_msg.linear_acceleration.y = accy
            self.imu_msg.linear_acceleration.z = accz
            self.imu_msg.angular_velocity.x = gyrox
            self.imu_msg.angular_velocity.y = gyroy
            self.imu_msg.angular_velocity.z = gyroz
            self.imu_msg.orientation.x = qua[0]
            self.imu_msg.orientation.y = qua[1]
            self.imu_msg.orientation.z = qua[2]
            self.imu_msg.orientation.w = qua[3]
            print("**************************",self.timer)
            #self.imu_msg.orientation.x = 0.0
            #self.imu_msg.orientation.y = 0.0
            #self.imu_msg.orientation.z = 0.0
            #self.imu_msg.orientation.w = 0.0
 
            self.imu_pub.publish(self.imu_msg)
            self.publisher_.publish(msg)
        except IndexError:
            self.get_logger().error(f'获取舵机状态失败', throttle_duration_sec=1)
            pass

    
    def compute_orientation(self, wx, wy, wz, ax, ay, az, dt):
        # 计算旋转矩阵
        Rx = np.array([[1, 0, 0],
                       [0, math.cos(ax), -math.sin(ax)],
                       [0, math.sin(ax), math.cos(ax)]])
        Ry = np.array([[math.cos(ay), 0, math.sin(ay)],
                       [0, 1, 0],
                       [-math.sin(ay), 0, math.cos(ay)]])
        Rz = np.array([[math.cos(wz), -math.sin(wz), 0],
                       [math.sin(wz), math.cos(wz), 0],
                       [0, 0, 1]])
        R = Rz.dot(Ry).dot(Rx)

        # 计算欧拉角
        roll = math.atan2(R[2][1], R[2][2])
        pitch = math.atan2(-R[2][0], math.sqrt(R[2][1] ** 2 + R[2][2] ** 2))
        yaw = math.atan2(R[1][0], R[0][0])
        print("roll", roll, "pitch", pitch, "yaw", yaw)

        return roll, pitch, yaw


    def get_quaternion_from_euler(self, roll, pitch, yaw):
        qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
        qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2)
        qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2)
        qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
        return [qx, qy, qz, qw]
        
        
    def roll_pitch_yaw_to_quaternion(self, roll, pitch, yaw):  
    
        x = math.radians(roll)  
        y = math.radians(pitch)  
        z = math.radians(yaw)  
  
        qx = math.cos(x) * math.cos(y) * math.cos(z)  
        qy = math.cos(x) * math.sin(y) * math.cos(z)  
        qz = math.cos(x) * math.sin(y) * math.sin(z)  
        qw = math.cos(x) * math.cos(z)  
        qn = math.cos(x) * math.sin(z)  
  
        return [qx, qy, qz, qw]




def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
