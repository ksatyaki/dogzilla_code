#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped

class TFPublisher(Node):
    def __init__(self):
        super().__init__('tf_publisher')

        # 创建一个TF广播器
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # 定义IMU和雷达之间的变换
        self.imu_to_lidar_transform = TransformStamped()
        self.imu_to_lidar_transform.header.frame_id = 'base_footprint'
        self.imu_to_lidar_transform.child_frame_id = 'base_link'
        self.imu_to_lidar_transform.transform.translation.x = 0.1154
        self.imu_to_lidar_transform.transform.translation.y = 0.0
        self.imu_to_lidar_transform.transform.translation.z = 0.047
        self.imu_to_lidar_transform.transform.rotation.x = 0.0
        self.imu_to_lidar_transform.transform.rotation.y = 0.0
        self.imu_to_lidar_transform.transform.rotation.z = 0.0
        self.imu_to_lidar_transform.transform.rotation.w = 1.0

        # 发布变换
        self.timer = self.create_timer(0.1, self.publish_tf)

    def publish_tf(self):
        self.imu_to_lidar_transform.header.stamp = self.get_clock().now().to_msg()
        self.tf_broadcaster.sendTransform(self.imu_to_lidar_transform)

def main(args=None):
    rclpy.init(args=args)
    tf_publisher = TFPublisher()
    rclpy.spin(tf_publisher)
    rclpy.shutdown()

if __name__ == '__main__':
    main()