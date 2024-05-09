#!/usr/bin/env python

import rospy
import os
from geometry_msgs.msg import Quaternion, Vector3
from sensor_msgs.msg import Imu
import math

class SenseGlove:
    def __init__(self):
        rospy.init_node('sense_glove')
        self.quat_pub = rospy.Publisher('/glove/quaternion', Quaternion, queue_size=10)
        self.euler_pub = rospy.Publisher('/glove/euler', Vector3, queue_size=10)
        self.imu_data = Imu()
        self.imu_data.orientation_covariance = [0.0]*9  # Set covariance to zero for now

    def start_publishing(self):
        rate = rospy.Rate(5)  # 5Hz
        while not rospy.is_shutdown():
            self.update_imu_data()
            self.publish_data()
            self.print_orientation()
            rate.sleep()

    def update_imu_data(self):
        # Dummy IMU data with fixed Euler angles (1, 1, 1)
        x = 1
        y = 1
        z = 1
        quaternion = self.euler_to_quaternion(x, y, z)
        self.imu_data.orientation = quaternion

    def publish_data(self):
        euler_angles = self.quaternion_to_euler(
            self.imu_data.orientation.x,
            self.imu_data.orientation.y,
            self.imu_data.orientation.z,
            self.imu_data.orientation.w
        )
        self.quat_pub.publish(self.imu_data.orientation)
        euler_msg = Vector3(x=euler_angles[0], y=euler_angles[1], z=euler_angles[2])
        self.euler_pub.publish(euler_msg)  # 发布欧拉角数据

    def euler_to_quaternion(self, x, y, z):
        qx = math.sin(x/2) * math.cos(y/2) * math.cos(z/2) - math.cos(x/2) * math.sin(y/2) * math.sin(z/2)
        qy = math.cos(x/2) * math.sin(y/2) * math.cos(z/2) + math.sin(x/2) * math.cos(y/2) * math.sin(z/2)
        qz = math.cos(x/2) * math.cos(y/2) * math.sin(z/2) - math.sin(x/2) * math.sin(y/2) * math.cos(z/2)
        qw = math.cos(x/2) * math.cos(y/2) * math.cos(z/2) + math.sin(x/2) * math.sin(y/2) * math.sin(z/2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)

    def print_orientation(self):
        os.system('clear')  # Clear terminal screen
        rospy.loginfo("Quaternion: x={}, y={}, z={}, w={}".format(
            self.imu_data.orientation.x,
            self.imu_data.orientation.y,
            self.imu_data.orientation.z,
            self.imu_data.orientation.w
        ))

        x, y, z = self.quaternion_to_euler(
            self.imu_data.orientation.x,
            self.imu_data.orientation.y,
            self.imu_data.orientation.z,
            self.imu_data.orientation.w
        )
        rospy.loginfo("Euler Angles: x={}, y={}, z={}".format(
            x, y, z
        ))

    def quaternion_to_euler(self, x, y, z, w):
        # Convert Quaternion to Euler angles
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        x = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        y = math.pi / 2 if abs(sinp) >= 1 else math.asin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        z = math.atan2(siny_cosp, cosy_cosp)

        return x, y, z

if __name__ == '__main__':
    sg = SenseGlove()
    sg.start_publishing()