#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
import math

current_position = PoseStamped()

def position_callback(msg):
    global current_position
    current_position = msg

def scan_callback(msg):
    avoidance_vector_x, avoidance_vector_y = 0, 0
    avoid = True
    d0, k = 0.55, 0.1  # 安全距离和控制参数

    # 遍历激光雷达数据
    for i in range(len(msg.ranges)):
        if d0 > msg.ranges[i] > 0.4:
            avoid = True
            angle = msg.angle_min + i * msg.angle_increment
            x = math.cos(angle)
            y = math.sin(angle)
            U = -0.5 * k * ((1/msg.ranges[i]) - (1/d0))**2

            avoidance_vector_x += x * U
            avoidance_vector_y += y * U

    # 如果需要避障，更新目标位置
    if avoid:
        magnitude = math.sqrt(avoidance_vector_x**2 + avoidance_vector_y**2)
        if magnitude > 3:
            scale = 3 / magnitude
            avoidance_vector_x *= scale
            avoidance_vector_y *= scale

        # 设置新位置并发布到 /myPose
        new_pose = PoseStamped()
        new_pose.header.stamp = rospy.Time.now()
        new_pose.header.frame_id = "map"
        new_pose.pose.position.x = current_position.pose.position.x + avoidance_vector_x
        new_pose.pose.position.y = current_position.pose.position.y + avoidance_vector_y
        new_pose.pose.position.z = current_position.pose.position.z  # Z坐标保持不变
        pose_pub.publish(new_pose)

def main():
    rospy.init_node('laser_scan_avoidance')
    # rospy.Subscriber('/vision_pose_unfused', PoseStamped, position_callback)
    rospy.Subscriber('/mavros/local_position/pose', PoseStamped, position_callback)
    # rospy.Subscriber('/scan', LaserScan, scan_callback)
    rospy.Subscriber('laser/scan', LaserScan, scan_callback)
    global pose_pub
    pose_pub = rospy.Publisher('/myPose', PoseStamped, queue_size=10)
    rospy.spin()

if __name__ == '__main__':
    main()
