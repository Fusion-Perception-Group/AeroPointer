#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
import math
import time

current_pose = PoseStamped()  # 使用PoseStamped类型

def pose_callback(msg):
    global current_pose
    # 更新当前位置，考虑到了出生点坐标
    # current_pose.pose.position.x = msg.pose.position.x + Spawn_X
    # current_pose.pose.position.y = msg.pose.position.y + Spawn_Y
    current_pose = msg

def is_close_to_target(target_pose, tolerance=0.1, ifZ = False):
    # rospy.loginfo("当前位置: {:.2f}, {:.2f}".format(current_pose.pose.position.x, current_pose.pose.position.y))
    """检查是否接近目标点"""
    dist = math.inf
    # rospy.loginfo(f":::::::::::::::::::{ifZ}")
    if ifZ :
        # rospy.loginfo("三维差值计算")
        dist = math.sqrt((current_pose.pose.position.x - target_pose.position.x) ** 2 +
                     (current_pose.pose.position.y - target_pose.position.y) ** 2+
                     (current_pose.pose.position.z - target_pose.position.z) ** 2)
    else:
        dist = math.sqrt((current_pose.pose.position.x - target_pose.position.x) ** 2 +
                     (current_pose.pose.position.y - target_pose.position.y) ** 2)
    return dist < tolerance

def fly_to(target_pose, ifZ = False):
    """阻塞式飞行至目标位置，并确保在目标点稳定停留2秒"""
    target_pub.publish(target_pose)
    rospy.loginfo("飞往目标点: {:.2f}, {:.2f}, {:.2f}".format(target_pose.position.x, 
                                                             target_pose.position.y, 
                                                             target_pose.position.z))
    stable_time = 0  # 目标点稳定时间

    while not rospy.is_shutdown():
        if is_close_to_target(target_pose, ifZ = ifZ):
            rospy.loginfo("位置已稳定!")
            if stable_time == 0:  # 开始计时
                stable_time = time.time()
            elif time.time() - stable_time >= 3:  # 如果已经在目标点稳定停留了至少2秒
                rospy.loginfo("目标点已稳定5秒!")
                break
        else:
            target_pub.publish(target_pose)
            stable_time = 0  # 如果离开目标点，重置计时器
        rospy.sleep(0.1)

def main():
    global target_pub
    rospy.init_node('fly_to_node')
    rospy.Subscriber('/mavros/local_position/pose', PoseStamped, pose_callback)
    target_pub = rospy.Publisher('/myPose', Pose, queue_size=10)

# 无旋转，面向X轴正方向
# no_rotation = (0, 0, 0, 1)
# # 绕Z轴逆时针旋转90度，面向Y轴正方向
# north = (0, 0, math.sqrt(2)/2, math.sqrt(2)/2)
# # 绕Z轴顺时针旋转90度，面向Y轴负方向
# south = (0, 0, -math.sqrt(2)/2, math.sqrt(2)/2)
# # 绕Z轴逆时针旋转180度，面向X轴负方向
# west = (0, 0, 1, 0)

    height = 1
    poses = [#13
        Pose(Point(0, 0, 0.5), Quaternion(0, 0, 0, 1)),
        Pose(Point(0, 0, -0.1), Quaternion(0, 0, 0, 1)),
    ]
    
    fly_to(poses[0], ifZ = True)
    fly_to(poses[1])
    
    rospy.loginfo("已完成")

if __name__ == '__main__':
    main()
