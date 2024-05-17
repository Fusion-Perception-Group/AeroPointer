#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
import os
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from sensor_msgs.msg import LaserScan
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

LaserData = LaserScan()
VisionData = PoseStamped()

spawn_x = 0.725
spawn_y = 0.884

Area_1 = ((0, 0), (5, 1.865))
Area_2 = ((0, 1.865), (5, 3.12))
Area_3 = ((0, 3.12), (5, 5))

def in_Area(poseTuple: tuple[float, float]):
    x, y = poseTuple
    # 检查 Area_1
    if x >= Area_1[0][0] and x <= Area_1[1][0] and y >= Area_1[0][1] and y <= Area_1[1][1]:
        return 1
    # 检查 Area_2
    if x >= Area_2[0][0] and x <= Area_2[1][0] and y >= Area_2[0][1] and y <= Area_2[1][1]:
        return 2
    # 检查 Area_3
    if x >= Area_3[0][0] and x <= Area_3[1][0] and y >= Area_3[0][1] and y <= Area_3[1][1]:
        return 3
    # 如果不在任何区域内
    return 0

def getLaserData(angle : str):
    bias = 3
    if angle == "front":
        min_val = float('inf')
        for i in range(0, 0 + bias):
            min_val = min(min_val, LaserData.ranges[i])
        return min_val
    elif angle == "left":
        min_val = float('inf')
        for i in range(90 - bias, 90 + bias):
            min_val = min(min_val, LaserData.ranges[i])
        return min_val
    elif angle == 'back':
        min_val = float('inf')
        for i in range(180 - bias, 180 + bias):
            min_val = min(min_val, LaserData.ranges[i])
        return min_val
    elif angle == "right":
        min_val = float('inf')
        for i in range(270 - bias, 270 + bias):
            min_val = min(min_val, LaserData.ranges[i])
        return min_val
    else:
        return None

def get_fusion_Pose(aera : int, vision_pose: PoseStamped) -> PoseStamped:
    vision_position = {'x':PoseStamped.pose.position.x,'y':PoseStamped.pose.position.y,'z':PoseStamped.pose.position.z}

    laser_distance = {'front':getLaserData('front'),'left':getLaserData('left'),'back':getLaserData('back'),'right':getLaserData('right')}
    
    finalpose = {'x':0, 'y':0, 'z':0}
    
    if(aera == 1):
        #第二个小屏障
        if vision_position['x'] < 2.65 and vision_position['y'] < 0.5:
            finalpose['x'] = 2.65 - laser_distance['front']
            finalpose['y'] = laser_distance['right']
        #第二个小屏障的厚度
        elif vision_position['x'] > 2.55  and vision_position['x'] < 2.85:
            finalpose['x'] = 5 - laser_distance['front']
            finalpose['y'] = vision_position['y']
        #激光雷达x y都正确
        else:
            finalpose['x'] = 5 - laser_distance['front']
            finalpose['y'] = laser_distance['right']
            
            
        
    elif(aera == 2):
        
        pass
    
    elif(aera == 3):
        
        pass
    
    else:
        return vision_pose


    
###############################################################
###############################################################
def laser_callback(msg) -> None:
    global LaserData
    LaserData = msg

def vision_pose_callback(msg) -> None:
    global VisionData
    VisionData = msg
###############################################################
###############################################################


if __name__ == '__main__':
    rospy.init_node('fusion_estimator')
    rospy.Subscriber('/scan', LaserScan, laser_callback, queue_size=5)
    rospy.Subscriber('/vision_pose_unfusioned', PoseStamped, vision_pose_callback,queue_size=5)
    rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=10)
    rospy.spin()
