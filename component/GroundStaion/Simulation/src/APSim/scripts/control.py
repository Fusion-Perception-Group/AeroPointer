#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

poses_1= []
poses_2=[]
poses_3= []

currentSetPose = Pose()

class droneControl():
    def __init__():
