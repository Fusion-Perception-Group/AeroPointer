#! /usr/bin/env python

import rospy
import tf
import math
import numpy as np
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Pose
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

X_prop = 0.95
Y_prop = 0.95

def local_pose_callback(pose):
    global LocalPose
    # print(LocalPose)
    LocalPose = pose

def get_yaw_degrees(pose):
    quaternion = pose.pose.orientation 
    _, _, yaw = tf.transformations.euler_from_quaternion(
        [quaternion.x, quaternion.y, quaternion.z, quaternion.w])
    yaw_degrees = math.degrees(yaw)
    return yaw_degrees

def fly_to(target,stay=8, ifLand = False, PoseBias = 0.10):
    X_Ban = 2*X_prop
    Y_Ban = 1.2*Y_prop
    global rate
    global LocalPose
    global PoseMap
    global pub_pose
    DegBias = 5
    print(target)
    while True:
        rate.sleep()
        pub_pose.publish(target)
        if (ifLand == True):
                if abs(LocalPose.pose.position.z - target.pose.position.z) <= 0.15:
                    return True
                else:
                    return False
        if (abs(LocalPose.pose.position.x - target.pose.position.x) <= PoseBias and
            abs(LocalPose.pose.position.y - target.pose.position.y) <= PoseBias and
            abs(get_yaw_degrees(LocalPose) - get_yaw_degrees(target)) <= DegBias):
            rospy.loginfo("IN target!")
            rospy.loginfo(stay)
            stay = stay - 1
            if (stay <= 0):
                return True

if __name__ == "__main__":
    rospy.init_node("pose_create_py")
    #PICTURE
    rospy.loginfo("POSE_CREATE_NODE")
    # create publisher
    global pub_pose

    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
    disarm_cmd = CommandBoolRequest(value=False)
    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

    pub_pose = rospy.Publisher("/myPose", PoseStamped, queue_size = 20)
    local_pose_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, local_pose_callback, queue_size=10)
    global rate
    rate = rospy.Rate(20)

    q_yaw_0 = Quaternion(0, 0, 0, 1)
    q_yaw_90 = Quaternion(0, 0, math.sin(math.pi / 4), math.sin(math.pi / 4))
    q_yaw_180 = Quaternion(0, 0, 1, 0)
    q_yaw_270 = Quaternion(0, 0, -math.sin(math.pi / 4), math.sin(math.pi / 4))

    global LocalPose
    LocalPose = PoseStamped()
    # default height = 0.5m
    global PoseMap
    PoseMap = [[PoseStamped() for x in range(6)] for y in range(5)]
    EdgeBias = 0.20
    # poses[Y][X]
    for y in range(5):
        for x in range (6):
            PoseMap[y][x] = PoseStamped(
                pose=Pose(
                    position=Point(0.8*x*X_prop, 0.8*y*Y_prop, 1.6),
                    orientation=q_yaw_90
                )
            )
            if(x == 0 and y == 0):
                PoseMap[y][x] = PoseStamped(
                    pose=Pose(
                        position=Point(0.1, 0.1, 1),
                        orientation=q_yaw_90
                    )
                )
                continue
            if(y == 4):
                PoseMap[y][x].pose.position.y -= EdgeBias
            elif(y == 0):
                PoseMap[y][x].pose.position.y += EdgeBias

            if(x == 5):
                PoseMap[y][x].pose.position.x -= EdgeBias
            elif(x == 0):
                PoseMap[y][x].pose.position.x += EdgeBias
    print("MAP CREATED!")

    while(not rospy.is_shutdown()):
        print("START")
        # print "SEND [1][1], CHANGE ANGLE"
        # TargetPose1 = PoseMap[1][1]
        # fly_to(TargetPose1)

        print ("SEND [3][4], CHANGE ANGLE")
        TargetPose1_1 = PoseMap[3][4]
        fly_to(TargetPose1_1)
        # TargetPose1_1.pose.orientation = q_yaw_180
        fly_to(TargetPose1_1, 8)
        # TargetPose1_1.pose.orientation = q_yaw_270
        fly_to(TargetPose1_1, 8)

        print ("SEND [4][5]")
        TargetPose2 = PoseMap[4][5]
        # TargetPose2.pose.orientation = q_yaw_270
        fly_to(TargetPose2)

        print ("SEND [4][1]")
        TargetPose2_1 = PoseMap[4][1]
        # TargetPose2_1.pose.orientation = q_yaw_270
        fly_to(TargetPose2_1, 10)

        print ("SEND [4][0]")
        TargetPose3 = PoseMap[4][0]
        # TargetPose3.pose.orientation = q_yaw_270
        fly_to(TargetPose3)

        print ("SEND [3][0]")
        TargetPose4 = PoseMap[3][0]
        # TargetPose4.pose.orientation = q_yaw_270
        fly_to(TargetPose4, 10)

        print ("SEND [3][4]")
        TargetPose4_1 = PoseMap[3][4]
        # TargetPose4_1.pose.orientation = q_yaw_270
        fly_to(TargetPose4_1, 10)

        print ("SEND [3][5]")
        TargetPose5 = PoseMap[3][5]
        # TargetPose5.pose.orientation = q_yaw_270
        fly_to(TargetPose5)

        print ("SEND [2][5]")
        TargetPose6 = PoseMap[2][5]
        # TargetPose6.pose.orientation = q_yaw_270
        fly_to(TargetPose6, 10)

        print ("SEND [2][1]")
        TargetPose6_1 = PoseMap[2][1]
        # TargetPose6_1.pose.orientation = q_yaw_270
        fly_to(TargetPose6_1, 10)
        # TargetPose6_1.pose.orientation = q_yaw_0
        fly_to(TargetPose6_1, 10)

        print ("SEND [2][0]")
        TargetPose7 = PoseMap[2][0]
        # TargetPose7.pose.orientation = q_yaw_0
        fly_to(TargetPose7)
        break