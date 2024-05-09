#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

current_state = State()  # 当前无人机状态
global ifSaid
ifSaid = False  # 是否已发送着陆信息标志

#######################################################################
# 初始化目标位置
#######################################################################
global pose
pose = PoseStamped(
    pose=Pose(
        position=Point(0, 0, 1),
        orientation=Quaternion(0, 0, math.sin(math.pi / 4), math.sin(math.pi / 4))
    )
)

# 回调函数：更新无人机当前状态
def state_cb(msg):
    global current_state
    current_state = msg

# 回调函数：接收目标位置并更新
# pose:PoseStamped类型对象
# msg:Pose类型对象
def myPose_callback(msg):
    global pose
    if msg is not None and pose.pose != msg:
        pose.pose = msg
        # rospy.logwarn("POSE CHANGED:\n %s" % pose)
        rospy.logwarn("POSE CHANGED")

# 程序关闭时的处理函数
def on_shutdown():
    global ifSaid
    if ifSaid:
        return
    global offb_set_mode
    offb_set_mode.custom_mode = "AUTO.LAND"
    
    if set_mode_client.call(offb_set_mode).mode_sent:
        ifSaid = True
        rospy.loginfo("offboard python Node Closing, Vehicle landing")
        rospy.loginfo("~Goodbye~")

# 主函数
if __name__ == "__main__":
    rospy.init_node("ap_actuator")  # 初始化节点
    rate = rospy.Rate(20)  # 设置运行频率为20Hz

    #######################################################################
    # 服务和话题初始化
    #######################################################################
    state_sub = rospy.Subscriber("mavros/state", State, callback=state_cb)
    get_pose = rospy.Subscriber("/myPose", Pose, callback=myPose_callback, queue_size=20)
    local_pose_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, queue_size=10)
    set_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)

    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
    rospy.wait_for_service("/mavros/set_mode")
    
    global set_mode_client
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)
    
    #######################################################################
    # 等待与飞控建立连接
    #######################################################################
    while not rospy.is_shutdown() and not current_state.connected:
        rate.sleep()

    #######################################################################
    # 主循环：发送控制指令
    #######################################################################
    MsgCounter = 0  # 消息发送计数器
    global offb_set_mode
    offb_set_mode = SetModeRequest(custom_mode='OFFBOARD')
    land_mode = SetModeRequest(custom_mode='POSCTL')
    arm_cmd = CommandBoolRequest(value=True)
    disarm_cmd = CommandBoolRequest(value=True)

    while not rospy.is_shutdown():
        # 检测是否需要降落,-0.1表示降落
        if pose.pose.position.z == -0.1:
            set_mode_client.call(land_mode)
            arming_client.call(disarm_cmd)
            break

        # 发布目标位置
        rospy.loginfo("SENT NEW POSE")
        set_pos_pub.publish(pose)
        MsgCounter += 1

        # 每40次循环尝试维持OFFBOARD模式并检查解锁状态
        if MsgCounter % 40 == 0:
            if current_state.mode != "OFFBOARD":
                if set_mode_client.call(offb_set_mode).mode_sent:
                    rospy.loginfo("OFFBOARD enabled")
            elif not current_state.armed:
                if arming_client.call(arm_cmd).success:
                    rospy.loginfo("Vehicle armed")
            MsgCounter = 0

        rospy.on_shutdown(on_shutdown)
        rate.sleep()
