#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Vector3
import math
import numpy
from tf.transformations import quaternion_from_euler, quaternion_multiply
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState

def vector_callback(vector):
    rospy.loginfo("Vector: x={:.2f}, y={:.2f}, z={:.2f}".format(vector.x, vector.y, vector.z))
    
    # 默认方向为Z轴正方向 (0, 0, 1)
    default_direction = [0, 0, 1]
    
    # 需要的方向
    target_direction = [vector.x, vector.y, vector.z]
    
    # 计算旋转的四元数
    q = quaternion_from_vector_to_vector(default_direction, target_direction)

    # 更新模型状态
    state = ModelState()
    state.model_name = 'stick_model'
    state.pose.position.x = 0
    state.pose.position.y = 0
    state.pose.position.z = 1.75
    state.pose.orientation.x = q[0]
    state.pose.orientation.y = q[1]
    state.pose.orientation.z = q[2]
    state.pose.orientation.w = q[3]
    
    try:
        set_state(state)
        rospy.loginfo("\tModel state updated successfully.")
    except rospy.ServiceException as e:
        rospy.loginfo("Service call failed: {}".format(e))

def quaternion_from_vector_to_vector(from_vec, to_vec):
    """计算从一个向量到另一个向量的旋转四元数"""
    v = numpy.cross(from_vec, to_vec)
    w = math.sqrt((numpy.linalg.norm(from_vec) ** 2) * (numpy.linalg.norm(to_vec) ** 2)) + numpy.dot(from_vec, to_vec)
    q = numpy.array([v[0], v[1], v[2], w])
    return q / numpy.linalg.norm(q)

def set_state(state):
    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        resp = set_model_state(state)
        return resp.success
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)

def listener():
    rospy.init_node('aeropointer/glove_model_listener', anonymous=True)
    rospy.Subscriber("/glove/vector", Vector3, vector_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
