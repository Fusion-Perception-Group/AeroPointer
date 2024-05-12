#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Vector3, Point
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
import math
import numpy


class ModelUpdater:
    def __init__(self):
        # 初始化节点
        rospy.init_node('glove_model_listener', anonymous=True)
        
        # 订阅向量和位置数据
        rospy.Subscriber("/senseglove/vector", Vector3, self.vector_callback, queue_size=30)
        rospy.Subscriber("/senseglove/position", Point, self.position_callback, queue_size=30)
        
        # 初始位置
        self.current_position = Point(x=0, y=0, z=1.75)
        self.current_vector = Vector3(x=0, y=0, z=1)  # 初始向量设定为Z轴方向

        # 更新频率设置为 20Hz
        self.timer = rospy.Timer(rospy.Duration(1.0/120), self.excuteUpdate)

    def position_callback(self, pos):
        self.current_position = pos
        rospy.loginfo("Start position to: x={:.2f}, y={:.2f}, z={:.2f}".format(pos.x, pos.y, pos.z))

    def vector_callback(self, vector):
        self.current_vector = vector
        rospy.loginfo("Vector: x={:.2f}, y={:.2f}, z={:.2f}".format(vector.x, vector.y, vector.z))

    def excuteUpdate(self, event):
        # 计算旋转的四元数
        q = self.quaternion_from_vector_to_vector([0, 0, 1], [self.current_vector.x, self.current_vector.y, self.current_vector.z])
        
        # 更新模型状态
        state = ModelState()
        state.model_name = 'stick_model'
        state.pose.position = self.current_position
        state.pose.orientation.x = q[0]
        state.pose.orientation.y = q[1]
        state.pose.orientation.z = q[2]
        state.pose.orientation.w = q[3]
        
        self.set_state(state)

    def quaternion_from_vector_to_vector(self, from_vec, to_vec):
        """计算从一个向量到另一个向量的旋转四元数"""
        v = numpy.cross(from_vec, to_vec)
        w = math.sqrt((numpy.linalg.norm(from_vec) ** 2) * (numpy.linalg.norm(to_vec) ** 2)) + numpy.dot(from_vec, to_vec)
        q = numpy.array([v[0], v[1], v[2], w])
        return q / numpy.linalg.norm(q)

    def set_state(self, state):
        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            resp = set_model_state(state)
            if resp.success:
                pass
                # rospy.loginfo("Model state updated successfully.")
            else:
                rospy.loginfo("Failed to update model state.")
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)

    def spin(self):
        rospy.spin()

if __name__ == '__main__':
    model_updater = ModelUpdater()
    model_updater.spin()
