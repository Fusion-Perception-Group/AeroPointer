#!/usr/bin/env python

import rospy
import os
from geometry_msgs.msg import Quaternion, Vector3, Point
from sensor_msgs.msg import Imu
import math

class SenseGlove:
    def __init__(self):
        rospy.init_node('senseglove')
        ############Position
        self.position_pub = rospy.Publisher('/senseglove/position', Point, queue_size=10)
        self.position = Point(x=0, y=0, z=1.75)
        ############Vector
        self.vec_pub = rospy.Publisher('/senseglove/vector', Vector3, queue_size=10)
        # Dummy vector data (1, 1, 1)
        self.vector = [1, 1, 1]
        #测试用flag
        self.flag_Xreverse = False
        self.flag_Yreverse = False
        ############Quaternion
        self.quat_pub = rospy.Publisher('/senseglove/quaternion', Quaternion, queue_size=10)
        self.imu_data = Imu()
        self.imu_data.orientation_covariance = [0.0]*9  # Set covariance to zero for now
        
    def start_publishing(self):
        rate = rospy.Rate(10)  # 5Hz
        while not rospy.is_shutdown():
            self.update_vector_data()
            self.publish()
            self.update_vector_data()
            rate.sleep()

    def update_vector_data(self):
        #测试用，增加减少
        self.updateIncrement(True,True)
        magnitude = math.sqrt(sum(x**2 for x in self.vector))
        self.normalized_vector = tuple(x / magnitude for x in self.vector)  # Normalizing the vector

    def publish(self):
        # Publish the normalized vector
        normalized_vector_msg = Vector3(x=self.normalized_vector[0], y=self.normalized_vector[1], z=self.normalized_vector[2])
        self.vec_pub.publish(normalized_vector_msg)

        # Publish the quaternion converted from vector
        quaternion = self.vector_to_quaternion(self.normalized_vector)
        self.imu_data.orientation = quaternion
        self.quat_pub.publish(self.imu_data.orientation)

        # Publish the default position
        self.position_pub.publish(self.position)

        # Print the current state
        self.print_orientation()

    def vector_to_quaternion(self, vector):
        # Dummy implementation for vector to quaternion conversion
        # This assumes the vector points along what will be the rotated x-axis
        angle = math.acos(vector[0])  # Assuming rotation around original x-axis
        half_angle = angle / 2
        sin_half_angle = math.sin(half_angle)

        return Quaternion(
            x=sin_half_angle * vector[0],
            y=sin_half_angle * vector[1],
            z=sin_half_angle * vector[2],
            w=math.cos(half_angle)
        )

    def print_orientation(self):
        os.system('clear')  # Clear terminal screen
        rospy.loginfo(rospy.get_name())
        # 打印位置信息
        rospy.loginfo("SENT:Position : x={:.2f}, y={:.2f}, z={:.2f}".format(
            self.position.x, self.position.y, self.position.z
        ))
        rospy.loginfo(">>>>>Original Vector: x={:.2f}, y={:.2f}, z={:.2f}".format(self.vector[0],self.vector[1],self.vector[2]))
        # 打印单位向量
        rospy.loginfo("SENT:Normalized Vector: x={:.2f}, y={:.2f}, z={:.2f}".format(
            self.normalized_vector[0], self.normalized_vector[1], self.normalized_vector[2]
        ))
        # 打印由单位向量计算出的四元数
        rospy.loginfo("SENT:Quaternion: x={:.2f}, y={:.2f}, z={:.2f}, w={:.2f}".format(
            self.imu_data.orientation.x,
            self.imu_data.orientation.y,
            self.imu_data.orientation.z,
            self.imu_data.orientation.w
        ))
        
    def updateIncrement(self, ifautoTestVec = False,ifautoTestPos = False):
        # 更新 x 值
        if ifautoTestVec:
            Increment = 0.01
            if self.vector[0] >= 5:
                self.flag_Xreverse = True
            elif self.vector[0] <= -5:
                self.flag_Xreverse = False

            self.vector[0] += -Increment if self.flag_Xreverse else Increment

            # 更新 y 值
            if self.vector[1] >= 5:
                self.flag_Yreverse = True
            elif self.vector[1] <= -5:
                self.flag_Yreverse = False

            self.vector[1] += -2*Increment if self.flag_Yreverse else 2*Increment
        
        if ifautoTestPos:  
            Increment = 0.01
            if self.position.y >= 5:
                self.flag_Xreverse = True
            elif self.position.y <= -5:
                self.flag_Xreverse = False

            self.position.y += -Increment if self.flag_Xreverse else Increment

            # 更新 y 值
            if self.position.x >= 5:
                self.flag_Yreverse = True
            elif self.position.x <= -5:
                self.flag_Yreverse = False

            self.position.x += -2*Increment if self.flag_Yreverse else 2*Increment
    
if __name__ == '__main__':
    sg = SenseGlove()
    sg.start_publishing()
