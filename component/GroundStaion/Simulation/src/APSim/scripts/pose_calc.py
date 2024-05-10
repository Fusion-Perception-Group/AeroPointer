#!/usr/bin/env python
# 根据四元数/glove/quaternion计算出目标坐标
import rospy
from geometry_msgs.msg import Quaternion, Pose
import math
import os

class IntersectionCalculator:
    def __init__(self):
        rospy.init_node('aeropointer/pose_calc')
        self.sub = rospy.Subscriber('/glove/quaternion', Quaternion, self.quaternion_callback)
        self.pub = rospy.Publisher('/myPose', Pose, queue_size=10)
        self.radius = 5  # Radius of the virtual sphere

    def quaternion_callback(self, quaternion_msg):
        # Convert Quaternion to direction vector
        direction_vector = self.quaternion_to_direction_vector(quaternion_msg)

        # Find intersection point with virtual sphere
        intersection_point = self.find_intersection_with_sphere(direction_vector)

        # Publish intersection point as Pose
        self.publish_intersection_point(intersection_point)

    def quaternion_to_direction_vector(self, quaternion_msg):
        # Convert Quaternion to direction vector
        # For simplicity, assume the quaternion represents a direction vector directly
        # You might need to adjust this conversion based on your specific use case
        return (quaternion_msg.x, quaternion_msg.y, quaternion_msg.z)

    def find_intersection_with_sphere(self, direction_vector):
        # Equation of the ray: P = P0 + t * direction_vector
        # Equation of the sphere: x^2 + y^2 + z^2 = radius^2

        # Solve for t in the equation ||P0 + t * direction_vector||^2 = radius^2
        # where P0 is the origin (0, 0, 0)

        # Since the origin is (0, 0, 0), the equation reduces to ||t * direction_vector||^2 = radius^2
        # => t^2 * ||direction_vector||^2 = radius^2
        # => t^2 = radius^2 / ||direction_vector||^2
        # => t = sqrt(radius^2 / ||direction_vector||^2)

        magnitude_squared = sum(coord**2 for coord in direction_vector)
        t = math.sqrt(self.radius**2 / magnitude_squared)

        # Intersection point coordinates
        intersection_point = (direction_vector[0]*t, direction_vector[1]*t, direction_vector[2]*t)
        return intersection_point

    def publish_intersection_point(self, intersection_point):
        pose_msg = Pose()
        pose_msg.position.x = intersection_point[0]
        pose_msg.position.y = intersection_point[1]
        pose_msg.position.z = intersection_point[2] + 1.75 #1.75是手套位置，没有传递进来。
        self.pub.publish(pose_msg)
        # Print the intersection point
        os.system('clear')
        rospy.loginfo("Intersection Point: ({:.2f}, {:.2f}, {:.2f})".format(*intersection_point))

if __name__ == '__main__':
    intersection_calculator = IntersectionCalculator()
    rospy.spin()
