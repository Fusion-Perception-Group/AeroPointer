#!/usr/bin/env python
# 根据四元数/senseglove/quaternion & /senseglove/position计算出Intersection Point
import rospy
from geometry_msgs.msg import Quaternion, Pose, Point
import math
import os

class IntersectionCalculator:
    def __init__(self):
        rospy.init_node('itsc_calc')
        self.sub_quaternion = rospy.Subscriber('/senseglove/quaternion', Quaternion, self.quaternion_callback)
        self.sub_position = rospy.Subscriber('/senseglove/position', Point, self.position_callback)
        self.pub = rospy.Publisher('/ap/intersection', Pose, queue_size=10)
        self.radius = 2.5 # Radius of the virtual sphere
        self.sphere_center = Point(0, 0, 1.75)  # Default sphere center

    def position_callback(self, position_msg):
        self.sphere_center = position_msg
        rospy.loginfo("Sphere center updated to: ({:.2f}, {:.2f}, {:.2f})".format(position_msg.x, position_msg.y, position_msg.z))

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
        # Equation of the sphere: (x - cx)^2 + (y - cy)^2 + (z - cz)^2 = radius^2
        # P0 is the sphere center

        # Solve for t in the equation ||(P0 - sphere_center) + t * direction_vector||^2 = radius^2

        # Direct computation considering sphere center
        cx, cy, cz = self.sphere_center.x, self.sphere_center.y, self.sphere_center.z
        t = self.radius / math.sqrt(sum(coord**2 for coord in direction_vector))  # Simplified for unit direction vector

        # Intersection point coordinates
        intersection_point = (cx + direction_vector[0]*t, cy + direction_vector[1]*t, cz + direction_vector[2]*t)
        return intersection_point

    def publish_intersection_point(self, intersection_point):
        pose_msg = Pose()
        pose_msg.position.x = intersection_point[0]
        pose_msg.position.y = intersection_point[1]
        pose_msg.position.z = intersection_point[2]
        pose_msg.orientation = Quaternion(0,0,0,1)
        self.pub.publish(pose_msg)
        # Print the intersection point
        os.system('clear')
        rospy.loginfo("Intersection Point: ({:.2f}, {:.2f}, {:.2f})".format(*intersection_point))

if __name__ == '__main__':
    intersection_calculator = IntersectionCalculator()
    rospy.spin()
