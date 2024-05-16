#!/usr/bin/env python
import rospy
import tf2_ros
import geometry_msgs
from geometry_msgs.msg import PoseStamped
from tf2_geometry_msgs import PoseStamped as TF2PoseStamped

def pose_callback(msg):
    # Convert PoseStamped to TF2PoseStamped
    tf2_msg = TF2PoseStamped()
    tf2_msg.header = msg.header
    tf2_msg.pose = msg.pose
    
    # Broadcast the transformation
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "map"
    t.child_frame_id = "base_link"
    t.transform.translation.x = msg.pose.position.x
    t.transform.translation.y = msg.pose.position.y
    t.transform.translation.z = msg.pose.position.z
    t.transform.rotation = msg.pose.orientation
    
    br.sendTransform(t)

def listener():
    rospy.init_node('tf2_base_link_broadcaster')
    rospy.Subscriber('/mavros/local_position/pose', PoseStamped, pose_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
