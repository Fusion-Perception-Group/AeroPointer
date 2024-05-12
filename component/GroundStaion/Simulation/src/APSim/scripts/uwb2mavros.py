#!/usr/bin/env python
import rospy
import serial
import re
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

class SerialToPose:
    def __init__(self, port='/dev/ttyS0', baudrate=115200):
        self.ser = serial.Serial(port, baudrate, timeout=1)
        self.pose_pub = rospy.Publisher('~vision_pose/pose', PoseStamped, queue_size=10)

    def read_serial_data(self):
        line = self.ser.readline().decode('ascii', errors='ignore')
        rospy.loginfo("Received: %s" % line)
        return line

    def parse_data(self, data):
        try:
            if "no solution" in data:
                rospy.logwarn("No solution found in the data.")
                return None
            matches = re.findall(r"LO=\[([-\d\.]+),([-\d\.]+),([-\d\.]+)\]", data)
            if matches:
                x, y, z = map(float, matches[0])
                return x, y, z
            else:
                rospy.logwarn("No valid data found.")
                return None
        except Exception as e:
            rospy.logerr("Error parsing data: %s" % e)
            return None

    def publish_pose(self, x, y, z):
        pose = PoseStamped()
        pose.header = Header(stamp=rospy.Time.now(), frame_id='vision')
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        self.pose_pub.publish(pose)
        rospy.loginfo("Published pose: x=%.2f, y=%.2f, z=%.2f" % (x, y, z))

    def run(self):
        rospy.init_node('serial_to_pose', anonymous=True)
        rate = rospy.Rate(10)  # 10Hz
        while not rospy.is_shutdown():
            data = self.read_serial_data()
            xyz = self.parse_data(data)
            if xyz:
                self.publish_pose(*xyz)
            rate.sleep()

if __name__ == '__main__':
    converter = SerialToPose()
    converter.run()
