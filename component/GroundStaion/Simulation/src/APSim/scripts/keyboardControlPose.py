#!/usr/bin/env python

import rospy
import os
from geometry_msgs.msg import Pose, Point, Quaternion
import sys, select, termios, tty

Spawn_X = 0.73
Spawn_Y = 0.88

# 定义键盘控制按键
MOVE_BINDINGS = {
    'w': (1, 0),
    's': (-1, 0),
    'a': (0, 1),
    'd': (0, -1),
}

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main():
    rospy.init_node('keyboard_teleop')
    pub = rospy.Publisher('/myPose', Pose, queue_size=10)
    rate = rospy.Rate(10)

    pose=   Pose(
        position=Point(2 - Spawn_X, 0.8842 - Spawn_Y, 1),
        # orientation=Quaternion(0, 0, math.sin(math.pi / 4), math.sin(math.pi / 4)) #y轴方向，北方
        orientation=Quaternion(0, 0, 0, 1) # 默认x轴方向，东方
    )
    
    pose.orientation.w = 1.0

    global settings
    settings = termios.tcgetattr(sys.stdin)

    print("Control Your Robot!")
    print("Use 'w' and 's' to control x axis, 'a' and 'd' to control y axis")

    try:
        while not rospy.is_shutdown():
            key = getKey()
            if key in MOVE_BINDINGS.keys():
                x_change, y_change = MOVE_BINDINGS[key]
                pose.position.x += x_change * 0.1
                pose.position.y += y_change * 0.1
                os.system('clear')
                rospy.loginfo("Publishing pose: %s", pose)
                pub.publish(pose)
            elif key == '\x03':  # Control-C to exit
                break
            rate.sleep()
    except Exception as e:
        rospy.logerr("Error: %s", e)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__ == "__main__":
    main()
