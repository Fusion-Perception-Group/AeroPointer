import rospy
from sensor_msgs.msg import LaserScan
from mavros_msgs.msg import SetPositionTarget
from geometry_msgs.msg import Point, Quaternion
import math

def scan_callback(msg):
    avoidance_vector_x, avoidance_vector_y = 0.0, 0.0
    avoid = False
    d0, k = 0.55, 0.5  # 安全距离和控制参数

    # 遍历激光雷达数据
    
    for i in range(len(msg.ranges)):
        if d0 > msg.ranges[i] > 0.4:  # 当障碍物距离在0.35米到3米之间
            avoid = True
            angle = msg.angle_min + i * msg.angle_increment
            x = math.cos(angle)
            y = math.sin(angle)
            U = -0.5 * k * ((1/msg.ranges[i]) - (1/d0))**2  # 计算潜在的避障力

            avoidance_vector_x += x * U
            avoidance_vector_y += y * U

    # 如果需要避障，更新目标位置
    if avoid:
        magnitude = math.sqrt(avoidance_vector_x**2 + avoidance_vector_y**2)
        if magnitude > 3:
            scale = 3 / magnitude
            avoidance_vector_x *= scale
            avoidance_vector_y *= scale

        # 使用 MAVROS 更新无人机的位置
        current_pos = get_current_location()  # 定义此函数以获取当前位置
        set_position(current_pos.x + avoidance_vector_x, current_pos.y + avoidance_vector_y, current_pos.z)  # 定义此函数以设置新位置

def main():
    rospy.init_node('laser_scan_avoidance')
    rospy.Subscriber('/scan', LaserScan, scan_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
