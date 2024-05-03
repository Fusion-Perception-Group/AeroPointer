#!/usr/bin/python3
import serial
import re
from pymavlink import mavutil

# 打开串口
ser = serial.Serial('/dev/ttyUSB0', 115200)

# 创建MAVLink连接（替换为适当的目标地址和端口）
mav = mavutil.mavlink_connection('udpin:localhost:14550')

def send_vision_position(x, y, z):
    # 发送VISION_POSITION_ESTIMATE消息
    mav.mav.vision_position_estimate_send(
        mav.mavlink10.MAVLink_time_usec_message(mavutil.mavlink.get_system_time()), # 时间戳
        x,  # X坐标
        y,  # Y坐标
        z   # Z坐标
    )

def matchPose(line):
    match = re.search(r'LO=\[(.*?),(.*?),(.*?)\]', line)
    if match:
        x = float(match.group(1))
        y = float(match.group(2))
        z = float(match.group(3))
        return x, y, z
    return None

while True:
    line = ser.readline().decode('utf-8')
    match = re.search(r'LO=\[(.*?),(.*?),(.*?)\]', line)
    if match:
        x = float(match.group(1))
        y = float(match.group(2))
        z = float(match.group(3))
        send_vision_position(x, y, z)
        
