from pymavlink import mavutil
import time
import math
import numpy as np

# 建立连接
master = mavutil.mavlink_connection('/dev/tty.usbmodem01', baud=115200)

# 等待接收飞控的心跳消息
master.wait_heartbeat()
print("Heartbeat from system (system %u component %u)\n\n" % (master.target_system, master.target_component))

# 发送心跳消息
master.mav.heartbeat_send(
    mavutil.mavlink.MAV_TYPE_GCS,
    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
    0, 0, 0
)

# 固定的VISION_POSITION_ESTIMATE数据
x = 1.0
y = 1.0
z = 1.0
roll = 0.0
pitch = 0.0
yaw = 0.0
covariance = [float('nan')] * 21  # 如果未知，设置为NaN
reset_counter = 0

def send_vision_position_estimate():
    # 获取当前时间戳（UNIX时间）
    usec = int(time.time() * 1e6)

    # 发送VISION_POSITION_ESTIMATE消息
    master.mav.vision_position_estimate_send(
        usec,
        x,
        y,
        z,
        roll,
        pitch,
        yaw,
        covariance,
        reset_counter
    )

    print(f"=========Sent VISION_POSITION_ESTIMATE at {usec}")

# 循环发送VISION_POSITION_ESTIMATE消息并接收消息
while True:
    start_time = time.time()
    # for _ in range(10):  # 每秒发送十次
    #     send_vision_position_estimate()
    #     time.sleep(0.1)  # 0.1秒间隔
    send_vision_position_estimate()

    # 无时间间隔地读取接收到的MAVLink消息
    while (time.time() - start_time) < 0.1:
        msg = master.recv_match(blocking=False)
        if msg:
            # print(msg.get_type())
            # 检查是否为视觉相关的数据
            if msg.get_type() == 'VISION_POSITION_ESTIMATE':
                print(f"Received VISION_POSITION_ESTIMATE: {msg}\n")
            elif msg.get_type() == 'VISION_POSITION_DELTA':
                print(f"Received VISION_POSITION_DELTA: {msg}\n")
            elif msg.get_type() == 'VISION_SPEED_ESTIMATE':
                print(f"Received VISION_SPEED_ESTIMATE: {msg}\n")
