from component.GroundStaion.utilities.UWBmodule import SerialToPose

def main():
    while True:
        converter = SerialToPose(port='/dev/ttyUSB0')  # 修改为你的实际串口设备名
        pose = converter.get_latest_pose()
        if pose == -1:
            print("RANGE_ERROR detected.")
        elif pose == 0:
            print("No solution detected.")
        elif pose is not None:
            x, y, z = pose
            print("Latest pose: x=%.2f, y=%.2f, z=%.2f" % (x, y, z))
        else:
            print("Failed to get the latest pose.")

if __name__ == '__main__':
    main()