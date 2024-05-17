import serial
import re

# -1: RANGE_ERROR, 0: No Solution, 1: Valid Position, None: else invalid
class SerialToPose:
    def __init__(self, port='/dev/cu.usbserial-120', baudrate=115200):
        self.ser = serial.Serial(port=port, baudrate=baudrate, timeout=1)

    def read_serial_data(self):
        line = self.ser.readline().decode('ascii', errors='ignore').strip()
        return line

    def parse_data(self, data):
        try:
            if "no solution" in data:
                return 0
            if "RANGE_ERROR" in data:
                return -1
            matches = re.findall(r"LO=\[([-\d\.]+),([-\d\.]+),([-\d\.]+)\]", data)
            if matches:
                return tuple(map(float, matches[0]))
            else:
                # print("No valid data found.")
                return None
        except Exception as e:
            # print("Error parsing data: %s" % e)
            return None

    def get_latest_pose(self):
        data = self.read_serial_data()
        pose = self.parse_data(data)

        if pose == -1:
            return -1
        elif pose == 0:
            return 0
        elif pose is not None:
            return pose[0], pose[1], pose[2]
        else:
            return None

if __name__ == '__main__':
    UWBserial = SerialToPose(port='/dev/ttyUSB0') 
    while True:
        pose = UWBserial.get_latest_pose()
        if pose == -1:
            print("RANGE_ERROR detected.")
        elif pose == 0:
            print("No solution detected.")
        elif pose is not None:
            x, y, z = pose
            print("Valid position detected: x={}, y={}, z={}".format(x, y, z))