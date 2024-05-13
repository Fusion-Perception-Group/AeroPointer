import serial
import re

class SerialToPose:
    def __init__(self, port='/dev/cu.usbserial-120', baudrate=115200):
        self.ser = serial.Serial(port=port, baudrate=baudrate, timeout=1)
        self.error_count = 0

    def read_serial_data(self):
        line = self.ser.readline().decode('ascii', errors='ignore').strip()
        # print("Received: %s" % line)
        return line

    def parse_data(self, data):
        try:
            if "no solution" in data:
                print("No solution found in the data.")
                return None
            if "RANGE_ERROR" in data:
                self.error_count += 1
                if self.error_count >= 3:
                    print("RANGE_ERROR")
                    return
                    
            else:
                self.error_count = 0

            matches = re.findall(r"LO=\[([-\d\.]+),([-\d\.]+),([-\d\.]+)\]", data)
            if matches:
                return map(float, matches[0])
            else:
                print("No valid data found.")
                return None
        except Exception as e:
            print("Error parsing data: %s" % e)
            return None

    def run(self):
        while True:
            data = self.read_serial_data()
            xyz = self.parse_data(data)
            if xyz:
                x, y, z = xyz
                print("Parsed pose: x=%.2f, y=%.2f, z=%.2f" % (x, y, z))

if __name__ == '__main__':
    # 注意：更改端口为适合你的系统的端口
    converter = SerialToPose(port='/dev/cu.usbserial-120')  # 修改为你的实际串口设备名
    converter.run()