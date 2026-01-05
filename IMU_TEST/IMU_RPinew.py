import smbus2
import time
import struct
import socket

class BNO055:
    def __init__(self, address=0x28, bus_num=1):
        self.address = address
        self.bus = smbus2.SMBus(bus_num)
        self.init_bno055()
        self.wait_for_full_calibration()

    def init_bno055(self):
        # Operation mode to NDOF (0x0C)
        self.bus.write_byte_data(self.address, 0x3D, 0x0C)
        time.sleep(0.02)

    def read_calibration_status(self):
        cal = self.bus.read_byte_data(self.address, 0x35)
        sys = (cal >> 6) & 0x03
        gyro = (cal >> 4) & 0x03
        accel = (cal >> 2) & 0x03
        mag = cal & 0x03
        return sys, gyro, accel, mag

    def wait_for_full_calibration(self):
        print("calibrate sensor")
        while True:
            sys, gyro, accel, mag = self.read_calibration_status()
            print(f"Calibration → Sys:{sys}, Gyro:{gyro}, Accel:{accel}, Mag:{mag}", end="\r")
            if sys == 3 and gyro == 3 and accel == 3 and mag == 3:
                print("\n calibrate complete")
                break
            time.sleep(0.5)

    def read_vector(self, register, count=3):
        data = self.bus.read_i2c_block_data(self.address, register, count*2)
        values = []
        for i in range(count):
            lsb = data[i*2]
            msb = data[i*2+1]
            val = struct.unpack('<h', bytes([lsb, msb]))[0]
            values.append(val)
        return values
    
    def read_quaternion(self, register=0x20):
        data = bus.read_i2c_block_data(self.address, register, 8)
        q = []
        for i in range(4):
            lsb = data[i*2]
            msb = data[i*2+1]
            val = struct.unpack('<h', bytes([lsb, msb]))[0]
            q.append(val / (1<<14))
        return q

    def read_all(self):
        return {
            "time": time.time(),
            "accel": [x/100.0 for x in self.read_vector(0x08)], # m/s^2
            "mag": [x/16.0 for x in self.read_vector(0x0E)], # μT
            "gyro": [x/16.0 for x in self.read_vector(0x14)], # deg/s
            "euler": [x/16.0 for x in self.read_vector(0x1A)], # degrees
            "lin_acc": [x/100.0 for x in self.read_vector(0x28)], # m/s^2
            "gravity": [x/100.0 for x in self.read_vector(0x2E)], # m/s^2
            "quat": self.read_quaternion() # w, x, y, z
        }

class IMUClient:
    def __init__(self, ip, port, log_file="imu_data_log.txt"):
        self.sensor = BNO055()
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.client_socket.settimeout(0.02)
        self.addr = (ip, port)
        self.log = open(log_file, "w")
        self.start_time = time.time()
        self.log.write("time,ax,ay,az,mx,my,mz,gx,gy,gz,ex,ey,ez,lax,lay,laz,gvx,gvy,gvz,q0,q1,q2,q3\n")

    def run(self):
        try:
            while True:
                data = self.sensor.read_all()
                t = data["time"] - self.start_time
                line = "{:.3f},".format(t) + ",".join(
                    f"{v:.3f}" for group in [
                        data["accel"], data["mag"], data["gyro"],
                        data["euler"], data["lin_acc"], data["gravity"],
                        data["quat"]
                    ] for v in group
                ) + "\n"

                self.client_socket.sendto(line.encode(), self.addr)
                self.log.write(line)
                self.log.flush()

                print(f"Accel     : {data['accel']}")
                print(f"Magnet    : {data['mag']}")
                print(f"Gyro      : {data['gyro']}")
                print(f"Euler     : {data['euler']}")
                print(f"LinearAcc : {data['lin_acc']}")
                print(f"Gravity   : {data['gravity']}")
                print(f"Quaternion: {data['quat']}")
                print("-" * 50)

                try:
                    recv, _ = self.client_socket.recvfrom(1024)
                    if recv.decode() == "exit":
                        print("EXIT DEPLOYED. CLIENT SHUTDOWN")
                        break
                except socket.timeout:
                    pass

                time.sleep(0.04)
        finally:
            self.close()

    def close(self):
        self.client_socket.close()
        self.log.close()
        print("END CONNECTION")

# 실행 코드
if __name__ == "__main__":
    client = IMUClient(ip="192.168.137.1", port=22)
    # server의 ip 입력, port는 수정 가능
    client.run()
