import smbus2
import time
import struct
import socket
from collections import deque
import numpy as np
import RPi.GPIO as GPIO
from IMU_Para_v2 import BNO055, QuaternionFilter, Servo

# ===== IMUClient 클래스 =====
class IMUClient:
    def __init__(self, ip, port, print_raw=False, log_filename="imu_raw_log.txt"):
        self.sensor = BNO055()
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.client_socket.settimeout(0.02)
        self.addr = (ip, port)
        self.start_time = time.time()

        self.q_filter = QuaternionFilter(maxlen=2)
        self.print_raw = print_raw
        self.log_filename = log_filename
        self.log_file = open(self.log_filename, "w", encoding="utf-8")
        self.index = 0

        headers = [
            "Index", "Time",
            "accel_x", "accel_y", "accel_z",
            "mag_x", "mag_y", "mag_z",
            "gyro_x", "gyro_y", "gyro_z",
            "euler_x", "euler_y", "euler_z",
            "linacc_x", "linacc_y", "linacc_z",
            "gravity_x", "gravity_y", "gravity_z",
            "quat_w", "quat_x", "quat_y", "quat_z"
        ]
        self.log_file.write(",".join(headers) + "\n")

    def run(self):
        try:
            while True:
                data = self.sensor.read_all()
                t = data["time"] - self.start_time
                raw_q = data["quat"]
                filtered_q = self.q_filter.update(raw_q)

                # 전송 라인에 Index 포함하도록 수정
                line = "{},{:.3f},".format(self.index, t) + ",".join(
                    f"{v:.3f}" for group in [
                        data["accel"], data["mag"], data["gyro"],
                        data["euler"], data["lin_acc"], data["gravity"],
                        data["quat"]
                    ] for v in group
                )
                self.client_socket.sendto(line.encode(), self.addr)

                # 로컬 로그 저장 (6자리 정밀도 유지)
                log_line = "{},{:.3f},".format(self.index, t) + ",".join(
                    f"{v:.6f}" for group in [
                        data["accel"], data["mag"], data["gyro"],
                        data["euler"], data["lin_acc"], data["gravity"],
                        raw_q
                    ] for v in group
                ) + "\n"
                self.log_file.write(log_line)
                self.log_file.flush()

                if self.print_raw:
                    print(f"Quaternion(raw)     : {raw_q}")
                print(f"Quaternion(filtered): {[round(v, 6) for v in filtered_q]}")

                try:
                    recv, _ = self.client_socket.recvfrom(1024)
                    if recv.decode() == "exit":
                        print("EXIT DEPLOYED. CLIENT SHUTDOWN")
                        break
                except socket.timeout:
                    pass

                self.index += 1
                time.sleep(0.04)

        finally:
            self.close()

    def close(self):
        self.client_socket.close()
        self.log_file.close()
        print("END CONNECTION")

# ===== 실행 =====
if __name__ == "__main__":
    client = IMUClient(ip="10.96.215.26", port=5005, print_raw=False) # 192.168.137.1, port=22
    client.run()
