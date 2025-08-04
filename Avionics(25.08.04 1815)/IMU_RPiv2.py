import smbus2
import time
import struct
import socket
from collections import deque

# ===== 평균 필터 함수 =====
def compute_moving_average(queue):
    if len(queue) == 0:
        return 0.0
    return sum(queue) / len(queue)

# ===== Quaternion 이동평균 필터 클래스 =====
class QuaternionFilter:
    def __init__(self, maxlen=2):
        self.q_w_queue = deque(maxlen=maxlen)
        self.q_x_queue = deque(maxlen=maxlen)
        self.q_y_queue = deque(maxlen=maxlen)
        self.q_z_queue = deque(maxlen=maxlen)

    def update(self, quat):
        self.q_w_queue.append(quat[0])
        self.q_x_queue.append(quat[1])
        self.q_y_queue.append(quat[2])
        self.q_z_queue.append(quat[3])
        return self.get_filtered()

    def get_filtered(self):
        return [
            compute_moving_average(self.q_w_queue),
            compute_moving_average(self.q_x_queue),
            compute_moving_average(self.q_y_queue),
            compute_moving_average(self.q_z_queue),
        ]

# ===== BNO055 센서 클래스 =====
class BNO055:
    def __init__(self, address=0x28, bus_num=1, calibrate=True):
        self.address = address
        self.bus = smbus2.SMBus(bus_num)
        self.init_bno055()
        if calibrate:
            self.wait_for_full_calibration()

    def init_bno055(self):
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
        print("🔄 센서 보정 중...")
        while True:
            sys, gyro, accel, mag = self.read_calibration_status()
            print(f"Calibration → Sys:{sys}, Gyro:{gyro}, Accel:{accel}, Mag:{mag}", end="\r")
            if sys >= 2 and gyro == 3 and accel == 3 and mag == 3:
                print("\n✅ 센서 보정 완료!")
                break
            time.sleep(0.5)

    def read_vector(self, register, count=3):
        data = self.bus.read_i2c_block_data(self.address, register, count * 2)
        values = []
        for i in range(count):
            lsb = data[i * 2]
            msb = data[i * 2 + 1]
            val = struct.unpack('<h', bytes([lsb, msb]))[0]
            values.append(val)
        return values

    def read_quaternion(self, register=0x20):
        data = self.bus.read_i2c_block_data(self.address, register, 8)
        q = []
        for i in range(4):
            lsb = data[i * 2]
            msb = data[i * 2 + 1]
            val = struct.unpack('<h', bytes([lsb, msb]))[0]
            q.append(val / (1 << 14))
        return q

    def read_all(self):
        return {
            "time": time.time(),
            "accel":   [x / 100.0 for x in self.read_vector(0x08)],
            "mag":     [x / 16.0  for x in self.read_vector(0x0E)],
            "gyro":    [x / 16.0  for x in self.read_vector(0x14)],
            "euler":   [x / 16.0  for x in self.read_vector(0x1A)],
            "lin_acc": [x / 100.0 for x in self.read_vector(0x28)],
            "gravity": [x / 100.0 for x in self.read_vector(0x2E)],
            "quat":    self.read_quaternion()
        }

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
