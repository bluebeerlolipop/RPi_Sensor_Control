# Final/raspi.py
import smbus2
import time
import struct
import socket
import threading
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
    def __init__(self, address=0x28, bus_num=1, calibrate=True, client_socket=None, start_event=None):
        self.address = address
        self.bus = smbus2.SMBus(bus_num)
        self.init_bno055()

        self.client_socket = client_socket
        self.start_event = start_event
        if self.client_socket:
            self.client_socket.settimeout(0.02)

        self.start_event = start_event
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

            command = ""
            if self.client_socket is not None:
                try:
                    recv, _ = self.client_socket.recvfrom(1024)
                    command = recv.decode().strip()
                except socket.timeout:
                    command = ""

            print(f"Calibration → Sys:{sys}, Gyro:{gyro}, Accel:{accel}, Mag:{mag}", end="\r")

            if (sys >= 2 and gyro == 3 and accel == 3 and mag == 3) or command == "start" or (self.start_event and self.start_event.is_set()):
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

# ===== FAIL 신호 리스너 =====
def listen_for_fail_signal(callback):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", 6006)) # para.py에서 지정한 포트
    print("📡 FAIL 신호 대기 중...")
    while True:
        data, _ = sock.recvfrom(1024)
        if data.decode().strip().upper() == "FAIL":
            print("🚨 FAIL 신호 수신!")
            callback()
            break

# ===== IMUClient 클래스 =====
class IMUClient:
    def __init__(self, ip, port, stport, print_raw=False, log_filename="Raspi_data.txt", start_event=None):
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.client_socket.settimeout(0.02)
        self.addr = (ip, port)
        self.start_time = time.time()
        self.print_raw = print_raw
        self.index = 0
        self.Loop = False
        self.Con_Lost = True
        self.running = True # 루프 종료 플래그
        self.start_event = start_event
        self.target_port = None


        self.client_socket.bind(("", stport))

        # 헤더를 먼저 서버에 전송
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
        header_str = ",".join(headers)
        self.client_socket.sendto(header_str.encode(), self.addr)

        self.sensor = None
        self.q_filter = QuaternionFilter(maxlen=2)

        self.log_filename = log_filename
        self.log_file = open(self.log_filename, "w", encoding="utf-8")
        self.log_file.write(header_str + "\n")
    
    def stop_and_send(self):
        self.running = False # run() 루프 탈출
        self.log_file.flush()
        self.log_file.close()
        self.send_log_file(override_port = 5010)
    
    def send_log_file(self, override_port=None):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.target_port = override_port if override_port else self.addr[1]
        try:
            with open(self.log_filename, "r", encoding="utf-8") as f:
                for line in f:
                    sock.sendto(line.encode(), self.addr)
                    time.sleep(0.01)
            print("✅ 로그 파일 전송 완료")
        except Exception as e:
            print(f"⚠️ 전송 실패: {e}")
        finally:
            sock.close()

    def run(self):
        try:
            if self.sensor is None:
                self.sensor = BNO055(client_socket=self.client_socket, start_event=self.start_event)

            while self.running:
                try:
                    recv, _ = self.client_socket.recvfrom(1024)
                    command = recv.decode().strip()
                    if command == "start":
                        self.start_time = time.time()      # start 명령 이후의 시간
                        self.index = 0                     # start 받으면 index 초기화
                        self.Loop = True
                        print("start input! IMU DATA COLLECTION STARTED")
                    elif command == "exit":
                        print("EXIT DEPLOYED. CLIENT SHUTDOWN")
                        break
                except socket.timeout:
                    pass

                while self.Loop and self.running:
                    data = self.sensor.read_all()
                    t = data["time"] - self.start_time    # start 명령 이후의 시간
                    raw_q = data["quat"]
                    filtered_q = self.q_filter.update(raw_q)

                    line = "{},{:.3f},".format(self.index, t) + ",".join(
                        f"{v:.3f}" for group in [
                            data["accel"], data["mag"], data["gyro"],
                            data["euler"], data["lin_acc"], data["gravity"],
                            data["quat"]
                        ] for v in group
                    )
                    try:
                        self.client_socket.sendto(line.encode(), self.addr)
                    except Exception as e:
                        if self.Con_Lost:
                            self.index = 0
                            self.Con_Lost = False
                        print(f"⚠️ 데이터 전송 실패: {e}")

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


                    self.index += 1
                    time.sleep(0.04)

        finally:
            self.close()

    def close(self):
        self.log_file.close()
        print("END CONNECTION")

# ===== 실행 =====
if __name__ == "__main__":
    start_event = threading.Event()
    
    client = IMUClient(ip="192.168.137.1", port=5005, stport=5004, print_raw=False, start_event = start_event) # 192.168.137.1, port=22
    
    listener = threading.Thread(target=listen_for_fail_signal, args=(client.stop_and_send,))
    listener.daemon = True
    listener.start()
    
    client.run()
