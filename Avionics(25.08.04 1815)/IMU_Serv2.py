import socket
import time
import threading
from collections import deque

# 평균 필터 함수
def compute_moving_average(queue):
    return sum(queue) / len(queue) if queue else 0.0

class IMUServer:
    def __init__(self, ip='10.96.215.26', port=5005):
        self.ip = ip
        self.port = port
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.server_socket.bind((self.ip, self.port))
        self.client_address = None
        self.measurement_active = False

        # 데이터 큐 초기화
        self.acc_x_queue = deque(maxlen=2)
        self.acc_y_queue = deque(maxlen=2)
        self.acc_z_queue = deque(maxlen=2)

        self.mag_x_queue = deque(maxlen=2)
        self.mag_y_queue = deque(maxlen=2)
        self.mag_z_queue = deque(maxlen=2)

        self.gyro_x_queue = deque(maxlen=2)
        self.gyro_y_queue = deque(maxlen=2)
        self.gyro_z_queue = deque(maxlen=2)

        self.heading_queue = deque(maxlen=2)
        self.roll_queue = deque(maxlen=2)
        self.pitch_queue = deque(maxlen=2)

        self.linacc_x_queue = deque(maxlen=2)
        self.linacc_y_queue = deque(maxlen=2)
        self.linacc_z_queue = deque(maxlen=2)

        self.gravity_x_queue = deque(maxlen=2)
        self.gravity_y_queue = deque(maxlen=2)
        self.gravity_z_queue = deque(maxlen=2)

        self.quat_w_queue = deque(maxlen=2)
        self.quat_x_queue = deque(maxlen=2)
        self.quat_y_queue = deque(maxlen=2)
        self.quat_z_queue = deque(maxlen=2)

        self.last_index = None
        self.last_timestamp = None
        self.logged_header = False

        print(f"📱 서버 실행 중... {self.ip}:{self.port}")

    def wait_for_client(self):
        print("⌛ 클라이언트 접속 대기 중...")
        _, address = self.server_socket.recvfrom(1024)
        self.client_address = address
        print("✅ 클라이언트 접속됨:", address[0])

    def monitor_keyboard_input(self):
        while True:
            try:
                cmd = input().strip().lower()
                if cmd == "start":
                    print("📥 입력 명령: start")
                    self.measurement_active = True
                elif cmd == "stop":
                    print("🔕 입력 명령: stop")
                    self.measurement_active = False
                elif cmd == "exit":
                    print("📥 입력 명령: exit")
                    self.send_exit_signal()
                    break
            except EOFError:
                break

    def parse_message(self, message):
        try:
            values = message.strip().split(',')
            if len(values) < 23:
                raise ValueError(f"필드 개수가 부족합니다. ({len(values)}개 필드 수신됨)")

            index = int(values[0])
            timestamp = float(values[1])
            floats = [float(v) for v in values[2:]]

            data = {
                "index": index,
                "timestamp": timestamp,
                "accel": floats[0:3],
                "mag": floats[3:6],
                "gyro": floats[6:9],
                "euler": floats[9:12],
                "lin_acc": floats[12:15],
                "gravity": floats[15:18],
                "quat": floats[18:22]
            }

            if self.last_index is not None and index != self.last_index + 1:
                print(f"⚠️ 인데크 누락! {self.last_index} → {index}")

            self.last_index = index
            self.last_timestamp = timestamp
            return data

        except Exception as e:
            print("❗ 메시지 파싱 오류:", e)
            return None

    def store_to_queues(self, data):
        self.acc_x_queue.append(data['accel'][0])
        self.acc_y_queue.append(data['accel'][1])
        self.acc_z_queue.append(data['accel'][2])

        self.mag_x_queue.append(data['mag'][0])
        self.mag_y_queue.append(data['mag'][1])
        self.mag_z_queue.append(data['mag'][2])

        self.gyro_x_queue.append(data['gyro'][0])
        self.gyro_y_queue.append(data['gyro'][1])
        self.gyro_z_queue.append(data['gyro'][2])

        self.heading_queue.append(data['euler'][0])
        self.roll_queue.append(data['euler'][1])
        self.pitch_queue.append(data['euler'][2])

        self.linacc_x_queue.append(data['lin_acc'][0])
        self.linacc_y_queue.append(data['lin_acc'][1])
        self.linacc_z_queue.append(data['lin_acc'][2])

        self.gravity_x_queue.append(data['gravity'][0])
        self.gravity_y_queue.append(data['gravity'][1])
        self.gravity_z_queue.append(data['gravity'][2])

        self.quat_w_queue.append(data['quat'][0])
        self.quat_x_queue.append(data['quat'][1])
        self.quat_y_queue.append(data['quat'][2])
        self.quat_z_queue.append(data['quat'][3])

    def display_filtered_data(self):
        while True:
            if not self.measurement_active:
                time.sleep(0.1)
                continue

            if len(self.acc_x_queue) > 0:
                values = [
                    compute_moving_average(self.acc_x_queue),
                    compute_moving_average(self.acc_y_queue),
                    compute_moving_average(self.acc_z_queue),
                    compute_moving_average(self.mag_x_queue),
                    compute_moving_average(self.mag_y_queue),
                    compute_moving_average(self.mag_z_queue),
                    compute_moving_average(self.gyro_x_queue),
                    compute_moving_average(self.gyro_y_queue),
                    compute_moving_average(self.gyro_z_queue),
                    compute_moving_average(self.heading_queue),
                    compute_moving_average(self.roll_queue),
                    compute_moving_average(self.pitch_queue),
                    compute_moving_average(self.linacc_x_queue),
                    compute_moving_average(self.linacc_y_queue),
                    compute_moving_average(self.linacc_z_queue),
                    compute_moving_average(self.gravity_x_queue),
                    compute_moving_average(self.gravity_y_queue),
                    compute_moving_average(self.gravity_z_queue),
                    compute_moving_average(self.quat_w_queue),
                    compute_moving_average(self.quat_x_queue),
                    compute_moving_average(self.quat_y_queue),
                    compute_moving_average(self.quat_z_queue)
                ]

                line = f"{self.last_index},{self.last_timestamp:.3f}," + \
                       ",".join(f"{v:.3f}" for v in values)

                print(line)

                with open("filtered_data.txt", "a", encoding="utf-8") as f:
                    if not self.logged_header:
                        header = [
                            "Index", "Time",
                            "accel_x", "accel_y", "accel_z",
                            "mag_x", "mag_y", "mag_z",
                            "gyro_x", "gyro_y", "gyro_z",
                            "heading", "roll", "pitch",
                            "linacc_x", "linacc_y", "linacc_z",
                            "gravity_x", "gravity_y", "gravity_z",
                            "quat_w", "quat_x", "quat_y", "quat_z"
                        ]
                        f.write(",".join(header) + "\n")
                        self.logged_header = True
                    f.write(line + "\n")

            time.sleep(0.06)

    def send_exit_signal(self):
        for _ in range(200):
            self.server_socket.sendto(b"exit", self.client_address)
            time.sleep(0.05)

    def run(self):
        threading.Thread(target=self.display_filtered_data, daemon=True).start()
        threading.Thread(target=self.monitor_keyboard_input, daemon=True).start()

        try:
            self.wait_for_client()
            while True:
                data, _ = self.server_socket.recvfrom(1024)
                message = data.decode("UTF-8").strip()

                if message.lower() == "exit":
                    print("🚪 종료 메시지 수신")
                    break

                if self.measurement_active:
                    parsed = self.parse_message(message)
                    if parsed:
                        self.store_to_queues(parsed)

        except KeyboardInterrupt:
            print("\n🚑 오류 종료")
            self.send_exit_signal()
        finally:
            self.close()

    def close(self):
        self.server_socket.close()
        print("🔧 서버 시작 종료")

if __name__ == "__main__":
    server = IMUServer()
    server.run()
