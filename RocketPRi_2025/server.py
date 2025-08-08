# Final/server.py
import socket
import time
import threading
from collections import deque

# ----------------------- 공통 함수 -----------------------
def compute_moving_average(queue):
    return sum(queue) / len(queue) if queue else 0.0

# ----------------------- 서버 클래스 -----------------------
class IMUServer:
    def __init__(self, ip='10.14.170.26', port=5005):
        self.ip = ip
        self.port = port
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.server_socket.bind((self.ip, self.port))
        self.client_address = None
        self.measurement_active = False
        self.last_logged_index = None

        # ---- 각 센서별 deque 초기화 ----
        self.acc_x_queue, self.acc_y_queue, self.acc_z_queue = deque(maxlen=2), deque(maxlen=2), deque(maxlen=2)
        self.mag_x_queue, self.mag_y_queue, self.mag_z_queue = deque(maxlen=2), deque(maxlen=2), deque(maxlen=2)
        self.gyro_x_queue, self.gyro_y_queue, self.gyro_z_queue = deque(maxlen=2), deque(maxlen=2), deque(maxlen=2)
        self.heading_queue, self.roll_queue, self.pitch_queue = deque(maxlen=2), deque(maxlen=2), deque(maxlen=2)
        self.linacc_x_queue, self.linacc_y_queue, self.linacc_z_queue = deque(maxlen=2), deque(maxlen=2), deque(maxlen=2)
        self.gravity_x_queue, self.gravity_y_queue, self.gravity_z_queue = deque(maxlen=2), deque(maxlen=2), deque(maxlen=2)
        self.quat_w_queue, self.quat_x_queue, self.quat_y_queue, self.quat_z_queue = (
            deque(maxlen=2), deque(maxlen=2), deque(maxlen=2), deque(maxlen=2)
        )

        self.last_index = None
        self.last_timestamp = None
        self.logged_header = False

        print(f"📱 서버 실행 중... {self.ip}:{self.port}")

    # ------------------- 클라이언트 대기 -------------------
    def wait_for_client(self):
        print("⌛ 클라이언트 접속 대기 중...")
        _, address = self.server_socket.recvfrom(1024)
        self.client_address = address
        print("✅ 클라이언트 접속됨:", address[0])

    # ------------------- 키보드 명령 -------------------
    def monitor_keyboard_input(self):
        while True:
            try:
                cmd = input().strip().lower()
                if cmd == "start":
                    print("📥 입력 명령: start")
                    self.send_start_signal()
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

    # ------------------- 패킷 파싱 -------------------
    def parse_message(self, message):
        try:
            values = message.strip().split(',')
            if len(values) != 24:
                raise ValueError(f"필드 개수가 올바르지 않음. ({len(values)}개 수신됨)")

            index = int(values[0])
            timestamp = float(values[1])
            floats = [float(v) for v in values[2:]]

            return {
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
        except Exception as e:
            print("❗ 메시지 파싱 오류:", e)
            return None

    # ------------------- deque 저장 -------------------
    def store_to_queues(self, data):
        self.acc_x_queue.append(data['accel'][0]);  self.acc_y_queue.append(data['accel'][1]);  self.acc_z_queue.append(data['accel'][2])
        self.mag_x_queue.append(data['mag'][0]);    self.mag_y_queue.append(data['mag'][1]);    self.mag_z_queue.append(data['mag'][2])
        self.gyro_x_queue.append(data['gyro'][0]);  self.gyro_y_queue.append(data['gyro'][1]);  self.gyro_z_queue.append(data['gyro'][2])
        self.heading_queue.append(data['euler'][0]); self.roll_queue.append(data['euler'][1]);   self.pitch_queue.append(data['euler'][2])
        self.linacc_x_queue.append(data['lin_acc'][0]); self.linacc_y_queue.append(data['lin_acc'][1]); self.linacc_z_queue.append(data['lin_acc'][2])
        self.gravity_x_queue.append(data['gravity'][0]); self.gravity_y_queue.append(data['gravity'][1]); self.gravity_z_queue.append(data['gravity'][2])
        self.quat_w_queue.append(data['quat'][0]);  self.quat_x_queue.append(data['quat'][1]);  self.quat_y_queue.append(data['quat'][2]);  self.quat_z_queue.append(data['quat'][3])

    # ------------------- 필터링·로그 출력 -------------------
    def display_filtered_data(self):
        while True:
            if not self.measurement_active:
                time.sleep(0.1)
                continue
            
            if self.last_index is None or self.last_index == self.last_logged_index:
                time.sleep(0.02)  # 아직 새 데이터 없음
                continue

            if self.acc_x_queue:
                vals = [compute_moving_average(q) for q in (
                    self.acc_x_queue, self.acc_y_queue, self.acc_z_queue,
                    self.mag_x_queue, self.mag_y_queue, self.mag_z_queue,
                    self.gyro_x_queue, self.gyro_y_queue, self.gyro_z_queue,
                    self.heading_queue, self.roll_queue, self.pitch_queue,
                    self.linacc_x_queue, self.linacc_y_queue, self.linacc_z_queue,
                    self.gravity_x_queue, self.gravity_y_queue, self.gravity_z_queue,
                    self.quat_w_queue, self.quat_x_queue, self.quat_y_queue, self.quat_z_queue
                )]
                line = f"{self.last_index},{self.last_timestamp:.3f}," + ",".join(f"{v:.3f}" for v in vals)
                #print(line)

                with open("Server_data.txt", "a", encoding="utf-8") as f:
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
                self.last_logged_index = self.last_index
            time.sleep(0.06)

    # ------------------- 신호 전송 -------------------
    def send_exit_signal(self):
        if self.client_address:
            cli_ip = self.client_address[0]
            port_raspi = 5004
            for _ in range(200):
                self.server_socket.sendto(b"exit", (cli_pi, port_raspi))
                time.sleep(0.05)

    def send_start_signal(self):
        if self.client_address:
            cli_ip = self.client_address[0]
            port_raspi = 5004
            port_para = 5006

            for _ in range(20):
                self.server_socket.sendto(b"start", (cli_ip, port_raspi))
                time.sleep(0.05)

            for _ in range(20):
                self.server_socket.sendto(b"start", (cli_ip, port_para))
                time.sleep(0.05)

    # ------------------- 메인 루프 -------------------
    def run(self):
        threading.Thread(target=self.display_filtered_data, daemon=True).start()
        threading.Thread(target=self.monitor_keyboard_input, daemon=True).start()

        try:
            self.wait_for_client()
            while True:
                data, _ = self.server_socket.recvfrom(1024)
                message = data.decode("UTF-8").strip()

                if message.lower() == "canpara":
                    print("📡 낙하산 사출 신호 수신")
                    print("📡 낙하산 사출 신호 수신")
                    continue
                
                if message.lower() == "parafail":
                    print("🚨 낙하산 사출 실패 신호 수신")
                    print("🚨 낙하산 사출 실패 신호 수신")
                    continue
                
                if message.lower() == "parasuc":
                    print("✅ 낙하산 사출 성공 신호 수신")
                    print("✅ 낙하산 사출 성공 신호 수신")
                    continue

                if message.lower() == "exit":
                    print("🚪 종료 메시지 수신")
                    break

                if self.measurement_active:
                    parsed = self.parse_message(message)
                    if parsed:
                        # --- 중복 검사 → 저장 → 인덱스 갱신 ---
                        if self.last_index is not None and parsed["index"] == self.last_index:
                            continue
                        self.store_to_queues(parsed)
                        self.last_index = parsed["index"]
                        self.last_timestamp = parsed["timestamp"]

        except KeyboardInterrupt:
            pass
        finally:
            self.close()

    # ------------------- 종료 -------------------
    def close(self):
        self.server_socket.close()
        print("🔧 서버 시작 종료")

# ----------------------- 실행 -----------------------
if __name__ == "__main__":
    IMUServer().run()
