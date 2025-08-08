# Final/para.py
import time
import numpy as np
import socket
import RPi.GPIO as GPIO
import threading
from raspi import BNO055, QuaternionFilter

SERVO_PIN = 18  # GPIO18 (물리 12번 핀)
FAIL_SIGNAL_IP = "127.0.0.1"  # 로컬 테스트용 IP
FAIL_SIGNAL_PORT = 6006 # raspi.py와 동일한 포트
START_LISTEN_PORT = 5006 # server.py(para port)와 동일한 포트
SERVER_IP   = "192.168.137.1"   # server.py가 바인드한 IP
SERVER_PORT = 5005


# start 신호 수신
def listen_start(event, port=START_LISTEN_PORT):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("", port))
    print(f"📡 start 신호 대기 중… (포트 {port})")
    while True:
        data, _ = sock.recvfrom(1024)
        if data.decode().strip().lower() == "start":
            print("🚀 start 신호 수신! 탐지 루프 시작")
            event.set()
            break
    sock.close()

class IMUParachuteSystem:
    def __init__(self, start_event, output_filename="output_log.txt"):
        self.notify_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.start_event = start_event
        self.output_filename = output_filename
        self.pwm = None
        self.sensor = BNO055(calibrate=True, start_event=self.start_event)
        self.q_filter = QuaternionFilter(maxlen=2)
        self.acc_total_queue = []  # 평균필터용 큐 수동 관리
        self.last_angle = None
        self.servo_activated = False
        self.armed_for_deploy = False

    def init_servo(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(SERVO_PIN, GPIO.OUT)
        self.pwm = GPIO.PWM(SERVO_PIN, 50)  # 50Hz PWM
        self.pwm.start(0)

    def rotate_servo(self, angle, delay=0.5):
        duty = 2.5 + (angle / 180.0) * 10
        self.pwm.ChangeDutyCycle(duty)
        time.sleep(delay)
        self.pwm.ChangeDutyCycle(0)

    def cleanup_servo(self):
        if self.pwm:
            self.pwm.stop()
        GPIO.cleanup()

    def quaternion_to_rotation_matrix(self, q):
        w, x, y, z = q
        return np.array([
            [1 - 2 * (y**2 + z**2), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x**2 + z**2), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x**2 + y**2)]
        ])

    def angle_between_vectors(self, v1, v2):
        dot = np.dot(v1, v2)
        norm1 = np.linalg.norm(v1)
        norm2 = np.linalg.norm(v2)
        cos_theta = dot / (norm1 * norm2)
        return np.arccos(np.clip(cos_theta, -1.0, 1.0))

    def quaternion_angle_to_normal(self, q, normal_vector=np.array([0, 0, 1])):
        R = self.quaternion_to_rotation_matrix(q)
        rotated_vector = R @ normal_vector
        return self.angle_between_vectors(rotated_vector, normal_vector)

    def send_fail_signal(self):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.sendto(b"FAIL", (FAIL_SIGNAL_IP, FAIL_SIGNAL_PORT))
            print("❌ FAIL 신호 전송 완료")

    def generate_filtered_log(self):
        self.start_event.wait()

        self.init_servo()
        print("⚙️ 초기화: 서보를 140도로 설정")
        self.rotate_servo(140)

        print("🖍️ 실시간 IMU 로그 시작")

        try:
            with open(self.output_filename, "w", encoding="utf-8") as f:
                f.write("Index,Timestamp,w,x,y,z,Angle(rad),Angle(deg),acc_total\n")
                index = 0

                while True:
                    data = self.sensor.read_all()
                    quat_filtered = self.q_filter.update(data["quat"])
                    angle_rad = self.quaternion_angle_to_normal(quat_filtered)
                    angle_deg = np.degrees(angle_rad)
                    timestamp = time.strftime('%Y-%m-%d %H:%M:%S')

                    lin_acc = data["lin_acc"]
                    acc_total = np.linalg.norm(lin_acc)
                    self.acc_total_queue.append(acc_total)
                    if len(self.acc_total_queue) > 2:
                        self.acc_total_queue.pop(0)
                    avg_acc_total = sum(self.acc_total_queue) / len(self.acc_total_queue)

                    if self.last_angle is not None and abs(angle_deg - self.last_angle) > 50:
                        print(f"⚠️ 각도 튐 감지! {self.last_angle:.2f}° → {angle_deg:.2f}°, 스킵됨")
                        continue
                    self.last_angle = angle_deg

                    log = (
                        f"{index},{timestamp}," +
                        ",".join(f"{round(q, 6)}" for q in quat_filtered) + "," +
                        f"{angle_rad:.4f},{angle_deg:.4f},{avg_acc_total:.2f}\n"
                    )
                    print(log.strip())
                    f.write(log)
                    f.flush()

                    if angle_deg < 20:
                        if not self.armed_for_deploy:
                            print("🟢 낙하산 사출 조건 감지 활성화됨 (20도 미만)")
                        self.armed_for_deploy = True

                    if self.armed_for_deploy and angle_deg > 41 and not self.servo_activated:
                        print("🚨 낙하산 사출 조건 충족! 낙하산 사출!")
                        f.write("🚨 낙하산 사출 조건 충족! 낙하산 사출!\n")
                        f.flush()
                        self.rotate_servo(100)
                        self.servo_activated = True
                        self.notify_socket.sendto(b"canpara", (SERVER_IP, SERVER_PORT))

                        acc_values = []
                        t0 = time.time()
                        while time.time() - t0 < 1.0:
                            lin_acc = self.sensor.read_all()["lin_acc"]
                            acc_magnitude = np.linalg.norm(lin_acc)
                            acc_values.append(acc_magnitude)
                            time.sleep(0.06)

                        max_acc = max(acc_values)
                        if max_acc - min(acc_values) < 5.0:
                            msg = f"❌ 낙하산 사출 실패로 판단됨 (최고 가속도 {max_acc:.2f} m/s²)"
                            print(msg)
                            f.write(msg + "\n")
                            f.flush()
                            self.send_fail_signal()
                            self.notify_socket.sendto(b"parafail", (SERVER_IP, SERVER_PORT))
                            exit(0)

                        elif max_acc - min(acc_values) >= 5.0:
                            print(f"✅ 낙하산 사출 성공! (최고 가속도 {max_acc:.2f} m/s²)")
                            f.write(f"✅ 낙하산 사출 성공! (최고 가속도 {max_acc:.2f} m/s²)\n")
                            f.flush()
                            self.notify_socket.sendto(b"parasuc", (SERVER_IP, SERVER_PORT))
                            break

                    index += 1
                    time.sleep(0.04)

        except KeyboardInterrupt:
            print("🚫 로그 수집 중단")
        finally:
            try:
                self.cleanup_servo()
            except Exception as e:
                print(f"⚠️ 서보 cleanup 중 예외 발생: {e}")
            print("✅ IMU 및 서보 연결 종료")

if __name__ == "__main__":
    start_event = threading.Event()
    threading.Thread(target=listen_start, args=(start_event,)).start()
    imu_system = IMUParachuteSystem(start_event=start_event)
    imu_system.generate_filtered_log()
