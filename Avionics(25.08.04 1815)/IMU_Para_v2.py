import time
import numpy as np
import RPi.GPIO as GPIO


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


class Servo:
    def __init__(self, servo_pin=18):
        self.servo_pin = servo_pin
        self.pwm = None
        self.sensor = None
        self.q_filter = QuaternionFilter(maxlen=2)
        self.last_angle = None
        self.servo_activated = False
        self.armed_for_deploy = False

    def init_servo(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.servo_pin, GPIO.OUT)
        self.pwm = GPIO.PWM(self.servo_pin, 50)
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

    def run(self):
        self.init_servo()
        print("⚙️ 초기화: 서보를 140도로 설정")
        self.rotate_servo(140)

        self.sensor = BNO055(calibrate=True)

        print("📝 실시간 IMU 로그 시작")

        try:
            while True:
                data = self.sensor.read_all()
                quat_filtered = self.q_filter.update(data["quat"])
                angle_rad = self.quaternion_angle_to_normal(quat_filtered)
                angle_deg = np.degrees(angle_rad)
                timestamp = time.strftime('%Y-%m-%d %H:%M:%S')

                if self.last_angle is not None and abs(angle_deg - self.last_angle) > 50:
                    print(f"⚠️ 각도 튐 감지! {last_angle:.2f}° → {angle_deg:.2f}°, 스킵됨")
                    continue
                self.last_angle = angle_deg

                log = (
                    f"{index},{timestamp}," +
                    ",".join(f"{round(q, 6)}" for q in quat_filtered) + "," +
                    f"{angle_rad:.4f},{angle_deg:.4f}\n"
                )

                print(log.strip())

                if angle_deg < 20 and not self.armed_for_deploy:
                    print("🟢 사출 조건 감지 활성화됨 (20도 미만)")
                    self.armed_for_deploy = True

                if self.armed_for_deploy and angle_deg > 41 and not self.servo_activated:
                    print("🚨 낙하산 사출 조건 충족! 낙하산 사출!")

                    self.rotate_servo(100)
                    self.servo_activated = True
                    exit(0)

                index += 1
                time.sleep(0.04)

        except KeyboardInterrupt:
            print("🛑 로그 수집 중단")
        finally:
            try:
                self.cleanup_servo()
            except Exception as e:
                print(f"⚠️ 서보 cleanup 중 예외 발생: {e}")
            print("✅ IMU 및 서보 연결 종료")

if __name__ == "__main__":
    logger = Servo()
    logger.run()