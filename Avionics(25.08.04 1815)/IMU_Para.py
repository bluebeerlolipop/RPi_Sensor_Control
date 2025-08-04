# main_log_generator.py
import time
import numpy as np
import RPi.GPIO as GPIO
from IMU_RPiv2 import BNO055, QuaternionFilter

SERVO_PIN = 18  # GPIO18 (물리 12번 핀)

def init_servo():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(SERVO_PIN, GPIO.OUT)
    pwm = GPIO.PWM(SERVO_PIN, 50)  # 50Hz PWM
    pwm.start(0)
    return pwm

def rotate_servo(pwm, angle, delay=0.5):
    duty = 2.5 + (angle / 180.0) * 10
    pwm.ChangeDutyCycle(duty)
    time.sleep(delay)
    pwm.ChangeDutyCycle(0)  # 떨림 방지

def cleanup_servo(pwm):
    pwm.stop()
    GPIO.cleanup()

def quaternion_to_rotation_matrix(q):
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y**2 + z**2), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x**2 + z**2), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x**2 + y**2)]
    ])

def angle_between_vectors(v1, v2):
    dot = np.dot(v1, v2)
    norm1 = np.linalg.norm(v1)
    norm2 = np.linalg.norm(v2)
    cos_theta = dot / (norm1 * norm2)
    return np.arccos(np.clip(cos_theta, -1.0, 1.0))

def quaternion_angle_to_normal(q, normal_vector=np.array([0, 0, 1])):
    R = quaternion_to_rotation_matrix(q)
    rotated_vector = R @ normal_vector
    return angle_between_vectors(rotated_vector, normal_vector)

def generate_filtered_log_from_sensor(output_filename="output_log.txt"):
    pwm = init_servo()
    print("⚙️ 초기화: 서보를 140도로 설정")
    rotate_servo(pwm, 140)

    sensor = BNO055(calibrate=True)  # 보정 활성화
    q_filter = QuaternionFilter(maxlen=2)

    print("📝 실시간 IMU 로그 시작")

    last_angle = None
    servo_activated = False
    armed_for_deploy = False  # 사출 조건 감지 활성화 여부

    try:
        with open(output_filename, "w", encoding="utf-8") as f:
            f.write("Index,Timestamp,w,x,y,z,Angle(rad),Angle(deg)\n")
            index = 0

            while True:
                data = sensor.read_all()
                quat_filtered = q_filter.update(data["quat"])
                angle_rad = quaternion_angle_to_normal(quat_filtered)
                angle_deg = np.degrees(angle_rad)
                timestamp = time.strftime('%Y-%m-%d %H:%M:%S')

                # 각도 튐 감지 필터
                if last_angle is not None and abs(angle_deg - last_angle) > 50:
                    print(f"⚠️ 각도 튐 감지! {last_angle:.2f}° → {angle_deg:.2f}°, 스킵됨")
                    continue
                last_angle = angle_deg

                # 로그 기록
                log = (
                    f"{index},{timestamp}," +
                    ",".join(f"{round(q, 6)}" for q in quat_filtered) + "," +
                    f"{angle_rad:.4f},{angle_deg:.4f}\n"
                )

                print(log.strip())
                f.write(log)
                f.flush()

                # 조건 1: 회전 각도가 20도 미만이면 사출 준비 활성화
                if angle_deg < 20:
                    if not armed_for_deploy:
                        print("🟢 사출 조건 감지 활성화됨 (20도 미만)")
                    armed_for_deploy = True

                # 조건 2: 사출 감지 활성화 후 41도 초과 시 낙하산 사출
                if armed_for_deploy and angle_deg > 41 and not servo_activated:
                    print("🚨 낙하산 사출 조건 충족! 낙하산 사출!")
                    f.write("🚨 낙하산 사출 조건 충족! 낙하산 사출!\n")
                    f.flush()

                    rotate_servo(pwm, 100)
                    servo_activated = True
                    exit(0)

                index += 1
                time.sleep(0.04)

    except KeyboardInterrupt:
        print("🛑 로그 수집 중단")
    finally:
        try:
            if pwm is not None:
                cleanup_servo(pwm)
        except Exception as e:
            print(f"⚠️ 서보 cleanup 중 예외 발생: {e}")
        print("✅ IMU 및 서보 연결 종료")

# 실행
if __name__ == "__main__":
    generate_filtered_log_from_sensor()
