import time
import numpy as np
import re
from pathlib import Path

# === 설정 ===
ANGLE_THRESHOLD_DEG = (90.0 - 41.0)  # 지표각 기준으로 환산 (지표각 41도 미만 시 낙하산 사출)
ACC_THRESHOLD = 5.0                 # delta sqrt lin_acc 값이 5.0 m/s² 초과 시 낙하산 사출 판정
DATA_FILE = "filtered_data.txt"     # 감시 대상 파일

# === 회전각 계산 함수 ===
def quaternion_to_rotation_matrix(q):
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y**2 + z**2),     2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),       1 - 2 * (x**2 + z**2),   2 * (y * z - x * w)],
        [2 * (x * z - y * w),       2 * (y * z + x * w),     1 - 2 * (x**2 + y**2)]
    ])

def angle_from_quaternion(q, normal=np.array([0, 0, 1])):
    R = quaternion_to_rotation_matrix(q)
    rotated = R @ normal
    cos_theta = np.dot(rotated, normal) / (np.linalg.norm(rotated) * np.linalg.norm(normal))
    return np.degrees(np.arccos(np.clip(cos_theta, -1.0, 1.0)))

# === 실시간 모니터링 함수 ===
def monitor_file_for_parachute_trigger():
    print("📄 낙하산 사출 감시 시작...")
    triggered = False
    angle_ready = False
    last_position = 0
    file = Path(DATA_FILE)

    while True:
        if not file.exists():
            print("❗ 데이터 파일 없음. 대기 중...")
            time.sleep(1)
            continue

        with open(file, "r", encoding="utf-8") as f:
            f.seek(last_position)
            lines = f.readlines()
            last_position = f.tell()

        for i in range(len(lines)):
            line = lines[i].strip()

            if line.startswith("Quaternion:"):
                q_vals = re.findall(r"[-+]?[0-9]*\.?[0-9]+", line)
                if len(q_vals) >= 4:
                    quat = [float(q) for q in q_vals[:4]]
                    angle = angle_from_quaternion(quat)

                    if not angle_ready and angle >= ANGLE_THRESHOLD_DEG:
                        print(f"✅ 회전각 임계값 초과 감지됨 (기준: {ANGLE_THRESHOLD_DEG:.2f}°), 현재: {angle:.2f}°")
                        angle_ready = True  # 임계각을 한 번 넘어가면 유지

            elif line.startswith("LinearAcc"):
                acc_vals = re.findall(r"[-+]?[0-9]*\.?[0-9]+", line)
                if len(acc_vals) >= 3:
                    acc = np.array([float(x) for x in acc_vals[:3]])
                    acc_mag = np.linalg.norm(acc)

                    if angle_ready and acc_mag >= ACC_THRESHOLD:
                        print(f"🚨 낙하산 사출 감지됨! | 각도: {angle:.2f}°, 가속도 크기: {acc_mag:.2f} m/s²")
                        triggered = True
                        break

        if triggered:
            break

        time.sleep(0.06)

# === 실행 ===
if __name__ == "__main__":
    monitor_file_for_parachute_trigger()
