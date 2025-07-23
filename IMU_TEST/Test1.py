import time

def generate_dummy_filtered_log(filename="filtered_test_data.txt",
                                 z_start=5.0, z_end=-20.0, step=-0.1, interval=0.1):

    acc_z = z_start
    jumped = False  # 급격한 튀는 값 발생 여부

    try:
        print("📝 더미 IMU 로그 생성 시작 (Ctrl+C 로 종료)")

        with open(filename, "w", encoding="utf-8") as f:
            while True:
                timestamp = time.strftime('%Y-%m-%d %H:%M:%S')

                dummy = lambda x: f"{x:.2f}"
                dummy3 = lambda x: f"[{x:.2f}, {x:.2f}, {x:.2f}]"
                dummy4 = lambda x: f"[w: {x:.3f}, x: {x:.3f}, y: {x:.3f}, z: {x:.3f}]"

                log = (
                    f"Timestamp : {timestamp}\n"
                    f"Accel     : [0.00, 0.00, {acc_z:.2f}] m/s^2\n"
                    f"Magnet    : {dummy3(2.22)} μT\n"
                    f"Gyro      : {dummy3(3.33)} deg/s\n"
                    f"Euler     : [heading: {dummy(10.0)}°, roll: {dummy(20.0)}°, pitch: {dummy(30.0)}°]\n"
                    f"LinearAcc : {dummy3(1.11)} m/s^2\n"
                    f"Gravity   : {dummy3(9.81)} m/s^2\n"
                    f"Quaternion: {dummy4(0.707)}\n"
                    f"Temperature: {dummy(25.0)} °C\n"
                    + ("-" * 50) + "\n"
                )

                print(f"출력됨: acc_z = {acc_z:.2f}")
                f.write(log)
                f.flush()

                # 급격히 튀는 값 삽입 (단 한 번)
                if not jumped and acc_z <= -5.2:
                    acc_z = -14.0
                    jumped = True
                else:
                    acc_z += step

                if acc_z < z_end:
                    acc_z = z_start
                    jumped = False  # 한 주기마다 다시 튀도록 허용

                time.sleep(interval)

    except KeyboardInterrupt:
        print("\n🛑 로그 생성 중단됨")

# 실행
if __name__ == "__main__":
    generate_dummy_filtered_log()
