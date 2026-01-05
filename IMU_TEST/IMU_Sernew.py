import socket
import time
import threading
from collections import deque

# Average filtering function
def compute_moving_average(queue):
    if len(queue) == 0:
        return None
    return sum(queue) / len(queue)

class IMUServer:
    def __init__(self, ip='192.168.137.1', port=22):
        self.ip = ip
        self.port = port
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.server_socket.bind((self.ip, self.port))
        self.client_address = None

        # 큐 선언
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

        self.temp_queue = deque(maxlen=2)

        print(f"📡 서버 실행 중... {self.ip}:{self.port}")


    def wait_for_client(self):
        print("waiting for client")
        _ , address = self.server_socket.recvfrom(1024) # (data, address) 중 [1] 값 할당
        self.client_address = address
        print("client detected:", address[0])

    def parse_message(self, message):
        try:
            values = message.strip().split(',')
            if len(values) < 23:
                raise ValueError("필드 개수가 부족합니다.")
            time = float(values[0])  # timestamp
            # float 변환
            values = [float(v) for v in values[1:]]  # [0]은 timestamp

            # 데이터 분할
            accel = values[0:3]
            mag = values[3:6]
            gyro = values[6:9]
            euler = values[9:12]
            lin_acc = values[12:15]
            gravity = values[15:18]
            quat = values[18:22]
            temp = values[22] # 22번 아니면 수정 필요

            # 분할 데이터 할당
            return {
                "time": time, 
                "accel": accel,
                "mag": mag,
                "gyro": gyro,
                "euler": euler,
                "lin_acc": lin_acc,
                "gravity": gravity,
                "quat": quat,
                "temp": temp
            }
        except Exception as e:
            print("Message error:", e)
            return None

    def store_to_queues(self, data):
        # Acceleration
        self.acc_x_queue.append(data['accel'][0]) # m/s^2
        self.acc_y_queue.append(data['accel'][1]) # m/s^2
        self.acc_z_queue.append(data['accel'][2]) # m/s^2

        # Magnitude
        self.mag_x_queue.append(data['mag'][0]) # μT
        self.mag_y_queue.append(data['mag'][1]) # μT
        self.mag_z_queue.append(data['mag'][2]) # μT

        # Gyroscope
        self.gyro_x_queue.append(data['gyro'][0]) # deg/s
        self.gyro_y_queue.append(data['gyro'][1]) # deg/s
        self.gyro_z_queue.append(data['gyro'][2]) # deg/s

        # Euler angles
        self.heading_queue.append(data['euler'][0]) # degrees
        self.roll_queue.append(data['euler'][1]) # degrees
        self.pitch_queue.append(data['euler'][2]) # degrees

        # Linear Acceleration
        self.linacc_x_queue.append(data['lin_acc'][0]) # m/s^2
        self.linacc_y_queue.append(data['lin_acc'][1]) # m/s^2
        self.linacc_z_queue.append(data['lin_acc'][2]) # m/s^2

        # Gravity
        self.gravity_x_queue.append(data['gravity'][0]) # m/s^2
        self.gravity_y_queue.append(data['gravity'][1]) # m/s^2
        self.gravity_z_queue.append(data['gravity'][2]) # m/s^2

        # Quaternion
        self.quat_w_queue.append(data['quat'][0]) # w
        self.quat_x_queue.append(data['quat'][1]) # x
        self.quat_y_queue.append(data['quat'][2]) # y
        self.quat_z_queue.append(data['quat'][3]) # z

        # Temperature
        self.temp_queue.append(data['temp']) # °C

    # Filtering + display
    def display_filtered_data(self):
        while True:
            if len(self.acc_x_queue) > 0:
                avg_acc_x = compute_moving_average(self.acc_x_queue)
                avg_acc_y = compute_moving_average(self.acc_y_queue)
                avg_acc_z = compute_moving_average(self.acc_z_queue)

                avg_mag_x = compute_moving_average(self.mag_x_queue)
                avg_mag_y = compute_moving_average(self.mag_y_queue)
                avg_mag_z = compute_moving_average(self.mag_z_queue)

                avg_gyro_x = compute_moving_average(self.gyro_x_queue)
                avg_gyro_y = compute_moving_average(self.gyro_y_queue)
                avg_gyro_z = compute_moving_average(self.gyro_z_queue)

                avg_heading = compute_moving_average(self.heading_queue)
                avg_roll = compute_moving_average(self.roll_queue)
                avg_pitch = compute_moving_average(self.pitch_queue)

                avg_linacc_x = compute_moving_average(self.linacc_x_queue)
                avg_linacc_y = compute_moving_average(self.linacc_y_queue)
                avg_linacc_z = compute_moving_average(self.linacc_z_queue)

                avg_gravity_x = compute_moving_average(self.gravity_x_queue)
                avg_gravity_y = compute_moving_average(self.gravity_y_queue)
                avg_gravity_z = compute_moving_average(self.gravity_z_queue)

                avg_quat_w = compute_moving_average(self.quat_w_queue)
                avg_quat_x = compute_moving_average(self.quat_x_queue)
                avg_quat_y = compute_moving_average(self.quat_y_queue)
                avg_quat_z = compute_moving_average(self.quat_z_queue)

                avg_temp = compute_moving_average(self.temp_queue)

                timestamp = time.strftime('%Y-%m-%d %H:%M:%S')
                log = (f"Timestamp : {timestamp}\n"
                       f"Accel     : [{avg_acc_x:.2f}, {avg_acc_y:.2f}, {avg_acc_z:.2f}] m/s^2\n"
                       f"Magnet    : [{avg_mag_x:.2f}, {avg_mag_y:.2f}, {avg_mag_z:.2f}] μT\n"
                       f"Gyro      : [{avg_gyro_x:.2f}, {avg_gyro_y:.2f}, {avg_gyro_z:.2f}] deg/s\n"
                       f"Euler     : [heading: {avg_heading:.2f}°, roll: {avg_roll:.2f}°, pitch: {avg_pitch:.2f}°]\n"
                       f"LinearAcc : [{avg_linacc_x:.2f}, {avg_linacc_y:.2f}, {avg_linacc_z:.2f}] m/s^2\n"
                       f"Gravity   : [{avg_gravity_x:.2f}, {avg_gravity_y:.2f}, {avg_gravity_z:.2f}] m/s^2\n"
                       f"Quaternion: [w: {avg_quat_w:.3f}, x: {avg_quat_x:.3f}, y: {avg_quat_y:.3f}, z: {avg_quat_z:.3f}]\n"
                       f"Temperature: {avg_temp:.1f} °C\n"
                       + ("-" * 50) + "\n")

                print(log)

                # 파일에 기록
                with open("filtered_data.txt", "a", encoding="utf-8") as f:
                    f.write(log)   
            else:
                print("waiting for data")
            time.sleep(0.06)

    def send_exit_signal(self):
        message = "exit"
        for _ in range(200):
            self.server_socket.sendto(message.encode("UTF-8"), self.client_address)
            time.sleep(0.05)

    def run(self):
        threading.Thread(target=self.display_filtered_data, daemon=True).start()  # filtered data 출력
        try:
            self.wait_for_client()
            while True:
                data, _ = self.server_socket.recvfrom(1024) # (data, address) 중 [0] 값 할당
                message = data.decode("UTF-8")

                if message == "exit":
                    print("client exit signal detected. Closing the server.")
                    break

                parsed = self.parse_message(message)
                if parsed:
                    self.store_to_queues(parsed)  # 큐에 데이터 저장
        
        except KeyboardInterrupt:
            print("\n keyboard interrrupt detected: Closing the server.")
            self.send_exit_signal()
        finally:
            self.close()

    def close(self):
        self.server_socket.close()
        print("Server socket is closed. Server closed.")

# 실행 코드
if __name__ == "__main__":
    server = IMUServer()
    server.run()
