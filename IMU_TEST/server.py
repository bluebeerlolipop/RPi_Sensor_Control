import smbus2
import time
import struct
from collections import deque

BNO055_ADDRESS = 0x28  # or 0x29

bus = smbus2.SMBus(1)  # I2C 버스 1

acc_x_queue = deque(maxlen=2)
acc_y_queue = deque(maxlen=2)
acc_z_queue = deque(maxlen=2)

gyro_x_queue = deque(maxlen=2)
gyro_y_queue = deque(maxlen=2)
gyro_z_queue = deque(maxlen=2)

quat_w_queue = deque(maxlen=2)
quat_x_queue = deque(maxlen=2)
quat_y_queue = deque(maxlen=2)
quat_z_queue = deque(maxlen=2)

temp_queue = deque(maxlen=2)
timestamp_queue = deque(maxlen=2)

heading_queue = deque(maxlen=2)
roll_queue = deque(maxlen=2)
pitch_queue = deque(maxlen=2)

# 데이터 읽는 함수
def read_vector(register, count=3):
    data = bus.read_i2c_block_data(BNO055_ADDRESS, register, count*2)
    values = []
    for i in range(count):
        lsb = data[i*2]
        msb = data[i*2+1]
        val = struct.unpack('<h', bytes([lsb, msb]))[0]
        values.append(val)
    return values

def read_quaternion(register=0x20):
    data = bus.read_i2c_block_data(BNO055_ADDRESS, register, 8)
    q = []
    for i in range(4):
        lsb = data[i*2]
        msb = data[i*2+1]
        val = struct.unpack('<h', bytes([lsb, msb]))[0]
        q.append(val / (1<<14))
    return q

def read_temperature():
    return bus.read_byte_data(BNO055_ADDRESS, 0x34)

def read_calibration_status():
    cal_data = bus.read_byte_data(BNO055_ADDRESS, 0x35)
    sys = (cal_data >> 6) & 0x03
    gyro = (cal_data >> 4) & 0x03
    accel = (cal_data >> 2) & 0x03
    mag = cal_data & 0x03
    return sys, gyro, accel, mag

def init_bno055():
    # Operation mode to NDOF (0x0C)
    bus.write_byte_data(BNO055_ADDRESS, 0x3D, 0x0C)
    time.sleep(0.02)

def wait_for_full_calibration():
    print("🔄 센서 보정 중...")
    while True:
        sys, gyro, accel, mag = read_calibration_status()
        print(f"Calibration → Sys:{sys}, Gyro:{gyro}, Accel:{accel}, Mag:{mag}", end='\r')
        if sys == 3 and gyro == 3 and accel == 3 and mag == 3:
            print("\n✅ 센서 보정 완료!")
            break
        time.sleep(0.5)

init_bno055()
wait_for_full_calibration()

while True:
    accel = [x/100.0 for x in read_vector(0x08)]  # m/s^2
    mag   = [x/16.0 for x in read_vector(0x0E)]   # uT
    gyro  = [x/16.0 for x in read_vector(0x14)]   # deg/s
    euler = [x/16.0 for x in read_vector(0x1A)]   # degrees
    lin_acc = [x/100.0 for x in read_vector(0x28)] # m/s^2
    gravity = [x/100.0 for x in read_vector(0x2E)] # m/s^2
    quat  = read_quaternion()
    temp = read_temperature()  # °C
    timestamp = time.time()  

    if euler is not None:
        acc_x_queue.append(lin_acc[0])
        acc_y_queue.append(lin_acc[1])
        acc_z_queue.append(lin_acc[2])

        gyro_x_queue.append(gyro[0])
        gyro_y_queue.append(gyro[1])
        gyro_z_queue.append(gyro[2])

        quat_w_queue.append(quat[0])
        quat_x_queue.append(quat[1])
        quat_y_queue.append(quat[2])
        quat_z_queue.append(quat[3])

        heading_queue.append(euler[0])
        roll_queue.append(euler[1])
        pitch_queue.append(euler[2])

        temp_queue.append(temp)
        timestamp_queue.append(timestamp)


    prinf(f"Timestamp : {timestamp:.2f} s")
    print(f"Temperature: {temp:.1f} °C")
    print(f"Accel     : {accel} m/s^2")
    print(f"Magnet    : {mag} uT")
    print(f"Gyro      : {gyro} deg/s")
    print(f"Euler     : {euler} deg")
    print(f"LinearAcc : {lin_acc} m/s^2")
    print(f"Gravity   : {gravity} m/s^2")
    print(f"Quaternion: {quat}")
    print("-" * 50)
    time.sleep(0.06)