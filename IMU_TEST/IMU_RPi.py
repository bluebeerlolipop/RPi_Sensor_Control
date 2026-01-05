import smbus2
import time
import struct
import socket

ip = '192.168.1.174' # server의 ip를 입력해야함. client에 할당된 ip를 적으면 안됨.
port = 22 # port번호는 수정 가능

client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

BNO055_ADDRESS = 0x28  # or 0x29

bus = smbus2.SMBus(1)  # I2C 버스 1

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
    print("calibrate sensor")
    while True:
        sys, gyro, accel, mag = read_calibration_status()
        print(f"Calibration → Sys:{sys}, Gyro:{gyro}, Accel:{accel}, Mag:{mag}", end='\r')
        if sys == 3 and gyro == 3 and accel == 3 and mag == 3:
            print("\n calibrate complete")
            break
        time.sleep(0.5)

init_bno055()
wait_for_full_calibration()

f = open("imu_data_log_Client.txt", "w")
f.write("time,ax,ay,az,mx,my,mz,gx,gy,gz,ex,ey,ez,lax,lay,laz,gvx,gvy,gvz,q0,q1,q2,q3\n")
start_time = time.time()

#client_socket.settimeout(0.02)
data_number = 1
try:
    while True:
        accel = [x/100.0 for x in read_vector(0x08)]  # m/s^2
        mag   = [x/16.0 for x in read_vector(0x0E)]   # uT
        gyro  = [x/16.0 for x in read_vector(0x14)]   # deg/s
        euler = [x/16.0 for x in read_vector(0x1A)]   # degrees
        lin_acc = [x/100.0 for x in read_vector(0x28)] # m/s^2
        gravity = [x/100.0 for x in read_vector(0x2E)] # m/s^2
        quat  = read_quaternion()
        
        line = "{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}\n".format(
                data_number, time.time() - start_time,
                *accel, *mag, *gyro, *euler, *lin_acc, *gravity, *quat
            )
        
        client_socket.sendto(line.encode("UTF-8"), (ip, port)) # line을 UDP로 전송

        f.write(line)
        f.flush() # 디스크에 기록
        
        print(f"DataNumber: {data_number}")
        print(f"Accel     : {accel} m/s^2")
        print(f"Magnet    : {mag} uT")
        print(f"Gyro      : {gyro} deg/s")
        print(f"Euler     : {euler} deg")
        print(f"LinearAcc : {lin_acc} m/s^2")
        print(f"Gravity   : {gravity} m/s^2")
        print(f"Quaternion: {quat}")
        print("-" * 50)
        time.sleep(0.04)
        data_number = data_number + 1
        
        client_socket.settimeout(0.02)
        try:
            data, address = client_socket.recvfrom(1024)
            message = data.decode("UTF-8")
            if message == "exit":
                print("EXIT DEPLOYED. CLIENT SHUTDOWN")
                break
        except socket.timeout:
            pass
finally:
    f.close()
    client_socket.close()
    print("END CONNECTION")
