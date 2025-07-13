import smbus2
import time
import struct

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

def init_bno055():
    # Operation mode to NDOF (0x0C)
    bus.write_byte_data(BNO055_ADDRESS, 0x3D, 0x0C)
    time.sleep(0.02)

init_bno055()

with open("imu_data_log.txt", "w") as f:
    f.write("time,ax,ay,az,mx,my,mz,gx,gy,gz,ex,ey,ez,lax,lay,laz,gvx,gvy,gvz,q0,q1,q2,q3\n")


while True:
    accel = [x/100.0 for x in read_vector(0x08)]  # m/s^2
    mag   = [x/16.0 for x in read_vector(0x0E)]   # uT
    gyro  = [x/16.0 for x in read_vector(0x14)]   # deg/s
    euler = [x/16.0 for x in read_vector(0x1A)]   # degrees
    lin_acc = [x/100.0 for x in read_vector(0x28)] # m/s^2
    gravity = [x/100.0 for x in read_vector(0x2E)] # m/s^2
    quat  = read_quaternion()

    line = "{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}\n".format(
            time.time(),
            *accel, *mag, *gyro, *euler, *lin_acc, *gravity, *quat
        )

    f.write(line)
    f.flush()  # 즉시 디스크에 기록
    time.sleep(0.06)

    print(f"Accel     : {accel} m/s^2")
    print(f"Magnet    : {mag} uT")
    print(f"Gyro      : {gyro} deg/s")
    print(f"Euler     : {euler} deg")
    print(f"LinearAcc : {lin_acc} m/s^2")
    print(f"Gravity   : {gravity} m/s^2")
    print(f"Quaternion: {quat}")
    print("-" * 50)
    time.sleep(0.06)
