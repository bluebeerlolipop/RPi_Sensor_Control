import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import threading
import time
from collections import deque

# lin_acc_z 값을 저장할 queue
lin_acc_z_queue = deque(maxlen=100)

# 실시간 그래프 그리는 함수
def plot_lin_acc():
    plt.ion()
    fig, ax = plt.subplots()
    x_data = []
    y_data = []
    line, = ax.plot([], [], label='Z Linear Acc', linewidth=2)
    ax.set_ylim(-12, 12)
    ax.set_xlim(0, 10)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Z Acceleration (m/s²)')
    ax.set_title('Real-Time Z-Axis Linear Acceleration')
    ax.legend()
    start_time = time.time()

    while True:
        if len(lin_acc_z_queue) > 0:
            now = time.time() - start_time
            z = lin_acc_z_queue[-1]

            x_data.append(now)
            y_data.append(z)

            x_data = x_data[-100:]
            y_data = y_data[-100:]

            line.set_data(x_data, y_data)
            ax.set_xlim(max(0, now - 10), now + 1)
            ax.relim()
            ax.autoscale_view()
            plt.draw()
            plt.pause(0.05)

# 메인 센서 루프에서 lin_acc z-axis 값을 queue에 추가
def sensor_loop():
    import smbus2
    import struct

    BNO055_ADDRESS = 0x28
    bus = smbus2.SMBus(1)

    def read_vector(register, count=3):
        data = bus.read_i2c_block_data(BNO055_ADDRESS, register, count*2)
        values = []
        for i in range(count):
            lsb = data[i*2]
            msb = data[i*2+1]
            val = struct.unpack('<h', bytes([lsb, msb]))[0]
            values.append(val)
        return values

    def init_bno055():
        bus.write_byte_data(BNO055_ADDRESS, 0x3D, 0x0C)
        time.sleep(0.02)

    init_bno055()

    while True:
        lin_acc = [x / 100.0 for x in read_vector(0x28)]
        lin_acc_z_queue.append(lin_acc[2])
        time.sleep(0.02)

# 센서 읽기 및 그래프 그리기 스레드 시작
threading.Thread(target=sensor_loop, daemon=True).start()
threading.Thread(target=plot_lin_acc, daemon=True).start()

# GUI 창 유지
import tkinter as tk
tk.mainloop()
