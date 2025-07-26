import socket
import time

ip ='192.168.1.174' # server의 ip를 입력
port = 22

server_socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
server_socket.bind((ip,port))
print('WAITING FOR CLIENT...')
data, address = server_socket.recvfrom(1024)
print("CLIENT ip address: ", address[0])

f = open("imu_data_log_Server.txt", "w")
f.write("time,ax,ay,az,mx,my,mz,gx,gy,gz,ex,ey,ez,lax,lay,laz,gvx,gvy,gvz,q0,q1,q2,q3\n")

try:
    while True:
        data, address = server_socket.recvfrom(1024)
        message = data.decode("UTF-8")

        if message == "exit": # 클라이언트에서 exit 메시지 보내면 서버 종료하는 코드
            print("EXIT DEPLOYED. SERVER SHUTDOWN")
            break

        message = message.strip()
        message = message.split(',')

        data_number = int(message(0))
        runtime = float(message[1])
        accel = [float(message[2]), float(message[3]), float(message[4])]
        mag = [float(message[5]), float(message[6]), float(message[7])]
        gyro = [float(message[8]), float(message[9]), float(message[10])]
        euler = [float(message[11]), float(message[12]), float(message[13])]
        lin_acc = [float(message[14]), float(message[15]), float(message[16])]
        gravity = [float(message[17]), float(message[18]), float(message[19])]
        quat = [float(message[20]), float(message[21]), float(message[22]), float(message[23])]

        print(f"DataNumber: {data_number}")
        print(f"Accel     : {accel} m/s^2")
        print(f"Magnet    : {mag} uT")
        print(f"Gyro      : {gyro} deg/s")
        print(f"Euler     : {euler} deg")
        print(f"LinearAcc : {lin_acc} m/s^2")
        print(f"Gravity   : {gravity} m/s^2")
        print(f"Quaternion: {quat}")
        print("-" * 50)

        line = "{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}\n".format(
            data_number, runtime,
            *accel, *mag, *gyro, *euler, *lin_acc, *gravity, *quat
            )
        f.write(line)
        f.flush() # server 측에서 즉시 저장

except KeyboardInterrupt:
    print("KeyboardInterrupt: Exiting gracefully...")
    message = "exit"
    for i in range(200):
        server_socket.sendto(message.encode("UTF-8"), address)
        time.sleep(0.05)

finally:
    f.close()
    server_socket.close()
    print("Server socket closed.")