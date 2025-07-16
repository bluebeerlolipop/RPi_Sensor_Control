import socket

ip ='192.168.137.1'
port = 22

server_socket = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
server_socket.bind((ip,port))
print('WAITING FOR CLIENT...')
data, address = server_socket.recvfrom(1024)
print("CLIENT ip address: ", address[0])

while True:
    data, address = server_socket.recvfrom(1024)

    if data == "exit":
        print("EXIT DEPLOYED. SERVER SHUTDOWN")
        break

    message = data.decode("UTF-8")
    message = message.strip()
    message = message.split(',')

    accel = [float(message[1]), float(message[2]), float(message[3])]
    mag = [float(message[4]), float(message[5]), float(message[6])]
    gyro = [float(message[7]), float(message[8]), float(message[9])]
    euler = [float(message[10]), float(message[11]), float(message[12])]
    lin_acc = [float(message[13]), float(message[14]), float(message[15])]
    gravity = [float(message[16]), float(message[17]), float(message[18])]
    quat = [float(message[19]), float(message[20]), float(message[21]), float(message[22])]

    print(f"Accel     : {accel} m/s^2")
    print(f"Magnet    : {mag} uT")
    print(f"Gyro      : {gyro} deg/s")
    print(f"Euler     : {euler} deg")
    print(f"LinearAcc : {lin_acc} m/s^2")
    print(f"Gravity   : {gravity} m/s^2")
    print(f"Quaternion: {quat}")
    print("-" * 50)

server_socket.close()