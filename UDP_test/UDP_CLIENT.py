import socket
import time

ip = '192.168.137.1' # server의 ip를 입력해야함. client에 할당된 ip를 적으면 안됨.
port = 22 # port번호는 수정 가능

client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

message = "UDP test"
for i in range(10):
    i = i + 1
    client_socket.sendto(message.encode("UTF-8"), (ip, port))
    if i == 10:
        message = "exit"
        print("%d번째 전송" %i)
        client_socket.sendto(message.encode("UTF-8"), (ip,port))
        break
    print("%d번째 전송" %i)
    time.sleep(1)

client_socket.close()
print("END CONNECTION")
