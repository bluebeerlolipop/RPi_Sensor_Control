import socket

ip ='192.168.0.9'
port = 22

server_socket = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
server_socket.bind((ip,port))
print('WAITING FOR CLIENT...')
data, address = server_socket.recvfrom(1024)

while True:
    data, address = server_socket.recvfrom(1024)
    message = data.decode("UTF-8")
    print("CLIENT ip address: ", address[0])
    print("message from client: ", message)
    if message == "exit":
        print("EXIT DEPLOYED. SERVER SHUTDOWN")
        break

server_socket.close()