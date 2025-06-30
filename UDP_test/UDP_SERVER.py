import socket

ip ='192.168.137.1'
port = 22

server_socket = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
server_socket.bind((ip,port))
print('WAITING FOR CLIENT...')
data, address = server_socket.recvfrom(1024)
print('CLIENT ip address:', address[0])
print(data.decode("UTF-8"))