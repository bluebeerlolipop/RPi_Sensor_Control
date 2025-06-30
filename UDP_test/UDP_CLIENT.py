import socket

ip = '192.168.137.1'
port = 22

client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

message = "UDP test"
client_socket.sendto(message.encode("UTF-8"), (ip, port))

client_socket.close()
print("END CONNECTION")