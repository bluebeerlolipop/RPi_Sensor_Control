import socket
import time

class UDP:
    def __init__(self):
        self.ip = '192.168.137.1'
        self.port = 22

    def Client(self):
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.message = "UDP test"

        for i in range(10):
            i = i + 1
            self.client_socket.sendto(self.message.encode("UTF-8"), (self.ip, self.port))
            print("%d번째 전송" % i)
            if i  == 10:
                self.message = "exit"
                self.client_socket.sendto(self.message.encode("UTF-8"), (self.ip, self.port))
                break
            time.sleep(1)
        self.client_socket.close()
        print("END CONNECTION")

    def Server(self):
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.server_socket.bind((self.ip, self.port))
        print('WAITING FOR CLIENT...')
        
        while True:
            self.data, self.address = self.server_socket.recvfrom(1024)
            self.message = self.data.decode("UTF-8")
            print("CLIENT ip address: ", self.address[0])
            print("message from client: ", self.message)
            if self.message == "exit":
                print("EXIT DEPLOYED. SERVER SHUTDOWN")
                break
        
        self.server_socket.close()  