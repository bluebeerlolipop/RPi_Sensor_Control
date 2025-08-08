# fail_log_receiver.py
import socket
import time

IP = "10.14.170.26"   # 서버 IP
PORT = 5010           # 클라이언트가 보내는 포트와 동일하게 유지

def receive_fail_safe_log():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((IP, PORT))

    print(f"📡 로그 수신 서버 시작됨: {IP}:{PORT}")
    log_file = open("Fail_safe.txt", "w", encoding="utf-8")

    try:
        while True:
            data, addr = sock.recvfrom(8192)
            message = data.decode("utf-8").strip()

            if message.lower() == "exit":
                print("✅ 로그 수신 종료")
                break

            if message.startswith("Index"):
                print("📂 헤더 수신됨")
            else:
                print(f"📝 수신: {message[:30]}...")

            log_file.write(message + "\n")
            log_file.flush()

    finally:
        log_file.close()
        sock.close()
        print("📁 로그 파일 저장 완료")

if __name__ == "__main__":
    receive_fail_safe_log()
