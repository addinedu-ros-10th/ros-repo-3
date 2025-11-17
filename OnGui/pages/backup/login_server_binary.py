
import socket

def start_server(ip="127.0.0.1", port=9500):
    # 소켓 생성 (TCP)
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
        server.bind((ip, port))
        server.listen(5)  # 동시에 최대 5개의 접속 대기 가능
        print(f"서버가 {ip}:{port}에서 대기 중...")

        while True:  # ★ 서버가 계속 동작하도록 반복문 유지
            print("클라이언트 접속 기다리는 중...")
            conn, addr = server.accept()  # 클라이언트 연결 수락

            # 연결된 클라이언트와 통신
            with conn:
                print(f"클라이언트 연결됨: {addr}")

                # 데이터 수신
                data = conn.recv(1024)
                if not data:
                    print("수신된 데이터 없음. 연결 종료.")
                    continue

                print("수신한 데이터(hex):", data.hex())

                # 응답 전송
                conn.sendall(b"ACK")
                print("ACK 응답 전송 완료")

                # 연결 종료 후 루프 다시 돌아가서 대기
                print("클라이언트 연결 종료.\n")


if __name__ == "__main__":
    start_server()

