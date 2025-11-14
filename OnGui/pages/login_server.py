
import socket
import struct

# 등록된 사용자 목록 (예시)
REGISTERED_USERS = ["1", "user", "test"]

# SERVER_IP = "0.0.0.0"
# SERVER_PORT = 9500


def handle_client(conn, addr):
    print(f"[클라이언트 접속] {addr}")

    try:
        # --------------------------
        # 1) 첫 1바이트 읽기: 길이 값인지? 아니면 문자 데이터인지?
        # --------------------------
        first_byte = conn.recv(1)

        if not first_byte:
            print("클라이언트가 데이터를 보내지 않음.")
            return

        # 바이트 값을 정수로 변환
        byte_value = first_byte[0]

        # ------------------------------------------------------
        # 🔹 바이너리 방식으로 보내진 경우
        #    조건: 첫 바이트 값이 문자열 길이와 동일해야 한다 (0~255)
        # ------------------------------------------------------
        if 1 <= byte_value <= 20:
            print("[바이너리 로그인 요청 수신]")

            username_length = byte_value  # username 길이
            username_bytes = conn.recv(username_length)

            username = username_bytes.decode("utf-8")
            print(f"바이너리 Username 수신 → {username}")

        # ------------------------------------------------------
        # 🔹 일반 문자열 방식으로 보내진 경우
        #    조건: 첫 바이트부터 바로 UTF-8 문자열로 간주
        # ------------------------------------------------------
        else:
            print("[문자열 로그인 요청 수신]")

            # 첫 바이트 포함하여 나머지도 수신
            rest = conn.recv(1024)
            username = (first_byte + rest).decode("utf-8").strip()
            print(f"문자열 Username 수신 → {username}")

        # ------------------------------------------------------
        # 🔹 사용자 검증 후 응답 전송
        # ------------------------------------------------------
        if username in REGISTERED_USERS:
            conn.sendall("OK".encode('utf-8'))
        else:
            conn.sendall("FAIL".encode('utf-8'))

    except Exception as e:
        print("서버 오류:", e)

    finally:
        conn.close()
        print(f"[클라이언트 종료] {addr}")


def start_server():
    SERVER_IP = "0.0.0.0"
    SERVER_PORT = 9500

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind((SERVER_IP, SERVER_PORT))
    server.listen(5)

    print(f"서버 시작: {SERVER_IP}:{SERVER_PORT}")

    while True:
        conn, addr = server.accept()
        handle_client(conn, addr)


if __name__ == "__main__":
    start_server()

