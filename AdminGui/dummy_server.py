# dummy_server.py
import socket
import struct
import threading
import time

SERVER_IP = "127.0.0.1"     # 로컬 테스트용
PORT_LOGIN = 7001           # 로그인 요청 수신용
PORT_PUSH  = 7002           # 데이터 실시간 push

# ======================================================
# FID 101 : 관리자 로그인 응답 (GUI 패턴과 100% 일치)
# ======================================================
def make_login_response():
    tid = 1
    length = 4
    fid = 101
    admin_id = 1
    return struct.pack("<iii i", tid, length, fid, admin_id)


# ======================================================
# FID 102 : 재고 정보
# ======================================================
def make_item_info_packet():
    tid = 1
    fid = 102
    admin_id = 1

    qty_list = [
        3,4,1,2, 5,1,2,0,
        3,2,1,5, 4,3,2,1
    ]

    header = struct.pack("<iii", tid, 68, fid)
    body   = struct.pack("<i16i", admin_id, *qty_list)
    return header + body


# ======================================================
# FID 103 : 카트 상태 정보
# ======================================================
def make_cart_packet(cid, user, stid, bat, estid, prog, x, y):

    tid = 1
    fid = 103
    length = 32

    header = struct.pack("<iii", tid, length, fid)
    body = struct.pack(
        "<iii f i i f f",
        cid, user, stid, bat,
        estid, prog,
        x, y
    )
    return header + body


# ======================================================
# 로그인 서버 (3001)
# ======================================================
def login_server():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((SERVER_IP, PORT_LOGIN))
        s.listen(5)

        print("[Dummy] Login server listening...")

        while True:
            conn, addr = s.accept()
            print("[Dummy] Login request from:", addr)

            data = conn.recv(1024)
            if len(data) >= 12:
                tid, length, fid = struct.unpack("<iii", data[:12])
                if fid == 1:
                    print("→ 관리자 로그인 요청 수신")
                    conn.sendall(make_login_response())

            conn.close()


# ======================================================
# Broadcaster (3002)
# ======================================================
def push_server():
    while True:
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                s.bind((SERVER_IP, PORT_PUSH))
                s.listen(5)

                print("[Dummy] Push server waiting for GUI...")
                conn, addr = s.accept()
                print("[Dummy] Connected:", addr)

                while True:
                    # FID 102
                    conn.sendall(make_item_info_packet())
                    time.sleep(0.3)

                    # Cart 1
                    conn.sendall(make_cart_packet(
                        cid=1, user=1, stid=5,
                        bat=78.2, estid=0, prog=0,
                        x=0.0, y=0.0
                    ))
                    time.sleep(0.3)

                    # Cart 2
                    conn.sendall(make_cart_packet(
                        cid=2, user=-2, stid=35,
                        bat=55.4, estid=1, prog=0,
                        x=-0.21, y=-1.57
                    ))
                    time.sleep(0.3)

        except Exception as e:
            print("[Dummy Push Error]", e)
            time.sleep(1)


# ======================================================
# MAIN
# ======================================================
def main():
    threading.Thread(target=login_server, daemon=True).start()
    push_server()


if __name__ == "__main__":
    main()


#1: x=0.35, y=0.36
#2: x=0.85, y=0.36
#3: x=1.32, y=0.36
#4: x=1.96, y=0.36
#5: x=1.96, y=0.63
#6: x=0.35, y=0.95
#7: x=0.85, y=0.95
#8: x=1.32, y=0.95
#9: x=1.96, y=0.95
#10: x=1.96, y=1.31
#11: x=0.35, y=1.78
#12: x=0.85, y=1.78
#13: x=1.32, y=1.78
#14: x=1.96, y=1.78
#15: x=1.96, y=2.11
#16: x=0.35, y=2.37
#17: x=0.85, y=2.37
#18: x=1.32, y=2.37
#19: x=1.96, y=2.37
#20: x=0.41, y=2.9
#21: x=1.77, y=2.9