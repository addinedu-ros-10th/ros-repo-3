# tcp_client.py
import socket
import struct
import threading

TCP_CTRL_IP = "0.0.0.0"
TCP_CTRL_PORT = 9100        # main controller가 접속할 포트 (현재는 로그용 정도)
SERVER_IP = "192.168.0.180"
SERVER_PORT = 6001

client = None   # 전역 소켓 (연결 후 유지)


def _tcp_client_loop():
    """
    별도 스레드에서 서버에 한 번 연결해두고,
    전역 client 소켓을 다른 모듈에서 사용.
    """
    global client
    print(f"[TCP] connecting to {SERVER_IP}:{SERVER_PORT} ...")
    try:
        c = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        c.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        c.connect((SERVER_IP, SERVER_PORT))
        client = c
        print("[TCP] connected.")
        # 여기서 굳이 루프 돌 필요 없이 연결만 유지
        while True:
            # 그냥 살아 있는 용도, 필요하면 나중에 heartbeat 등 추가 가능
            # 너무 바쁘지 않게 sleep
            import time
            time.sleep(10)
    except Exception as e:
        print("[TCP] connection error:", e)


def start_tcp_thread():
    """main에서 한 번만 호출해서 TCP 연결 스레드 시작"""
    t = threading.Thread(target=_tcp_client_loop, daemon=True)
    t.start()
    return t


def send_to_server(packet: bytes):
    """
    서버로 TCP 데이터 전송
    """
    global client
    if client is None:
        print("[TCP] client not connected yet.")
        return

    try:
        print("[TCP] sending data...")
        client.sendall(packet)
        client.sendall(b"DONE")   # 프로토콜 명세에 DONE 전송이 있다고 했던 부분
        print("[TCP] send done.")
    except Exception as e:
        print("[TCP] send error:", e)


def make_update_packet(transaction_id: int, function_id: int, payload: dict) -> bytes:
    """
    transaction_id : 33(controller)
    function_id    : 1(사용자 데이터) 또는 2(영상 판별 등)
    payload        : function별 데이터(dict)
    """
    # 1) Function ID에 따라 바디 만들기
    if function_id == 1:
        # Function 1 - 카트 사용자 데이터 업데이트
        recognized = 1 if payload.get("recognized", False) else 0
        body = struct.pack("<i", recognized)   # 4 bytes

    elif function_id == 2:
        # Function 2 - 영상 판별 데이터 예시
        cam = int(payload.get("cam", 0))         # 카메라 ID
        obj_type = int(payload.get("type", 0))   # 물품/사용자/손동작 등
        data = int(payload.get("data", 0))       # 세부 ID 또는 bit mask
        posx = float(payload.get("posx", 0.0))
        posy = float(payload.get("posy", 0.0))
        dist = float(payload.get("dist", 0.0))
        # int,int,int,float,float,float  → <iii fff
        body = struct.pack("<iii fff", cam, obj_type, data, posx, posy, dist)
    else:
        raise ValueError(f"지원하지 않는 function_id: {function_id}")

    # 2) Length_of_data = 바디 길이
    length_of_data = len(body)

    # 3) 헤더 pack (Transaction_ID, Length_of_data, Function_ID)
    header = struct.pack("<iii", transaction_id, length_of_data, function_id)

    # 4) 최종 패킷 = 헤더 + 바디
    packet = header + body
    return packet


# --- 편의 함수들: HSVAnalyzer에서 바로 호출해서 사용 ---

def user_result():
    """
    예: 로그인 성공 → 영상 인식 및 AI 적용 완료
    """
    function_id = 1
    transaction_id = 33
    payload = {"recognized": True}
    packet = make_update_packet(transaction_id, function_id, payload)
    send_to_server(packet)


def send_detection_result(cam_id, obj_type, obj_data, posx, posy, dist):
    """
    cam_id  : 정면:1, 후면:2 ...
    obj_type: 물품:1, 카드 사용자:2, 손동작:3 ...
    obj_data: 세부 ID 또는 bit mask
    posx,posy,dist: 추가 정보
    """
    function_id = 2
    transaction_id = 33
    payload = {
        "cam": cam_id,
        "type": obj_type,
        "data": obj_data,
        "posx": posx,
        "posy": posy,
        "dist": dist,
    }
    packet = make_update_packet(transaction_id, function_id, payload)
    send_to_server(packet)
