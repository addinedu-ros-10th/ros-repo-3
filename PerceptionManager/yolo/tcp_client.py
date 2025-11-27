# tcp_client.py
import socket
import struct

# TCP (메인 컨트롤러에게 결과 전송)
TCP_CTRL_IP = "0.0.0.0"
TCP_CTRL_PORT = 9100        # main controller가 접속할 포트

client = None


def make_update_packet(transaction_id: int, function_id: int, payload: dict):
    """
    transaction_id : 33(controller)
    function_id    : 1(사용자 데이터) 또는 2(영상 판별)
    payload        : function별 데이터(dict)
    """

    # 1) Function ID에 따라 바디 만들기
    if function_id == 1:
        # Function 1 - 카트 사용자 데이터 업데이트
        # BOOL: 0 또는 1
        # 예) payload = {"recognized": True}
        recognized = 1 if payload.get("recognized", False) else 0
        body = struct.pack("<i", recognized)   # 4 bytes
    elif function_id == 2:
        # Function 2 - 영상 판별 데이터 송신
        # 예) payload = {"cam":1, "type":2, "data":0, "posx":123.4, "posy":56.7}
        cam  = int(payload.get("cam",  1))
        type_ = int(payload.get("type", 0))
        data = int(payload.get("data", 0))
        posx = float(payload.get("posx", 0.0))
        posy = float(payload.get("posy", 0.0))
        dist = float(payload.get("dist", 0.0))

        body = struct.pack("<iii fff", cam, type_, data, posx, posy, dist)  # 24 bytes
    else:
        raise ValueError(f"지원하지 않는 function_id: {function_id}")

    # 2) Length_of_data = 바디 길이
    length_of_data = len(body)

    # 3) 헤더 pack (Transaction_ID, Length_of_data, Function_ID)
    header = struct.pack("<iii", transaction_id, length_of_data, function_id)

    # 4) 최종 패킷 = 헤더 + 바디
    packet = header + body

    # 5) 전송
    # SERVER_IP = "192.168.0.180"
    # SERVER_PORT = 6001
    SERVER_IP = "192.168.4.17"
    SERVER_PORT = 3000    
    
    send_to_server(SERVER_IP, SERVER_PORT, packet)

    return packet


def send_to_server(ip, port, packet: bytes):
    global client
    
    """
    서버로 TCP 데이터 전송
    """
    try:
        print(f"서버({ip}:{port})에 연결 중...")
        print("데이터 전송 중...")

        # 길이가 있는 프로토콜이면 sendall 사용이 안전
        client.send(packet)

        # 프로토콜 문서에 'DONE' 같은 문자열을 보내라고 되어 있지 않으면 지우는 게 안전
        client.send(b"DONE")
        print("전송 완료")
            
    except Exception as e:
        print("TCP 통신 오류:", e)
        return None


def send_detection_result(cam_id, obj_type, obj_data, posx, posy, dist):
    function_id = 2
    transaction_id = 33 

    payload = {
        "cam": cam_id,    # 정면: 1, 후면: 2
        "type": obj_type, # 물품:1, 카드 사용자:2, 손동작:3 ...
        "data": obj_data, # 세부 ID 또는 bit mask
        "posx": posx,
        "posy": posy,
        "dist": dist,
    }

    make_update_packet(transaction_id, function_id, payload)


def tcp_server_loop():
    global client
    # SERVER_IP = "192.168.0.180"
    # SERVER_PORT = 6001
    SERVER_IP = "192.168.4.17"
    SERVER_PORT = 3000
    print(f"[TCP] listening on {TCP_CTRL_IP}:{TCP_CTRL_PORT}")
    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    client.connect((SERVER_IP, SERVER_PORT))
