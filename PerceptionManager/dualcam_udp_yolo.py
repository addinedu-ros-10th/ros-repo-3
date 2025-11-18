#!/usr/bin/env python3
# multi_cam_yolo_server.py
import socket, struct, threading, time, math, json
import cv2, numpy as np
from ultralytics import YOLO
import torch
from flask import Flask, request, jsonify, render_template_string, Response

# ===================== 설정 =====================
FRAME_MAGIC = 0xC0DE

UDP_PORT_CAM0 = 5000
UDP_PORT_CAM1 = 5001
MAX_PACKET_SIZE = 1500

# YOLO 모델
YOLO_MODEL_PATH = "best.pt"   # 네가 학습한 pt
TARGET_CLASS_NAME = "blue_w"         # 따라갈 기본 타깃 클래스
YOLO_CONF = 0.6
DEVICE = 0 if torch.cuda.is_available() else "cpu"

# 카메라 캘리브레이션
CALIB_FILE_CAM0 = "camera_calibration.npz"   # 정면 카메라
CALIB_FILE_CAM1 = "camera_calibration_back.npz"    # 뒤 카메라

# 각 클래스별 실제 높이 (미터 단위)
CLASS_REAL_HEIGHTS = {
    "red_m":  0.065,  # 6.5cm
    "red_w":  0.065,  # 6.5cm
    "blue_m": 0.055,  # 5.5cm
    "blue_w": 0.055,  # 5.5cm
}
DEFAULT_HEIGHT_M = 0.06     # 혹시 매핑 안되면 쓸 기본값


# TCP (메인 컨트롤러에게 결과 전송)
TCP_CTRL_IP = "0.0.0.0"
TCP_CTRL_PORT = 9100        # main controller가 접속할 포트


# ===================== 공유 상태 =====================
latest_frame_cam0 = None
latest_frame_cam1 = None
frame_lock_cam0 = threading.Lock()
frame_lock_cam1 = threading.Lock()

# YOLO ON/OFF
enable_cam0 = True
enable_cam1 = False
mode_lock = threading.Lock()

# 디버그 프레임
debug_frame_cam0 = None
debug_frame_cam1 = None
debug_lock = threading.Lock()

# TCP 클라이언트 리스트 (브로드캐스트용)
tcp_clients = []
tcp_clients_lock = threading.Lock()

app = Flask(__name__)

def handle_client_recv(conn: socket.socket, addr):
    """
    해당 클라이언트에서 오는 JSON 라인을 계속 읽으면서
    type == 'control' 메시지를 처리한다.
    """
    buf = b""
    try:
        while True:
            data = conn.recv(4096)
            if not data:
                print("[TCP] client disconnected:", addr)
                break
            buf += data
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                line = line.strip()
                if not line:
                    continue
                try:
                    msg = json.loads(line.decode("utf-8"))
                except Exception as e:
                    print("[TCP] JSON parse error from", addr, ":", e)
                    continue

                if msg.get("type") == "control":
                    handle_control_message(msg)
                # 필요하면 다른 타입 추가 가능 (예: 'ping', 'config' 등)

    except Exception as e:
        print("[TCP] client error:", addr, e)
    finally:
        # 보내기 리스트에서 제거
        with tcp_clients_lock:
            if conn in tcp_clients:
                tcp_clients.remove(conn)
        try:
            conn.close()
        except Exception:
            pass
        print("[TCP] closed connection:", addr)


def handle_control_message(msg: dict):
    """
    클라이언트로부터 받은 제어 메시지 처리.
    예시:
    { "type": "control", "cam0": "on", "cam1": "off" }
    """
    global enable_cam0, enable_cam1
    cam0 = msg.get("cam0", "").lower()
    cam1 = msg.get("cam1", "").lower()

    with mode_lock:
        if cam0 in ["on", "off"]:
            enable_cam0 = (cam0 == "on")
        if cam1 in ["on", "off"]:
            enable_cam1 = (cam1 == "on")

        st = {"cam0": enable_cam0, "cam1": enable_cam1}
    print("[CTRL/TCP] mode changed by client:", st)


def send_to_all_clients(msg_dict):
    """YOLO 결과를 JSON 라인으로 모든 TCP 클라이언트에 전송"""
    data = (json.dumps(msg_dict) + "\n").encode("utf-8")
    dead = []
    with tcp_clients_lock:
        for s in tcp_clients:
            try:
                s.sendall(data)
            except Exception:
                dead.append(s)
        for d in dead:
            try:
                d.close()
            except Exception:
                pass
            tcp_clients.remove(d)


# ===================== UDP 수신 (JPEG 조립) =====================
def udp_recv_loop(port, cam_id):
    """
    cam_id: 0 또는 1
    port: cam0용(5000), cam1용(5001)
    """
    global latest_frame_cam0, latest_frame_cam1

    packet_buffer = {}  # frame_id -> [chunks]
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", port))
    sock.settimeout(2.0)
    print(f"[UDP{cam_id}] listening on :{port}")

    while True:
        try:
            data, _ = sock.recvfrom(MAX_PACKET_SIZE)
            if len(data) < 18:
                continue

            magic, fid, ts, idx, total = struct.unpack("!H I d H H", data[:18])
            if magic != FRAME_MAGIC:
                continue
            payload = data[18:]

            if fid not in packet_buffer:
                packet_buffer[fid] = [None] * total
            if 0 <= idx < total:
                packet_buffer[fid][idx] = payload

            buf = packet_buffer[fid]
            if all(chunk is not None for chunk in buf):
                jpg = b"".join(buf)
                arr = np.frombuffer(jpg, dtype=np.uint8)
                frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                if frame is not None:
                    if cam_id == 0:
                        with frame_lock_cam0:
                            latest_frame_cam0 = frame
                    else:
                        with frame_lock_cam1:
                            latest_frame_cam1 = frame
                del packet_buffer[fid]

                # 너무 쌓이면 통째로 비워서 메모리 보호
                if len(packet_buffer) > 100:
                    packet_buffer.clear()

        except socket.timeout:
            continue
        except Exception as e:
            print(f"[UDP{cam_id} ERR]", e)

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

        body = struct.pack("<iii ff", cam, type_, data, posx, posy)  # 20 bytes
    else:
        raise ValueError(f"지원하지 않는 function_id: {function_id}")

    # 2) Length_of_data = 바디 길이
    length_of_data = len(body)

    # 3) 헤더 pack (Transaction_ID, Length_of_data, Function_ID)
    header = struct.pack("<iii", transaction_id, length_of_data, function_id)

    # 4) 최종 패킷 = 헤더 + 바디
    packet = header + body

    # 5) 전송
    SERVER_IP = "192.168.4.17"
    SERVER_PORT = 3000
    send_to_server(SERVER_IP, SERVER_PORT, packet)

    return packet

def send_to_server(ip, port, packet: bytes):
    """
    서버로 TCP 데이터 전송
    """
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client:
            print(f"서버({ip}:{port})에 연결 중...")
            client.connect((ip, port))
            print("데이터 전송 중...")

            # 길이가 있는 프로토콜이면 sendall 사용이 안전
            client.sendall(packet)

            # 프로토콜 문서에 'DONE' 같은 문자열을 보내라고 되어 있지 않으면 지우는 게 안전
            client.sendall(b"DONE")
            print("전송 완료")

            response = client.recv(1024)
            print("서버 응답:", response)

    except Exception as e:
        print("TCP 통신 오류:", e)
        return None

def send_detection_result(cam_id, obj_type, obj_data, posx, posy):
    function_id = 2
    transaction_id = 33 

    payload = {
        "cam": cam_id,    # 정면: 1, 후면: 2
        "type": obj_type, # 물품:1, 카드 사용자:2, 손동작:3 ...
        "data": obj_data, # 세부 ID 또는 bit mask
        "posx": posx,
        "posy": posy
    }

    make_update_packet(transaction_id, function_id, payload)

# ===================== YOLO 루프 =====================
def yolo_loop():
    global debug_frame_cam0, debug_frame_cam1

    # ===== 캘리브레이션 로드 (카메라별) =====
    data0 = np.load(CALIB_FILE_CAM0)
    camera_matrix_cam0 = data0["camera_matrix"]
    fy_cam0 = float(camera_matrix_cam0[1, 1])
    print(f"[YOLO] Cam0 calib loaded from {CALIB_FILE_CAM0}, fy0={fy_cam0:.2f}")

    data1 = np.load(CALIB_FILE_CAM1)
    camera_matrix_cam1 = data1["camera_matrix"]
    fy_cam1 = float(camera_matrix_cam1[1, 1])
    print(f"[YOLO] Cam1 calib loaded from {CALIB_FILE_CAM1}, fy1={fy_cam1:.2f}")

    # YOLO 모델 로드
    model = YOLO(YOLO_MODEL_PATH)
    names = model.names
    print(f"[YOLO] classes = {names}, device={DEVICE}")

    # 타깃 클래스 (팔로잉용 기본 타깃 하나 정해두는 용도)
    target_id = None
    if TARGET_CLASS_NAME is not None:
        for cid, name in names.items():
            if name == TARGET_CLASS_NAME:
                target_id = cid
                break
        if target_id is None:
            print(f"[YOLO] WARNING: TARGET_CLASS_NAME '{TARGET_CLASS_NAME}' not found in classes")
        else:
            print(f"[YOLO] target class: {TARGET_CLASS_NAME} (id={target_id})")

    while True:
        # 어떤 카메라에 YOLO를 돌릴지 확인
        with mode_lock:
            use_cam0 = enable_cam0
            use_cam1 = enable_cam1

        frames = []
        cam_ids = []

        # cam0 프레임 준비
        if use_cam0:
            with frame_lock_cam0:
                f0 = None if latest_frame_cam0 is None else latest_frame_cam0.copy()
            if f0 is not None:
                frames.append(f0)
                cam_ids.append(0)

        # cam1 프레임 준비
        if use_cam1:
            with frame_lock_cam1:
                f1 = None if latest_frame_cam1 is None else latest_frame_cam1.copy()
            if f1 is not None:
                frames.append(f1)
                cam_ids.append(1)

        if not frames:
            time.sleep(0.02)
            continue

        # YOLO 배치 추론
        try:
            results = model.predict(
                source=frames,
                conf=YOLO_CONF,
                verbose=False,
                imgsz=640,
                device=DEVICE
            )
        except Exception as e:
            print("[YOLO ERR]", e)
            time.sleep(0.05)
            continue

        # 카메라별 결과 처리
        for res, cid, frame in zip(results, cam_ids, frames):
            h, w, _ = frame.shape

            # ---- 여기서 카메라별 fy 선택 ----
            if cid == 0:
                fy = fy_cam0
            else:
                fy = fy_cam1
            # --------------------------------

            best = None
            best_score = -1.0

            best_cls_id = None
            best_cls_name = None

            if res.boxes is not None and len(res.boxes) > 0:
                for b in res.boxes:
                    cls_id = int(b.cls.item())
                    conf   = float(b.conf.item())
                    x1, y1, x2, y2 = b.xyxy[0].tolist()
                    cx = 0.5 * (x1 + x2)
                    cy = 0.5 * (y1 + y2)
                    h_px = (y2 - y1)

                    cls_name = names.get(cls_id, str(cls_id))
                    label = f"{cls_name} {conf:.2f}"

                    # 디버깅: 모든 박스 표시
                    cv2.rectangle(frame,
                                  (int(x1), int(y1)), (int(x2), int(y2)),
                                  (0, 255, 0), 2)
                    cv2.putText(frame, label,
                                (int(x1), max(0, int(y1) - 6)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                (0, 255, 0), 2, cv2.LINE_AA)

                    # 타깃 선택 기준:
                    # - TARGET_CLASS_NAME 설정되어 있으면 그 클래스만 사용
                    if target_id is not None and cls_id != target_id:
                        continue

                    # 가장 큰(가까운) 타깃 1개 선택
                    score = h_px
                    if score > best_score:
                        best_score   = score
                        best         = (cx, cy, h_px, conf)
                        best_cls_id  = cls_id
                        best_cls_name = cls_name

            # 타깃이 있으면 거리/yaw 계산 + TCP 전송
            if best is not None:
                cx, cy, h_px, conf = best
                
                if cid == 0:
                    send_detection_result(2, 2, 1, cx, cy)
                else :
                    send_detection_result(1, 2, 1, cx, cy)

                # 1) 클래스 이름에 따라 실제 높이 선택
                if best_cls_name in CLASS_REAL_HEIGHTS:
                    real_h = CLASS_REAL_HEIGHTS[best_cls_name]
                else:
                    real_h = DEFAULT_HEIGHT_M

                # 2) 거리 계산 (카메라별 fy 사용)
                dist = (fy * real_h) / (h_px + 1e-6)

                # 3) yaw 계산
                x_err_px = cx - (w / 2.0)
                yaw = math.atan2(x_err_px, fy)

                # 디버그 라인/텍스트 (서버 화면에 거리도 같이 표시)
                cv2.line(frame, (w // 2, 0), (w // 2, h), (0, 255, 255), 1)
                info = f"{best_cls_name} d={dist:.2f}m yaw={yaw:.2f}rad"
                cv2.putText(frame, info,
                            (10, h - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                            (255, 255, 255), 2)
                print("[Debug] cam", cid, "frame size =", frame.shape)


                # 4) TCP로 main controller에 전송 (거리 포함)
                msg = {
                    "type": "result",
                    "cam_id": cid,
                    "timestamp": time.time(),
                    "visible": True,
                    "class": best_cls_name,
                    "dist": float(dist),
                    "yaw": float(yaw),
                    "conf": float(conf),
                    "cx": float(cx),
                    "cy": float(cy),
                    "h_px": float(h_px),
                    "real_height_m": float(real_h),
                }
                send_to_all_clients(msg)
            else:
                msg = {
                    "type": "result",
                    "cam_id": cid,
                    "timestamp": time.time(),
                    "visible": False,
                }
                send_to_all_clients(msg)

            # 디버그 프레임 업데이트
            with debug_lock:
                if cid == 0:
                    debug_frame_cam0 = frame.copy()
                else:
                    debug_frame_cam1 = frame.copy()

        time.sleep(0.01)
        



# ===================== TCP 서버 (메인 컨트롤러용) =====================
def tcp_server_loop():
    print(f"[TCP] listening on {TCP_CTRL_IP}:{TCP_CTRL_PORT}")
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((TCP_CTRL_IP, TCP_CTRL_PORT))
    srv.listen(5)

    while True:
        conn, addr = srv.accept()
        print("[TCP] client connected:", addr)
        conn.setblocking(True)

        # 보내기용 리스트 등록
        with tcp_clients_lock:
            tcp_clients.append(conn)

        # 이 클라이언트로부터 제어 메시지를 읽을 스레드 시작
        t = threading.Thread(target=handle_client_recv, args=(conn, addr), daemon=True)
        t.start()



# ===================== Flask (제어 + 디버그) =====================
HTML_MAIN = """
<!doctype html>
<html>
<head>
  <meta charset="utf-8"/>
  <title>Multi-Cam YOLO Server</title>
  <style>
    body{
      margin:0; padding:0;
      background:#020617;
      color:#e5e7eb;
      font-family:system-ui,-apple-system,BlinkMacSystemFont,"Segoe UI",sans-serif;
    }
    .topbar{
      padding:10px 16px;
      border-bottom:1px solid #111827;
      display:flex;
      justify-content:space-between;
      align-items:center;
    }
    .cams{
      display:flex;
      gap:8px;
      padding:8px;
      height:calc(100vh - 50px);
      box-sizing:border-box;
    }
    .cam-box{
      flex:1;
      background:#020617;
      border-radius:12px;
      border:1px solid #111827;
      padding:8px;
      box-sizing:border-box;
      display:flex;
      flex-direction:column;
    }
    .cam-box h2{
      margin:0 0 8px 0;
      font-size:16px;
    }
    .cam-box img{
      flex:1;
      max-width:100%;
      max-height:100%;
      object-fit:contain;
      background:#000;
      border-radius:8px;
    }
    button{
      padding:4px 10px;
      margin-right:4px;
      border-radius:8px;
      border:none;
      cursor:pointer;
      background:#1f2937;
      color:#e5e7eb;
      font-size:12px;
    }
    button.on{
      background:#2563eb;
    }
  </style>
</head>
<body>
  <div class="topbar">
    <div>Multi-Cam YOLO Server</div>
    <div>
      <button onclick="setMode('cam0_on')">Cam0 ON only</button>
      <button onclick="setMode('cam1_on')">Cam1 ON only</button>
      <button onclick="setMode('both_on')">Both ON</button>
      <button onclick="setMode('both_off')">Both OFF</button>
    </div>
  </div>
  <div class="cams">
    <div class="cam-box">
      <h2>Cam0</h2>
      <img src="/cam0/video">
    </div>
    <div class="cam-box">
      <h2>Cam1</h2>
      <img src="/cam1/video">
    </div>
  </div>
<script>
async function setMode(mode){
  let params;
  if(mode === 'cam0_on') params = 'cam0=on&cam1=off';
  else if(mode === 'cam1_on') params = 'cam0=off&cam1=on';
  else if(mode === 'both_on') params = 'cam0=on&cam1=on';
  else if(mode === 'both_off') params = 'cam0=off&cam1=off';
  try{
    await fetch('/control?' + params);
  }catch(e){
    console.log(e);
  }
}
</script>
</body>
</html>
"""

@app.route("/")
def index():
    return render_template_string(HTML_MAIN)


@app.route("/status")
def status():
    with mode_lock:
        st = {"cam0": enable_cam0, "cam1": enable_cam1}
    return jsonify(st)


@app.route("/control")
def control():
    global enable_cam0, enable_cam1
    cam0 = request.args.get("cam0", "").lower()
    cam1 = request.args.get("cam1", "").lower()

    with mode_lock:
        if cam0 in ["on", "off"]:
            enable_cam0 = (cam0 == "on")
        if cam1 in ["on", "off"]:
            enable_cam1 = (cam1 == "on")
        st = {"cam0": enable_cam0, "cam1": enable_cam1}
    print("[CTRL] mode:", st)
    return jsonify(st)


def mjpeg_gen_cam(cid):
    global debug_frame_cam0, debug_frame_cam1
    while True:
        with debug_lock:
            if cid == 0:
                frame = None if debug_frame_cam0 is None else debug_frame_cam0.copy()
            else:
                frame = None if debug_frame_cam1 is None else debug_frame_cam1.copy()
        if frame is None:
            time.sleep(0.03)
            continue
        ok, enc = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if not ok:
            time.sleep(0.03)
            continue
        jpg = enc.tobytes()
        yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" +
               jpg + b"\r\n")
        time.sleep(0.03)


@app.route("/cam0/video")
def cam0_video():
    return Response(mjpeg_gen_cam(0),
                    mimetype="multipart/x-mixed-replace; boundary=frame")


@app.route("/cam1/video")
def cam1_video():
    return Response(mjpeg_gen_cam(1),
                    mimetype="multipart/x-mixed-replace; boundary=frame")


# ===================== main =====================
def main():
    # UDP 수신 스레드 2개
    t_udp0 = threading.Thread(target=udp_recv_loop, args=(UDP_PORT_CAM0, 0), daemon=True)
    t_udp1 = threading.Thread(target=udp_recv_loop, args=(UDP_PORT_CAM1, 1), daemon=True)
    t_udp0.start()
    t_udp1.start()

    # YOLO 스레드
    t_yolo = threading.Thread(target=yolo_loop, daemon=True)
    t_yolo.start()

    # TCP 서버 스레드
    t_tcp = threading.Thread(target=tcp_server_loop, daemon=True)
    t_tcp.start()

    print(f"[HTTP] Flask: http://0.0.0.0:8000/")
    app.run(host="0.0.0.0", port=8000, threaded=True)


if __name__ == "__main__":
    main()
