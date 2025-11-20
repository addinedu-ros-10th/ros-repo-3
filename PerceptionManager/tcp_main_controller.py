#!/usr/bin/env python3
# tcp_client_controller.py
#
# - YOLO 서버(multi_cam_yolo_server.py)에 TCP 클라이언트로 접속
# - 서버에서 오는 YOLO 결과(type="result")를 계속 읽어 출력
# - 키보드 입력으로 YOLO on/off 제어(type="control" JSON 전송)
#
# 사용법:
#   python3 tcp_client_controller.py
#
#   콘솔에서:
#     0 : cam0 ON only
#     1 : cam1 ON only
#     b : both ON
#     n : both OFF
#     q : quit

import socket
import json
import threading

# ===== 서버 주소 설정 =====
SERVER_IP   = "192.168.4.17"   # YOLO 서버 IP로 바꿔줘
SERVER_PORT = 9100             # multi_cam_yolo_server.py 의 TCP_CTRL_PORT 와 동일하게


# 전역 소켓
sock = None


def handle_result_msg(msg: dict):
    """
    서버에서 온 YOLO 결과 메시지 처리.
    msg 예시:
    {
      "type": "result",
      "cam_id": 0,
      "timestamp": ...,
      "visible": true,
      "dist": 0.52,
      "yaw":  -0.14,
      "conf": 0.87,
      ...
    }
    """
    cam_id = msg.get("cam_id")
    visible = msg.get("visible", False)

    if not visible:
        print(f"[CTRL] cam{cam_id}: target LOST")
        return

    dist = msg.get("dist", None)
    yaw  = msg.get("yaw", None)
    conf = msg.get("conf", None)

    cls_name = msg.get("class", "")
    print(f"[CTRL] cam{cam_id}: {cls_name} dist={dist:.2f} yaw={yaw:.2f}")


    # 여기서부터는 네가 원래 하던 제어 로직(/cmd_vel 등)으로 연결하면 됨.
    # 아래는 예시용 P 제어 스켈레톤.

    TARGET_DIST = 0.5   # m (원하는 거리)
    Kp_dist = 0.8
    Kp_yaw  = 1.5

    if dist is not None and yaw is not None:
        e_dist = dist - TARGET_DIST
        v = Kp_dist * e_dist
        w = -Kp_yaw * yaw   # 화면 기준 오른쪽 타깃 → 음수 회전(오른쪽 도는 방향)

        # TODO: 여기서 v, w를 실제 로봇에 보내는 코드로 교체
        # 예: ROS2라면 /cmd_vel 퍼블리시, 소켓이라면 모터보드로 송신 등
        print(f"      → cmd: v={v:.2f}, w={w:.2f}")


def recv_loop():
    """
    서버에서 오는 TCP 데이터를 줄 단위(JSON 라인)로 읽어서 처리.
    type == "result" 인 메시지는 handle_result_msg 로 보낸다.
    """
    global sock
    buf = b""
    try:
        while True:
            data = sock.recv(4096)
            if not data:
                print("[CTRL] server closed")
                break

            buf += data
            # '\n' 기준으로 한 줄씩 파싱
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                line = line.strip()
                if not line:
                    continue

                try:
                    msg = json.loads(line.decode("utf-8"))
                except Exception as e:
                    print("[CTRL] JSON parse error:", e, line)
                    continue

                mtype = msg.get("type")
                if mtype == "result":
                    handle_result_msg(msg)
                else:
                    print("[CTRL] unknown msg type:", msg)

    except Exception as e:
        print("[CTRL] recv_loop error:", e)
    finally:
        try:
            sock.close()
        except Exception:
            pass
        print("[CTRL] recv_loop terminated")


def send_control(cam0=None, cam1=None):
    """
    YOLO 서버에 제어 메시지를 보냄.
    cam0, cam1: "on" / "off" / None
      예) cam0="on", cam1="off"  → cam0만 YOLO ON, cam1 OFF
    """
    global sock
    msg = {"type": "control"}
    if cam0 is not None:
        msg["cam0"] = cam0
    if cam1 is not None:
        msg["cam1"] = cam1

    data = (json.dumps(msg) + "\n").encode("utf-8")
    try:
        sock.sendall(data)
    except Exception as e:
        print("[CTRL] send_control error:", e)


def input_loop():
    """
    간단한 키보드 인터페이스:
      0 : cam0만 YOLO ON
      1 : cam1만 YOLO ON
      b : cam0, cam1 둘 다 ON
      n : cam0, cam1 둘 다 OFF
      q : 종료
    """
    print("=== Control keys ===")
    print("  0 : cam0 ON only")
    print("  1 : cam1 ON only")
    print("  b : both ON")
    print("  n : both OFF")
    print("  q : quit")
    print("====================")

    while True:
        try:
            cmd = input("> ").strip().lower()
        except EOFError:
            break

        if cmd == "0":
            # cam0 ON, cam1 OFF
            send_control(cam0="on", cam1="off")
        elif cmd == "1":
            # cam0 OFF, cam1 ON
            send_control(cam0="off", cam1="on")
        elif cmd == "b":
            # 둘 다 ON
            send_control(cam0="on", cam1="on")
        elif cmd == "n":
            # 둘 다 OFF
            send_control(cam0="off", cam1="off")
        elif cmd == "q":
            print("[CTRL] quitting...")
            break
        else:
            print("unknown command")


def main():
    global sock
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((SERVER_IP, SERVER_PORT))
    print(f"[CTRL] connected to {SERVER_IP}:{SERVER_PORT}")

    # 결과 수신 스레드 시작
    t_recv = threading.Thread(target=recv_loop, daemon=True)
    t_recv.start()

    # 키보드 입력 루프 (제어)
    input_loop()

    # 입력 루프가 끝나면 소켓 닫기
    try:
        sock.close()
    except Exception:
        pass
    print("[CTRL] client terminated")


if __name__ == "__main__":
    main()
