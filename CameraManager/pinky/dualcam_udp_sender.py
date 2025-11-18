#!/usr/bin/env python3
# 라즈베리파이: 카메라 2대 → UDP 2포트로 JPEG 전송
import socket, struct, time, cv2, numpy as np
from picamera2 import Picamera2
import threading

SERVER_IP = "192.168.4.17"
CHUNK_SIZE   = 1200
JPEG_QUALITY = 60
FPS_LIMIT    = 20
FRAME_MAGIC  = 0xC0DE

def make_sender(camera_id, udp_port, rotate=False, size=(960, 540)):
    cam = Picamera2(camera_id)
    cam.configure(cam.create_preview_configuration(
        main={"format": "RGB888", "size": size}))
    cam.start()
    print(f"[SND{camera_id}] Camera started on port {udp_port}")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    frame_id = 0
    interval = 1.0 / FPS_LIMIT

    def loop():
        nonlocal frame_id
        print(f"[SND{camera_id}] → {SERVER_IP}:{udp_port}  JPEG={JPEG_QUALITY} CHUNK={CHUNK_SIZE}")
        try:
            while True:
                t0 = time.time()
                frame = cam.capture_array()
                if rotate:
                    frame = cv2.rotate(frame, cv2.ROTATE_180)

                ok, enc = cv2.imencode(".jpg", frame,
                                       [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY,
                                        cv2.IMWRITE_JPEG_OPTIMIZE, 1])
                if not ok:
                    continue

                data = enc.tobytes()
                total = (len(data) + CHUNK_SIZE - 1) // CHUNK_SIZE

                for idx in range(total):
                    start = idx * CHUNK_SIZE
                    chunk = data[start:start + CHUNK_SIZE]
                    header = struct.pack("!H I d H H",
                                         FRAME_MAGIC,
                                         frame_id,
                                         time.time(),
                                         idx,
                                         total)
                    sock.sendto(header + chunk, (SERVER_IP, udp_port))

                frame_id = (frame_id + 1) & 0xFFFFFFFF
                dt = time.time() - t0
                if dt < interval:
                    time.sleep(interval - dt)
        except KeyboardInterrupt:
            pass
        finally:
            cam.stop()
            sock.close()
            print(f"[SND{camera_id}] stopped")

    return loop

if __name__ == "__main__":
    # cam0: 앞,  port 5000
    t0 = threading.Thread(target=make_sender(0, 5000, rotate=False), daemon=True)
    # cam1: 뒤/옆, port 5001
    t1 = threading.Thread(target=make_sender(1, 5001, rotate=True), daemon=True)

    t0.start()
    t1.start()

    print("[MAIN] multi_cam_sender running. Press Ctrl+C to stop.")
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("[MAIN] exiting...")
