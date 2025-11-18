#!/usr/bin/env python3
# dual_cam_server.py
import time
import threading

from flask import Flask, Response, render_template_string
import cv2
from picamera2 import Picamera2


# ========= 카메라 스트림 클래스 =========
class PiCamStream:
    def __init__(self, camera_index=0, width=640, height=480, rotate_180=False):
        self.camera_index = camera_index
        self.width = width
        self.height = height
        self.rotate_180 = rotate_180

        self.picam2 = Picamera2(camera_num=camera_index)
        # [변경①] format=RGB888 그대로 두고, 아래에서 RGB→BGR 변환 안 함
        config = self.picam2.create_video_configuration(
            main={"size": (width, height), "format": "RGB888"}
        )
        self.picam2.configure(config)
        self.picam2.start()

        self.frame = None
        self.lock = threading.Lock()
        self.running = True

        self.thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.thread.start()

    def _capture_loop(self):
        while self.running:
            frame = self.picam2.capture_array()  # RGB888
            if self.rotate_180:
                frame = cv2.rotate(frame, cv2.ROTATE_180)
            with self.lock:
                self.frame = frame
            time.sleep(0.01)

    def get_frame(self):
        with self.lock:
            if self.frame is None:
                return None
            return self.frame.copy()

    def close(self):
        self.running = False
        try:
            self.thread.join(timeout=1.0)
        except:
            pass
        try:
            self.picam2.close()
        except:
            pass


# ========= Flask 앱 & 전역 카메라 =========
app = Flask(__name__)

# ⚠️ 여기서 0, 1은 libcamera-hello --list-cameras 기준 인덱스
# [변경①-2] cam0은 그대로, cam1만 180도 회전시키도록 설정
cam0 = PiCamStream(camera_index=0, width=640, height=480, rotate_180=False)
cam1 = PiCamStream(camera_index=1, width=640, height=480, rotate_180=True)


def mjpeg_generator(cam: PiCamStream, label: str):
    while True:
        frame = cam.get_frame()
        if frame is None:
            time.sleep(0.05)
            continue

        # [변경②] RGB→BGR 변환 제거 (파란 화면 방지)
        frame_draw = frame  # RGB 그대로 사용

        # 텍스트 색은 G 채널만 쓰기 때문에 색 뒤집힘 문제 없음
        cv2.putText(
            frame_draw, label, (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2, cv2.LINE_AA
        )

        ok, buf = cv2.imencode(".jpg", frame_draw, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if not ok:
            continue
        jpg = buf.tobytes()

        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n" + jpg + b"\r\n"
        )
        time.sleep(0.03)


# ========= 라우트 =========
@app.route("/cam0/video")
def cam0_video():
    return Response(
        mjpeg_generator(cam0, "CAM0"),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )


@app.route("/cam1/video")
def cam1_video():
    return Response(
        mjpeg_generator(cam1, "CAM1"),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )


# [변경③] 두 화면 나란히(2분할) 레이아웃 HTML
HTML_MAIN = """
<!doctype html>
<html>
<head>
  <meta charset="utf-8"/>
  <title>Dual Camera View</title>
  <style>
    body {
      margin: 0;
      padding: 0;
      background: #020617;
      color: #e5e7eb;
      font-family: system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
    }
    .wrap {
      display: flex;
      flex-direction: row;
      justify-content: center;
      align-items: stretch;
      height: 100vh;
      gap: 8px;
      padding: 8px;
      box-sizing: border-box;
      background: radial-gradient(circle at top, #1f2937, #020617);
    }
    .cam-pane {
      flex: 1 1 0;
      display: flex;
      flex-direction: column;
      background: rgba(15,23,42,0.8);
      border-radius: 12px;
      border: 1px solid #111827;
      overflow: hidden;
      box-shadow: 0 18px 45px rgba(0,0,0,0.65);
      min-width: 0;
    }
    .cam-header {
      padding: 6px 10px;
      font-size: 13px;
      letter-spacing: 0.08em;
      text-transform: uppercase;
      color: #9ca3af;
      background: linear-gradient(to right, rgba(15,23,42,0.95), rgba(30,64,175,0.6));
      border-bottom: 1px solid rgba(15,23,42,0.9);
    }
    .cam-header span {
      color: #38bdf8;
      font-weight: 500;
    }
    .cam-body {
      flex: 1;
      display: flex;
      align-items: center;
      justify-content: center;
      background: #020617;
    }
    .cam-body img {
      max-width: 100%;
      max-height: 100%;
      display: block;
    }
  </style>
</head>
<body>
  <div class="wrap">
    <div class="cam-pane">
      <div class="cam-header">Primary · <span>CAM0</span></div>
      <div class="cam-body">
        <img src="/cam0/video" />
      </div>
    </div>
    <div class="cam-pane">
      <div class="cam-header">Secondary · <span>CAM1 (rotated)</span></div>
      <div class="cam-body">
        <img src="/cam1/video" />
      </div>
    </div>
  </div>
</body>
</html>
"""

@app.route("/main")
def main_page():
    return render_template_string(HTML_MAIN)


# ========= 종료 처리 =========
def cleanup():
    cam0.close()
    cam1.close()

if __name__ == "__main__":
    try:
        print("[INFO] Dual camera server running at http://0.0.0.0:8092/main")
        app.run(host="0.0.0.0", port=8092, threaded=True)
    finally:
        cleanup()
