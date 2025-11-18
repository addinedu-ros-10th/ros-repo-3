#!/usr/bin/env python3
import time
import math
import threading

import cv2
import numpy as np
from picamera2 import Picamera2
from ultralytics import YOLO  # pip install ultralytics

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from flask import Flask, Response, jsonify, render_template_string


# ==========================
# 설정값
# ==========================

YOLO_MODEL_PATH = "best.pt"   # 네가 학습한 모델 경로
TARGET_CLASS    = "blue_m"           # 따라갈 대상 클래스 이름

CALIB_FILE = "camera_calibration.npz"

OBJ_REAL_HEIGHT_M = 0.065            # 미니어쳐 실제 높이 6.5cm
TARGET_DIST_M     = 0.40              # 유지하고 싶은 거리 (50cm)
MIN_SAFE_DIST_M   = 0.20             # 너무 가까우면 멈추는 거리

CONF_THRESH = 0.7                    # YOLO confidence threshold

Kp_dist = 0.8                        # 거리 P제어 gain
Kp_yaw  = 1.5                        # yaw P제어 gain

MAX_V = 0.2                         # 최대 전진 속도 (m/s)
MIN_V = 0.03                         # 최소 전진 속도 (dead-zone 방지)
MAX_W = 1.5                          # 최대 회전 속도 (rad/s)

LOST_STOP_SEC = 0.2                  # 이 시간 이상 안 보이면 정지

FLASK_PORT = 8094                    # 디버그 서버 포트


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


# ==========================
# 카메라 스트림
# ==========================

class PiCamStream:
    def __init__(self, width=640, height=480, rotate_180=False):
        self.picam2 = Picamera2()
        cfg = self.picam2.create_video_configuration(
            main={"size": (width, height), "format": "RGB888"}
        )
        self.picam2.configure(cfg)
        self.picam2.start()

        self.rotate_180 = rotate_180
        self.frame = None
        self.lock = threading.Lock()
        self.running = True

        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def _loop(self):
        while self.running:
            img = self.picam2.capture_array()
            if self.rotate_180:
                img = cv2.rotate(img, cv2.ROTATE_180)
            with self.lock:
                self.frame = img
            time.sleep(0.01)

    def get_frame(self):
        with self.lock:
            return None if self.frame is None else self.frame.copy()

    def close(self):
        self.running = False
        try: self.thread.join()
        except: pass
        try: self.picam2.close()
        except: pass


# ==========================
# 미니어쳐 따라가기 노드
# ==========================

class MiniFollowerNode(Node):
    def __init__(self):
        super().__init__("mini_follower_node")

        # cmd_vel 퍼블리셔
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # 캘리브레이션 로드
        data = np.load(CALIB_FILE)
        self.camera_matrix = data["camera_matrix"]
        self.fy = float(self.camera_matrix[1, 1])  # 세로 방향 focal length

        self.get_logger().info(f"[MiniFollower] Loaded calibration. fy = {self.fy:.2f}")

        # 카메라 시작
        self.cam = PiCamStream(width=640, height=480, rotate_180=False)

        # YOLO 모델
        self.model = YOLO(YOLO_MODEL_PATH)
        self.names = self.model.names

        # 타겟 클래스 ID 찾기
        self.target_class_id = None
        for cid, name in self.names.items():
            if name == TARGET_CLASS:
                self.target_class_id = cid
                break
        if self.target_class_id is None:
            raise RuntimeError(f"TARGET_CLASS '{TARGET_CLASS}' not found in model classes {self.names}")

        self.get_logger().info(f"[MiniFollower] Following class: {TARGET_CLASS} (id={self.target_class_id})")

        self.last_seen_time = self.get_clock().now()

        # ------------------------
        # 디버그용 공유 데이터
        # ------------------------
        self.debug_lock = threading.Lock()
        self.debug_frame = None
        self.debug_info = {
            "visible": False,
            "class": TARGET_CLASS,
            "dist": None,
            "v": 0.0,
            "w": 0.0,
            "yaw": None,
            "cx": None,
            "cy": None,
            "h_px": None,
        }

        # 제어 루프 타이머 (20Hz)
        self.timer = self.create_timer(0.05, self.control_loop)

    # ------------------------------
    # 디버그 getter/setter
    # ------------------------------
    def set_debug(self, frame, **info):
        with self.debug_lock:
            self.debug_frame = frame
            for k, v in info.items():
                self.debug_info[k] = v

    def get_debug_frame(self):
        with self.debug_lock:
            return None if self.debug_frame is None else self.debug_frame.copy()

    def get_debug_info(self):
        with self.debug_lock:
            return dict(self.debug_info)

    # ------------------------------
    # YOLO: 타겟 1개 찾기
    # ------------------------------
    def detect_target(self, frame):
        """
        frame: RGB
        return: (cx, cy, h_px, conf, (x1,y1,x2,y2)) or None
        """
        h, w, _ = frame.shape
        bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        results = self.model.predict(bgr, conf=CONF_THRESH, verbose=False)
        if not results:
            return None

        det = results[0]
        if det.boxes is None or len(det.boxes) == 0:
            return None

        best = None
        best_score = -1.0

        for box in det.boxes:
            cls_id = int(box.cls.item())
            if cls_id != self.target_class_id:
                continue

            conf = float(box.conf.item())
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            cx = 0.5 * (x1 + x2)
            cy = 0.5 * (y1 + y2)
            h_px = (y2 - y1)

            # 가장 큰 타겟 또는 conf 높은 타겟
            score = h_px  # 필요하면 conf도 섞을 수 있음
            if score > best_score:
                best_score = score
                best = (cx, cy, h_px, conf, (x1, y1, x2, y2))

        return best

    # ------------------------------
    # 메인 제어 루프
    # ------------------------------
    def control_loop(self):
        frame = self.cam.get_frame()
        if frame is None:
            return

        twist = Twist()
        now = self.get_clock().now()

        target = self.detect_target(frame)

        # 디버그용 오버레이 프레임
        overlay = frame.copy()
        h_img, w_img, _ = frame.shape

        if target is None:
            # 타겟 미발견 → 일정 시간 지나면 정지
            dt_lost = (now - self.last_seen_time).nanoseconds * 1e-9

            # 중앙선 표시만
            cv2.line(overlay,
                     (w_img // 2, 0),
                     (w_img // 2, h_img),
                     (0, 255, 255), 1)

            if dt_lost > LOST_STOP_SEC:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.cmd_pub.publish(twist)

            self.set_debug(
                overlay,
                visible=False,
                dist=None,
                v=0.0,
                w=0.0,
                yaw=None,
                cx=None,
                cy=None,
                h_px=None,
            )
            self.cmd_pub.publish(twist)
            return

        # 타겟이 보임
        self.last_seen_time = now

        cx, cy, h_px, conf, (x1, y1, x2, y2) = target

        # 바운딩 박스 & 중심, 중앙선 표시
        cv2.rectangle(overlay,
                      (int(x1), int(y1)), (int(x2), int(y2)),
                      (0, 255, 0), 2)
        cv2.circle(overlay,
                   (int(cx), int(cy)),
                   4, (0, 0, 255), -1)
        cv2.line(overlay,
                 (w_img // 2, 0),
                 (w_img // 2, h_img),
                 (0, 255, 255), 1)

        # 1) 거리 추정: z ≈ fy * H_real / h_px
        dist = (self.fy * OBJ_REAL_HEIGHT_M) / (h_px + 1e-6)

        # 2) 거리 오차 → v
        e_dist = dist - TARGET_DIST_M

        if dist < MIN_SAFE_DIST_M:
            v = 0.0
        else:
            v = Kp_dist * e_dist
            v = clamp(v, -MAX_V, MAX_V)
            if abs(e_dist) < 0.05:
                v = 0.0
            elif abs(v) < MIN_V:
                v = MIN_V * (1 if v >= 0 else -1)

        # 3) 좌/우 각도: x_err_px → yaw
        x_err_px = cx - (w_img / 2.0)
        yaw = math.atan2(x_err_px, self.fy)

        w = Kp_yaw * yaw
        w = clamp(w, -MAX_W, MAX_W)

        twist.linear.x = v
        twist.angular.z = w
        self.cmd_pub.publish(twist)

        # 디버그 텍스트
        info_text = f"d={dist:.2f}m v={v:.2f} w={w:.2f} yaw={yaw:.2f}"
        cv2.putText(overlay, info_text,
                    (10, h_img - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (255, 255, 255), 2)

        self.set_debug(
            overlay,
            visible=True,
            dist=float(dist),
            v=float(v),
            w=float(w),
            yaw=float(yaw),
            cx=float(cx),
            cy=float(cy),
            h_px=float(h_px),
        )

    def shutdown(self):
        self.cam.close()
        stop = Twist()
        self.cmd_pub.publish(stop)
        self.destroy_node()


# ==========================
# Flask 디버그 서버
# ==========================

app = Flask(__name__)
node_ref: MiniFollowerNode = None

HTML_MAIN = """
<!doctype html>
<html>
<head>
  <meta charset="utf-8"/>
  <title>Mini Follower Debug</title>
  <style>
    body {
      margin:0; padding:0;
      background:#020617;
      color:#e5e7eb;
      font-family:system-ui,-apple-system,BlinkMacSystemFont,"Segoe UI",sans-serif;
    }
    .wrap {
      display:flex;
      flex-direction:row;
      gap:8px;
      padding:8px;
      height:100vh;
      box-sizing:border-box;
    }
    .video-pane {
      flex:2;
      background:#020617;
      border-radius:12px;
      border:1px solid #111827;
      display:flex;
      justify-content:center;
      align-items:center;
      overflow:hidden;
    }
    .video-pane img {
      max-width:100%;
      max-height:100%;
    }
    .info-pane {
      flex:1;
      background:#020617;
      border-radius:12px;
      border:1px solid #111827;
      padding:12px;
      font-size:14px;
      box-sizing:border-box;
    }
    pre {
      background:#020617;
      border-radius:8px;
      border:1px solid #1f2937;
      padding:8px;
      font-size:13px;
      overflow-x:auto;
    }
    h2 {margin-top:0; font-size:18px;}
  </style>
</head>
<body>
  <div class="wrap">
    <div class="video-pane">
      <img src="/video">
    </div>
    <div class="info-pane">
      <h2>Mini Follower Debug</h2>
      <pre id="info">Loading...</pre>
    </div>
  </div>
<script>
async function poll(){
  try{
    const r = await fetch('/info');
    const j = await r.json();
    document.getElementById('info').textContent = JSON.stringify(j,null,2);
  }catch(e){
    document.getElementById('info').textContent = "Error fetching info";
  }
  setTimeout(poll, 300);
}
poll();
</script>
</body>
</html>
"""

@app.route("/")
def main_page():
    return render_template_string(HTML_MAIN)

@app.route("/video")
def video():
    def gen():
        while True:
            if node_ref is None:
                time.sleep(0.1)
                continue
            frame = node_ref.get_debug_frame()
            if frame is None:
                time.sleep(0.03)
                continue
            # RGB → BGR 변환 후 JPEG 인코딩
            bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            ok, buf = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if not ok:
                continue
            jpg = buf.tobytes()
            yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpg + b"\r\n"
            time.sleep(0.03)
    return Response(gen(), mimetype="multipart/x-mixed-replace; boundary=frame")

@app.route("/info")
def info():
    if node_ref is None:
        return jsonify({"status": "no_node"})
    return jsonify(node_ref.get_debug_info())


def ros_thread_fn(node: MiniFollowerNode):
    rclpy.spin(node)
    node.shutdown()


def main():
    global node_ref
    rclpy.init()
    node = MiniFollowerNode()
    node_ref = node

    t = threading.Thread(target=ros_thread_fn, args=(node,), daemon=True)
    t.start()

    app.run(host="0.0.0.0", port=FLASK_PORT, debug=False, threaded=True)


if __name__ == "__main__":
    main()
