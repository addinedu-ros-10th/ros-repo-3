#!/usr/bin/env python3
import time
import threading
import signal
import math
import cv2
import numpy as np
from picamera2 import Picamera2

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from flask import Flask, Response, jsonify


# ==========================================================
# CONFIG
# ==========================================================

CAM_INDEX = 1
ROTATE_180_IMAGE = True

if CAM_INDEX == 1:
    CALIB_FILE = "camera_calibration.npz"
else:
    CALIB_FILE = "camera_calibration_back.npz"

ARUCO_DICT_TYPE = cv2.aruco.DICT_4X4_250
MARKER_SIZE_M   = 0.05

TARGET_MARKER_ID = 2


# 최종 주차 조건
TARGET_DIST_M = 0.125
DIST_TOL_M    = 0.006
LAT_TOL_M     = 0.018
YAW_TOL       = 0.04   # rad

# 속도/게인
MAX_V = 0.12
MIN_V = 0.03
MAX_W = 3.5

# slide 단계: 마커를 화면 끝에 유지하며 호를 그리며 이동
SLIDE_V      = 0.07          # slide 동안 전진 속도
K_EDGE       = 7           # 마커의 가로 위치(픽셀)에 대한 회전 게인
SLIDE_LAT_TOL = 0.008
EDGE_FRAC    = 0.8           # 타겟 위치: 화면 폭*0.2 또는 *0.8 지점
# slide 단계에서 x 크기에 따라 edge ↔ center 보간
X_REF_FOR_SLIDE = 0.18   # |x|가 이 정도 이상이면 '크다'고 보고 edge를 더 강하게 사용

# approach 단계: 수직선 위에서 직선 접근
APPROACH_V   = 0.08
K_DIST_APP   = 0.8

# final_align 단계: 정밀 보정
Kp_yaw_final = 1.3
Kp_lat_final = 2.0
Kp_dist_final= 0.6
Ki_dist_final= 0.3

# done 이후 90도 회전
ROTATE_SPEED = 1.35          # rad/s
ROTATE_ANGLE = math.pi * 1.06  # 90 degrees

# 마커를 잠시 놓쳤을 때, 마지막 자세를 얼마나 유지할지 (초)
LOST_TIMEOUT = 0.15   # 0.3초 정도, 필요하면 0.2~0.5 사이에서 튜닝
# slide 단계에서 x도 같이 줄이기 위한 회전 게인
K_X_SLIDE = 3.5   # 필요하면 1.0~3.0 사이에서 튜닝



def clamp(x, lo, hi):
    return max(lo, min(hi, x))


# ==========================================================
# CAMERA STREAM
# ==========================================================

class PiCamStream:
    def __init__(self, camera_index=0, width=640, height=480, rotate_180=False):
        self.picam2 = Picamera2(camera_num=camera_index)
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


# ==========================================================
# ARUCO PARKING NODE (새 FSM)
# ==========================================================

class ArucoParkingNode(Node):
    def __init__(self):
        super().__init__("aruco_parking_node")

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # Calibration
        data = np.load(CALIB_FILE)
        self.camera_matrix = data["camera_matrix"]
        self.dist_coeffs   = data["distortion_coefficients"]

        self.cam = PiCamStream(CAM_INDEX, 640, 480, ROTATE_180_IMAGE)

        # ArUco detector
        self.dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICT_TYPE)
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.parameters)

        # FSM
        # search → slide → approach → final_align → done
        self.state = "search"
        self.slide_side = None         # "left" or "right"
        self.edge_u_target = None      # 화면 내 타겟 x 픽셀 위치

        # final_align용
        self.dist_integral = 0.0
        self.align_ok_time = None

        self.last_time = self.get_clock().now()

        # Debug shared
        self.debug_frame = None
        self.debug_lock = threading.Lock()
        self.debug_info = {
            "state": self.state,
            "slide_side": self.slide_side,
            "marker_id": None,
            "x": None,
            "z": None,
            "yaw": None,
            "u": None,
            "v_cmd": 0.0,
            "w_cmd": 0.0,
        }
        
        self.done_start_time = None
        # slide 단계에서 마커를 잃었을 때 타이머
        # ── 마지막으로 유효하게 본 마커 포즈/픽셀 위치 저장 ──
        self.last_valid_res      = None   # (mid, x, y, z)
        self.last_valid_rvec     = None
        self.last_valid_center   = None   # (u_center, v_center)
        self.last_valid_img_w    = None
        self.last_valid_img_h    = None
        self.last_seen_time      = None   # time.time()



        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info("ArucoParkingNode (new slide-based FSM) started.")

    # ------------------------------------------------------
    # DEBUG SHARED
    # ------------------------------------------------------
    def set_debug_frame(self, frame):
        with self.debug_lock:
            self.debug_frame = frame

    def get_debug_frame(self):
        with self.debug_lock:
            return None if self.debug_frame is None else self.debug_frame.copy()

    def set_debug_info(self, **kw):
        with self.debug_lock:
            for k, v in kw.items():
                self.debug_info[k] = v

    def get_debug_info(self):
        with self.debug_lock:
            return dict(self.debug_info)

    # ------------------------------------------------------
    # ARUCO DETECTION
    # ------------------------------------------------------
    def detect_marker_pose_and_center(self, frame, draw=True):
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)

        h, w = frame.shape[:2]
        overlay = frame.copy()

        if ids is None:
            return None, overlay, None, None, w, h

        ids = ids.flatten()
        obj = np.array([
            [-MARKER_SIZE_M/2,  MARKER_SIZE_M/2, 0],
            [ MARKER_SIZE_M/2,  MARKER_SIZE_M/2, 0],
            [ MARKER_SIZE_M/2, -MARKER_SIZE_M/2, 0],
            [-MARKER_SIZE_M/2, -MARKER_SIZE_M/2, 0]
        ], np.float32)

        best = None
        best_idx = None
        best_z = 999
        best_r = None
        best_t = None

        for i, mid in enumerate(ids):
            if TARGET_MARKER_ID is not None and mid != TARGET_MARKER_ID:
                continue

            ok, rvec, tvec = cv2.solvePnP(
                obj, corners[i][0],
                self.camera_matrix, self.dist_coeffs
            )
            if not ok:
                continue

            x, y, z = tvec.flatten()
            if z < best_z:
                best = (int(mid), x, y, z)
                best_idx = i
                best_z = z
                best_r = rvec
                best_t = tvec

        if best is None:
            return None, overlay, None, None, w, h

        # marker 중심 픽셀 계산
        pts = corners[best_idx][0]  # (4,2)
        u_center = float(np.mean(pts[:, 0]))
        v_center = float(np.mean(pts[:, 1]))

        if draw:
            cv2.aruco.drawDetectedMarkers(overlay, [corners[best_idx]])
            cv2.drawFrameAxes(overlay, self.camera_matrix,
                              self.dist_coeffs, best_r, best_t, 0.03)
            # 중심점도 찍기
            cv2.circle(overlay, (int(u_center), int(v_center)), 4, (0,255,0), -1)

        return best, overlay, best_r, (u_center, v_center), w, h

    # ------------------------------------------------------
    def control_loop(self):
        frame = self.cam.get_frame()
        if frame is None:
            return

        twist = Twist()
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        if dt <= 0:
            dt = 1e-3
        self.last_time = now

        res, overlay, rvec, center_uv, img_w, img_h = \
            self.detect_marker_pose_and_center(frame)

        # ─────────────────────────────────────────────
        # 1) 마커를 본 적이 있으면 마지막 유효 포즈를 저장
        # 2) 잠깐(LOST_TIMEOUT 이내) 놓치면 마지막 포즈를 재사용
        # ─────────────────────────────────────────────
        now_wall = time.time()

        if res is not None and rvec is not None and center_uv is not None:
            # 이번 프레임에서 정상적으로 마커를 봤다면 그대로 저장
            self.last_valid_res    = res
            self.last_valid_rvec   = rvec
            self.last_valid_center = center_uv
            self.last_valid_img_w  = img_w
            self.last_valid_img_h  = img_h
            self.last_seen_time    = now_wall
        else:
            # 못 봤지만, 최근(LOST_TIMEOUT 이내)에 본 적이 있다면 마지막 포즈 재사용
            if (self.last_valid_res is not None and
                self.last_seen_time is not None and
                (now_wall - self.last_seen_time) < LOST_TIMEOUT):

                res       = self.last_valid_res
                rvec      = self.last_valid_rvec
                center_uv = self.last_valid_center
                img_w     = self.last_valid_img_w
                img_h     = self.last_valid_img_h

                # 디버그용 표시 (원하면 빼도 됨)
                cv2.putText(overlay, "LOST → using last pose",
                            (10, 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                            (0, 0, 255), 2)
            else:
                # LOST_TIMEOUT 이상 완전히 놓친 상태면 기존 로직대로 None 유지
                pass

        marker_id = None
        x = z = yaw = None
        u_center = None
        v_cmd = 0.0
        w_cmd = 0.0


        # --------------------------------------------------
        # 공통: pose 파싱
        # --------------------------------------------------
        if res is not None and rvec is not None:
            mid, x_, y_, z_ = res
            marker_id = mid
            x = float(x_)
            z = float(z_)

            R, _ = cv2.Rodrigues(rvec)
            yaw = math.atan2(R[1, 0], R[0, 0])   # Marker 축 기준 yaw

        if center_uv is not None:
            u_center, v_center = center_uv
        else:
            u_center = None

        # --------------------------------------------------
        # FSM
        # --------------------------------------------------
        if self.state == "search":
            # 마커 발견될 때까지 정지 상태
            if res is not None and center_uv is not None:
                # --- 🔴 기존: u_center 기준으로 left/right 판단 ---
                if u_center > img_w / 2:
                    self.slide_side = "right"
                    self.edge_u_target = EDGE_FRAC * img_w  # 오른쪽 끝 근처
                else:
                    self.slide_side = "left"
                    self.edge_u_target = (1.0 - EDGE_FRAC) * img_w  # 왼쪽 끝 근처

                # --- 🟢 변경: solvePnP에서 나온 x 부호 기준으로 판단 ---
                # x > 0  → 마커가 카메라 기준 오른쪽
                # x < 0  → 마커가 카메라 기준 왼쪽
                # if x is not None and x > 0.0:
                #     self.slide_side = "right"
                # else:
                #     self.slide_side = "left"

                # if self.slide_side == "right":
                #     self.edge_u_target = EDGE_FRAC * img_w      # 오른쪽 근처
                # else:
                #     self.edge_u_target = (1.0 - EDGE_FRAC) * img_w  # 왼쪽 근처

                self.state = "slide"
                self.align_ok_time = None
                self.dist_integral = 0.0

                self.get_logger().info(
                    f"[search] Marker detected → slide (side={self.slide_side}, "
                    f"x={x:.3f}, edge_u_target={self.edge_u_target:.1f})"
                )

            twist.linear.x = 0.0
            twist.angular.z = 0.0


            # 마커가 보이는 동안:
            # - 마커 가로 위치를 화면 끝 근처(edge_u_target)에 유지하도록 회전
            # - 일정 속도로 전진
            # ⇒ 마커가 화면 끝에 걸쳐진 상태로 호를 그리며 이동
            
        elif self.state == "slide":
            # slide: 마커를 화면 "옆 → 점점 중앙"으로 끌어가며
            #        수직선(x≈0)에 올라탈 때까지 호를 그리며 이동
            if res is None or center_uv is None or rvec is None:
                # 마커를 잃으면 안전하게 정지하고 search로 돌아감
                self.get_logger().warn("[slide] Marker lost → search")
                self.state = "search"
                self.align_ok_time = None
                twist.linear.x = 0.0
                twist.angular.z = 0.0

            else:
                # 1) x 기반 수직선 도달 체크
                if x is not None and abs(x) < SLIDE_LAT_TOL:
                    # 여기서 바로 approach로 전환 + 정지
                    self.state = "approach"
                    self.align_ok_time = None
                    self.dist_integral = 0.0
                    self.get_logger().info(
                        f"[slide] Reached normal line (x={x:.3f}) → approach"
                    )
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    v_cmd, w_cmd = 0.0, 0.0
                    # 수직선 넘지 않게 바로 리턴
                    self.set_debug_frame(overlay)
                    self.set_debug_info(
                        state=self.state,
                        slide_side=self.slide_side,
                        marker_id=marker_id,
                        x=x,
                        z=z,
                        yaw=yaw,
                        u=u_center,
                        v_cmd=twist.linear.x,
                        w_cmd=twist.angular.z,
                    )
                    self.cmd_pub.publish(twist)
                    return

                # 2) x 크기에 따라 edge ↔ center 보간해서 u_target 결정
                #    |x|가 클수록 edge 쪽(EDGE_FRAC 근처),
                #    |x|가 작아질수록 center(0.5) 쪽으로 edge_frac를 줄임
                abs_x = abs(x) if x is not None else 0.0

                # 0~1 사이: 1이면 "아직 많이 틀어졌다", 0이면 "거의 중앙"
                x_ratio = clamp(abs_x / X_REF_FOR_SLIDE, 0.0, 1.0)

                # EDGE_FRAC(예: 0.8) → x가 줄어들수록 0.5로 수렴
                edge_frac_dyn = 0.5 + (EDGE_FRAC - 0.5) * x_ratio
                #  x_ratio=1   → edge_frac_dyn ≈ EDGE_FRAC (0.8)
                #  x_ratio=0   → edge_frac_dyn = 0.5 (완전 중앙)

                u_center_target = 0.5 * img_w
                if self.slide_side == "right":
                    u_edge_side = edge_frac_dyn * img_w          # 오른쪽 쪽
                else:
                    u_edge_side = (1.0 - edge_frac_dyn) * img_w  # 왼쪽 쪽

                # 최종 목표 픽셀 위치
                #   - edge_frac_dyn 안에 이미 "중앙으로 점점 당기는 효과"가 들어있음
                u_target = u_edge_side
                # (더 부드럽게 하고 싶으면 아래처럼 한 번 더 보간해도 됨)
                # u_target = 0.7 * u_edge_side + 0.3 * u_center_target

                # 3) 마커의 현재 u가 u_target을 향하도록 회전 제어
                u_err = (u_center - u_target) / img_w  # 정규화
                w_u = -K_EDGE * u_err

                # 🔹 x 기반 보정: x>0이면 왼쪽으로, x<0이면 오른쪽으로 더 꺾어서
                #    실제 3D lateral error도 줄이도록 유도
                w_x = K_X_SLIDE * (-x)    # x 양수 → 음수 회전, x 음수 → 양수 회전

                w = w_u + w_x
                w = clamp(w, -MAX_W, MAX_W)

                # 4) x가 작아질수록 속도 줄여서 overshoot 방지
                v = SLIDE_V
                if abs_x < 2.0 * SLIDE_LAT_TOL:
                    v *= 0.5
                if abs_x < 1.5 * SLIDE_LAT_TOL:
                    v *= 0.3

                twist.linear.x = v
                twist.angular.z = w
                v_cmd, w_cmd = v, w
                
        elif self.state == "approach":
            # 수직선(x≈0) 위에서, 마커와의 거리를 TARGET_DIST까지 맞추는 직선 접근 단계
            if res is None or rvec is None:
                # 마커를 잃으면 일단 정지 (원하면 여기서 search로 돌려도 됨)
                self.get_logger().warn("[approach] Marker lost → stop (stay in approach)")
                twist.linear.x = 0.0
                twist.angular.z = 0.0
            else:
                # 거리 오차
                e_dist = z - TARGET_DIST_M

                # 목표 거리에 거의 도달 → final_align으로 넘어가서 정밀 조정
                if abs(e_dist) < DIST_TOL_M:
                    self.state = "final_align"
                    self.align_ok_time = None
                    self.dist_integral = 0.0
                    self.get_logger().info(
                        f"[approach] Distance OK (z={z:.3f}) → final_align"
                    )
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                else:
                    # 단순 거리 제어: 직선으로만 접근 (회전 X)
                    v = K_DIST_APP * e_dist
                    v = clamp(v, -APPROACH_V, APPROACH_V)

                    if abs(v) < MIN_V:
                        v = MIN_V * (1 if v >= 0 else -1)

                    twist.linear.x = v
                    twist.angular.z = 0.0
                    v_cmd, w_cmd = v, 0.0

        elif self.state == "final_align":
            # x, yaw, z 모두 정밀하게 맞추는 단계
            if res is None or rvec is None:
                twist.linear.x = twist.angular.z = 0.0
                self.align_ok_time = None
            else:
                v_cmd, w_cmd = self._final_align_control(x, z, yaw, dt, twist)

        elif self.state == "done":
            # first time entering done → start rotation timer
            if self.done_start_time is None:
                self.done_start_time = time.time()
                self.get_logger().info("[done] Start 90-degree rotation")

            elapsed = time.time() - self.done_start_time

            if elapsed < (ROTATE_ANGLE / ROTATE_SPEED):
                # keep rotating
                twist.linear.x = 0.0
                twist.angular.z = ROTATE_SPEED
                v_cmd, w_cmd = 0.0, ROTATE_SPEED
            else:
                # rotation finished → final stop
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                v_cmd, w_cmd = 0.0, 0.0
                # optional: lock final stop
                # self.state = "finished"


        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        # 디버그 업데이트
        self.set_debug_frame(overlay)
        self.set_debug_info(
            state=self.state,
            slide_side=self.slide_side,
            marker_id=marker_id,
            x=x,
            z=z,
            yaw=yaw,
            u=u_center,
            v_cmd=twist.linear.x,
            w_cmd=twist.angular.z,
        )

        self.cmd_pub.publish(twist)

    # ------------------------------------------------------
    # final_align 제어
    # ------------------------------------------------------
    def _final_align_control(self, x, z, yaw, dt, twist: Twist):
        v = w = 0.0

        # 만족 조건
        yaw_ok = abs(yaw) < YAW_TOL
        lat_ok = abs(x) < LAT_TOL_M
        dist_ok = abs(z - TARGET_DIST_M) < DIST_TOL_M

        if yaw_ok and lat_ok and dist_ok:
            if self.align_ok_time is None:
                self.align_ok_time = time.time()
            else:
                if time.time() - self.align_ok_time > 0.3:
                    self.state = "done"
                    self.get_logger().info(
                        f"[final_align] PERFECT: x={x:.3f}, z={z:.3f}, yaw={yaw:.3f} → done"
                    )
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    return 0.0, 0.0
        else:
            self.align_ok_time = None

        # 거리 오차
        e_dist = z - TARGET_DIST_M
        self.dist_integral += e_dist * dt

        v = Kp_dist_final * e_dist + Ki_dist_final * self.dist_integral
        v = clamp(v, -MAX_V, MAX_V)
        if abs(v) < MIN_V:
            v = MIN_V * (1 if v >= 0 else -1)

        # 좌우 + yaw 보정
        w_lat = Kp_lat_final * (-x)
        w_yaw = Kp_yaw_final * (-yaw)
        w = w_lat + w_yaw
        w = clamp(w, -MAX_W, MAX_W)

        if z < 0.25:
            v *= 0.5
            w *= 0.7

        twist.linear.x = v
        twist.angular.z = w
        return v, w

    # ------------------------------------------------------
    def shutdown(self):
        stop = Twist()
        self.cmd_pub.publish(stop)
        self.cam.close()
        self.destroy_node()


# ==========================================================
# FLASK WEB SERVER
# ==========================================================

app = Flask(__name__)
node_ptr = None

@app.route("/")
def main_page():
    return """
    <html><body>
    <h1>Aruco Parking Debug (slide → line → approach)</h1>
    <img src="/video" width="640">
    <pre id='info' style="font-size:18px;"></pre>
    <script>
    setInterval(()=>{
      fetch('/info').then(r=>r.json()).then(j=>{
        document.getElementById('info').textContent =
          JSON.stringify(j, null, 2);
      })
    }, 200);
    </script>
    </body></html>
    """

@app.route("/video")
def video():
    def gen():
        while True:
            f = node_ptr.get_debug_frame()
            if f is None:
                time.sleep(0.02)
                continue
            _, jpeg = cv2.imencode(".jpg", f)
            yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" +
                   jpeg.tobytes() + b"\r\n")
    return Response(gen(), mimetype="multipart/x-mixed-replace; boundary=frame")

@app.route("/info")
def info():
    return jsonify(node_ptr.get_debug_info())


def main():
    rclpy.init()
    global node_ptr
    node_ptr = ArucoParkingNode()
    import logging
    log = logging.getLogger('werkzeug')
    log.setLevel(logging.ERROR)


    def sigint(*args):
        node_ptr.shutdown()
        rclpy.shutdown()
        exit(0)

    signal.signal(signal.SIGINT, sigint)

    threading.Thread(target=lambda: rclpy.spin(node_ptr),
                     daemon=True).start()

    app.run(host="0.0.0.0", port=8080, debug=False, threaded=True)


if __name__ == "__main__":
    main()