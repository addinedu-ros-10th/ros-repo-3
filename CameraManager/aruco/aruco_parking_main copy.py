#!/usr/bin/env python3
import time
import threading
import signal
import math
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from aruco_camera import PiCamStream
import aruco_parking_server


# ==========================================================
# CONFIGURATION (파라미터)
# ==========================================================

CAM_INDEX = 1
ROTATE_180_IMAGE = True        # cam1 회전 여부

CALIB_FILE = "camera_calibration.npz"
ARUCO_DICT_TYPE = cv2.aruco.DICT_4X4_250
MARKER_SIZE_M = 0.08            # 아루코 한 변 길이 (m)

TARGET_MARKER_ID = 3         # 특정 ID로 제한하려면 숫자 입력

TARGET_DIST_M = 0.20            # 목표 거리 (10cm)
DIST_TOL_M   = 0.01             # 거리 허용 오차
LAT_TOL_M    = 0.01             # 좌우 허용 오차
ANGLE_TOL    = 0.02             # 정면 헤딩 각도 허용 오차 (라디안 ≈ 1.1°)

# PID gains
Kp_dist = 0.6
Ki_dist = 0.0
Kp_lat  = 2.0
Kd_lat  = 0.1

MAX_V = 0.12
MIN_V = 0.03
MAX_W = 1.2

ROTATE_SPEED = 0.8
ROTATE_TIME  = math.pi / ROTATE_SPEED   # 대략 180도


def clamp(x, lo, hi): 
    return max(lo, min(hi, x))


# ==========================================================
# ARUCO PARKING NODE
# ==========================================================

class ArucoParkingNode(Node):
    def __init__(self):
        super().__init__("aruco_parking_node")

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # Load calibration
        data = np.load(CALIB_FILE)
        self.camera_matrix = data["camera_matrix"]
        self.dist_coeffs   = data["distortion_coefficients"]

        self.cam = PiCamStream(CAM_INDEX, 640, 480, ROTATE_180_IMAGE)

        # ArUco detector
        self.dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICT_TYPE)
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.parameters)

        # PID internal variables
        self.e_lat_prev = 0.0
        self.dist_integral = 0.0
        self.last_time = self.get_clock().now()

        # ==========================================================
        # NEW: repeatable FSM
        # ==========================================================
        self.state = "idle"     ### NEW
        self.rotate_start_time = None

        # Debug
        self.debug_frame = None
        self.debug_lock = threading.Lock()
        self.debug_info = {
            "state": self.state,
            "marker_id": None,
            "x": None,
            "z": None,
            "v": 0.0,
            "w": 0.0,
        }

        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info("ArucoParkingNode started. Waiting in idle...")

    # --- 디버그 공유 데이터 ---
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

    # --- ArUco pose estimation ---
    def detect_marker_pose(self, frame, draw=True):
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)

        overlay = frame.copy()
        if ids is None:
            cv2.putText(overlay, "NO MARKER", (10,30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7,(255,0,0),2)
            return None, overlay

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

        for i, mid in enumerate(ids):
            if TARGET_MARKER_ID is not None and mid != TARGET_MARKER_ID:
                continue

            ok, rvec, tvec = cv2.solvePnP(obj, corners[i][0],
                                          self.camera_matrix, self.dist_coeffs)
            if not ok: 
                continue

            x,y,z = tvec.flatten()
            if z < best_z:
                best = (int(mid), x,y,z)
                best_idx = i
                best_z = z
                r_best, t_best = rvec, tvec

        if best is None:
            cv2.putText(overlay, "NO MATCH ID", (10,30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7,(255,0,0),2)
            return None, overlay

        if draw:
            cv2.aruco.drawDetectedMarkers(overlay, [corners[best_idx]])
            cv2.drawFrameAxes(overlay, self.camera_matrix,
                              self.dist_coeffs, r_best, t_best, 0.03)
            mid, x,y,z = best
            cv2.putText(overlay,
                        f"id={mid} x={x:.3f} z={z:.33f}",
                        (10, overlay.shape[0]-20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6,(0,255,0),2)

        return best, overlay

    # ======================================================
    # MAIN CONTROL LOOP
    # ======================================================
    def control_loop(self):
        frame = self.cam.get_frame()
        if frame is None:
            return

        twist = Twist()
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds*1e-9
        if dt<=0: 
            dt=1e-3
        self.last_time = now

        # 상태별 동작
        if self.state == "idle":           ### NEW
            res, overlay = self.detect_marker_pose(frame)
            if res is not None:
                self.state = "approach"
                self.get_logger().info("Marker detected → approach start")
            v_out = w_out = 0.0

        elif self.state == "approach":
            res, overlay = self.detect_marker_pose(frame)
            v_out, w_out = self._approach(res, twist, dt)

        elif self.state == "align":        ### NEW
            res, overlay = self.detect_marker_pose(frame)
            v_out, w_out = self._align(res, twist)

        elif self.state == "rotate":
            overlay = frame.copy()
            v_out, w_out = self._rotate(twist)

        else:  # done or unknown
            overlay = frame.copy()
            v_out = w_out = 0.0

        # 디버그 업데이트
        marker = None; x=z=None
        if 'res' in locals() and res is not None:
            marker, x,_,z = res

        self.set_debug_frame(overlay)
        self.set_debug_info(state=self.state,
                            marker_id=marker, x=x, z=z,
                            v=v_out, w=w_out)

        self.cmd_pub.publish(twist)

    # ======================================================
    # APPROACH (곡선접근)
    # ======================================================

    def _approach(self, res, twist, dt):
        if res is None:
            twist.linear.x = twist.angular.z = 0.0
            return 0.0, 0.0

        mid, x, y, z = res

        # 거리만 맞으면 align 단계로 (정면 맞추기)
        e_dist = z - TARGET_DIST_M
        if abs(e_dist) < DIST_TOL_M:
            self.state = "align"
            self.get_logger().info("Distance OK → align")
            return 0.0, 0.0

        # 거리 PID
        self.dist_integral += e_dist*dt
        v = Kp_dist*e_dist + Ki_dist*self.dist_integral
        v = clamp(v, -MAX_V, MAX_V)
        if abs(v) < MIN_V: 
            v = MIN_V * (1 if v>=0 else -1)

        # 좌우 PD — 부호 반전 e_lat = -x
        e_lat = -x
        de_lat = (e_lat - self.e_lat_prev)/dt
        self.e_lat_prev = e_lat

        w = Kp_lat*e_lat + Kd_lat*de_lat
        w = clamp(w, -MAX_W, MAX_W)

        # 가까워지면 감속
        if z < 0.25: 
            v *= 0.5
            w *= 0.7

        twist.linear.x = v
        twist.angular.z = w

        return v, w

    # ======================================================
    # ALIGN (정면 맞추기)
    # ======================================================

    def _align(self, res, twist):
        if res is None:
            twist.linear.x = twist.angular.z = 0.0
            return 0.0, 0.0

        mid, x, y, z = res

        # Heading error: atan2(x, z)
        heading_err = math.atan2(x, z)

        # 완료 조건 (정면)
        if abs(heading_err) < ANGLE_TOL :
            twist.linear.x = twist.angular.z = 0.0
            self.state = "rotate"
            self.rotate_start_time = time.time()
            self.get_logger().info("Align OK → rotate")
            return 0.0, 0.0

        # 정면 맞추기: 제자리 회전
        w = Kp_lat * (-heading_err)     # 부호 맞추기 (필요 시 -/+ 조정)
        w = clamp(w, -MAX_W, MAX_W)
        w *= 0.6

        twist.linear.x  = 0.0
        twist.angular.z = w

        return 0.0, w

    # ======================================================
    # ROTATE (180도 회전)
    # ======================================================

    def _rotate(self, twist):
        if self.rotate_start_time is None:
            self.rotate_start_time = time.time()

        elapsed = time.time() - self.rotate_start_time
        if elapsed >= ROTATE_TIME:
            twist.linear.x = twist.angular.z = 0.0
            self.state = "idle"    ### NEW
            self.get_logger().info("Rotate done → idle (ready for next run)")
            return 0.0, 0.0

        twist.linear.x = 0.0
        twist.angular.z = ROTATE_SPEED
        return 0.0, ROTATE_SPEED

    # ======================================================
    def shutdown(self):
        stop = Twist()
        self.cmd_pub.publish(stop)
        self.cam.close()
        self.destroy_node()


# ===================== 메인 =====================

def main():
    rclpy.init()
    node = ArucoParkingNode()

    # Flask 서버에 node 주입
    aruco_parking_server.node_for_web = node

    # Flask 서버 스레드 시작
    flask_thread = threading.Thread(
        target=aruco_parking_server.run_flask, daemon=True
    )
    flask_thread.start()
    node.get_logger().info("Flask debug server at http://0.0.0.0:8093/main")

    def sigint_handler(sig, frame):
        node.get_logger().info("SIGINT received, stopping...")
        node.shutdown()
        rclpy.shutdown()

    signal.signal(signal.SIGINT, sigint_handler)

    try:
        rclpy.spin(node)
    finally:
        node.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
