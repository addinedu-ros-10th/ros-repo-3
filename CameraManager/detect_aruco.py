#!/usr/bin/env python3
# detect_aruco_server.py
import cv2, time
import numpy as np
from flask import Flask, Response, render_template_string, jsonify
from pinkylib.camera import Camera
# from picamera2 import Picamera2

# --------------------- 설정 ---------------------
PORT = 8091
ARUCO_DICT = cv2.aruco.DICT_4X4_250
MARKER_SIZE_M = 0.08     # 마커 한 변의 실제 길이 (m)
CALIB_PATH = "./camera_calibration.npz"
camera_num = 1
# ------------------------------------------------

app = Flask(__name__)
cam = Camera(camera_num)
cam.start(width=640, height=480)
cam.set_calibration(CALIB_PATH)

# 웹에서 공유할 전역 상태
latest_pose = []
latest_time = 0.0

# --------------------- HTML 템플릿 ---------------------
HTML = """
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <title>ArUco Detection</title>
  <style>
    body { font-family: system-ui, sans-serif; background:#0b0e12; color:#e5e7eb; margin:24px; }
    .wrap { max-width: 960px; margin:auto; }
    img { width:100%; border-radius:10px; border:1px solid #1f2937; }
    .info { margin-top:14px; background:#11161d; padding:10px 14px; border-radius:10px; }
    code { background:#0f141a; padding:2px 6px; border-radius:6px; color:#a7f3d0; }
  </style>
</head>
<body>
  <div class="wrap">
    <h2>ArUco Marker Detection</h2>
    <img src="/video" />
    <div class="info">
      <h4>Latest Detections</h4>
      <pre id="pose">No marker detected</pre>
    </div>
  </div>
<script>
async function poll(){
  try{
    const r = await fetch('/pose');
    const j = await r.json();
    if(j.length===0){
      document.getElementById('pose').textContent = 'No marker detected';
    }else{
      let txt = '';
      for(const [id,x,y,z] of j){
        txt += `ID=${id}, X=${x.toFixed(1)}cm, Y=${y.toFixed(1)}cm, Z=${z.toFixed(1)}cm\\n`;
      }
      document.getElementById('pose').textContent = txt;
    }
  }catch(e){}
  setTimeout(poll, 300);
}
poll();
</script>
</body>
</html>
"""
# --------------------------------------------------------

def detect_aruco_frame():
    """YOLO처럼 스트리밍 루프에서 마커 감지"""
    global latest_pose, latest_time

    dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(dictionary, parameters)

    while True:
        frame = cam.get_frame()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = detector.detectMarkers(gray)
        overlay = frame.copy()
        pose_list = []

        if ids is not None:
            for i in range(len(ids)):
                # 각 마커별 pose 계산
                obj_points = np.array([
                    [-MARKER_SIZE_M / 2, MARKER_SIZE_M / 2, 0],
                    [ MARKER_SIZE_M / 2, MARKER_SIZE_M / 2, 0],
                    [ MARKER_SIZE_M / 2,-MARKER_SIZE_M / 2, 0],
                    [-MARKER_SIZE_M / 2,-MARKER_SIZE_M / 2, 0]
                ], dtype=np.float32)

                success, rvec, tvec = cv2.solvePnP(
                    obj_points,
                    corners[i][0],
                    cam.calibration_matrix,
                    cam.dist_coeffs
                )
                if success:
                    x, y, z = tvec.flatten() * 100  # cm 단위 변환
                    cv2.aruco.drawDetectedMarkers(overlay, corners)
                    cv2.drawFrameAxes(
                        overlay, cam.calibration_matrix, cam.dist_coeffs, rvec, tvec, 0.01
                    )
                    cv2.putText(
                        overlay,
                        f"id:{ids[i][0]} ({x:.1f},{y:.1f},{z:.1f})cm",
                        (int(corners[i][0][0][0]), int(corners[i][0][0][1])-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2
                    )
                    pose_list.append([int(ids[i][0]), float(x), float(y), float(z)])

        latest_pose = pose_list
        latest_time = time.time()

        ok, buf = cv2.imencode(".jpg", overlay, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if not ok:
            continue
        jpg = buf.tobytes()
        yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpg + b"\r\n")
        time.sleep(0.03)

@app.route("/")
def index():
    return render_template_string(HTML)

@app.route("/video")
def video_feed():
    return Response(detect_aruco_frame(), mimetype="multipart/x-mixed-replace; boundary=frame")

@app.route("/pose")
def pose():
    return jsonify(latest_pose)

@app.route("/shutdown")
def shutdown():
    try:
        cam.close()
    except:
        pass
    return "Camera closed"

if __name__ == "__main__":
    print(f"[INFO] ArUco Detection Server running at http://0.0.0.0:{PORT}/")
    try:
        app.run(host="0.0.0.0", port=PORT, threaded=True)
    finally:
        cam.close()
