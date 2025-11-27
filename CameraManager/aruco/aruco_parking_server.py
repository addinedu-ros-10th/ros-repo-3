# aruco_parking_server.py
import time
import cv2
from flask import Flask, Response, render_template_string, jsonify
import logging
log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR)

app = Flask(__name__)
node_for_web = None  # main에서 주입


HTML_MAIN = """
<!doctype html>
<html>
<head>
  <meta charset="utf-8"/>
  <title>Aruco Parking Debug</title>
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
      justify-content:space-between;
      height:100vh;
      box-sizing:border-box;
      padding:8px;
      gap:8px;
    }
    .video-pane {
      flex:2;
      background:#020617;
      border-radius:12px;
      border:1px solid #111827;
      overflow:hidden;
      display:flex;
      align-items:center;
      justify-content:center;
    }
    .video-pane img {
      max-width:100%;
      max-height:100%;
      display:block;
    }
    .info-pane {
      flex:1;
      background:#020617;
      border-radius:12px;
      border:1px solid #111827;
      padding:12px;
      box-sizing:border-box;
      font-size:14px;
    }
    pre {
      background:#020617;
      padding:8px;
      border-radius:8px;
      border:1px solid #1f2937;
      font-size:13px;
      overflow-x:auto;
    }
    h2 { margin-top:0; font-size:18px; }
  </style>
</head>
<body>
  <div class="wrap">
    <div class="video-pane">
      <img src="/video" />
    </div>
    <div class="info-pane">
      <h2>Aruco Parking Debug</h2>
      <pre id="info">Loading...</pre>
    </div>
  </div>
<script>
async function poll(){
  try{
    const r = await fetch('/debug');
    const j = await r.json();
    let txt = "";
    txt += "STATE       : " + j.state + "\\n";
    txt += "marker_id   : " + j.marker_id + "\\n";
    txt += "x (m)       : " + j.x + "\\n";
    txt += "z (m)       : " + j.z + "\\n";
    txt += "v cmd (m/s) : " + j.v.toFixed(3) + "\\n";
    txt += "w cmd (rad/s): " + j.w.toFixed(3) + "\\n";
    document.getElementById('info').textContent = txt;
  }catch(e){
    document.getElementById('info').textContent = "Error fetching debug info";
  }
  setTimeout(poll, 300);
}
poll();
</script>
</body>
</html>
"""


@app.route("/video")
def video():
    def gen():
        global node_for_web
        while True:
            if node_for_web is None:
                time.sleep(0.1)
                continue
            frame = node_for_web.get_debug_frame()
            if frame is None:
                time.sleep(0.05)
                continue
            # RGB → BGR로 바꿔서 JPEG 인코딩 (여기서는 색 맞추기 위해)
            frame_bgr = frame
            # frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            ok, buf = cv2.imencode(".jpg", frame_bgr, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if not ok:
                continue
            jpg = buf.tobytes()
            yield (
                b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpg + b"\r\n"
            )
            time.sleep(0.03)
    return Response(gen(), mimetype="multipart/x-mixed-replace; boundary=frame")


@app.route("/debug")
def debug():
    global node_for_web
    if node_for_web is None:
        return jsonify({"state": "no_node"})
    info = node_for_web.get_debug_info()
    # None 값이 있을 수 있으니 문자열 처리
    for k in ["x", "z"]:
        if info[k] is None:
            info[k] = None
    return jsonify(info)


@app.route("/main")
def main_page():
    return render_template_string(HTML_MAIN)


def run_flask():
    app.run(host="0.0.0.0", port=8093, threaded=True)
