# multi_cam_yolo_flask.py
import time
import cv2
from flask import Flask, request, jsonify, render_template_string, Response

# 여기서 multi_cam_yolo_server 를 import 하지 않는다!!
# 대신 서버 쪽에서 함수들을 넘겨받는 방식으로 설계

app = Flask(__name__)

# 서버에서 주입받을 함수 포인터들
_get_mode = None
_set_mode = None
_get_debug_frame = None


def init_flask(get_mode_fn, set_mode_fn, get_debug_frame_fn):
    """
    multi_cam_yolo_server 쪽에서 호출해서
    현재 모드 조회/변경, 디버그 프레임 접근 함수를 등록해준다.
    """
    global _get_mode, _set_mode, _get_debug_frame
    _get_mode = get_mode_fn
    _set_mode = set_mode_fn
    _get_debug_frame = get_debug_frame_fn


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
    # 서버에서 주입된 함수 사용
    st = _get_mode()
    return jsonify(st)


@app.route("/control")
def control():
    cam0 = request.args.get("cam0", "").lower()
    cam1 = request.args.get("cam1", "").lower()

    st = _set_mode(cam0, cam1)
    print("[CTRL] mode:", st)
    return jsonify(st)


def mjpeg_gen_cam(cid):
    while True:
        frame = _get_debug_frame(cid)
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
