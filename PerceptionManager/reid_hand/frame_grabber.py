# frame_grabber.py
import cv2
import threading
import time


class FrameGrabber:
    def __init__(self, src=0, width=960, height=540, fps=30, fourcc='MJPG'):
        self.src = src
        self.width = width
        self.height = height
        self.fps = fps
        self.fourcc = fourcc
        self.cap = None
        self.latest = None
        self.lock = threading.Lock()
        self.running = False
        self.thread = threading.Thread(target=self._loop, daemon=True)

    def start(self):
        self.running = True
        self.thread.start()
        # 첫 프레임 들어올 때까지 잠깐 대기(최대 1초)
        t0 = time.time()
        while self.latest is None and (time.time() - t0) < 1.0:
            time.sleep(0.01)
        return self

    def _open(self):
        cap = cv2.VideoCapture(self.src)
        if self.fourcc:
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.fourcc))
        if self.width:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        if self.height:
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        if self.fps:
            cap.set(cv2.CAP_PROP_FPS, self.fps)
        return cap

    def _loop(self):
        self.cap = self._open()
        fail_cnt = 0
        target_dt = 1.0 / max(1, self.fps)
        while self.running:
            if self.cap is None or not self.cap.isOpened():
                time.sleep(0.1)
                self.cap = self._open()
                continue

            ok, frame = self.cap.read()
            if not ok:
                fail_cnt += 1
                if fail_cnt >= 10:
                    try:
                        self.cap.release()
                    except:
                        pass
                    self.cap = None
                    fail_cnt = 0
                time.sleep(0.01)
                continue
            fail_cnt = 0

            # 최신 프레임만 저장
            with self.lock:
                self.latest = frame

            time.sleep(max(0.0, target_dt * 0.2))

        if self.cap is not None:
            try:
                self.cap.release()
            except:
                pass
        self.cap = None

    def read(self):
        with self.lock:
            if self.latest is None:
                return None
            return self.latest.copy()

    def stop(self):
        self.running = False
        try:
            self.thread.join(timeout=1.0)
        except:
            pass
