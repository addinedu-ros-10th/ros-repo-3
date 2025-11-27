# aruco_camera.py
import time
import threading
import cv2
from picamera2 import Picamera2


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
        try:
            self.thread.join()
        except:
            pass
        try:
            self.picam2.close()
        except:
            pass
