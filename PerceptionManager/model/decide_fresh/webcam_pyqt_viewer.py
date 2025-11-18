# webcam_pyqt_viewer.py

# --- add: must be first ---
import os
# X11 사용 시
os.environ.setdefault("QT_QPA_PLATFORM", "xcb")
# Wayland를 쓰는 배포판이면 필요 시: os.environ["QT_QPA_PLATFORM"] = "wayland"

# cv2가 설정한 Qt 플러그인 경로를 무력화
os.environ.pop("QT_PLUGIN_PATH", None)
os.environ.pop("QT_QPA_PLATFORM_PLUGIN_PATH", None)

# PyQt를 먼저 import 해서 Qt 경로를 고정
from PyQt5 import QtWidgets, QtCore, QtGui
# PyQt6를 쓰려면 위 줄을 다음으로 교체:
# from PyQt6 import QtWidgets, QtCore, QtGui

# (선택) PyQt의 platform plugin 경로를 명시적으로 지정
try:
    import PyQt5, sys
    pyqt_dir = os.path.dirname(PyQt5.__file__)
    qt_plugins = os.path.join(pyqt_dir, "Qt5", "plugins", "platforms")  # PyQt5
    if os.path.isdir(qt_plugins):
        os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"] = qt_plugins
except Exception:
    pass

# 이제야 다른 것들을 import
import sys, time, cv2, torch
import torch.nn.functional as F
import numpy as np

from PyQt5 import QtWidgets, QtCore, QtGui
from model_io import load_checkpoint  # 같은 폴더에 두세요

class WebcamViewer(QtWidgets.QWidget):
    def __init__(self, ckpt_path: str, cam_index: int = 0, use_cam: bool = False, device: str = None, parent=None):
        super().__init__(parent)
        self.setWindowTitle("RexNet Freshness - Webcam Viewer")

        self.device = device or ("cuda" if torch.cuda.is_available() else "cpu")
        self.model, self.preprocess, self.classes = load_checkpoint(ckpt_path, device=self.device)
        self.use_cam = use_cam

        # OpenCV 카메라
        self.cap = cv2.VideoCapture(cam_index)
        if not self.cap.isOpened():
            raise RuntimeError(f"Cannot open camera index {cam_index}")

        # UI
        self.image_label = QtWidgets.QLabel(alignment=QtCore.Qt.AlignCenter)
        self.pred_label  = QtWidgets.QLabel("Pred: -", alignment=QtCore.Qt.AlignCenter)
        font = QtGui.QFont(); font.setPointSize(14); font.setBold(True)
        self.pred_label.setFont(font)

        layout = QtWidgets.QVBoxLayout()
        layout.addWidget(self.image_label, stretch=1)
        layout.addWidget(self.pred_label)
        self.setLayout(layout)

        # 타이머로 주기적 프레임 갱신
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(30)  # 30ms ≈ ~33 FPS

        self.last_t = time.time()

    def closeEvent(self, event):
        self.cap.release()
        super().closeEvent(event)

    @torch.no_grad()
    def predict_frame(self, frame_bgr: np.ndarray):
        # BGR -> RGB, PIL 변환 없이 tensor로 직접 변환
        rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        img = cv2.resize(rgb, (self.preprocess.transforms[0].size[0], self.preprocess.transforms[0].size[1])) \
              if hasattr(self.preprocess.transforms[0], "size") else rgb
        # 전처리: torchvision Compose와 동일하게 구성 (여기선 Resize만 반영)
        # 간단히 ToTensor+Normalize 처리:
        x = torch.from_numpy(img).float().permute(2,0,1) / 255.0
        # Normalize
        mean = torch.tensor(self.preprocess.transforms[-1].mean).view(3,1,1)
        std  = torch.tensor(self.preprocess.transforms[-1].std).view(3,1,1)
        x = (x - mean) / std
        x = x.unsqueeze(0).to(self.device)

        logits = self.model(x)
        probs = F.softmax(logits, dim=1)[0]
        conf, idx = torch.max(probs, dim=0)
        return int(idx.item()), float(conf.item()), probs.cpu().numpy()

    @torch.no_grad()
    def compute_simple_cam(self, frame_bgr: np.ndarray, pred_idx: int):
        """
        간단 CAM: forward_features + classifier weight
        - 히트맵 크기를 원본 프레임 크기로 리사이즈 후 overlay
        """
        # 입력과 동일 전처리
        rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        size = self.preprocess.transforms[0].size if hasattr(self.preprocess.transforms[0], "size") else (224, 224)
        img = cv2.resize(rgb, size)
        x = torch.from_numpy(img).float().permute(2,0,1) / 255.0
        mean = torch.tensor(self.preprocess.transforms[-1].mean).view(3,1,1)
        std  = torch.tensor(self.preprocess.transforms[-1].std).view(3,1,1)
        x = ((x - mean) / std).unsqueeze(0).to(self.device)

        # 특징맵
        # timm rexnet: forward_features → [B,C,H,W]
        feats = self.model.forward_features(x)  # [1,C,H,W]

        # 분류기 weight
        classifier = getattr(self.model, "get_classifier")() if hasattr(self.model, "get_classifier") else self.model.classifier
        if not hasattr(classifier, "weight"):
            return None
        w = classifier.weight[pred_idx]  # [C]

        cam = torch.einsum("c, bchw -> bhw", w, feats)  # [1,H,W]
        cam = cam[0]
        cam = cam - cam.min()
        cam = cam / (cam.max() + 1e-6)
        cam = cam.cpu().numpy()

        # 원본 프레임 크기로
        H, W = frame_bgr.shape[:2]
        cam_resized = cv2.resize(cam, (W, H), interpolation=cv2.INTER_LINEAR)
        heat = np.uint8(255 * cam_resized)
        heat = cv2.applyColorMap(heat, cv2.COLORMAP_JET)
        overlay = cv2.addWeighted(frame_bgr, 0.6, heat, 0.4, 0)
        return overlay

    def update_frame(self):
        ok, frame = self.cap.read()
        if not ok:
            return

        # 추론
        idx, conf, probs = self.predict_frame(frame)
        label = f"{self.classes[idx]} ({conf*100:.1f}%)"
        self.pred_label.setText(f"Pred: {label}")

        # CAM 옵션
        if self.use_cam:
            try:
                frame = self.compute_simple_cam(frame, idx) or frame
            except Exception:
                pass

        # 화면 표시 (BGR→RGB)
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb.shape
        qimg = QtGui.QImage(rgb.data, w, h, ch*w, QtGui.QImage.Format_RGB888)
        self.image_label.setPixmap(QtGui.QPixmap.fromImage(qimg))

        # FPS 가볍게 표기(윈도우 타이틀)
        now = time.time()
        fps = 1.0 / max(now - self.last_t, 1e-6)
        self.last_t = now
        self.setWindowTitle(f"RexNet Freshness - Webcam Viewer  |  {fps:.1f} FPS")

def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--ckpt", type=str, required=True, help="Path to checkpoint.pt (from model_io.py)")
    parser.add_argument("--cam_index", type=int, default=0)
    parser.add_argument("--cam", action="store_true", help="Overlay simple CAM heatmap")
    args = parser.parse_args()

    app = QtWidgets.QApplication(sys.argv)
    w = WebcamViewer(ckpt_path=args.ckpt, cam_index=args.cam_index, use_cam=args.cam)
    w.resize(960, 720)
    w.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
