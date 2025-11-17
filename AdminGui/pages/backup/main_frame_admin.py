
import sys
import cv2
import time
from PyQt6.QtWidgets import (
    QApplication, QWidget, QLabel, QPushButton, QVBoxLayout, QHBoxLayout, QFileDialog
)
from PyQt6.QtCore import QTimer, Qt
from PyQt6.QtGui import QImage, QPixmap


class VideoMonitor(QWidget):
    def __init__(self,manager):
        super().__init__()

        

        # ----- [1] 윈도우 기본 설정 -----
        self.setWindowTitle("실시간 영상 모니터링 (캡처 + 녹화 기능)")
        self.setGeometry(200, 200, 800, 600)

        # ----- [2] 영상 표시용 QLabel 생성 -----
        self.video_label = QLabel(self)
        self.video_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.video_label.setText("카메라를 여는 중...")

        # ----- [3] 버튼 생성 -----
        self.capture_btn = QPushButton("📸 캡처")
        self.record_btn = QPushButton("🔴 녹화 시작")
        self.stop_btn = QPushButton("⏹ 녹화 정지")
        self.cart_status_btn = QPushButton("Cart 상태 보기")

        # ----- [4] 버튼 이벤트 연결 -----
        self.capture_btn.clicked.connect(self.capture_image)
        self.record_btn.clicked.connect(self.start_recording)
        self.stop_btn.clicked.connect(self.stop_recording)

        # ----- [5] 버튼 레이아웃 -----
        btn_layout = QHBoxLayout()
        btn_layout.addWidget(self.cart_status_btn)
        btn_layout.addWidget(self.capture_btn)
        btn_layout.addWidget(self.record_btn)
        btn_layout.addWidget(self.stop_btn)

        # ----- [6] 전체 레이아웃 -----
        layout = QVBoxLayout()
        layout.addWidget(self.video_label)
        layout.addLayout(btn_layout)
        self.setLayout(layout)

        # ----- [7] 카메라 초기화 -----
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.video_label.setText("❌ 카메라를 열 수 없습니다.")
            return

        # ----- [8] QTimer 설정 (30ms마다 영상 업데이트) -----
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(30)

        # ----- [9] 녹화 관련 변수 초기화 -----
        self.recording = False
        self.video_writer = None

    # ===============================
    # 📸 실시간 영상 업데이트
    # ===============================
    def update_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        # [A] BGR → RGB 색상 변환
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # [B] 녹화 중이면 프레임 저장
        if self.recording and self.video_writer is not None:
            # 저장은 BGR 형식으로 해야 하므로 다시 변환
            bgr_frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            self.video_writer.write(bgr_frame)

        # [C] 영상 크기, 데이터 형식 변환
        height, width, channel = frame.shape
        bytes_per_line = 3 * width
        q_img = QImage(frame.data, width, height, bytes_per_line, QImage.Format.Format_RGB888)

        # [D] QPixmap으로 변환해 QLabel에 표시
        self.video_label.setPixmap(QPixmap.fromImage(q_img))

    # ===============================
    # 📸 이미지 캡처 기능
    # ===============================
    def capture_image(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        # 저장할 경로 선택
        save_path, _ = QFileDialog.getSaveFileName(
            self, "캡처 이미지 저장", "capture.jpg", "Image Files (*.jpg *.png)"
        )
        if save_path:
            cv2.imwrite(save_path, frame)
            print(f"✅ 캡처 이미지 저장 완료: {save_path}")

    # ===============================
    # 🔴 녹화 시작 기능
    # ===============================
    def start_recording(self):
        if self.recording:
            print("이미 녹화 중입니다.")
            return

        # 비디오 저장 경로 설정
        save_path, _ = QFileDialog.getSaveFileName(
            self, "녹화 파일 저장", "record.avi", "Video Files (*.avi *.mp4)"
        )
        if not save_path:
            return

        # 녹화 설정 (코덱, FPS, 해상도)
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        fps = 30.0
        width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

        # 비디오 작성 객체 생성
        self.video_writer = cv2.VideoWriter(save_path, fourcc, fps, (width, height))

        # 녹화 상태 변경
        self.recording = True
        self.record_btn.setText("⏺ 녹화 중...")
        print(f"🎥 녹화 시작: {save_path}")

    # ===============================
    # ⏹ 녹화 정지 기능
    # ===============================
    def stop_recording(self):
        if not self.recording:
            return

        self.recording = False
        self.record_btn.setText("🔴 녹화 시작")
        if self.video_writer is not None:
            self.video_writer.release()
            self.video_writer = None
        print("🛑 녹화 종료")

    # ===============================
    # 🧹 창 닫힐 때 리소스 정리
    # ===============================
    def closeEvent(self, event):
        self.timer.stop()
        if self.cap.isOpened():
            self.cap.release()
        if self.video_writer is not None:
            self.video_writer.release()
        event.accept()


# ===============================
# 🚀 메인 실행 부분
# ===============================
if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = VideoMonitor()
    window.show()
    sys.exit(app.exec())
