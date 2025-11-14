
import sys
import math
import cv2
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

from PyQt6.QtWidgets import (
    QApplication, QWidget, QLabel, QPushButton, QVBoxLayout,
    QHBoxLayout, QStackedWidget, QScrollBar
)
from PyQt6.QtGui import QPixmap, QPainter, QPen, QColor, QImage
from PyQt6.QtCore import QTimer, Qt


# ============================================
# ROS2 노드: 로봇 위치를 받아오는 Subscriber
# ============================================
class PoseSubscriber(Node):
    def __init__(self, manager):
        super().__init__('pose_subscriber')
        self.manager = manager
        self.robot_pose = None
        self.create_subscription(PoseStamped, '/robot_pose', self.pose_callback, 10)

    def pose_callback(self, msg):
        self.robot_pose = msg.pose.position


# ============================================
# PyQt6: 지도 + 좌표 표시 메인 위젯
# ============================================
class MapWindow(QWidget):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node

        self.setWindowTitle("로봇 위치 모니터링 (세로 스크롤 지원)")
        self.setGeometry(100, 100, 1000, 600)

        # ----- 지도 이미지 로드 -----
        self.map_img = cv2.imread("shopping_map.png")
        if self.map_img is None:
            raise FileNotFoundError("shopping_map.png 파일을 같은 폴더에 넣어주세요!")

        # OpenCV → QPixmap 변환
        height, width, ch = self.map_img.shape
        bytes_per_line = ch * width
        q_img = QImage(self.map_img.data, width, height, bytes_per_line, QImage.Format.Format_BGR888)
        self.map_pixmap = QPixmap.fromImage(q_img)

        # 지도 표시용 QLabel
        self.map_label = QLabel()
        self.map_label.setPixmap(self.map_pixmap)
        self.map_label.setFixedSize(width, height)

        # 화면에 표시될 영역 높이 제한
        self.view_height = 1200  # 한 화면에 보일 높이
        self.offset_y = 0       # 현재 지도 표시 오프셋

        # 지도 컨테이너 위젯 (뷰 역할)
        self.map_container = QWidget()
        self.map_container_layout = QVBoxLayout()
        self.map_container_layout.setContentsMargins(0, 0, 0, 0)
        self.map_container_layout.addWidget(self.map_label)
        self.map_container.setLayout(self.map_container_layout)
        self.map_container.setFixedHeight(self.view_height)
        self.map_container.setFixedWidth(width)

        # ✅ 세로 스크롤바 추가
        self.scroll_bar = QScrollBar(Qt.Orientation.Vertical)
        self.scroll_bar.setMinimum(0)
        self.scroll_bar.setMaximum(max(0, height - self.view_height))
        self.scroll_bar.setSingleStep(20)
        self.scroll_bar.valueChanged.connect(self.scroll_moved)

        # 좌표 표시 라벨
        self.coord_label = QLabel("현재 좌표: (0.0, 0.0)")
        self.coord_label.setAlignment(Qt.AlignmentFlag.AlignTop)
        self.coord_label.setStyleSheet("font-size: 16px; padding: 10px;")

        # 페이지 이동 버튼
        self.next_btn = QPushButton("➡ 쇼핑카트 상태정보 보기")
        self.next_btn.clicked.connect(self.go_to_next_page)

        # 지도 + 스크롤바 배치
        map_layout = QHBoxLayout()
        map_layout.addWidget(self.map_container)
        map_layout.addWidget(self.scroll_bar)

        right_layout = QVBoxLayout()
        right_layout.addWidget(self.coord_label)
        right_layout.addWidget(self.next_btn)
        right_layout.addStretch()
        
        top_layout = QHBoxLayout()
        top_layout.addLayout(map_layout)
        top_layout.addLayout(right_layout)
       
        bottom_layout = QHBoxLayout()
        bottom_layout.addStretch()
        

        main_layout = QVBoxLayout()
        main_layout.addLayout(top_layout)
        main_layout.addLayout(bottom_layout)
       
        self.setLayout(main_layout)

        # ROS 데이터 업데이트 타이머
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_display)
        self.timer.start(30)

    # ---------------------------
    # 스크롤 이동 시 지도 위치 조정
    # ---------------------------
    def scroll_moved(self, value):
        """스크롤바 이동 시 지도 이미지의 표시 위치를 위아래로 이동"""
        self.offset_y = value
        self.map_label.move(0, -self.offset_y)

    # ---------------------------
    # ROS2 데이터 갱신 및 지도 표시
    # ---------------------------
    def update_display(self):
        if self.ros_node.robot_pose is None:
            return

        x = self.ros_node.robot_pose.x
        y = self.ros_node.robot_pose.y

        img_h, img_w, _ = self.map_img.shape
        px = int((x + 5) * (img_w / 10))
        py = int((5 - y) * (img_h / 10))

        display_img = self.map_pixmap.copy()
        painter = QPainter(display_img)
        pen = QPen(QColor(255, 0, 0), 10)
        painter.setPen(pen)
        painter.drawPoint(px, py)
        painter.end()

        self.map_label.setPixmap(display_img)
        self.coord_label.setText(f"현재 좌표: ({x:.2f}, {y:.2f})")

    def go_to_next_page(self):
        self.parentWidget().setCurrentIndex(1)


# ============================================
# 로그 페이지
# ============================================
class LogWindow(QWidget):
    def __init__(self):
        super().__init__()
        label = QLabel("📜 여기는 로그 화면입니다.", self)
        label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout = QVBoxLayout()
        layout.addWidget(label)
        self.setLayout(layout)


# ============================================
# 실행부
# ============================================
def main():
    rclpy.init()
    node = PoseSubscriber()

    app = QApplication(sys.argv)

    stacked = QStackedWidget()
    map_window = MapWindow(node)
    log_window = LogWindow()

    stacked.addWidget(map_window)
    stacked.addWidget(log_window)
    stacked.setCurrentIndex(0)
    stacked.show()

    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.01))
    timer.start(10)

    sys.exit(app.exec())


if __name__ == "__main__":
    main()
