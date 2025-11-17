
import sys
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from PyQt6.QtWidgets import QApplication, QLabel, QWidget, QVBoxLayout
from PyQt6.QtCore import QTimer, Qt
from PyQt6.QtGui import QImage, QPixmap

"""
🧩 핵심 개념 요약

Gazebo에서 카메라가 publish하는 토픽은 보통
/camera/image_raw (또는 /camera/color/image_raw) 입니다.

ROS2에서는 이 영상을 rclpy와 cv_bridge를 통해 numpy 배열로 변환할 수 있습니다.

변환한 이미지를 QLabel에 띄우면 됩니다.

"""



class RosVideoSubscriber(Node):
    """ROS2 카메라 토픽 구독 노드"""
    def __init__(self):
        super().__init__('camera_subscriber')
        self.bridge = CvBridge()
        self.latest_frame = None

        # Gazebo 또는 카메라 토픽 구독
        # (필요에 따라 토픽 이름 변경 가능)
        self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

    def image_callback(self, msg):
        """ROS2에서 수신된 이미지를 OpenCV 이미지로 변환"""
        try:
            # ROS Image 메시지를 numpy(OpenCV 형식)로 변환
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.latest_frame = cv_image
        except Exception as e:
            self.get_logger().error(f"Image 변환 실패: {e}")


class VideoMonitor(QWidget):
    """PyQt6 영상 표시용 클래스"""
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node

        self.setWindowTitle("Gazebo 실시간 영상 모니터링")
        self.setGeometry(200, 200, 800, 600)

        self.video_label = QLabel("카메라 연결 중...", self)
        self.video_label.setAlignment(Qt.AlignmentFlag.AlignCenter)

        layout = QVBoxLayout()
        layout.addWidget(self.video_label)
        self.setLayout(layout)

        # 주기적으로 영상 업데이트
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(30)

    # ===============================
    # 🧠 여기서 Gazebo 영상을 표시
    # ===============================
    def update_frame(self):
        frame = self.ros_node.latest_frame
        if frame is None:
            return

        # [A] BGR → RGB 변환
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # [B] QImage 변환
        height, width, channel = frame.shape
        bytes_per_line = 3 * width
        q_img = QImage(frame.data, width, height, bytes_per_line, QImage.Format.Format_RGB888)

        # [C] QLabel에 표시
        self.video_label.setPixmap(QPixmap.fromImage(q_img))

    def closeEvent(self, event):
        """PyQt 종료 시 ROS2 노드도 함께 종료"""
        self.timer.stop()
        rclpy.shutdown()
        event.accept()


# ===============================
# 🚀 메인 실행부
# ===============================
def main():
    rclpy.init()

    # ROS 노드 실행
    ros_node = RosVideoSubscriber()

    # PyQt 실행
    app = QApplication(sys.argv)
    viewer = VideoMonitor(ros_node)
    viewer.show()

    # ROS 스피너와 PyQt 이벤트 루프를 병렬 실행
    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(ros_node, timeout_sec=0.01))
    timer.start(10)

    sys.exit(app.exec())


if __name__ == '__main__':
    main()
