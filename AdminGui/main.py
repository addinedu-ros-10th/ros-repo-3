
# main.py
import sys
import rclpy
from PyQt6.QtWidgets import QApplication
from PyQt6.QtCore import QTimer
from main_window_manager import MainWindowManager


# 개별 페이지 import
from pages.login_admin import LoginAdminWindow
from pages.admin_manager import PoseSubscriber, MapWindow
from pages.admin_detail import AdminDetailClass
from pages.Cart_detail import CartDetailClass

def main():

    # ✅ ROS2 초기화
    rclpy.init()

    app = QApplication(sys.argv)

    # 화면 관리 객체 생성
    manager = MainWindowManager()

    # ROS 노드 (데이터 수신용)
    ros_node = PoseSubscriber(manager)

    # 페이지 등록
    manager.add_page("LoginAdminWindow", LoginAdminWindow(manager))
    manager.add_page("AdminDetailClass", AdminDetailClass(manager))
    manager.add_page("CartDetailClass", CartDetailClass(manager))
    
    manager.add_page("PoseSubscriber", MapWindow(ros_node))
    
    # 시작 페이지 설정
    manager.show_page("LoginAdminWindow")
    

    manager.show()

    # ROS spin 주기적으로 호출
    # 이 코드의 목적과 효과는 
    # “ROS2 노드(rclpy)를 PyQt GUI 이벤트 루프와 동시에 동작시키기 위한 것”입니다.
    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(ros_node, timeout_sec=0.01))
    timer.start(10)

    sys.exit(app.exec())

    # ✅ ROS2 종료
    rclpy.shutdown()
    sys.exit(exit_code)

    
if __name__ == "__main__":
    main()


# 페이지 전환 폴더 구조 
# pyqt_app/
# ├── main.py                     ← 프로그램 실행 진입점
# ├── main_window_manager.py      ← QMainWindow + 화면 전환 관리
# └── pages/
#     ├── __init__.py
#     ├── main_page.py            ← 첫 번째 화면
#     └── sub_page.py             ← 두 번째 화면


# 실행 흐름 
# [main.py]  
#     ↓
# [MainWindowManager]
#     ↓
# [QStackedWidget] ← 여러 화면을 관리
#     ├─ MainPage
#     └─ SubPage

# 버튼 클릭 시 self.manager.show_page("다음_페이지이름") 호출
# → MainWindowManager가 QStackedWidget의 현재 페이지를 변경