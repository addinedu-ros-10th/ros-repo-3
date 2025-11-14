
# main.py
import sys
from PyQt6.QtWidgets import QApplication
from main_window_manager import MainWindowManager

# 개별 페이지 import
from pages.login import LoginWindow
from pages.stt import Sttclass
from pages.main_frame import MainFrame
from pages.shopping_list import AutoShoppingClass
from pages.pc_browser import PC_BrowserClass
# from pages.login_admin import LoginAdminWindow
from pages.login_tcp_req import LoginTcpWindow


def main():
    app = QApplication(sys.argv)

    # 화면 관리 객체 생성
    manager = MainWindowManager()

    # 페이지 등록
    manager.add_page("LoginWindow", LoginWindow(manager))
    manager.add_page("Sttclass", Sttclass(manager))
    # manager.add_page("MainFrame", MainFrame(manager, username="test"))
    manager.add_page("MainFrame", MainFrame(manager))
    manager.add_page("AutoShoppingClass", AutoShoppingClass(manager))
    manager.add_page("PC_BrowserClass", PC_BrowserClass(manager))
    # manager.add_page("LoginAdminWindow", LoginAdminWindow(manager))
    manager.add_page("LoginTcpWindow", LoginTcpWindow(manager))



    # ⚠️ MainFrame을 사용하려면 이처럼 등록해야 합니다.
    # main_frame_instance = MainFrame(manager=manager, username="test")
    # manager.add_page("MainFrame", main_frame_instance)

    # 시작 페이지 설정
    manager.show_page("LoginTcpWindow")
    

    manager.show()
    sys.exit(app.exec())

    
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