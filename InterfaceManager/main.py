
# main.py
import sys
from PyQt6.QtWidgets import QApplication
from main_window_manager import MainWindowManager

# 개별 페이지 import


from pages.off_main_frame import MainFrame
from pages.off_shopping_list import AutoShoppingClass
from pages.off_pc_browser import PC_BrowserClass
from pages.off_login_tcp_req import LoginTcpWindow
from pages.off_stt import Sttclass



def main():
    app = QApplication(sys.argv)

    # 화면 관리 객체 생성
    manager = MainWindowManager()

    # 페이지 등록
    
    manager.add_page("Sttclass", Sttclass(manager))
    manager.add_page("MainFrame", MainFrame(manager))
    manager.add_page("AutoShoppingClass", AutoShoppingClass(manager))
    manager.add_page("PC_BrowserClass", PC_BrowserClass(manager))
    manager.add_page("LoginTcpWindow", LoginTcpWindow(manager))
    
    # 시작 페이지 설정
    manager.show_page("LoginTcpWindow")
    
    manager.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()

    