# main.py
import sys
from PyQt6.QtWidgets import QApplication
from main_window_manager import MainWindowManager

from pages.admin_login import AdminLoginWindow
from pages.admin_manage import AdminManageWindow


def main():

    app = QApplication(sys.argv)

    manager = MainWindowManager()

    # 페이지 등록
    manager.add_page("AdminLoginWindow", AdminLoginWindow(manager))
    manager.add_page("AdminManage",      AdminManageWindow(manager))

    # 첫 화면
    manager.show_page("AdminLoginWindow")

    manager.show()

    sys.exit(app.exec())


if __name__ == "__main__":
    main()
