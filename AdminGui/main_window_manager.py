
# main_window_manager.py
from PyQt6.QtWidgets import QMainWindow, QStackedWidget, QWidget


class MainWindowManager(QMainWindow):
    def __init__(self):
        super().__init__()

        # ✅ 주석 해제: 이 창이 메인 창이므로 제목과 크기를 설정합니다.
        self.setWindowTitle("Smart Cart 관리 시스템 - Admin")
        self.resize(1280, 720)

        # QStackedWidget 생성 (모든 페이지 관리)
        self.stack = QStackedWidget()
        self.setCentralWidget(self.stack)

        # 페이지 이름 - 위젯 매핑
        self.pages: dict[str, QWidget] = {}

    def add_page(self, name: str, widget: QWidget):
        """페이지를 이름으로 등록"""
        self.pages[name] = widget
        self.stack.addWidget(widget)

    def show_page(self, name: str):
        """등록된 이름으로 페이지 전환"""
        if name in self.pages:
            self.stack.setCurrentWidget(self.pages[name])
        else:
            print(f"⚠️ 페이지 '{name}'가 등록되어 있지 않습니다.")

