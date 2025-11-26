import sys
from PyQt6.QtWidgets import QMainWindow, QWidget, QVBoxLayout, QLabel, QPushButton, QApplication
from PyQt6.QtGui import QPixmap, QColor
from PyQt6.QtCore import Qt

class MainFrame(QMainWindow):
    def __init__(self, manager, username=None, photo_path=None, login_window=None):
        super().__init__()
        self.manager = manager
        self.login_window = login_window

        # 🔥 1) 로그인에서 넘겨준 user_id 읽기
        user_id = getattr(self.manager, "RECV_user_id", None)

        # 🔥 2) user_id → username 매핑
        name_map = {
            1: "김철수",
            2: "박영희"
        }

        photo_map = {
            1: "./pages/user1.png",
            2: "./pages/user2.png"
        }

        # 기본 username, photo_path 처리
        username = name_map.get(user_id, "Guest")
        photo_path = photo_map.get(user_id, None)

        self.setWindowTitle("회원 정보 시스템 - 메인")
        self.setGeometry(100, 100, 1280, 720)

        # 🔥 3) 이제 username, photo_path 를 이용해 UI 세팅
        self.initUI(username, photo_path)

    def initUI(self, username, photo_path):
        # 메인 윈도우의 중앙 위젯 설정
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        # ❗ 중요: 수치(절대) 위치 지정을 위해 레이아웃을 사용하지 않습니다.
        # layout = QVBoxLayout()
        # central_widget.setLayout(layout) # <- 이 코드들을 제거합니다.

        # --- 1. 환영 메시지 라벨 ---
        # 부모 위젯을 central_widget으로 지정합니다.
        self.name_label = QLabel(f"환영합니다, {username}님!", central_widget) 
        font = self.name_label.font()
        font.setPointSize(16)
        self.name_label.setFont(font)
        
        # ❗ 수치로 위치와 크기 지정
        self.name_label.move(550, 50)      # X=20, Y=20 위치
        self.name_label.resize(360, 40)   # 너비 360, 높이 40
        
        # ❗ 라벨 '내부' 텍스트 정렬 (이전 코드와 동일하게 왼쪽 정렬)
        self.name_label.setAlignment(Qt.AlignmentFlag.AlignLeft) 
        # (만약 중앙 정렬을 원하면: Qt.AlignmentFlag.AlignCenter)

        # --- 2. 사용자 사진 라벨 ---
        self.photo_label = QLabel(central_widget) # 부모 지정
        self.photo_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        
        pixmap = QPixmap(photo_path)
        if pixmap.isNull():
            pixmap = QPixmap(450,300)
            pixmap.fill(QColor("gray"))
        
        self.photo_label.setPixmap(pixmap.scaled(450, 450, Qt.AspectRatioMode.KeepAspectRatio))

        # ❗ 수치로 위치와 크기 지정 (창의 중앙 근처)
        self.photo_label.move(400, 20)    # X=150, Y=80
        self.photo_label.resize(500, 500) # 너비 100, 높이 100

        self.stt_button = QPushButton("구매희망 리스트 담기", central_widget) # 부모 지정
        #self.stt_button.clicked.connect(self.stt_move)
        self.stt_button.clicked.connect(
            lambda: self.manager.show_page("ItemRadio"))
        # ❗ 수치로 위치와 크기 지정
        self.stt_button.move(420, 450)   # X=150, Y=220
        self.stt_button.resize(460, 50)  # 너비 100, 높이 30


        # --- 3. 로그아웃 버튼 ---
        self.logout_button = QPushButton("로그아웃", central_widget) # 부모 지정
        self.logout_button.clicked.connect(self.logout)
        
        # ❗ 수치로 위치와 크기 지정
        self.logout_button.move(420, 510)   # X=150, Y=220
        self.logout_button.resize(460, 50)  # 너비 100, 높이 30

    def stt_move(self):
        self.manager.show_page("Sttclass")

    def logout(self):
        if self.login_window:
            self.login_window.show()

        self.manager.show_page("LoginWindow")
        # self.close()

    def closeEvent(self, event):
        if self.login_window:
            self.login_window.show()
        event.accept()

if __name__ == "__main__":

    app = QApplication(sys.argv)
    
    # 아래의 코드는 main_frame.py 파일 단독으로 테스트 할 때 필요함 
    class FakeLoginWindow(QWidget):
        def show(self):
            print("로그인 창이 다시 나타났습니다.")
            
    test_frame = MainFrame( 
        username="test", 
        photo_path="invalid_path.png", 
        login_window=FakeLoginWindow()
    )
    test_frame.show()

    sys.exit(app.exec())