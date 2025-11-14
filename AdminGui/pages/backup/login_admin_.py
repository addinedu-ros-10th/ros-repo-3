

import os
import sys
from PyQt6.QtWidgets import QApplication, QMainWindow, QLabel, QLineEdit, QPushButton, QMessageBox, QWidget, QVBoxLayout, QFormLayout
from PyQt6.QtCore import Qt

# MainFrame 클래스를 pages/main_frame.py에서 가져옵니다.
from .admin_manager import PoseSubscriber

# 상위 폴더 경로 추가
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# db_connect 모듈이 상위 폴더에 있다고 가정합니다.
# 실제 환경에 맞게 db_connect.py의 위치를 확인하세요.
try:
    from db_connect import get_connection
except ImportError:
    print("Error: 'db_connect.py'를 찾을 수 없습니다. 상위 디렉토리에 있는지 확인하세요.")
    sys.exit(1)


class LoginAdminWindow(QWidget): 
    def __init__(self, manager=None):
        super().__init__()
        self.manager = manager
        
        # ❌ 아래 두 줄은 Manager가 설정하므로 삭제합니다.
        # self.setWindowTitle("회원정보 관리 시스템 - 로그인")
        # self.setGeometry(100, 100, 1280, 720)
        
        self.main_frame = None
        self.initUI()

    def initUI(self):
        # ❌ QWidget은 Central Widget이 필요 없습니다.
        # central_widget = QWidget()
        # self.setCentralWidget(central_widget)

        # 수직 레이아웃
        main_layout = QVBoxLayout()
                
        # 폼 레이아웃 (라벨 + 입력 필드)
        form_layout = QFormLayout()

        self.input_user = QLineEdit(self)

        self.input_pw = QLineEdit(self)
        self.input_pw.setEchoMode(QLineEdit.EchoMode.Password)
        
        form_layout.addRow("사용자 이름:", self.input_user)
        form_layout.addRow("비밀번호:", self.input_pw)

        # 로그인 버튼
        self.btn_login = QPushButton("로그인", self)
        self.btn_login.clicked.connect(self.login)

        # ❗ 수치로 위치와 크기 지정
        self.btn_login.move(300, 470)   # X=150, Y=220
        self.btn_login.resize(520, 50)  # 너비 100, 높이 30


        # 레이아웃에 위젯 추가
        main_layout.addLayout(form_layout)
        # main_layout.addWidget(self.btn_login)

        
        # ✅ QWidget은 setLayout을 바로 사용합니다.
        self.setLayout(main_layout)

    def login(self):
        username = self.input_user.text()
        password = self.input_pw.text()


        if not username or not password:
            QMessageBox.warning(self, "경고", "사용자 이름과 비밀번호를 입력하세요.")
            return

        conn = get_connection()
        if conn:
            try:
                cursor = conn.cursor()
                
                # 중요: DB에 username (이름)과 photo_path (사진 경로) 컬럼이 있어야 합니다.
                query = "SELECT username, password FROM users WHERE username=%s AND password=%s"
            
                cursor.execute(query, (username, password))
                result = cursor.fetchone()
                print("SQL result : ", result)
            except Exception as e:
                QMessageBox.critical(self, "데이터베이스 오류", f"조회 중 오류 발생: {e}")
                result = None
            finally:
                conn.close()

        if result:
            # 로그인 성공
            # username, = result

            self.main_frame = PoseSubscriber(
                manager=self.manager,         # ✅ manager 전달
                # username=username, 
                # photo_path=photo_path,
                # login_window=self
            )

            self.manager.show_page("PoseSubscriber")

            # 비밀번호 필드 지우기
            self.input_pw.clear()
        
        else:
            # 로그인 실패
            QMessageBox.warning(self, "로그인 실패", "사용자 이름 또는 비밀번호가 틀립니다.")
            self.input_pw.clear()
    

if __name__ == "__main__":
    
    app = QApplication(sys.argv)
    window = LoginAdminWindow()
    window.show()
    sys.exit(app.exec())