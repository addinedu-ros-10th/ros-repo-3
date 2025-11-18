

import os
import sys
from PyQt6.QtWidgets import QApplication, QMainWindow, QLabel, QLineEdit, QPushButton, QMessageBox, QWidget, QVBoxLayout, QFormLayout
from PyQt6.QtCore import Qt

# MainFrame 클래스를 pages/main_frame.py에서 가져옵니다.
from ..main_frame import MainFrame

# 상위 폴더 경로 추가
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# db_connect 모듈이 상위 폴더에 있다고 가정합니다.
# 실제 환경에 맞게 db_connect.py의 위치를 확인하세요.
try:
    from db_connect import get_connection
except ImportError:
    print("Error: 'db_connect.py'를 찾을 수 없습니다. 상위 디렉토리에 있는지 확인하세요.")
    sys.exit(1)



# ✅ QMainWindow 대신 QWidget을 상속받습니다.
class LoginWindow(QWidget): 
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
        self.btn_login.resize(220, 50)  # 너비 100, 높이 30

        # Admin 로그인 버튼
        self.btn_login_admin = QPushButton("Admin 로그인", self)
        self.btn_login_admin.clicked.connect(self.admin_login)

        # ❗ 수치로 위치와 크기 지정
        self.btn_login_admin.move(530, 470)   # X=150, Y=220
        self.btn_login_admin.resize(220, 50)  # 너비 100, 높이 30


        # 레이아웃에 위젯 추가
        main_layout.addLayout(form_layout)
        # main_layout.addWidget(self.btn_login)

        
        # ✅ QWidget은 setLayout을 바로 사용합니다.
        self.setLayout(main_layout)

    def login(self):
        username = self.input_user.text()
        # password = self.input_pw.text()


        if not username :
        # if not username or not password:
        
            QMessageBox.warning(self, "경고", "사용자 이름과 비밀번호를 입력하세요.")
            return

        conn = get_connection()
        if conn:
            try:
                cursor = conn.cursor()
                
                # 중요: DB에 username (이름)과 photo_path (사진 경로) 컬럼이 있어야 합니다.
                # query = "SELECT username, password FROM users WHERE username=%s AND password=%s"
                query = "SELECT username FROM users WHERE username=%s"
            
                # cursor.execute(query, (username, password))
                cursor.execute(query, (username))

                result = cursor.fetchone()
                print("SQL result : ", result)
            except Exception as e:
                QMessageBox.critical(self, "데이터베이스 오류", f"조회 중 오류 발생: {e}")
                result = None
            finally:
                conn.close()

        if result:
            # 로그인 성공
            username, photo_path = result
            # username, photo_path = result


                # ⚠️ 중요:
                # 기존 코드는 MainFrame이라는 새 '창'을 띄우고
                # 로그인 '창'을 숨겼습니다.
                #
                # ❌ 기존 방식
                # self.main_frame = MainFrame(...)
                # self.main_frame.show()
                # self.hide() 
                
                # ✅ 변경된 방식:
                # MainFrame도 QWidget으로 만들고 main.py에 등록한 뒤,
                # manager를 통해 페이지를 '전환'해야 합니다.
                
                # 1. MainFrame을 main.py에 등록해야 합니다. (예시)
                #    (main.py에)
                #    from pages.main_frame import MainFrame
                #    manager.add_page("MainFrame", MainFrame(manager, ...)) 
                #
                # 2. 로그인 성공 시, manager에게 페이지 전환을 요청합니다.
                #    (여기서 "MainFrame"은 main.py에서 등록한 이름입니다.)
                
                # self.manager.show_page("MainFrame") 
                
                # ✅ 테스트를 위해 'Sttclass'로 전환하는 예시입니다.
                #    MainFrame을 페이지로 등록했다면 "MainFrame"으로 바꾸세요.

            self.main_frame = MainFrame(
                manager=self.manager,         # ✅ manager 전달
                username=username, 
                photo_path=photo_path,
                login_window=self
            )



            self.manager.show_page("MainFrame")

            #비밀번호 필드 지우기
            self.input_pw.clear()
        
        else:
            # 로그인 실패
            QMessageBox.warning(self, "로그인 실패", "사용자 이름 또는 비밀번호가 틀립니다.")
            self.input_pw.clear()

    def admin_login(self):
        self.manager.show_page("LoginAdminWindow") 
    

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = LoginWindow()
    window.show()
    sys.exit(app.exec())
    
