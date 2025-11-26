import os
import sys
from PyQt6.QtGui import QPixmap, QColor
from PyQt6.QtWidgets import *
from PyQt6 import uic
from PyQt6.QtCore import Qt

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from db_connect import get_connection


class AutoShoppingClass(QMainWindow):

    def __init__(self, manager=None):
        super().__init__()
        self.manager = manager

        self.setWindowTitle("구매희망 쇼핑 리스트")
        self.setGeometry(100, 100, 1280, 720)
        self.main_frame = None                                      
        self.initUI()


    def initUI(self):
        # QMainWindow는 레이아웃을 위해 Central Widget이 필요합니다.
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        # 수직 레이아웃
        main_layout = QVBoxLayout()

        # --- 1. 환영 메시지 라벨 ---
        # 부모 위젯을 central_widget으로 지정합니다.
        self.name_label = QLabel("스마트 쇼핑카트 쇼핑 안내화면", central_widget) 
        font = self.name_label.font()
        font.setPointSize(16)
        self.name_label.setFont(font)
        
        # ❗ 수치로 위치와 크기 지정
        self.name_label.move(510, 50)      # X=20, Y=20 위치
        self.name_label.resize(360, 40)   # 너비 360, 높이 40

        # --- 2. 유저의 구매희망 리스트 표시 라벨 ---
        self.shopping_list_label = QLabel("", central_widget) 

        # ❗ 수치로 위치와 크기 지정
        self.shopping_list_label.move(430, 100)         # X=20, Y=20 위치
        self.shopping_list_label.resize(450, 400)        # 너비 360, 높이 40

        self.shopping_list_label.setStyleSheet("""
            border: 1px solid gray;      /* 검은색 실선 */
            border-radius: 5px;           /* 모서리를 살짝 둥글게 */
            background-color: #f9f9f9;    /* 연한 배경색 (선이 더 잘 보이게) */
            color: black;                 /* 글자색 */
            padding-left: 10px;           /* 글자 여백 */
                                               
        """)

        # border: 3px dashed blue;   # 파란색 점선
        # border: 1px solid gray;    # 얇은 회색 실선
        # border: 2px solid black;      /* 검은색 실선 */
        
        # Stt_button 
        self.Stt_button = QPushButton("구매희망 리스트 다시 선택하기", self)
        self.Stt_button.clicked.connect(self.Sttclass_move)
        

        # ❗ 수치로 위치와 크기 지정
        self.Stt_button.move(430, 550)   # X=150, Y=220
        self.Stt_button.resize(450, 50)  # 너비 100, 높이 30
        
        # Guide_mode_button 
        
        self.Guide_mode_button = QPushButton("가이드 모드", self)
        # self.Guide_mode_button.clicked.connect(self.Guide_mode)

        # ❗ Guide_mode 수치로 위치와 크기 지정
        self.Guide_mode_button.move(100, 100)   # X=150, Y=220
        self.Guide_mode_button.resize(210, 50)  # 너비 100, 높이 30

        # Folling_mode_button 
        
        self.Folling_mode_button = QPushButton("팔로잉 모드", self)
        # self.Folling_mode_button.clicked.connect(self.Folling_mode)

        # ❗ Folling_mode 수치로 위치와 크기 지정
        self.Folling_mode_button.move(100, 200)   # X=150, Y=220
        self.Folling_mode_button.resize(210, 50)  # 너비 100, 높이 30
     
        

        # back_button 
        self.back_button = QPushButton("뒤로가기", self)
        self.back_button.clicked.connect(self.backMove)

        # ❗ back_button > 수치로 위치와 크기 지정
        self.back_button.move(100, 40)   # X=150, Y=220
        self.back_button.resize(200, 40)  # 너비 100, 높이 30

        # 온라인 쇼핑 시작_button 
        self.shopping_start_button = QPushButton("온라인 쇼핑 종료", self)
        # self.shopping_start_button.clicked.connect(self.shopping_start)

        # ❗ 수치로 위치와 크기 지정
        self.shopping_start_button.move(430, 620)   # X=150, Y=220
        self.shopping_start_button.resize(450, 50)  # 너비 100, 높이 30

        # 레이아웃에 위젯 추가
        central_widget.setLayout(main_layout)

  

    def Sttclass_move(self):
        self.manager.show_page("Sttclass")

    # def shopping_start(self):
    #     self.manager.show_page("MainFrame")


    def backMove(self):
        self.manager.show_page("MainFrame")
     

if __name__ == "__main__":
    app = QApplication(sys.argv)
    myWindows = AutoShoppingClass()
    myWindows.show()
    sys.exit(app.exec())