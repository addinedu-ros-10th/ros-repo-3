

import os
import sys
from PyQt6.QtWidgets import QSizePolicy,QTableWidget, QApplication, QMainWindow, QLabel, QLineEdit, QPushButton, QMessageBox, QWidget, QVBoxLayout, QFormLayout
from PyQt6.QtCore import Qt


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
class PC_BrowserClass(QMainWindow): 

    def __init__(self, manager=None):
        super().__init__()
        self.manager = manager
        self.main_frame = None
        self.initUI()

    def initUI(self):

        self.setWindowTitle(" PC화면에서 장바구니 담기")
        self.resize(1280, 720)
        
        # QMainWindow는 레이아웃을 위해 Central Widget이 필요합니다.
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        # 수직 레이아웃
        main_layout = QVBoxLayout(central_widget)

        # --- 1. 환영 메시지 라벨 ---
        # 부모 위젯을 central_widget으로 지정합니다.
        self.name_label = QLabel("원하시는 상품류를 1개만 선택하세요.!!", central_widget) 
        font = self.name_label.font()
        font.setPointSize(16)
        self.name_label.setFont(font)
        
        # ❗ 1번 수치로 위치와 크기 지정
        self.name_label.move(500, 50)      
        self.name_label.resize(360, 40)  


        # 1. 음료류 저장 버튼
        self.product_save_btn_ = QPushButton("음료류 저장", self)
        # self.product_save_btn_.clicked.connect(self.on_add_button_clicked)

        # ❗ 1. 음료류 저장 버튼 수치로 위치와 크기 지정
        self.product_save_btn_.move(200, 150)      
        self.product_save_btn_.resize(200, 40)  

        
        # 2. 소스류 버튼
        self.reset_btn = QPushButton("소스류", self)
        # self.reset_btn.clicked.connect(self.reset)

        # ❗ 2. 소스류 수치로 위치와 크기 지정
        self.reset_btn.move(400, 150)      
        self.reset_btn.resize(200, 40)   
        
        
        # 3. 과자류 저장 버튼
        self.product_save_btn = QPushButton("과자류 저장", self)
        # self.product_save_btn.clicked.connect(self.product_list_create)

        # ❗ 3. 과자류 저장 수치로 위치와 크기 지정
        self.product_save_btn.move(600, 150)      
        self.product_save_btn.resize(200, 40) 

         # 4. 음식류 저장 버튼
        self.product_save_btn = QPushButton("음식류 저장", self)
        # self.product_save_btn.clicked.connect(self.product_list_create)

        # ❗ 4. 음식류 저장 수치로 위치와 크기 지정
        self.product_save_btn.move(200, 250)      
        self.product_save_btn.resize(200, 40)

         # 5. 가정용품 버튼
        self.product_save_btn = QPushButton("가정용품 저장", self)
        # self.product_save_btn.clicked.connect(self.product_list_create)

        # ❗ 5. 가정용품 저장 수치로 위치와 크기 지정
        self.product_save_btn.move(400, 250)      
        self.product_save_btn.resize(200, 40)

         # 6. 과일류 저장 버튼
        self.product_save_btn = QPushButton("과일류 저장", self)
        # self.product_save_btn.clicked.connect(self.product_list_create)

        # ❗ 6. 과일류 저장 수치로 위치와 크기 지정
        self.product_save_btn.move(600, 250)      
        self.product_save_btn.resize(200, 40)

        # 7. 장식품류 저장 버튼
        self.product_save_btn = QPushButton("장식품 저장", self)
        # self.product_save_btn.clicked.connect(self.product_list_create)

        # ❗7. 장식품류 저장 수치로 위치와 크기 지정
        self.product_save_btn.move(800, 250)      
        self.product_save_btn.resize(200, 40)
        
        
        # 8. 구매희망 쇼핑 리스트 이동 버튼
        self.product_move_btn = QPushButton("쇼핑리스트 보러가기", self)
        self.product_move_btn.clicked.connect(self.AutoShoppingClass)

        # ❗ 8. 구매희망 쇼핑 리스트 이동 수치로 위치와 크기 지정
        self.product_move_btn.move(500,450)      
        self.product_move_btn.resize(200, 40)   
             


        # ✅ QWidget은 setLayout을 바로 사용합니다.

        # main_layout.addWidget(self.product_info)
        self.setLayout(main_layout)
        

        
    def AutoShoppingClass(self):
        self.manager.show_page("AutoShoppingClass")
        


if __name__ == "__main__":
    
    app = QApplication(sys.argv)
    window = PC_BrowserClass()
    window.show()
    sys.exit(app.exec())
    