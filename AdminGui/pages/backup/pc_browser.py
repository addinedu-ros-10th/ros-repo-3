

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
        self.name_label = QLabel("빈칸에 상품명을 입력해주세요.!", central_widget) 
        font = self.name_label.font()
        font.setPointSize(16)
        self.name_label.setFont(font)
        
        # ❗ 1번 수치로 위치와 크기 지정
        self.name_label.move(300, 50)      
        self.name_label.resize(360, 40)  

        # --- 2. 상품명 입력 창 ---
        self.input_product = QLineEdit(self)
        
        # 비밀번호 숨김처리 기능
        # self.input_product.setEchoMode(QLineEdit.EchoMode.Password)

        # ❗ 2. 수치로 위치와 크기 지정
        self.input_product.move(300, 100)      
        self.input_product.resize(360, 40)   
        

        # 3. 상품명 저장 버튼
        self.product_save_btn_ = QPushButton("상품명 저장", self)
        self.product_save_btn_.clicked.connect(self.on_add_button_clicked)

        # ❗ 3. 상품명 저장 버튼 수치로 위치와 크기 지정
        self.product_save_btn_.move(300, 150)      
        self.product_save_btn_.resize(360, 40)  

        # ✅ QTableWidget 생성
        self.product_info = QTableWidget()
        self.product_info.setRowCount(0)  # 초기 행 개수
        self.product_info.setColumnCount(3)  # 3개의 컬럼 생성
        self.product_info.setHorizontalHeaderLabels(["Iid", "QTY", "RECV"])  # 헤더 이름 설정

        # ✅ 테이블 크기 조정
        self.product_info.resizeColumnsToContents()
        self.product_info.setColumnWidth(0, 150)
        self.product_info.setColumnWidth(1, 150)
        self.product_info.setColumnWidth(2, 150)

        # ✅ 스타일 지정 (이미지와 유사하게)
        self.product_info.setStyleSheet("""
            QTableWidget {
                border: 1px solid gray;
                border-radius: 3px;
                gridline-color: gray;
                background-color: white;
                font-size: 13px;
            }
            QHeaderView::section {
                background-color: #f0f0f0;
                border: 1px solid lightgray;
                font-weight: bold;
                padding: 4px;
            }
        """)

        # “2️⃣ QWidget + QVBoxLayout 또는 QHBoxLayout을 사용하는 경우”에서는,
        # 레이아웃 기반 배치이므로 .move() 같은 “절대 좌표” 이동은 사용할 수 없습니다.
        # 대신, 위젯의 위치를 조정하려면 **레이아웃 안에서 정렬(Alignment) 
        # 또는 여백(Margin, Spacing)**을 조절해야 합니다.

        self.product_info.setMinimumSize(130, 130)  # 최소 크기 지정
        self.product_info.setMaximumSize(450, 300)  # 최대 크기 제한
        self.product_info.setSizePolicy(
            QSizePolicy.Policy.Expanding,
            QSizePolicy.Policy.Expanding
        )

         # ✅ 테이블 정렬 (화면 상단 / 중앙 / 하단 등)
        # main_layout.addWidget(self.product_info, alignment=Qt.AlignmentFlag.AlignTop)   # 상단
        main_layout.addWidget(self.product_info, alignment=Qt.AlignmentFlag.AlignCenter) # 중앙
        # main_layout.addWidget(self.product_info, alignment=Qt.AlignmentFlag.AlignBottom) # 하단

        # 왼쪽, 위, 오른쪽, 아래 여백(px)
        main_layout.setContentsMargins(220, 200, 380, 200)  
        main_layout.setSpacing(20)  # 위젯 간 간격


        #========================================================== 

         # --- 4. 구매희망 리스트 라벨 ---
        self.name_label1 = QLabel("구매희망 쇼핑리스트.!", central_widget) 
        font = self.name_label1.font()
        font.setPointSize(16)
        self.name_label1.setFont(font)

        # ❗ 4. 구매희망 리스트 라벨 수치로 위치와 크기 지정
        self.name_label1.move(700, 40)      
        self.name_label1.resize(360, 50) 


        # --- 5. 구매희망 리스트 노출 화면 라벨 ---
        self.product_label = QLabel("", central_widget) 

        # ❗ 5. 구매희망 리스트 노출 화면 수치로 위치와 크기 지정
        self.product_label.move(700, 100)      
        self.product_label.resize(360, 350)  

        self.product_label.setStyleSheet("""
            border: 1px solid gray;      /* 검은색 실선 */
            border-radius: 5px;           /* 모서리를 살짝 둥글게 */
            background-color: #f9f9f9;    /* 연한 배경색 (선이 더 잘 보이게) */
            color: black;                 /* 글자색 */
            padding-left: 10px;           /* 글자 여백 */
                                               
        """) 
        
        # 6. 구매희망 리스트 작성 Reset 버튼
        self.reset_btn = QPushButton("Reset", self)
        self.reset_btn.clicked.connect(self.reset)

        # ❗ 6. 구매희망 리스트 작성 Reset 수치로 위치와 크기 지정
        self.reset_btn.move(700, 460)      
        self.reset_btn.resize(360, 40)   
        
        
        # 7. 구매희망 리스트 작성 저장 버튼
        self.product_save_btn = QPushButton("구매희망 리스트 저장", self)
        # self.product_save_btn.clicked.connect(self.product_list_create)

        # ❗ 7. 구매희망 리스트 작성 저장 수치로 위치와 크기 지정
        self.product_save_btn.move(700, 510)      
        self.product_save_btn.resize(360, 40) 

 
        # 8. 구매희망 쇼핑 리스트 이동 버튼
        self.product_move_btn = QPushButton("쇼핑리스트 보러가기", self)
        self.product_move_btn.clicked.connect(self.AutoShoppingClass)

        # ❗ 8. 구매희망 쇼핑 리스트 이동 수치로 위치와 크기 지정
        self.product_move_btn.move(700, 560)      
        self.product_move_btn.resize(360, 40)   


        # ✅ QWidget은 setLayout을 바로 사용합니다.

        main_layout.addWidget(self.product_info)
        self.setLayout(main_layout)
        
        
    def reset(self):
        self.product_label.clear()

        
    def AutoShoppingClass(self):
        self.manager.show_page("AutoShoppingClass")
        

    # ✅ 버튼 클릭 시 실행되는 함수
    def on_add_button_clicked(self):
        text = self.input_product.text().strip()
        if text:
            self.add_text(text)
            self.input_product.clear()

     
    def add_text(self, new_text):
        # 기존 텍스트 가져오기 (공백 제거)
        old_text = self.product_label.text().strip()

        # 줄 단위로 분리
        lines = old_text.split("\n") if old_text else []

        # 기존 번호 라벨링 제거 (예: "1. 내용" → "내용")
        clean_lines = [line.split(". ", 1)[1] if ". " in line else line for line in lines]

        # 새 텍스트를 최상단에 추가
        clean_lines.insert(0, new_text.strip())

        # 번호 다시 라벨링
        numbered_lines = [f"{i+1}. {line}" for i, line in enumerate(clean_lines)]

        # 라벨에 최종 텍스트 적용
        updated_text = "\n".join(numbered_lines)
        self.product_label.setText(updated_text)


if __name__ == "__main__":
    
    app = QApplication(sys.argv)
    window = PC_BrowserClass()
    window.show()
    sys.exit(app.exec())
    