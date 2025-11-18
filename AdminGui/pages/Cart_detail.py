

import os
import sys
import socket
import struct
from PyQt6.QtGui import QPixmap, QColor
from PyQt6.QtWidgets import *
from PyQt6 import uic
from PyQt6.QtCore import Qt

import mysql.connector
import soundfile as sf
import numpy as np
import whisper
import sounddevice as sd


sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from db_connect import get_connection


class CartDetailClass(QMainWindow):


    def __init__(self, manager=None):
        super().__init__()
        self.manager = manager

        self.setWindowTitle("Cart 상태 상세화면")
        self.setGeometry(100, 100, 1280, 720)
        self.main_frame = None                      # 메인 프레임 창을 저장할 변수

        # 로그인 단계에서 받아온 상품 데이터를 자동 로딩
        # if hasattr(self.manager, "shared_product_data"):
        #     self.load_product_table(self.manager.shared_product_data)


        self.initUI()

    def initUI(self):
        # QMainWindow는 레이아웃을 위해 Central Widget이 필요합니다.
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        # 수직 레이아웃
        main_layout = QVBoxLayout(central_widget)
        
         # ✅ QTableWidget 생성
        self.product_info = QTableWidget()
        self.product_info.setRowCount(0)  # 초기 행 개수
        self.product_info.setColumnCount(5)  # 5개의 컬럼 생성
        
        # 헤더 이름 설정
        self.product_info.setHorizontalHeaderLabels(["Robot_name", "Taget_id", "battery", "Cart_Error_status", "Order Status"])  

        # ✅ 테이블 크기 조정
        self.product_info.resizeColumnsToContents()
        self.product_info.setColumnWidth(0, 100)
        self.product_info.setColumnWidth(1, 100)
        self.product_info.setColumnWidth(2, 100)
        self.product_info.setColumnWidth(3, 250)
        self.product_info.setColumnWidth(4, 150)


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
        self.product_info.setMaximumSize(900, 900)  # 최대 크기 제한
        self.product_info.setSizePolicy(
            QSizePolicy.Policy.Expanding,
            QSizePolicy.Policy.Expanding
        )

         # ✅ 테이블 정렬 (화면 상단 / 중앙 / 하단 등)
        main_layout.addWidget(self.product_info, alignment=Qt.AlignmentFlag.AlignTop)   # 상단
        # main_layout.addWidget(self.product_info, alignment=Qt.AlignmentFlag.AlignCenter) # 중앙
        # main_layout.addWidget(self.product_info, alignment=Qt.AlignmentFlag.AlignBottom) # 하단

        main_layout.setContentsMargins(250, 30, 250, 300)  # 왼쪽, 위, 오른쪽, 아래 여백(px)
        main_layout.setSpacing(20)  # 위젯 간 간격

        # Stt_button 
        self.Stt_button = QPushButton("음성으로 장바구니 담기.!!", self)
        # self.Stt_button.clicked.connect(self.record_once)

        # ❗ 수치로 위치와 크기 지정
        self.Stt_button.move(430, 470)   # X=150, Y=220
        self.Stt_button.resize(220, 50)  # 너비 100, 높이 30

         # PC화면_button 
        self.browser_button = QPushButton("PC에서 장바구니 담기.!!", self)
        self.browser_button.clicked.connect(self.pc_browser)

        # ❗ 수치로 위치와 크기 지정
        self.browser_button.move(660, 470)   # X=150, Y=220
        self.browser_button.resize(220, 50)  # 너비 100, 높이 30


        # back_button 
        self.back_button = QPushButton("뒤로가기", self)
        self.back_button.clicked.connect(self.backMove)

        # ❗ back_button > 수치로 위치와 크기 지정
        self.back_button.move(100, 600)   # X=150, Y=220
        self.back_button.resize(200, 40)  # 너비 100, 높이 30

    
        # 온라인 쇼핑 시작_button 
        self.shopping_start_button = QPushButton("쇼핑리스트 보러가기", self)
        self.shopping_start_button.clicked.connect(self.shopping_start)

        # ❗ 수치로 위치와 크기 지정
        self.shopping_start_button.move(660, 530)   # X=150, Y=220
        self.shopping_start_button.resize(220, 50)  # 너비 100, 높이 30


        # 레이아웃에 위젯 추가
        # main_layout.addWidget(self.Stt_button)
        main_layout.addWidget(self.product_info)
        central_widget.setLayout(main_layout)



    def pc_browser(self):
        self.manager.show_page("PC_BrowserClass")

    def shopping_start(self):
        self.manager.show_page("AutoShoppingClass")


    def backMove(self):
        self.manager.show_page("MainFrame")



if __name__ == "__main__":
    app = QApplication(sys.argv)
    myWindows = CartDetailClass()
    myWindows.show()
    sys.exit(app.exec())