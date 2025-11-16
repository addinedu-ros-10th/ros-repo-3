
import os
import sys
import socket
import struct
from PyQt6.QtGui import QPixmap, QColor
from PyQt6.QtWidgets import *
from PyQt6 import uic
from PyQt6.QtCore import Qt

import mysql.connector


sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from db_connect import get_connection


class AdminDetailClass(QMainWindow):


    def __init__(self, manager=None):
        super().__init__()

        self.manager = manager

        self.setWindowTitle("구매희망 리스트 선택 화면")
        self.setGeometry(100, 100, 1400, 720)
        self.main_frame = None                                      # 메인 프레임 창을 저장할 변수
        self.initUI()

    def initUI(self):
        # QMainWindow는 레이아웃을 위해 Central Widget이 필요합니다.
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        # 수직 레이아웃
        main_layout = QVBoxLayout(central_widget)
        
        # -----------------------------
        # 첫 번째 테이블(product_info)
        # -----------------------------
        self.product_info = QTableWidget()
        self.product_info.setRowCount(0)
        self.product_info.setColumnCount(3)
        self.product_info.setHorizontalHeaderLabels(["Iid", "QTY", "RECV"])
        self.product_info.resizeColumnsToContents()
        self.product_info.setColumnWidth(0, 150)
        self.product_info.setColumnWidth(1, 200)
        self.product_info.setColumnWidth(2, 150)
        self.product_info.setMinimumSize(130, 130)
        self.product_info.setMaximumSize(470, 450)

        # -----------------------------
        # 두 번째 테이블(product_info_1)
        # -----------------------------
        self.product_info_1 = QTableWidget()
        self.product_info_1.setRowCount(0)
        self.product_info_1.setColumnCount(3)
        self.product_info_1.setHorizontalHeaderLabels(["Iid", "QTY", "RECV"])
        self.product_info_1.resizeColumnsToContents()
        self.product_info_1.setColumnWidth(0, 150)
        self.product_info_1.setColumnWidth(1, 200)
        self.product_info_1.setColumnWidth(2, 150)
        self.product_info_1.setMinimumSize(130, 130)
        self.product_info_1.setMaximumSize(470, 450)
        # self.product_info_1.setMaximumSize(470, 450)


        # -----------------------------
        # 세 번째 테이블(product_info_2)
        # -----------------------------
        self.product_info_2 = QTableWidget()
        self.product_info_2.setRowCount(0)
        self.product_info_2.setColumnCount(3)
        self.product_info_2.setHorizontalHeaderLabels(["Iid", "QTY", "RECV"])
        self.product_info_2.resizeColumnsToContents()
        self.product_info_2.setColumnWidth(0, 150)
        self.product_info_2.setColumnWidth(1, 200)
        self.product_info_2.setColumnWidth(2, 150)
        self.product_info_2.setMinimumSize(130, 130)
        self.product_info_2.setMaximumSize(470, 450)

        # -----------------------------
        # 테이블 3개를 가로로 나란히 배치
        # -----------------------------
        table_layout = QHBoxLayout()
        table_layout.addWidget(self.product_info)
        table_layout.addWidget(self.product_info_1)
        table_layout.addWidget(self.product_info_2)

  
        # Stt_button 
        self.Stt_button = QPushButton("음성으로 장바구니 담기.!!", self)
        # self.Stt_button.clicked.connect(self.record_once)

        # ❗ 수치로 위치와 크기 지정
        self.Stt_button.move(10, 470)   # X=150, Y=220
        self.Stt_button.resize(455, 50)  # 너비 100, 높이 30


        # 물품정보 업데이트_button 
        self.product_update_button = QPushButton("물품 정보 업데이트", self)
        self.product_update_button.clicked.connect(self.product_update)

        # ❗ 물품정보 업데이트 수치로 위치와 크기 지정
        self.product_update_button.move(470, 470)   # X=150, Y=220
        self.product_update_button.resize(455, 50)  # 너비 100, 높이 30

        # PC화면_button 
        self.browser_button = QPushButton("PC에서 장바구니 담기.!!", self)
        self.browser_button.clicked.connect(self.pc_browser)

        # ❗ 수치로 위치와 크기 지정
        self.browser_button.move(930, 470)   # X=150, Y=220
        self.browser_button.resize(455, 50)  # 너비 100, 높이 30


        self.back_button = QPushButton("뒤로가기", self)
        self.back_button.clicked.connect(self.backMove)

        # ❗ back_button > 수치로 위치와 크기 지정
        self.back_button.move(10,670)   # X=150, Y=220
        self.back_button.resize(200, 40)  # 너비 100, 높이 30

        # # 온라인 쇼핑 시작_button 
        # self.shopping_start_button = QPushButton("쇼핑리스트 보러가기", self)
        # self.shopping_start_button.clicked.connect(self.shopping_start)

        # # ❗ 수치로 위치와 크기 지정
        # self.shopping_start_button.move(660, 530)   # X=150, Y=220
        # self.shopping_start_button.resize(220, 50)  # 너비 100, 높이 30
        # back_button 


        # 레이아웃에 위젯 추가
        # main_layout.addWidget(self.Stt_button)
        main_layout.addLayout(table_layout)
        
         # ✅ 3개의 테이블이 무조건 창의 상단에 붙도록 정렬
        main_layout.setAlignment(table_layout, Qt.AlignmentFlag.AlignTop)
        
        central_widget.setLayout(main_layout)


    def product_update(self):
        customer_id = 2  # 고객 ID(예: 2)

        # 1~16번까지 모든 물품 선택
        selected_items = list(range(1, 17))

        # 모든 물품의 수량은 0 (요청 목적)
        quantities = {i: 0 for i in selected_items}

        # 패킷 생성 및 전송
        packet = self.make_store_item_request(customer_id, selected_items, quantities)
        self.send_tcp_packet(packet)

    # ---------------------------------------------------------
    # ② 프로토콜 형식에 맞는 패킷 구성
    # ---------------------------------------------------------
    def make_store_item_request(self, customer_id, selected_items, quantities):
        function_id = 1  # 매장 물품 정보 요청

        # -----------------------------------------------------
        # (1) 물품 ID bit flag 생성 (1~16번 전부 활성화)
        # -----------------------------------------------------
        item_flag = 0
        for item in selected_items:
            item_flag |= (1 << (item - 1))  # 모든 비트 켜짐

        # -----------------------------------------------------
        # (2) 수량 리스트 생성 (항상 16개)
        # -----------------------------------------------------
        qty_list = []
        for i in range(1, 17):
            qty = quantities.get(i, 0)  # 없으면 0
            qty_list.append(qty)

        # -----------------------------------------------------
        # (3) body 생성 - 고객ID + 물품ID(bit) + 수량(16개)
        # -----------------------------------------------------
        body = struct.pack(
            "!B I" + "H" * 16,
            customer_id,     # 1바이트
            item_flag,       # 4바이트 물품 비트ID
            *qty_list        # 16개 x 2바이트
        )

        # -----------------------------------------------------
        # (4) 전체 패킷 길이 계산 = header + body
        # -----------------------------------------------------
        packet_length = len(body) + 3  # FunctionID(1) + Length(2)

        header = struct.pack("!BH", function_id, packet_length)
        return header + body

    # ---------------------------------------------------------
    # ③ TCP 패킷 전송
    # ---------------------------------------------------------
    def send_tcp_packet(self, packet):
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
                sock.connect(("127.0.0.1", 9500))
                sock.sendall(packet)

                print("전송된 패킷:", packet)

                response = sock.recv(1024)
                print("서버 응답:", response)

        except Exception as e:
            print("TCP 통신 오류:", e)

    def pc_browser(self):
        self.manager.show_page("PC_BrowserClass")

    def shopping_start(self):
        self.manager.show_page("AutoShoppingClass")


    def backMove(self):
        self.manager.show_page("MainFrame")

     

if __name__ == "__main__":
    app = QApplication(sys.argv)
    myWindows = AdminDetailClass()
    myWindows.show()
    sys.exit(app.exec())