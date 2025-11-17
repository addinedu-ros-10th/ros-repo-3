
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

from pro_update import ProUpdateClass

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from db_connect import get_connection

RATE = 16000
CHANNELS = 1
DURATION = 5
OUTPUT_FILE = "temp.wav"

class Sttclass(QMainWindow):


    def __init__(self, manager=None):
        super().__init__()
        # self.setupUi(self)
        # self.setWindowTitle("SSCart_STT")
        self.manager = manager

        self.setWindowTitle("구매희망 리스트 선택 화면")
        self.setGeometry(100, 100, 1280, 720)
        self.main_frame = None                                      # 메인 프레임 창을 저장할 변수
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
        self.product_info.setColumnCount(3)  # 3개의 컬럼 생성
        
        # 헤더 이름 설정
        self.product_info.setHorizontalHeaderLabels(["Iid", "QTY", "RECV"])  

        # ✅ 테이블 크기 조정
        self.product_info.resizeColumnsToContents()
        self.product_info.setColumnWidth(0, 150)
        self.product_info.setColumnWidth(1, 200)
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
        self.product_info.setMaximumSize(470, 450)  # 최대 크기 제한
        self.product_info.setSizePolicy(
            QSizePolicy.Policy.Expanding,
            QSizePolicy.Policy.Expanding
        )

         # ✅ 테이블 정렬 (화면 상단 / 중앙 / 하단 등)
        main_layout.addWidget(self.product_info, alignment=Qt.AlignmentFlag.AlignTop)   # 상단
        # main_layout.addWidget(self.product_info, alignment=Qt.AlignmentFlag.AlignCenter) # 중앙
        # main_layout.addWidget(self.product_info, alignment=Qt.AlignmentFlag.AlignBottom) # 하단

        main_layout.setContentsMargins(430, 30, 300, 300)  # 왼쪽, 위, 오른쪽, 아래 여백(px)
        main_layout.setSpacing(20)  # 위젯 간 간격

        
        
        # --- 2. 마이크 이모티콘 라벨 ---
        # self.photo_label = QLabel(central_widget) # 부모 지정
        # self.photo_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        
        # pixmap = QPixmap()
        # if pixmap.isNull():
        #     pixmap = QPixmap(450,300)
        #     pixmap.fill(QColor("gray"))
        
        # self.photo_label.setPixmap(pixmap.scaled(450, 450, Qt.AspectRatioMode.KeepAspectRatio))

        # # ❗ 수치로 위치와 크기 지정 (창의 중앙 근처)
        # self.photo_label.move(410, 30)    # X=150, Y=80
        # self.photo_label.resize(500, 300) # 너비 100, 높이 100

        # Stt_button 
        self.Stt_button = QPushButton("음성으로 장바구니 담기.!!", self)
        self.Stt_button.clicked.connect(self.record_once)

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
        self.back_button.move(100, 40)   # X=150, Y=220
        self.back_button.resize(200, 40)  # 너비 100, 높이 30

        # 물품정보 업데이트_button 
        self.product_update_button = QPushButton("물품 정보 업데이트", self)
        self.product_update_button.clicked.connect(self.product_update)

        # ❗ 물품정보 업데이트 수치로 위치와 크기 지정
        self.product_update_button.move(430, 530)   # X=150, Y=220
        self.product_update_button.resize(210, 50)  # 너비 100, 높이 30


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

        # Whisper 모델 미리 로드
        self.stt_model = whisper.load_model("base")


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

     

    def save_to_db(self, text):
        """
        MySQL DB에 STT 결과를 저장하는 함수
        """
        try:
            conn = get_connection()
            cursor = conn.cursor()
            
            # sql = "INSERT INTO users (username, password ,stt_results) VALUES (%s,%s ,%s)" \
            
            sql = """
                INSERT INTO users (username, password , stt_results)
                VALUES (%s, %s, %s)
                ON DUPLICATE KEY UPDATE
                stt_results = VALUES(stt_results)
                """

            cursor.execute(sql, ("test", 1234, text))

            
            conn.commit()
            return True, "DB 저장 성공"
        except Exception as e:
            return False, f"DB 저장 실패: {e}"
        finally:
            cursor.close()
            conn.close()


    def record_once(self):
        # 1. 녹음
        QMessageBox.information(self, "녹음 시작", f"{DURATION}초 동안 말하세요...")
        recording = sd.rec(int(DURATION * RATE), samplerate=RATE, channels=CHANNELS, dtype='int16')
        sd.wait()
        QMessageBox.information(self, "녹음 종료", "녹음이 종료되었습니다.")

        # 2. 파일 저장
        audio_np = recording.astype(np.float32) / 32768.0
        sf.write(OUTPUT_FILE, audio_np, RATE)

        # 3. STT 변환
        result = self.stt_model.transcribe(OUTPUT_FILE, language="ko")
        text = result["text"]
        print("STT 결과:", text)

        # 4. DB 저장
        success, msg = self.save_to_db(text)

        # 5. 저장 결과 GUI 알림
        if success:
            QMessageBox.information(self, "완료", f"STT 결과가 DB에 저장되었습니다.\n\n{text}")
        else:
            QMessageBox.critical(self, "오류", msg)

        return text


if __name__ == "__main__":
    app = QApplication(sys.argv)
    myWindows = Sttclass()
    myWindows.show()
    sys.exit(app.exec())