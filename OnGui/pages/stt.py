
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

# from pages.pro_update import ProUpdateClass

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from db_connect import get_connection

RATE = 16000
CHANNELS = 1
DURATION = 5
OUTPUT_FILE = "temp.wav"

class Sttclass(QMainWindow):


    def __init__(self, manager=None):
        super().__init__()
        self.manager = manager

        self.setWindowTitle("구매희망 리스트 선택 화면")
        self.setGeometry(100, 100, 1280, 720)
        self.main_frame = None     # 메인 프레임 창을 저장할 변수

        # 로그인 단계에서 받아온 상품 데이터를 자동 로딩
        if hasattr(self.manager, "shared_product_data"):
            self.load_product_table(self.manager.shared_product_data)


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
        self.product_info.setHorizontalHeaderLabels(["id", "QTY", "RECV"])  

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

        # # 물품정보 업데이트_button 
        # # self.receive_product_info("0.0.0.0", 3000)
        # self.product_update_button = QPushButton("물품 정보 업데이트", self)
        # self.product_update_button.clicked.connect(self.receive_product_info)

        # # ❗ 물품정보 업데이트 수치로 위치와 크기 지정
        # self.product_update_button.move(430, 530)   # X=150, Y=220
        # self.product_update_button.resize(210, 50)  # 너비 100, 높이 30


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

     

        def load_product_table(self, raw_data: bytes):
            """
            login_tcp_req.py → manager.shared_product_data 로 전달된 상품정보를
            QTableWidget 에 표시하는 함수
            """

            # 1) 앞뒤 DONE 제거
            data = raw_data[4:-4]

            # 2) int32 32개 파싱 (ID 16개 + QTY 16개)
            total_count = len(data) // 4
            values = struct.unpack("<" + "i" * total_count, data)

            ids = values[:16]
            qtys = values[16:32]

            # 3) GUI 테이블 초기화
            self.product_info.setRowCount(16)

            for i in range(16):
                id_item = QTableWidgetItem(str(ids[i]))
                qty_item = QTableWidgetItem(str(qtys[i]))
                recv_item = QTableWidgetItem("OK")   # RECV 임의값 저장

                self.product_info.setItem(i, 0, id_item)
                self.product_info.setItem(i, 1, qty_item)
                self.product_info.setItem(i, 2, recv_item)

            self.product_info.resizeColumnsToContents()


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