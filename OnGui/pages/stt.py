
import os
import sys
import socket
import struct
from PyQt6.QtGui import QPixmap, QColor, QCloseEvent
from PyQt6.QtCore import Qt, QEvent     # QEvent 또는 QShowEvent
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

RATE = 16000
CHANNELS = 1
DURATION = 5
OUTPUT_FILE = "temp.wav"

class Sttclass(QMainWindow):

    def parse_product_response(self, data: bytes):
        """
        서버가 보내온 상품 정보 응답 파싱
        ID 16개 + QTY 16개 구조로 구성됨
        """
        # 예: DONE(4) + ID16개 + QTY16개 + DONE(4)
        # 단순화를 위해 숫자 데이터만 추출

        # 4바이트 'DONE' 제거
        data = data[4:]  # 앞 'DONE' 제거
        data = data[:-4] # 뒤 'DONE' 제거

        # 남은 데이터는 32개의 int32 구성
        count = len(data) // 4
        nums = struct.unpack("<" + "i" * count, data)

        ids = nums[:16]
        qtys = nums[16:32]

        return ids, qtys


    def __init__(self, manager=None):
        super().__init__()
        self.manager = manager

        # self.setWindowTitle("구매희망 리스트 선택 화면")
        # self.setGeometry(100, 100, 1280, 720)
        self.main_frame = None     # 메인 프레임 창을 저장할 변수
        self.initUI()

        # 로그인 단계에서 받아온 상품 데이터를 자동 로딩
        if hasattr(self.manager, "shared_product_data"):
            self.load_product_table(self.manager.shared_product_data)



    def initUI(self):

        self.setWindowTitle("구매희망 리스트 선택 화면")
        self.setGeometry(100, 100, 1280, 720)
        # QMainWindow는 레이아웃을 위해 Central Widget이 필요합니다.
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        # 수직 레이아웃
        main_layout = QVBoxLayout(central_widget)
        
         # ✅ QTableWidget 생성
        self.product_info = QTableWidget()
        self.product_info.setRowCount(0)        # 초기 행 개수
        self.product_info.setColumnCount(3)     # 2개의 컬럼 생성
        
        # 헤더 이름 설정
        self.product_info.setHorizontalHeaderLabels(["Name", "QTY", "RECV"])  

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
        self.product_info.setMaximumSize(700, 700)  # 최대 크기 제한
        self.product_info.setSizePolicy(
            QSizePolicy.Policy.Expanding,
            QSizePolicy.Policy.Expanding
        )
        
        
        
         # ✅ 테이블 정렬 (화면 상단 / 중앙 / 하단 등)
        main_layout.addWidget(self.product_info, alignment=Qt.AlignmentFlag.AlignTop)   # 상단
        # main_layout.addWidget(self.product_info, alignment=Qt.AlignmentFlag.AlignCenter) # 중앙
        # main_layout.addWidget(self.product_info, alignment=Qt.AlignmentFlag.AlignBottom) # 하단


        # 왼쪽, 위, 오른쪽, 아래 여백(px)
        main_layout.setContentsMargins(330, 10, 330, 330)  
        
        # main_layout.setSpacing(20)  # 위젯 간 간격

     
        # Stt_button 
        # self.Stt_button = QPushButton("음성으로 장바구니 담기.!!", self)
        # self.Stt_button.clicked.connect(self.record_once)

        # ❗ 수치로 위치와 크기 지정
        # self.Stt_button.move(430, 470)   # X=150, Y=220
        # self.Stt_button.resize(220, 50)  # 너비 100, 높이 30

         # PC화면_button 
        self.browser_button = QPushButton("PC에서 장바구니 담기.!!", self)
        self.browser_button.clicked.connect(self.pc_browser)

        # # ❗ 수치로 위치와 크기 지정
        self.browser_button.move(660,600)   # X=150, Y=220
        self.browser_button.resize(220, 50)  # 너비 100, 높이 30


        # back_button 
        self.back_button = QPushButton("뒤로가기", self)
        self.back_button.clicked.connect(self.backMove)

        # ❗ back_button > 수치로 위치와 크기 지정
        self.back_button.move(430,600)   # X=150, Y=220
        self.back_button.resize(200, 50)  # 너비 100, 높이 30


        # 레이아웃에 위젯 추가
        main_layout.addWidget(self.product_info)
        # self.setLayout(main_layout)
        central_widget.setLayout(main_layout)

        # Whisper 모델 미리 로드
        self.stt_model = whisper.load_model("base")



    # ✅ 3. showEvent 함수 추가
    def showEvent(self, event: QEvent):
        """
        이 창이 화면에 나타날 때마다 Qt에 의해 자동으로 호출됩니다.
        """
        print("Sttclass 창이 보입니다. 데이터를 로드합니다.")
        
        # 매니저에 공유 데이터가 있는지 확인
        if hasattr(self.manager, "shared_product_data") and self.manager.shared_product_data:
            # 데이터가 있다면 테이블 로드 함수 호출
            self.load_product_table(self.manager.shared_product_data)
        else:
            # 데이터가 없으면 테이블을 비웁니다 (선택 사항)
            self.product_info.setRowCount(0) 
            print("아직 공유된 상품 데이터가 없습니다.")
            
        # 부모의 showEvent를 호출해줘야 합니다.
        super().showEvent(event)

 

    def load_product_table(self, raw_data: bytes):
        """
        login_tcp_req.py → manager.shared_product_data 로 전달된 상품정보를
        QTableWidget 에 표시하는 함수
        """

        # 1) 앞뒤 DONE 제거
        data = raw_data[20:-4]

        # 2. 4바이트씩 나누어 숫자로 변환
        numbers = []
        for i in range(0, len(data), 4):
            value = struct.unpack('<I', data[i:i+4])[0]
            numbers.append(value)

        # 3. 결과 출력
        print(numbers)

        # 2) int32 32개 파싱 (ID 16개 + QTY 16개)
        total_count = len(data) // 4
        values = struct.unpack("<" + "i" * total_count, data)

        # 3) 받은 데이터의 절반을 ID/QTY 개수로 계산
        #    (total_count가 20이면, num_items는 10이 됨)
        num_items = total_count 
        

        # ids = values[:num_items]   # 0번부터 9번까지 (10개)
        # qtys = values[num_items:]  # 10번부터 19번까지 (10개)
        qtys = values  # 10번부터 19번까지 (10개)

        # (1) Name 16개 고정 리스트
        fixed_names = [
            "soju", "beer", "ketchap", "mayonaise",
            "Snack_Org", "Snack_Ylw", "sushi", "pizza",
            "frypan", "pot", "strawberry", "watermelon",
            "grape", "deco_tree", "deco_santa", "deco_ring"
        ]


        # 3) GUI 테이블 초기화 (16줄이 아닌, 받은 만큼만)
        self.product_info.setRowCount(num_items)

        # 16번이 아닌, 받은 개수(num_items)만큼만 반복
        for i in range(num_items):
            # id_item = QTableWidgetItem(str(ids[i]))
            
             # Name 컬럼
            name_item = QTableWidgetItem(fixed_names[i])
            qty_item = QTableWidgetItem(str(qtys[i]))
            recv_item = QTableWidgetItem("OK")      # RECV 임의값 저장
                    
            self.product_info.setItem(i, 0, name_item)
            self.product_info.setItem(i, 1, qty_item)
            self.product_info.setItem(i, 2, recv_item)
            


    def pc_browser(self):
        self.manager.show_page("PC_BrowserClass")

    def shopping_start(self):
        self.manager.show_page("AutoShoppingClass")


    def backMove(self):
        self.manager.show_page("MainFrame")


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

        # # 4. DB 저장
        # success, msg = self.save_to_db(text)

        # ⭐ 4. 새로 만든 로직 호출! (매칭 및 패킷 생성)
        success, msg = self.process_stt_result(text)


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