
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

        self.setWindowTitle("구매희망 리스트 선택 화면")
        self.setGeometry(100, 100, 1280, 720)
        self.main_frame = None     # 메인 프레임 창을 저장할 변수
        self.initUI()

        # 로그인 단계에서 받아온 상품 데이터를 자동 로딩
        if hasattr(self.manager, "shared_product_data"):
            self.load_product_table(self.manager.shared_product_data)



    def initUI(self):
        # QMainWindow는 레이아웃을 위해 Central Widget이 필요합니다.
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        # 수직 레이아웃
        main_layout = QVBoxLayout(central_widget)
        
         # ✅ QTableWidget 생성
        self.product_info = QTableWidget()
        self.product_info.setRowCount(0)  # 초기 행 개수
        self.product_info.setColumnCount(3)  # 2개의 컬럼 생성
        
        # 헤더 이름 설정
        self.product_info.setHorizontalHeaderLabels(["Name", "QTY"])  

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
            recv_item = QTableWidgetItem("OK")              # RECV 임의값 저장
                    
            self.product_info.setItem(i, 0, name_item)
            self.product_info.setItem(i, 1, qty_item)
            self.product_info.setItem(i, 2, recv_item)
            
        # self.product_info.resizeColumnsToContents()


    def pc_browser(self):
        self.manager.show_page("PC_BrowserClass")

    def shopping_start(self):
        self.manager.show_page("AutoShoppingClass")


    def backMove(self):
        self.manager.show_page("MainFrame")


    # QTY 배열 생성 및 TCP 통신을 위한 메서드 추가
    def process_stt_result(self, stt_text):
        """
        STT 결과를 고정 리스트와 비교하여 QTY 배열을 만들고 서버에 전송합니다.
        """
        
        # 2. Name 16개 고정 리스트 (클래스 내부 또는 함수 내에서 정의)
        fixed_names = [
            "소주", "beer", "ketchap", "mayonaise",
            "Snack_Org", "Snack_Ylw", "스시", "피자",
            "frypan", "pot", "strawberry", "watermelon",
            "grape", "deco_tree", "deco_santa", "deco_ring"
        ]
        
        # 3. 'QTY' 변수 명의 배열을 만들고 초기값을 모두 0으로 설정 (16개)
        # 이 배열의 값(0 또는 1)이 서버에 전송할 최종 수량 정보가 됩니다.
        QTY_array = [0] * 16

        # STT 결과를 소문자로 변환하여 비교 준비
        search_text = stt_text.lower().strip()
        print(f"매칭을 시도할 STT 텍스트: '{search_text}'")

        # 16개의 상품 이름을 순회하며 STT 결과와 매칭
        # ⭐ 핵심: 순서(인덱스)를 유지하며 비교합니다.
        for i, name in enumerate(fixed_names):
            # STT 결과 텍스트 안에 고정된 이름이 포함되어 있는지 확인
            # 예: '소주 하나' 라는 텍스트 안에 'soju'가 포함되어 있는지 (한글이므로 한글 매칭 필요)
            
            # 실제 서비스에서는 'soju'에 대응되는 한국어 명칭 '소주'와 비교해야 하지만, 
            # 여기서는 예시로 영문 명칭을 텍스트에 직접 포함시킨다고 가정하거나,
            # 한국어로 발음되는 상품 이름으로 매칭해야 합니다.
            
            # 일단, 고등학생의 이해를 돕기 위해 'soju'라는 이름이 STT 결과에 'soju'로 나왔다고 가정합니다.
            if name.lower() in search_text:
                # 매칭된 경우, 해당 위치에 수량 1을 넣습니다.
                QTY_array[i] = 1
                print(f"✅ 매칭 성공: '{name}' (인덱스 {i}) -> QTY: 1")
            
        print("최종 QTY 배열:", QTY_array)
        
        # 4. QTY 배열을 포함한 TCP 패킷 생성 및 전송
        # 여기서는 테스트를 위해 임의의 Header 값(Transaction_ID 등)을 사용합니다.
        Transaction_ID = 17# 예시: 장바구니 업데이트 기능 ID
        Function_ID = 2    # 예시: 특정 기능 ID
        username = 1       # 예시 사용자 ID

        # QTY 배열(16개의 정수)의 총 길이 = 16 * 4 bytes = 64 bytes
        QTY_data_length = len(QTY_array) * 4
        
        # Header (4 int) + QTY Array (16 int) = 총 20개의 정수
        Length_of_data = 4 * 4 + QTY_data_length # Header(16바이트) + QTY(64바이트) = 80바이트
                
        Transaction_ID = int(Transaction_ID)
        username = int(username)
        Function_ID = int(Function_ID)
        Length_of_data = int(Length_of_data)
        
        
        # 패킷 생성 함수 호출 (Header 4개 + QTY 16개)
        success, msg = self.create_and_send_packet(
            Transaction_ID, 
            Length_of_data, 
            Function_ID, 
            username, 
            QTY_array
        )
        
        return success, msg

    # TCP 패킷 생성 및 전송 함수 (이름 변경 및 QTY 배열 포함하도록 수정)
    def create_and_send_packet(self, Transaction_ID, Length_of_data, Function_ID, username, QTY_array):
        """
        Header(4개)와 QTY 배열(16개)을 포함하여 바이너리 패킷을 만들고 서버에 전송합니다.
        """
        SERVER_IP = "192.168.0.180"
        SERVER_PORT = 3000
        
        # struct.pack의 포맷 문자열 생성:
        # 4개의 i (Header) + 16개의 i (QTY 배열)
        format_string = "<iiii" + "i" * 16 

        # 패킷 데이터: Header 값 4개와 QTY 배열의 16개 값을 합쳐서 전달
        packet = struct.pack(
            format_string,
            Transaction_ID,
            Length_of_data,
            Function_ID,
            username,
            *QTY_array  # ⭐ QTY_array의 16개 요소가 여기에 언팩되어 들어갑니다.
        )
        
        print(f"생성된 패킷 길이: {len(packet)} 바이트")

        # ⭐⭐ 핵심 수정: manager의 공유 소켓 사용 확인
        if not hasattr(self.manager, 'active_tcp_socket') or self.manager.active_tcp_socket is None:
            return False, "로그인 소켓 연결을 찾을 수 없습니다."

        try:
            # ⭐ 공유된 소켓 객체를 가져와서 사용
            tcp_socket = self.manager.active_tcp_socket

            print("데이터 전송 중...")
            tcp_socket.send(packet) # 공유 소켓으로 전송
            tcp_socket.send(b"DONE") # 공유 소켓으로 전송

            # 서버로부터 응답 받기 (최대 1024바이트)
            response = tcp_socket.recv(1024) # 공유 소켓으로 수신
            print("서버 응답:", response)

        # 기존의 send_to_server 함수를 사용하여 전송
        # 참고: send_to_server는 반환 값이 None이면 통신 오류로 간주합니다.
        # response = self.send_to_server(SERVER_IP, SERVER_PORT, packet)

            if response is not None:
                return True, "수량 업데이트 패킷 서버 전송 완료."
            else:
                return False, "수량 업데이트 패킷 서버 전송 실패 (통신 오류)."
            
        except Exception as e:
            print("TCP 통신 오류:", e)
        return False, f"수량 업데이트 패킷 서버 전송 실패 (통신 오류: {e})."


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
    
    
    # def send_to_server(self, ip, port, packet):
    #     """
    #     서버로 TCP 데이터 전송
    #     """
    #     try:
    #         # ⭐ socket.socket()을 with 문 내에서 생성
    #         with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as tcp_socket:
    #             print(f"서버({ip}:{port})에 연결 중...")
    #             # self.tcp_socket.connect((ip, port))  <- 기존 코드
    #             tcp_socket.connect((ip, port)) # ⭐ 새로운 소켓 연결
    #             print("데이터 전송 중...")
                
    #             # self.tcp_socket.send(packet) <- 기존 코드
    #             tcp_socket.send(packet) # ⭐ 새로운 소켓으로 전송
    #             # self.tcp_socket.send(b"DONE") <- 기존 코드
    #             tcp_socket.send(b"DONE") # ⭐ 새로운 소켓으로 전송
                
    #             # 서버로부터 응답 받기 (최대 1024바이트)
    #             # response = self.tcp_socket.recv(1024) <- 기존 코드
    #             response = tcp_socket.recv(1024) # ⭐ 새로운 소켓으로 수신
    #             # print("서버 응답:", response.hex())
    #             print("서버 응답:", response)
                
    #             # # ... (로그인 성공/실패 처리 로직은 동일) ...
    #             # if response == b'\x11\x00\x00\x00\x01\x00\x00\x00\x01\x00\x00\x00\x01':

    #             # # ... (로그인 성공 시) ...
    #             #     QMessageBox.information(self, "로그인 성공", "스마트 쇼핑에 오신 걸 환영합니다!")

    #             #     # self.main_frame = MainFrame(
    #             #     #     manager=self.manager,
    #             #     #     # username=username,
    #             #     #     # photo_path="",      
    #             #     #     # login_window=self
    #             #     # )
    #             #     self.manager.show_page("MainFrame")

    #             #     # self.input_pw.clear()
    #             #     # self.input_user.clear()


    #             #     # ⭐ 매장 상품 데이터 응답(2번째 응답) 수신 > 최대 1024
    #             #     # prd_response = self.tcp_socket.recv(1024) <- 기존 코드
    #             #     prd_response = tcp_socket.recv(1024) # ⭐ 새로운 소켓으로 수신
    #             #     print("상품 정보 응답:", prd_response)

    #             # # ⭐ Sttclass 에 전달
    #             #     self.manager.shared_product_data = prd_response


    #             # elif response == b'\x11\x00\x00\x00\x01\x00\x00\x00\x01\x00\x00\x00\x00' :
    #             #         QMessageBox.warning(self, "로그인 실패", "등록되지 않은 사용자입니다.")
    #             #         self.input_pw.clear()
    #             # else:
    #             #         QMessageBox.critical(self, "연결 실패", "서버 응답이 없거나 통신 오류가 발생했습니다.")

                    
    #     except Exception as e:
    #         print("TCP 통신 오류:", e)
    #         return None




if __name__ == "__main__":
    app = QApplication(sys.argv)
    myWindows = Sttclass()
    myWindows.show()
    sys.exit(app.exec())