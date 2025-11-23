

import os
import sys
import struct
from PyQt6.QtWidgets import QSizePolicy,QTableWidget, QApplication, QMainWindow, QLabel, QLineEdit, QPushButton, QMessageBox, QWidget, QVBoxLayout, QFormLayout, QTableWidgetItem
from PyQt6.QtCore import Qt, QEvent



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
        self.main_frame = None
        self.initUI()

        # 로그인 단계에서 받아온 상품 데이터를 자동 로딩
        if hasattr(self.manager, "shared_product_data"):
            self.load_product_table(self.manager.shared_product_data)


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
        self.name_label.move(300, 45)      
        self.name_label.resize(360, 40)  

        # --- 2. 상품명 입력 창 ---
        self.input_product = QLineEdit(self)
        
        # 비밀번호 숨김처리 기능
        # self.input_product.setEchoMode(QLineEdit.EchoMode.Password)

        # ❗ 2. 수치로 위치와 크기 지정
        self.input_product.move(220, 460)      
        self.input_product.resize(450, 40)   
        

        # 3. 상품명 저장 버튼
        self.product_save_btn_ = QPushButton("상품명 저장", self)
        self.product_save_btn_.clicked.connect(self.on_add_button_clicked)

        # ❗ 3. 상품명 저장 버튼 수치로 위치와 크기 지정
        self.product_save_btn_.move(220, 510)      
        self.product_save_btn_.resize(450, 40)  

        # ✅ QTableWidget 생성
        self.product_info = QTableWidget()
        self.product_info.setRowCount(0)  # 초기 행 개수
        self.product_info.setColumnCount(3)  # 3개의 컬럼 생성
        self.product_info.setHorizontalHeaderLabels(["Name", "QTY", "RECV"])  # 헤더 이름 설정

        # ✅ 테이블 크기 조정
        self.product_info.resizeColumnsToContents()
        self.product_info.setColumnWidth(0, 145)
        self.product_info.setColumnWidth(1, 145)
        self.product_info.setColumnWidth(2, 145)

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
        self.product_info.setMaximumSize(450, 500)  # 최대 크기 제한
        self.product_info.setSizePolicy(
            QSizePolicy.Policy.Expanding,
            QSizePolicy.Policy.Expanding
        )

         # ✅ 테이블 정렬 (화면 상단 / 중앙 / 하단 등)
        # main_layout.addWidget(self.product_info, alignment=Qt.AlignmentFlag.AlignTop)   # 상단
        main_layout.addWidget(self.product_info, alignment=Qt.AlignmentFlag.AlignCenter) # 중앙
        # main_layout.addWidget(self.product_info, alignment=Qt.AlignmentFlag.AlignBottom) # 하단

        # 왼쪽, 위, 오른쪽, 아래 여백(px)
        main_layout.setContentsMargins(220, 100, 400, 270)  
        main_layout.setSpacing(20)  # 위젯 간 간격


        #========================================================== 


        # back_button 
        self.back_button = QPushButton("뒤로가기", self)
        self.back_button.clicked.connect(self.backMove)

        # ❗ back_button > 수치로 위치와 크기 지정
        self.back_button.move(30, 50)   # X=150, Y=220
        self.back_button.resize(150, 40)

         # --- 4. 구매희망 리스트 라벨 ---
        self.name_label1 = QLabel("구매희망 쇼핑리스트.!", central_widget) 
        font = self.name_label1.font()
        font.setPointSize(16)
        self.name_label1.setFont(font)

        # ❗ 4. 구매희망 리스트 라벨 수치로 위치와 크기 지정
        self.name_label1.move(800, 40)      
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
        self.product_save_btn.clicked.connect(self.send_wishlist_to_server)

        # ❗ 7. 구매희망 리스트 작성 저장 수치로 위치와 크기 지정
        self.product_save_btn.move(700, 510)      
        self.product_save_btn.resize(360, 40) 

 
        # # 8. 구매희망 쇼핑 리스트 이동 버튼
        # self.product_move_btn = QPushButton("쇼핑리스트 보러가기", self)
        # self.product_move_btn.clicked.connect(self.AutoShoppingClass)

        # # ❗ 8. 구매희망 쇼핑 리스트 이동 수치로 위치와 크기 지정
        # self.product_move_btn.move(700, 560)      
        # self.product_move_btn.resize(360, 40)   


        # ✅ QWidget은 setLayout을 바로 사용합니다.

        main_layout.addWidget(self.product_info)
        self.setLayout(main_layout)
        
        
    def reset(self):
        self.product_label.clear()

        
    def AutoShoppingClass(self):
        self.manager.show_page("AutoShoppingClass")

    def backMove(self):
        self.manager.show_page("Sttclass")
        

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

    def clear_socket_buffer(self, tcp_socket):
        """
        소켓 버퍼에 남아있는 데이터를 강제로 읽어 비우는 함수.
        """
        import time
        tcp_socket.settimeout(0.01) # 짧은 타임아웃 설정
        
        # 반복적으로 recv를 호출하여 버퍼를 비웁니다.
        while True:
            try:
                # 최대 1024바이트를 읽습니다.
                data = tcp_socket.recv(1024) 
                if not data:
                    break # 읽을 데이터가 없으면 루프 종료
                print(f"잔여 버퍼 데이터 발견 및 제거: {data}")
            except TimeoutError:
                # 타임아웃이 발생하면 버퍼가 비워진 것으로 간주하고 종료
                break
            except Exception as e:
                print(f"버퍼 비우는 중 오류 발생: {e}")
                break
        
        tcp_socket.settimeout(None) # 원래대로 블로킹 모드 복원 (필요하다면)

    def send_wishlist_to_server(self):
        """
        구매희망 리스트(화면 오른쪽 product_label)를 읽어서
        서버로 4바이트 int 들을 바이너리로 변환해 전송하는 함수
        ※ ID는 보내지 않고, QTY(=사용자가 입력한 순번별 수량)만 전송
        """
        fixed_names = [
            "soju", "beer", "ketchap", "mayonaise",
            "Snack_Org", "Snack_Ylw", "sushi", "pizza",
            "frypan", "pot", "strawberry", "watermelon",
            "grape", "deco_tree", "deco_santa", "deco_ring"
        ]

        # 1) 구매리스트에서 줄 단위 텍스트 가져오기
        raw_text = self.product_label.text().strip()

        if not raw_text:
            QMessageBox.warning(self, "오류", "저장할 구매 희망 리스트가 없습니다!")
            return

        # 2) "1. 사과" → "사과" 형태로 변환
        # qty_list = []
        qty_list = [0] * 16

        # 3) 구매 희망 상품 목록 추출 및 맵핑
        for line in raw_text.split("\n"):
            # 번호 제거 → "1. 라면" → "라면"
            if ". " in line:
                item_name = line.split(". ", 1)[1].strip()
            else:
                item_name = line.strip()

            # 상품명을 fixed_names에서 찾아서 QTY 리스트에 1 입력
            try:
                # 대소문자 구분을 위해 모두 소문자로 변환하여 비교 (필요 시)
                # item_name_lower = item_name.lower()
                
                # fixed_names는 이미 소문자 형태이므로, 입력도 소문자 비교가 안전합니다.
                idx = fixed_names.index(item_name)
                
                # 해당 위치에 1을 입력 (수량 = 1)
                qty_list[idx] = 1 
                
            except ValueError:
                # fixed_names에 없는 상품은 무시하거나 경고를 표시할 수 있습니다.
                print(f"경고: 알 수 없는 상품명 '{item_name}'은 전송 목록에서 제외됩니다.")

        
        # 예) [1,1,1] → b'\x01\x00\x00\x00\x01\x00\x00\x00\x01\x00\x00\x00'

        # struct.pack의 포맷 문자열 생성:
        Transaction_ID = 1 
        Function_ID = 2    
        username = 1
        Length_of_data = 69   #69
        
        Transaction_ID = int(Transaction_ID)
        username = int(username)
        Function_ID = int(Function_ID)
        Length_of_data = int(Length_of_data)
        recv = True

        # 4개의 i (Header) + 16개의 i (QTY 배열)
        # binary_data = struct.pack("<" + "i" * len(qty_list), *qty_list)
        format_string = "<iiii" + "i" * 16 + "?" # bool 타입은 ?로 표기 

        # 패킷 데이터: Header 값 4개와 QTY 배열의 
        packet = struct.pack(
            format_string,
            Transaction_ID,
            Length_of_data,
            Function_ID,
            username,
            *qty_list,
            recv   
        )
        # packet = struct.pack(
        #     format_string,
        #     Transaction_ID,
        #     Length_of_data,
        #     Function_ID,
        #     username,
        #     *[i for i in range(16)],   
        #     recv                        
        # )

        # ⭐⭐ 핵심 수정: manager의 공유 소켓 사용 확인
        if not hasattr(self.manager, 'active_tcp_socket') or self.manager.active_tcp_socket is None:
            return False, "로그인 소켓 연결을 찾을 수 없습니다."

        try:
            # ⭐ 공유된 소켓 객체를 가져와서 사용
            tcp_socket = self.manager.active_tcp_socket

            # ⭐ 추가: 전송 전에 소켓 버퍼를 강제 초기화
            self.clear_socket_buffer(tcp_socket)

            print("데이터 전송 중...")
            tcp_socket.send(packet)      # 공유 소켓으로 전송
            tcp_socket.send(b"DONE")   # 공유 소켓으로 전송

            # 서버로부터 응답 받기 (최대 1024바이트)
            response = tcp_socket.recv(1024)      # 공유 소켓으로 수신
            
            print("서버 응답:", response)
        
            if response is not None:
                return True, "수량 업데이트 패킷 서버 전송 완료."
            else:
                return False, "수량 업데이트 패킷 서버 전송 실패 (통신 오류)."
            
        except Exception as e:
            print("TCP 통신 오류:", e)
            try: self.tcp_socket.close() 
            except: pass
        return False, f"수량 업데이트 패킷 서버 전송 실패 (통신 오류: {e})."



if __name__ == "__main__":
    
    app = QApplication(sys.argv)
    window = PC_BrowserClass()
    window.show()
    sys.exit(app.exec())
    