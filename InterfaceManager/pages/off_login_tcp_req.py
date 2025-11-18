
import os
import sys
import socket
import struct
from PyQt6.QtGui import QPixmap, QColor
from PyQt6.QtWidgets import (
    QApplication, QWidget, QLabel, QLineEdit, QPushButton,
    QMessageBox, QVBoxLayout, QFormLayout, QMainWindow
)
from PyQt6.QtCore import Qt

# MainFrame 가져오기
from .off_main_frame import MainFrame

# SERVER_IP = "127.0.0.1"   # 서버 IP
# SERVER_PORT = 9500        # 서버 포트


class LoginTcpWindow(QMainWindow):
    def __init__(self, manager=None):
        super().__init__()
        self.manager = manager
        self.main_frame = None

        self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        
        self.initUI()

    def initUI(self):
        # main_layout = QVBoxLayout()
        # form_layout = QFormLayout()

        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        # --- 1. 환영 메시지 라벨 ---
        # 부모 위젯을 central_widget으로 지정합니다.
        self.name_label = QLabel(f"고객님 카드를 태그해주세요.!!", central_widget) 
        font = self.name_label.font()
        font.setPointSize(16)
        self.name_label.setFont(font)
        
        # ❗ 수치로 위치와 크기 지정
        self.name_label.move(500, 50)       # X=20, Y=20 위치
        self.name_label.resize(360, 40)     # 너비 360, 높이 40

        # --- 2. 사용자 사진 라벨 ---

        # 사진 파일 경로
        
        self.photo_label = QLabel(central_widget) # 부모 지정
        self.photo_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        
        img_path = "./pages/rfid_tag.png"
        
        pixmap = QPixmap(img_path)
        if pixmap.isNull():
            pixmap = QPixmap(250,250)
            pixmap.fill(QColor("gray"))
        
        self.photo_label.setPixmap(pixmap.scaled(250, 250, Qt.AspectRatioMode.KeepAspectRatio))

        # ❗ 수치로 위치와 크기 지정 (창의 중앙 근처)
        self.photo_label.move(400, 20)    # X=150, Y=80
        self.photo_label.resize(500, 500) # 너비 100, 높이 100

        self.btn_login = QPushButton("로그인", self)
        self.btn_login.move(530, 470)
        self.btn_login.resize(220, 50)
        # self.btn_login.clicked.connect(self.login)


    def make_update_packet(self, Transaction_ID, Length_of_data, function_id, username):


        SERVER_IP = "192.168.0.180"     # 서버 IP
        # SERVER_IP = "0.0.0.0"         # 서버 IP
        SERVER_PORT = 3000              # 서버 포트

        username = int(username)
        function_id = int(function_id)
        Length_of_data = int(Length_of_data)

        packet = struct.pack("<i i i i",
                            Transaction_ID,
                            Length_of_data,
                            function_id,
                            username
                            )
        
        self.send_to_server(SERVER_IP, SERVER_PORT, packet)

        return packet
    
        

    def send_to_server(self, ip, port, packet):
        """
        서버로 TCP 데이터 전송
        """
        
        try:
                
                print(f"서버({ip}:{port})에 연결 중...")
                self.tcp_socket.connect((ip, port))

                print("데이터 전송 중...")
                # client.sendall(packet)
                self.tcp_socket.send(packet)
                self.tcp_socket.send(b"DONE")

            
                # 서버로부터 응답 받기 (최대 1024바이트)
                response = self.tcp_socket.recv(1024)

                # print("서버 응답:", response.hex())
                print("서버 응답:", response)

    
                if response == b'\x11\x00\x00\x00\x01\x00\x00\x00\x01\x00\x00\x00\x01':
                    # 로그인 성공
                    # QMessageBox.information(self, "로그인 성공", f"{username}님 환영합니다!")
                    QMessageBox.information(self, "로그인 성공", "스마트 쇼핑에 오신 걸 환영합니다!")

                    self.main_frame = MainFrame(
                        manager=self.manager,
                        # username=username,
                        # photo_path="",      
                        # login_window=self
                    )
                    self.manager.show_page("MainFrame")


                    print("서버 응답:", response)

                    # ⭐ 매장 상품 데이터 응답(2번째 응답) 수신 > 최대 1024
                    # prd_response = self.tcp_socket.recv(1024)
                    # print("상품 정보 응답:", prd_response)

                    # ⭐ Sttclass 에 전달
                    # self.manager.shared_product_data = prd_response


                elif response == b'\x11\x00\x00\x00\x01\x00\x00\x00\x01\x00\x00\x00\x00' :
                        QMessageBox.warning(self, "로그인 실패", "등록되지 않은 사용자입니다.")
                        # self.input_pw.clear()
                else:
                        QMessageBox.critical(self, "연결 실패", "서버 응답이 없거나 통신 오류가 발생했습니다.")



        except Exception as e:
                        print("TCP 통신 오류:", e)
                        
                        return None
            
    # def __del__(self):
    #     self.tcp_socket.close()

    def close_connection(self):
        try:
            self.tcp_socket.close()
        except:
            pass


    # -------------------------------
    # 로그인 버튼 클릭 시 동작
    # -------------------------------
    def login(self):
        username = self.input_user.text().strip()
        function_id = 1
        Transaction_ID = 1
        Length_of_data = 4

        if not username:
            QMessageBox.warning(self, "경고", "사용자 이름을 입력하세요.")
            return
        

        # packet = struct.pack("<i i i i",
        #                     Transaction_ID,
        #                     Length_of_data,
        #                     Function_ID,
    #                           username                

        # TCP 서버에 로그인 요청
        response = self.make_update_packet(Transaction_ID, Length_of_data, function_id, username)
        


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = LoginTcpWindow()
    window.show()
    sys.exit(app.exec())
