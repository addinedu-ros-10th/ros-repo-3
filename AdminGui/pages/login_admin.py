

import os
import sys
import socket
import struct
from PyQt6.QtWidgets import (
    QApplication, QWidget, QLabel, QLineEdit, QPushButton,
    QMessageBox, QVBoxLayout, QFormLayout
)
from PyQt6.QtCore import Qt

# MainFrame 가져오기
from .admin_manager import PoseSubscriber


class LoginAdminWindow(QWidget):
    def __init__(self, manager=None):
        super().__init__()
        self.manager = manager
        self.main_frame = None
        self.initUI()

    def initUI(self):
        main_layout = QVBoxLayout()
        form_layout = QFormLayout()

        self.input_user = QLineEdit(self)
        self.input_pw = QLineEdit(self)
        self.input_pw.setEchoMode(QLineEdit.EchoMode.Password)

        form_layout.addRow("사용자 이름:", self.input_user)
        form_layout.addRow("비밀번호:", self.input_pw)

        # self.btn_login = QPushButton("로그인", self)
        # self.btn_login.move(530, 470)
        # self.btn_login.resize(220, 50)
        # self.btn_login.clicked.connect(self.login)

        self.btn_login_admin = QPushButton("Admin 로그인", self)
        self.btn_login_admin.move(530, 470)
        self.btn_login_admin.resize(220, 50)
        self.btn_login_admin.clicked.connect(self.admin_login)

        main_layout.addLayout(form_layout)
        self.setLayout(main_layout)


    # def make_update_packet(item_id, quantities, recv_flag):
    def make_update_packet(self, username, function_id):
    
        """
        매장 물품 정보 업데이트 패킷 생성
        item_id   : 물품 ID (uint16)
        quantities: 물품 수량 16개 (uint8 리스트)
        recv_flag : 0x00(포장), 0x01(배송)
        """
        # function_id = 0x01

        # if len(quantities) != 16:
        #     raise ValueError("QTY는 16개여야 합니다.")
        # if not (0 <= item_id <= 0xFFFF):
        #     raise ValueError("item_id는 0~65535 범위여야 합니다.")
        # if recv_flag not in (0x00, 0x01):
        #     raise ValueError("recv_flag는 0x00 또는 0x01만 가능.")

        # 19바이트 패킷 구성
        # packet = struct.pack("<B H 16B B",
        #                     function_id,
        #                     item_id,
        #                     *quantities,
        #                     recv_flag)
        #     return packet

        
        
        # ✅ 1. b'\x01' 의 뜻

        # 앞의 b 는 이 값이 바이트(byte) 타입이라는 의미
        # \x01 은 16진수 01(hex 01) 을 뜻함
        # 16진수 01 = 십진수 1

        SERVER_IP = "127.0.0.1"   # 서버 IP
        SERVER_PORT = 9500        # 서버 포트

        username = int(username)
        function_id = int(function_id)

        packet = struct.pack("<B B",
                            username,
                            function_id,
        )
        
        self.send_to_server(SERVER_IP, SERVER_PORT, packet)

        return packet
    
        

    def send_to_server(self, ip, port, packet):
        """
        서버로 TCP 데이터 전송
        """

        try:
            
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client:
                print(f"서버({ip}:{port})에 연결 중...")
                client.connect((ip, port))

                print("데이터 전송 중...")
                client.sendall(packet)

                # 서버로부터 응답 받기 (최대 1024바이트)
                response = client.recv(1024)
                print("서버 응답:", response.hex())

        except Exception as e:
                print("TCP 통신 오류:", e)
            
                return None


  
    def admin_login(self):
        username = self.input_user.text().strip()
        function_id = 1

        if not username:
            QMessageBox.warning(self, "경고", "사용자 이름을 입력하세요.")
            return

        # TCP 서버에 로그인 요청
        response = self.make_update_packet(username, function_id)
        
        print("response : ", response)

        if response == b'\x01\x01':
            # 로그인 성공
            QMessageBox.information(self, "로그인 성공", f"{username}님 환영합니다!")

            self.main_frame = PoseSubscriber(
                manager=self.manager,         # ✅ manager 전달
                # username=username, 
                # photo_path=photo_path,
                # login_window=self
            )
            self.manager.show_page("PoseSubscriber")

            self.input_pw.clear()
            self.input_user.clear()


        elif response == "FAIL":
            QMessageBox.warning(self, "로그인 실패", "등록되지 않은 사용자입니다.")
            self.input_pw.clear()
        else:
            QMessageBox.critical(self, "연결 실패", "서버 응답이 없거나 통신 오류가 발생했습니다.")



if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = LoginAdminWindow()
    window.show()
    sys.exit(app.exec())
