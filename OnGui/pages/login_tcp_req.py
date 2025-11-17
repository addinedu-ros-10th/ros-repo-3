
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
from .main_frame import MainFrame

# SERVER_IP = "127.0.0.1"   # 서버 IP
# SERVER_PORT = 9500        # 서버 포트


class LoginTcpWindow(QWidget):
    def __init__(self, manager=None):
        super().__init__()
        self.manager = manager
        self.main_frame = None

        self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        
        self.initUI()

    def initUI(self):
        main_layout = QVBoxLayout()
        form_layout = QFormLayout()

        self.input_user = QLineEdit(self)
        self.input_pw = QLineEdit(self)
        self.input_pw.setEchoMode(QLineEdit.EchoMode.Password)

        form_layout.addRow("사용자 이름:", self.input_user)
        form_layout.addRow("비밀번호:", self.input_pw)

        self.btn_login = QPushButton("로그인", self)
        self.btn_login.move(530, 470)
        self.btn_login.resize(220, 50)
        self.btn_login.clicked.connect(self.login)

        # self.btn_login_admin = QPushButton("Admin 로그인", self)
        # self.btn_login_admin.move(530, 470)
        # self.btn_login_admin.resize(220, 50)
        # self.btn_login_admin.clicked.connect(self.admin_login)

        main_layout.addLayout(form_layout)
        self.setLayout(main_layout)


    # def make_update_packet(item_id, quantities, recv_flag):
    def make_update_packet(self, Transaction_ID, Length_of_data, function_id, username):
    
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

        # packet = struct.pack("<i i i i",
        #                     Transaction_ID,
        #                     Length_of_data,
        #                     Function_ID,
        #                     username
        #                     )
        #  return packet


        
        
        # ✅ 1. b'\x01' 의 뜻

        # 앞의 b 는 이 값이 바이트(byte) 타입이라는 의미
        # \x01 은 16진수 01(hex 01) 을 뜻함
        # 16진수 01 = 십진수 1

        SERVER_IP = "192.168.0.180"     # 서버 IP
        SERVER_PORT = 3000          # 서버 포트

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
            
            # with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client:
            
                # socket.socket(socket.AF_INET, socket.SOCK_STREAM)
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

                
                # b'\x11\x00\x00\x00\x01\x00\x00\x00\x01\x00\x00\x00\x00 > false
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

                    self.input_pw.clear()
                    self.input_user.clear()


                    # 서버로부터 응답 받기 (최대 1024바이트)
                    response = self.tcp_socket.recv(1024)

                    # print("서버 응답:", response.hex())
                    print("서버 응답:", response)


                elif response == "FAIL":
                    QMessageBox.warning(self, "로그인 실패", "등록되지 않은 사용자입니다.")
                    self.input_pw.clear()
                else:
                    QMessageBox.critical(self, "연결 실패", "서버 응답이 없거나 통신 오류가 발생했습니다.")

                    # return response

        except Exception as e:
                    print("TCP 통신 오류:", e)
                    
                    return None


    # def tcp_request_login(self, username):
    #     """
    #     서버에 로그인 요청을 보내고 응답을 받는다.
    #     서버로 username을 전송하면, 서버는 OK 또는 FAIL 응답을 돌려줌.
    #     """
    #     SERVER_IP = "127.0.0.1"   # 서버 IP
    #     SERVER_PORT = 9500        # 서버 포트


        # try:
        #     with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client:
        #         client.connect((SERVER_IP, SERVER_PORT))

        #         # username을 bytes로 변환
        #         username_bytes = username.encode('utf-8')
        #         length = len(username_bytes)

        #         # struct.pack("!B", 길이) → 1바이트 길이 정보
        #         header = struct.pack("!B", length)

        #         # 최종 전송 데이터 = 헤더(1바이트) + 실제 문자열 데이터
        #         send_data = header + username_bytes
        #         print("send_data : ", send_data)

        #         client.sendall(send_data)

        #         # 서버 응답 수신
        #         response = client.recv(1024).decode('utf-8').strip()

                
        #         return response
        # except Exception as e:
        #     print("TCP 통신 오류:", e)
        #     return None
        
        # try:
        #     with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client:
        #         client.connect((SERVER_IP, SERVER_PORT))
        #         # username 전송
        #         client.sendall(username.encode('utf-8'))

        #         # 서버 응답 수신
        #         response = client.recv(1024).decode('utf-8').strip()
        #         return response
        # except Exception as e:
        #     print("TCP 통신 오류:", e)
        #     return None

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
        
        # print("response : ", response)

        # if response == b'\x02\x01':
        #     # 로그인 성공
        #     QMessageBox.information(self, "로그인 성공", f"{username}님 환영합니다!")

        #     self.main_frame = MainFrame(
        #         manager=self.manager,
        #         username=username,
        #         photo_path="",      # 서버에서 따로 받을 수도 있음
        #         login_window=self
        #     )
        #     self.manager.show_page("MainFrame")

        #     self.input_pw.clear()
        #     self.input_user.clear()


        # elif response == "FAIL":
        #     QMessageBox.warning(self, "로그인 실패", "등록되지 않은 사용자입니다.")
        #     self.input_pw.clear()
        # else:
        #     QMessageBox.critical(self, "연결 실패", "서버 응답이 없거나 통신 오류가 발생했습니다.")

    # def admin_login(self):
    #     self.manager.show_page("LoginAdminWindow")


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = LoginTcpWindow()
    window.show()
    sys.exit(app.exec())
