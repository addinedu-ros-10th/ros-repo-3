

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

        self.btn_login_admin = QPushButton("Admin 로그인", self)
        self.btn_login_admin.move(530, 470)
        self.btn_login_admin.resize(220, 50)
        self.btn_login_admin.clicked.connect(self.admin_login)

        main_layout.addLayout(form_layout)
        self.setLayout(main_layout)

    def make_update_packet(self, Transaction_ID, Length_of_data, function_id, username):
    
        
        SERVER_IP = "192.168.0.180"     # 서버 IP
        SERVER_PORT = 3001              # 서버 포트


        username = int(username)
        function_id = int(function_id)
        Length_of_data = int(Length_of_data)

        packet = struct.pack("<iiii",
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
                        
                print(f"서버({ip}:{port})에 연결 중...")
                self.tcp_socket.connect((ip, port))

                print("데이터 전송 중...")
                self.tcp_socket.send(packet)
                self.tcp_socket.send(b"DONE")

                # 서버로부터 응답 받기 (최대 1024바이트)
                response = self.tcp_socket.recv(1024)
                print("서버 응답:", response)


                if response == b'\x11\x00\x00\x00\x01\x00\x00\x00\x01\x00\x00\x00\x01':
            
                # 로그인 성공
                    QMessageBox.information(self, "로그인 성공", " 관리자 시스템에 오신걸 환영합니다!")

                    self.main_frame = PoseSubscriber(
                            manager=self.manager,         # ✅ manager 전달
                            # username=username, 
                            # photo_path=photo_path,
                            # login_window=self
                        )
                        
                    self.manager.show_page("PoseSubscriber")

                    self.input_pw.clear()
                    self.input_user.clear()


                elif response == b"":
                    QMessageBox.warning(self, "로그인 실패", "등록되지 않은 사용자입니다.")
                    self.input_pw.clear()
                else:
                    QMessageBox.critical(self, "연결 실패", "서버 응답이 없거나 통신 오류가 발생했습니다.")

        except Exception as e:
                    print("TCP 통신 오류:", e)
                    try: self.tcp_socket.close() 
                    except: pass
                    return None
                


    def admin_login(self):
        username = self.input_user.text().strip()
        function_id = 1
        Transaction_ID = 1
        Length_of_data = 4

        if not username:
            QMessageBox.warning(self, "경고", "사용자 이름을 입력하세요.")
            return

        # TCP 서버에 로그인 요청
        response = self.make_update_packet(Transaction_ID, Length_of_data, function_id, username)
        

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = LoginAdminWindow()
    window.show()
    sys.exit(app.exec())
