import os
import sys
import socket
import struct
from PyQt6.QtWidgets import (
    QApplication, QDialog, QMessageBox, QPushButton, QTextEdit
)
from PyQt6 import uic


class AdminLoginWindow(QDialog):
    #SERVER_IP = "192.168.0.180"
    SERVER_IP = "127.0.0.1"
    SERVER_PORT = 7001

    def __init__(self, manager=None):
        super().__init__()
        self.manager = manager

        # UI 로딩
        ui_path = os.path.join(os.path.dirname(__file__), "admin_login.ui")
        uic.loadUi(ui_path, self)

        # UI 객체 가져오기
        self.input_id: QTextEdit = self.findChild(QTextEdit, "textEdit_ID")
        self.input_pw: QTextEdit = self.findChild(QTextEdit, "textEdit_PW")
        self.btn_login: QPushButton = self.findChild(QPushButton, "pushButton_Login")

        # 시그널 연결
        if self.btn_login:
            self.btn_login.clicked.connect(self.admin_login)

    # ---------------------------------------------------------
    # 로그인 패킷 생성
    # ---------------------------------------------------------
    def make_login_packet(self, user_id: int) -> bytes:
        """
        Function ID 1 : 관리자 로그인 (ID 1개 = 4byte)
        [Transaction_ID(int), Length_of_data(int), Function_ID(int), ID(int)]
        """
        Transaction_ID = 1
        Length_of_data = 4
        Function_ID = 1

        return struct.pack(
            "<i i i i",
            Transaction_ID,
            Length_of_data,
            Function_ID,
            user_id
        )

    # ---------------------------------------------------------
    # 서버로 패킷 전송 (timeout 포함)
    # ---------------------------------------------------------
    def send_to_server(self, packet: bytes):
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
                sock.settimeout(2.0)
                sock.connect((self.SERVER_IP, self.SERVER_PORT))

                sock.sendall(packet)
                sock.sendall(b"DONE")

                # 🔥 로그인 응답은 16바이트이므로 정확히 16바이트 읽기
                response = self.recv_exact(sock, 16)
                return response

        except Exception as e:
            print("[AdminLogin] TCP 오류:", e)
            return None


    def recv_exact(self, sock, size):
        """size 바이트가 모두 수신될 때까지 반복해서 읽기"""
        data = b""
        while len(data) < size:
            chunk = sock.recv(size - len(data))
            if not chunk:
                return None
            data += chunk
        return data

    # ---------------------------------------------------------
    # 로그인 처리
    # ---------------------------------------------------------
    def admin_login(self):
        user_text = self.input_id.toPlainText().strip()
        #pw_text = self.input_pw.toPlainText().strip()  # PW는 UI용

        if not user_text:
            QMessageBox.warning(self, "오류", "ID를 입력하세요.")
            return

        # -----------------------
        # 오프라인 로그인
        # -----------------------
        if user_text == "local":
            QMessageBox.information(self, "로그인", "서버 없이 관리자 접속!")
            if self.manager is not None:
                self.manager.show_page("AdminManage")
            return

        if not user_text.isdigit():
            QMessageBox.warning(self, "오류", "ID는 숫자만 입력 가능합니다.")
            return

        user_id = int(user_text)

        # 패킷 생성 + 전송
        packet = self.make_login_packet(user_id)
        response = self.send_to_server(packet)

        if response is None:
            QMessageBox.critical(self, "연결 오류", "서버 응답이 없습니다.")
            return

        # 서버 로그인 성공 패턴
        #success_pattern = b'\x12\x00\x00\x00\x01\x00\x00\x00\x65\x00\x00\x00\x01'
        success_pattern = (b'\x01\x00\x00\x00'
                            b'\x04\x00\x00\x00'
                            b'\x65\x00\x00\x00'
                            b'\x01\x00\x00\x00')
        if response == success_pattern:
            QMessageBox.information(self, "성공", "관리자 로그인 성공!")
            self.manager.show_page("AdminManage")
        else:
            QMessageBox.warning(self, "실패", "로그인 실패. ID를 확인하세요.")


# 단독 실행용
if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = AdminLoginWindow()
    window.show()
    sys.exit(app.exec())
