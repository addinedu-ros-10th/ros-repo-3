import os
import sys
import socket
import struct
import serial
import threading

from PyQt6.QtGui import QPixmap, QColor
from PyQt6.QtWidgets import (
    QApplication, QWidget, QLabel, QPushButton,
    QMessageBox, QVBoxLayout, QMainWindow
)
from PyQt6.QtCore import Qt, QTimer

from .off_main_frame import MainFrame
from functools import partial


class LoginTcpWindow(QMainWindow):
    def __init__(self, manager=None):
        super().__init__()
        self.manager = manager
        self.main_frame = None

        # TCP 소켓 (cart_state_manager와 통신)
        self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        # RFID Serial
        self.serial_port = "/dev/ttyACM0"
        self.baudrate = 9600
        self.ser = None

        # 중복 로그인 방지 플래그
        self.login_in_progress = False

        self.initUI()
        self.start_serial_thread()


    def initUI(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        self.name_label = QLabel("고객님 카드를 태그해주세요.!!", central_widget)
        font = self.name_label.font()
        font.setPointSize(16)
        self.name_label.setFont(font)
        self.name_label.move(500, 50)
        self.name_label.resize(360, 40)

        self.photo_label = QLabel(central_widget)
        self.photo_label.setAlignment(Qt.AlignmentFlag.AlignCenter)

        img_path = "./pages/rfid_tag.png"
        pixmap = QPixmap(img_path)
        if pixmap.isNull():
            pixmap = QPixmap(250, 250)
            pixmap.fill(QColor("gray"))
        
        self.photo_label.setPixmap(
            pixmap.scaled(250, 250, Qt.AspectRatioMode.KeepAspectRatio)
        )
        self.photo_label.move(400, 20)
        self.photo_label.resize(500, 500)

        self.btn_login = QPushButton("로그인", self)
        self.btn_login.move(530, 470)
        self.btn_login.resize(220, 50)
        # 필요하면 수동 로그인 기능도 여기서 연결 가능


    # --------------------------------------
    # RFID Thread 시작
    # --------------------------------------
    def start_serial_thread(self):
        try:
            self.ser = serial.Serial(self.serial_port, self.baudrate, timeout=1)
            print("RFID 포트 연결 성공:", self.serial_port)
        except Exception as e:
            print("RFID 포트 연결 실패:", e)
            return

        thread = threading.Thread(target=self.read_serial_loop, daemon=True)
        thread.start()
        print("RFID 읽기 스레드 시작됨")


    # --------------------------------------
    # RFID 시리얼 루프 (아두이노 출력 읽기)
    # --------------------------------------
    def read_serial_loop(self):
        while True:
            try:
                if self.ser.in_waiting > 0:
                    line = self.ser.readline().decode(errors="ignore").strip()

                    # 예: "Converted UID (HEX): 00 00 00 01"
                    if "Converted UID" in line:
                        parts = line.split(":")
                        if len(parts) >= 2:
                            hex_uid = parts[1].strip()
                            print("Converted HEX UID 감지됨 →", hex_uid)

                            # GUI 스레드에서 처리
                            QTimer.singleShot(0, partial(self.process_converted_hex, hex_uid))

            except Exception as e:
                print("RFID 읽기 오류:", e)
                break


    # --------------------------------------
    # "00 00 00 01" → user_id 정수 변환
    # --------------------------------------
    def process_converted_hex(self, hex_uid: str):
        # 로그인 중이면 중복 요청 방지
        if self.login_in_progress:
            print("[INFO] 이미 로그인 처리 중입니다. RFID 입력시.")
            return

        try:
            cleaned = (
                hex_uid.replace(" ", "")
                       .replace("\r", "")
                       .replace("\n", "")
            )  # "00000001"
            user_id = int(cleaned, 16)
            print("Converted HEX → user_id =", user_id)
        except Exception as e:
            print("HEX UID 변환 실패:", hex_uid, e)
            return
        
        # 강제 로그인 조건
        if user_id in (1, 2):
            print("[LOCAL] 서버 응답 없이 강제 로그인 처리")
            self.do_login_success()
            return

        # manager에도 저장 (원하면 다른 페이지에서 참고 가능)
        if self.manager is not None:
            setattr(self.manager, "RECV_user_id", user_id)
            print(f"[MANAGER] RECV_user_id = {user_id}")

        # cart_state_manager로 송신 및 자동 로그인 시도
        self.auto_login(user_id)


    # --------------------------------------
    # cart_state_manager에 user_id 전송
    # --------------------------------------
    def auto_login(self, user_id: int):
        print(f"RFID 자동 로그인 실행 → user_id={user_id}")

        # 중복 로그인 방지 플래그
        self.login_in_progress = True

        # CartStateManager 프로토콜
        Transaction_ID = 33     # cart_state_manager ID
        Length_of_data = 4      # user_id int32 = 4바이트
        function_id = 1         # 예: 로그인/유저확인 기능 ID

        self.make_update_packet(Transaction_ID, Length_of_data, function_id, user_id)


    # --------------------------------------
    # TCP 패킷 생성 및 송신
    # --------------------------------------
    def make_update_packet(self, Transaction_ID, Length_of_data, function_id, user_id):

        SERVER_IP = "192.168.0.180"  # cart_state_manager IP
        SERVER_PORT = 6000           # cart_state_manager Port

        packet = struct.pack(
            "<i i i i",
            int(Transaction_ID),
            int(Length_of_data),
            int(function_id),
            int(user_id)
        )
        
        self.send_to_server(SERVER_IP, SERVER_PORT, packet)
        return packet


    # --------------------------------------
    # 정확히 n바이트 수신하는 헬퍼 함수
    # --------------------------------------
    def recv_exact(self, sock: socket.socket, num_bytes: int) -> bytes:
        data = b""
        while len(data) < num_bytes:
            chunk = sock.recv(num_bytes - len(data))
            if not chunk:
                break
            data += chunk
        return data


    # --------------------------------------
    # cart_state_manager와 통신 및 응답 처리
    # --------------------------------------
    def send_to_server(self, ip, port, packet: bytes):
        try:
            print(f"서버({ip}:{port}) 연결 중...")
            self.tcp_socket.connect((ip, port))

            print("데이터 전송 중...")
            self.tcp_socket.sendall(packet)
            self.tcp_socket.sendall(b"DONE")

            # ✅ cart_state_manager가 int32 user_id(1 또는 2)를 회신한다고 가정
            response = self.recv_exact(self.tcp_socket, 13)

            if len(response) == 13:
                recv_user_id = struct.unpack("<iii?", response)[3]
                print(f"[SERVER] 수신 user_id = {recv_user_id}")

                if (recv_user_id):
                    print("서버 인증 성공 → 자동 로그인 수행")
                    self.do_login_success()
                # elif recv_user_id in (0):
                else:
                    print("서버에서 미등록 user_id 수신:", recv_user_id)
                    QMessageBox.warning(self, "로그인 실패", "등록되지 않은 사용자입니다.")
            else:
                print("서버 응답 길이 오류:", response)
                QMessageBox.critical(self, "오류", "서버 응답 데이터가 올바르지 않습니다.")

        except Exception as e:
            print("TCP 통신 오류:", e)
            QMessageBox.critical(self, "연결 실패", f"TCP 통신 오류: {e}")

        finally:
            # 로그인 처리 완료 → 락 해제
            self.login_in_progress = False

            try:
                self.tcp_socket.close()
                self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            except:
                pass


    # --------------------------------------
    # 로그인 성공 시 화면 전환
    # --------------------------------------
    def do_login_success(self):
        QMessageBox.information(self, "로그인 성공", "스마트 쇼핑에 오신 걸 환영합니다!")
        
        # 이미 MainFrame은 main.py에서 등록되어 있음 → 새로 만들 필요 없음
        self.manager.show_page("MainFrame")



if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = LoginTcpWindow()
    window.show()
    sys.exit(app.exec())
