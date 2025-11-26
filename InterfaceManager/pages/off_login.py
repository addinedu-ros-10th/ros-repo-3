import os
import sys
import socket
import struct
import serial
import threading

from PyQt6.QtWidgets import QDialog, QMessageBox
from PyQt6 import uic
from PyQt6.QtCore import QTimer
from functools import partial


class OffLoginWindow(QDialog):
    def __init__(self, manager=None):
        super().__init__()
        self.manager = manager

        # UI 파일 로드
        ui_path = os.path.join(os.path.dirname(__file__), "off_login.ui")
        uic.loadUi(ui_path, self)

        # RFID 설정
        self.serial_port = "/dev/ttyACM0"
        self.baudrate = 9600
        self.ser = None

        # TCP 설정
        self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.SERVER_IP = "192.168.0.180"
        self.SERVER_PORT = 6000

        # 중복 로그인 방지
        self.login_in_progress = False

        # RFID 스레드 시작
        self.start_serial_thread()

    # ----------------------------------------------------------
    # RFID Thread 시작
    # ----------------------------------------------------------
    def start_serial_thread(self):
        try:
            self.ser = serial.Serial(self.serial_port, self.baudrate, timeout=1)
            print("RFID 포트 연결 성공:", self.serial_port)
        except Exception as e:
            print("RFID 포트 연결 실패:", e)
            QMessageBox.critical(self, "RFID 오류", "RFID 리더기를 찾을 수 없습니다.")
            return

        th = threading.Thread(target=self.read_serial_loop, daemon=True)
        th.start()
        print("RFID 읽기 스레드 시작됨")

    # ----------------------------------------------------------
    # RFID 반복 읽기
    # ----------------------------------------------------------
    def read_serial_loop(self):
        while True:
            try:
                if self.ser.in_waiting > 0:
                    line = self.ser.readline().decode(errors="ignore").strip()

                    if "Converted UID" in line:
                        parts = line.split(":")
                        if len(parts) >= 2:
                            hex_uid = parts[1].strip()
                            print("[RFID] HEX UID 감지:", hex_uid)

                            QTimer.singleShot(
                                0, partial(self.process_hex_uid, hex_uid)
                            )

            except Exception as e:
                print("RFID 읽기 오류:", e)
                break

    # ----------------------------------------------------------
    # HEX UID → 정수 user_id 변환
    # ----------------------------------------------------------
    def process_hex_uid(self, hex_uid: str):
        if self.login_in_progress:
            print("[INFO] 로그인 처리 중 → RFID 무시")
            return

        try:
            cleaned = hex_uid.replace(" ", "")
            user_id = int(cleaned, 16)
            print("변환된 user_id =", user_id)
        except Exception as e:
            print("HEX UID 변환 실패:", e)
            return

        # =======================================================
        # ① 개발용 임시 강제 로그인 모드
        # =======================================================
        if getattr(self, "FORCE_LOCAL_LOGIN", False):
            print("[DEV] FORCE_LOCAL_LOGIN=True → 서버 무시하고 강제 로그인")
            QMessageBox.information(self, "개발 모드 로그인",
                                    f"개발자 강제 로그인 모드로 접속되었습니다.\n(user_id={user_id})")
            self.manager.RECV_user_id = user_id
            self.do_login_success()
            return

        # 서버 응답 없이도 1,2는 강제로 로그인하는 기존 임시 코드
        if user_id in (1, 2):
            print("[LOCAL] 서버 응답없이 user_id 1,2를 강제 인증")
            QMessageBox.information(self, "임시 로그인",
                                    f"임시 강제 로그인 처리되었습니다.\n(user_id={user_id})")
            self.manager.RECV_user_id = user_id
            self.do_login_success()
            return

        # =======================================================
        # ② 그 외 정상 서버 인증 절차 진행
        # =======================================================
        if self.manager:
            setattr(self.manager, "RECV_user_id", user_id)

        self.login_in_progress = True
        self.auto_login(user_id)


    # ----------------------------------------------------------
    # TCP 통신으로 cart_state_manager에 user_id 전송
    # ----------------------------------------------------------
    def auto_login(self, user_id: int):
        print(f"[TCP] 서버 인증 요청 → user_id={user_id}")

        Transaction_ID = 33
        Length_of_data = 4
        function_id = 1

        packet = struct.pack(
            "<i i i i",
            int(Transaction_ID),
            int(Length_of_data),
            int(function_id),
            int(user_id)
        )

        self.send_to_server(self.SERVER_IP, self.SERVER_PORT, packet)

    # ----------------------------------------------------------
    # 정확히 n 바이트 수신
    # ----------------------------------------------------------
    def recv_exact(self, sock: socket.socket, num_bytes: int) -> bytes:
        data = b""
        while len(data) < num_bytes:
            chunk = sock.recv(num_bytes - len(data))
            if not chunk:
                break
            data += chunk
        return data

    # ----------------------------------------------------------
    # TCP 응답 처리
    # ----------------------------------------------------------
    def send_to_server(self, ip, port, packet: bytes):
        try:
            print(f"[TCP] 서버({ip}:{port}) 연결 중...")
            self.tcp_socket.connect((ip, port))

            print("[TCP] 데이터 전송 중…")
            self.tcp_socket.sendall(packet)
            self.tcp_socket.sendall(b"DONE")

            # 서버 응답: 13바이트 수신
            response = self.recv_exact(self.tcp_socket, 13)

            if len(response) == 13:
                recv_user_id = struct.unpack("<iii?", response)[3]
                print(f"[TCP] 서버 인증 결과 user_id={recv_user_id}")

                if recv_user_id:
                    self.do_login_success()
                else:
                    QMessageBox.warning(self, "로그인 실패", "등록되지 않은 사용자입니다.")
            else:
                print("[TCP Error] 응답 길이 오류")
                QMessageBox.critical(self, "오류", "서버 응답 데이터 오류.")

        except Exception as e:
            print("[TCP] 통신 오류:", e)
            QMessageBox.warning(self, "오프라인 모드", "서버 연결 실패 → 오프라인 인증으로 진행합니다.")

            # fallback: 1 or 2일 경우 로컬 로그인 처리
            user_id = getattr(self.manager, "RECV_user_id", None)
            if user_id in (1, 2):
                print("[LOCAL] 서버 실패 → 로컬 강제 로그인")
                self.do_login_success()
            else:
                QMessageBox.warning(self, "로그인 실패", "오프라인 인증 실패.")

        finally:
            self.login_in_progress = False
            try:
                self.tcp_socket.close()
                self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            except:
                pass

    # ----------------------------------------------------------
    # 로그인 성공 처리
    # ----------------------------------------------------------
    def do_login_success(self):
        QMessageBox.information(self, "로그인 성공", "스마트 쇼핑에 오신 걸 환영합니다!")
        self.manager.show_page("MainFrame")


# 테스트 실행용
if __name__ == "__main__":
    from PyQt6.QtWidgets import QApplication

    app = QApplication(sys.argv)

    class DummyManager:
        def show_page(self, name):
            print(f"[Manager] 화면 이동 → {name}")

    w = OffLoginWindow(manager=DummyManager())
    w.show()

    sys.exit(app.exec())
