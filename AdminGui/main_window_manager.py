# main_window_manager.py
import socket
import struct
import threading
import time

from PyQt6.QtWidgets import QMainWindow, QStackedWidget
from PyQt6.QtCore import pyqtSignal

class MainWindowManager(QMainWindow):
    """
    - 페이지 전환 관리
    - push_server(3002)에서 보내는 FID102/103 패킷을 수신하여
      AdminManageWindow 로 전달
    """

    packet_102 = pyqtSignal(bytes)     # header + body
    packet_103 = pyqtSignal(bytes)     # body only (32 bytes)

    SERVER_IP = "127.0.0.1"
    #SERVER_IP = "192.168.0.180"
    PUSH_PORT = 7002

    def __init__(self):
        super().__init__()

        self.setWindowTitle("스마트 쇼핑카트 - Admin")
        self.resize(1280, 720)

        self.stacked = QStackedWidget()
        self.setCentralWidget(self.stacked)

        self.pages = {}

        # 시그널 연결
        self.packet_102.connect(self._dispatch_102)
        self.packet_103.connect(self._dispatch_103)

        # push_server 수신 스레드
        self.running = True
        threading.Thread(target=self.listen_push_server, daemon=True).start()

    # ---------------------------
    # 페이지 관리
    # ---------------------------
    def add_page(self, name, widget):
        self.pages[name] = widget
        self.stacked.addWidget(widget)

    def show_page(self, name):
        if name in self.pages:
            self.stacked.setCurrentWidget(self.pages[name])

    # ---------------------------
    # push_server(3002) 수신 처리
    # ---------------------------
    def listen_push_server(self):
        """FID102 / FID103 실시간 수신"""
        while self.running:
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.connect((self.SERVER_IP, self.PUSH_PORT))
                print("[GUI] Connected to push server")

                while self.running:
                    data = sock.recv(4096)
                    if not data:
                        break

                    # 최소 12바이트 헤더
                    if len(data) < 12:
                        continue

                    tid, length, fid = struct.unpack("<iii", data[:12])

                    # FID102
                    if fid == 102:
                        self.packet_102.emit(data)

                    # FID103
                    elif fid == 103:
                        self.packet_103.emit(data[12:])  # body 32 bytes

                sock.close()

            except Exception as e:
                print("[MainWindowManager] Push listen error:", e)
                time.sleep(1)

    # ---------------------------
    # 페이지에게 데이터 전달
    # ---------------------------
    def _dispatch_102(self, packet: bytes):
        page = self.pages.get("AdminManage")
        if page is not None and hasattr(page, "parse_item_update"):
            page.parse_item_update(packet)

    def _dispatch_103(self, body):
        page = self.pages.get("AdminManage")
        if page:
            page.handle_cart_state_packet(body)

    # ---------------------------
    # 종료
    # ---------------------------
    def closeEvent(self, event):
        self.running = False
        super().closeEvent(event)
