import os
from PyQt6.QtWidgets import QDialog, QPushButton, QRadioButton, QMessageBox, QButtonGroup
from PyQt6 import uic
import socket
import struct


class ItemRadioButtonPage(QDialog):
    def __init__(self, manager=None):
        super().__init__()
        self.manager = manager

        # UI 파일 로드
        ui_path = os.path.join(os.path.dirname(__file__), "item_radiobutton.ui")
        uic.loadUi(ui_path, self)

        # 버튼 등록
        self.pushButton_confirm = self.findChild(QPushButton, "pushButton_confirm")
        self.pushButton_back = self.findChild(QPushButton, "pushButton_back")
        self.pushButton_selectD = self.findChild(QPushButton, "pushButton_selectD")
        self.pushButton_selectV = self.findChild(QPushButton, "pushButton_selectV")

        # 라디오버튼 그룹 설정
        self.button_group = QButtonGroup(self)
        self.button_group.setExclusive(True)
        self.radio_buttons = []

        for i in range(1, 17):  # radioButton_1 ~ radioButton_16
            rb = self.findChild(QRadioButton, f"radioButton_{i}")
            if rb:
                self.button_group.addButton(rb, i - 1)
                self.radio_buttons.append(rb)

        # 버튼 기능 연결
        self.pushButton_confirm.clicked.connect(self.send_selected_item)
        self.pushButton_back.clicked.connect(self.back_move)

        # 🔥 모드 변경 버튼
        self.pushButton_selectD.clicked.connect(self.go_radio_mode)
        self.pushButton_selectV.clicked.connect(self.go_voice_mode)

    # ------------------------------
    # 선택된 ID 가져오기
    # ------------------------------
    def send_selected_item(self):
        selected_id = self.button_group.checkedId()

        if selected_id == -1:
            QMessageBox.warning(self, "주의", "상품을 선택해주세요.")
            return

        # item_name 찾기 (라디오버튼 텍스트)
        rb = self.button_group.button(selected_id)
        item_name = rb.text()

        # manager 공유 데이터로 저장
        self.manager.selected_item_id = selected_id
        self.manager.selected_item_name = item_name

        # 서버로 즉시 전송 (COM=1)
        self.send_to_server(selected_id)

        # ItemShopping 페이지로 이동
        self.manager.show_page("ItemShopping")
    # ------------------------------
    # 서버 송신
    # ------------------------------
    def send_to_server(self, item_id):
        SERVER_IP = "192.168.0.180"
        SERVER_PORT = 6000

        TXID = 33
        FID = 2
        COM = 1
        DATA = item_id

        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.connect((SERVER_IP, SERVER_PORT))

            LEN = 4 + 4
            packet = struct.pack("<iiii", TXID, LEN, FID, COM)
            packet += struct.pack("<i", DATA)

            sock.sendall(packet)
            sock.sendall(b"DONE")

            print(f"[전송 완료] item_id={DATA} (COM=1)")
            #QMessageBox.information(self, "성공", f"물품(ID {DATA}) 전송 완료")

        except Exception as e:
            QMessageBox.critical(self, "오류", str(e))
        finally:
            sock.close()

        # 메인으로 이동
        self.manager.show_page("MainFrame")

    # ------------------------------
    # 페이지 이동
    # ------------------------------
    def back_move(self):
        self.manager.show_page("MainFrame")

    def go_radio_mode(self):
        self.manager.show_page("ItemRadio")

    def go_voice_mode(self):
        self.manager.show_page("ItemVoice")
