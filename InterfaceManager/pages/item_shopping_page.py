import os
from PyQt6.QtWidgets import QDialog, QPushButton, QListView
from PyQt6.QtCore import QStringListModel
from PyQt6 import uic
import socket
import struct


class ItemShoppingPage(QDialog):
    def __init__(self, manager=None):
        super().__init__()
        self.manager = manager

        ui_path = os.path.join(os.path.dirname(__file__), "item_shopping.ui")
        uic.loadUi(ui_path, self)

        # UI widgets
        self.btn_end = self.findChild(QPushButton, "pushButton_end")
        self.btn_reset = self.findChild(QPushButton, "pushButton_reset_list")
        self.btn_guide = self.findChild(QPushButton, "pushButton_guideM")
        self.btn_follow = self.findChild(QPushButton, "pushButton_followM")
        self.shopping_list = self.findChild(QListView, "shopping_list")

        # 리스트 모델
        self.list_model = QStringListModel()
        self.shopping_list.setModel(self.list_model)

        # 현재 모드 (0=대기, 2=팔로잉, 3=가이드)
        self.current_mode = 0

        # 버튼 연결
        self.btn_guide.clicked.connect(self.toggle_guide_mode)
        self.btn_follow.clicked.connect(self.toggle_follow_mode)
        self.btn_reset.clicked.connect(self.reset_list)
        self.btn_end.clicked.connect(self.finish_shopping)

        # 선택한 물품 이름 저장용
        self.selected_item = None


    # ------------------------------------------------------------
    # 페이지 오픈될 때 (선택한 물품 표시)
    # ------------------------------------------------------------
    def showEvent(self, event):
        super().showEvent(event)

        item_name = getattr(self.manager, "selected_item_name", None)
        self.selected_item = item_name

        if item_name:
            self.list_model.setStringList([f"선택한 물품: {item_name} (대기)"])
        else:
            self.list_model.setStringList(["(대기)"])


    # ------------------------------------------------------------
    # 서버 송신 (공용)
    # ------------------------------------------------------------
    def send_mode(self, data_value):
        SERVER_IP = "192.168.0.180"
        SERVER_PORT = 6000

        TXID = 33
        FID = 2
        COM = 2
        DATA = data_value

        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.connect((SERVER_IP, SERVER_PORT))

            LEN = 8
            packet = struct.pack("<iiii", TXID, LEN, FID, COM)
            packet += struct.pack("<i", DATA)

            sock.sendall(packet)
            sock.sendall(b"DONE")

        except Exception as e:
            print("통신 오류:", e)
        finally:
            try:
                sock.close()
            except:
                pass


    # ------------------------------------------------------------
    # 가이드 모드 로직
    # ------------------------------------------------------------
    def toggle_guide_mode(self):
        # 1) 선택한 물품이 없는 경우 → 가이드 모드 동작 X
        if not self.selected_item:
            return

        # 2) 이미 가이드 모드면 → 대기 상태 복귀
        if self.current_mode == 3:
            self.current_mode = 0
            self.list_model.setStringList([f"선택한 물품: {self.selected_item} (대기)"])
            self.send_mode(0x0001)
            return

        # 3) 팔로잉 모드였다면 → 가이드 모드 불가
        if self.current_mode == 2:
            return

        # 4) 대기 상태에서 가이드 모드로 전환
        self.current_mode = 3
        self.list_model.setStringList([f"선택한 물품: {self.selected_item} (가이드)"])
        self.send_mode(0x0003)


    # ------------------------------------------------------------
    # 팔로잉 모드 로직
    # ------------------------------------------------------------
    def toggle_follow_mode(self):
        # 1) 팔로잉 모드 ON → 다시 누르면 대기 복귀
        if self.current_mode == 2:
            self.current_mode = 0
            self.list_model.setStringList(["(대기)"])
            self.send_mode(0x0001)
            return

        # 2) 선택 모드(가이드/대기)에서 팔로잉 시작
        self.current_mode = 2
        self.selected_item = None   # 선택한 물품 제거
        self.list_model.setStringList(["(팔로잉)"])
        self.send_mode(0x0002)


    # ------------------------------------------------------------
    # 쇼핑 리스트 다시 선택하기
    # ------------------------------------------------------------
    def reset_list(self):
        self.current_mode = 0
        self.list_model.setStringList(["(대기)"])
        self.manager.show_page("ItemRadio")


    # ------------------------------------------------------------
    # 쇼핑 종료
    # ------------------------------------------------------------
    def finish_shopping(self):
        self.list_model.setStringList(["쇼핑을 종료합니다"])
        self.send_mode(0x0004)
