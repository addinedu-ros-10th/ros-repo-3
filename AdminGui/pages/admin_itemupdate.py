import os
import socket
import struct
from typing import List, Optional

from PyQt6 import uic
from PyQt6.QtWidgets import (
    QDialog, QPushButton, QComboBox, QLineEdit, QMessageBox
)


class AdminItemUpdateWindow(QDialog):
    """
    재고 수정 화면.
    - 콤보박스에서 물품 선택
    - 수량(1~5) 입력 후 서버로 QTY 16개 배열 전송(Function ID 2)
    """

    SERVER_IP = "192.168.0.180"
    SERVER_PORT = 7001

    def __init__(self, manager=None):
        super().__init__()
        self.manager = manager

        ui_path = os.path.join(os.path.dirname(__file__), "admin_itemupdate.ui")
        uic.loadUi(ui_path, self)

        # 위젯
        self.combo_item: QComboBox = self.findChild(QComboBox, "comboBox_itemlist")
        self.edit_qty: QLineEdit = self.findChild(QLineEdit, "lineEdit_QTY")
        self.btn_update: QPushButton = self.findChild(QPushButton, "pushButton_update")
        self.btn_back: QPushButton = self.findChild(QPushButton, "pushButton_back")

        # IID ↔ 인덱스 매핑 (콤보박스 순서와 동일)
        self.item_names: List[str] = [
            "소주", "맥주", "케첩", "마요네즈",
            "과자_주황색", "과자_노란색",
            "초밥", "피자",
            "후라이팬", "냄비",
            "딸기", "수박", "포도",
            "장식_나무", "장식_산타", "장식_고리",
        ]

        if self.btn_update:
            self.btn_update.clicked.connect(self.on_update_clicked)
        if self.btn_back:
            self.btn_back.clicked.connect(self.on_back_clicked)

    # ------------------------------------------------------------------
    # 공통 TCP 전송 (timeout 포함)
    # ------------------------------------------------------------------
    def _send_packet(self, payload: bytes):
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
                sock.settimeout(1.0)
                sock.connect((self.SERVER_IP, self.SERVER_PORT))
                sock.sendall(payload)
                sock.sendall(b"DONE")
                try:
                    response = sock.recv(4096)
                except socket.timeout:
                    print("[AdminItemUpdate] 서버 응답 timeout")
                    return None
                return response
        except OSError as e:
            print("[AdminItemUpdate] TCP 오류:", e)
            QMessageBox.critical(self, "통신 오류", f"서버 통신 중 오류가 발생했습니다.\n{e}")
            return None

    def make_update_packet(self, qty_list: List[int]) -> bytes:
        """
        Function ID 2 : 매장 물품 정보 업데이트
        QTY 16개 전체를 전송.
        """
        if len(qty_list) != 16:
            raise ValueError("QTY 리스트는 반드시 16개여야 합니다.")

        transaction_id = 1
        length_of_data = 64
        function_id = 2

        packet = struct.pack("<iii16i", transaction_id, length_of_data, function_id, *qty_list)
        return packet

    # ------------------------------------------------------------------
    # 버튼 핸들러
    # ------------------------------------------------------------------
    def on_update_clicked(self):
        # 전체 재고 리스트 (없으면 0으로 초기화)
        qty_list = getattr(self.manager, "qty_list", [0] * 16) if self.manager else [0] * 16

        # 선택된 IID
        iid = self.combo_item.currentIndex()
        if iid < 0 or iid >= 16:
            QMessageBox.warning(self, "오류", "물품을 선택하세요.")
            return

        item_name = self.item_names[iid]

        # 수량 검증 (1~5)
        text = self.edit_qty.text().strip()
        if not text.isdigit():
            QMessageBox.warning(self, "오류", "수량은 숫자만 입력 가능합니다.")
            return
        qty = int(text)
        if not (1 <= qty <= 5):
            QMessageBox.warning(self, "오류", "수량은 1~5 범위만 허용됩니다.")
            return

        # 🔥 확인 메시지
        reply = QMessageBox.question(
            self,
            "재고 수정 확인",
            f"물품명: {item_name}\n수량: {qty} 개로 수정하시겠습니까?",
            QMessageBox.Yes | QMessageBox.No,
        )
        if reply != QMessageBox.Yes:
            return

        # 리스트 수정
        qty_list = list(qty_list)
        qty_list[iid] = qty

        # 패킷 생성 + 전송
        packet = self.make_update_packet(qty_list)
        response = self._send_packet(packet)
        if response is None:
            return

        QMessageBox.information(self, "완료", "재고 수정 요청을 전송했습니다.")

        if self.manager is not None:
            setattr(self.manager, "qty_list", qty_list)
            # 메인 화면으로 복귀
            self.manager.show_page("AdminManage")

    def on_back_clicked(self):
        """돌아가기 → AdminManage 화면으로"""
        if self.manager is not None:
            self.manager.show_page("AdminManage")
