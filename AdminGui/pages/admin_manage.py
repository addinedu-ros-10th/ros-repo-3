# pages/admin_manage.py
import os
import socket
import struct
from typing import List, Optional

from PyQt6 import uic
from PyQt6.QtWidgets import (
    QDialog, QTableWidget, QTableWidgetItem, QPushButton,
    QMessageBox, QProgressBar, QComboBox, QLineEdit,
    QWidget, QVBoxLayout
)
from PyQt6.QtGui import QColor
from .map_widget import MapWidget


class AdminManageWindow(QDialog):
    """
    통합 관리자 화면

    - FID 102 : 매장 물품 정보 수신 → tableWidget_item
    - FID 103 : 카트 상태 정보 수신
        * tableWidget_cartstate  (CID, robot_status 문자열, updated_time)
        * tableWidget_error      (CID, error_status 문자열, updated_time)
        * tableWidget_online     (온라인 카트 목록)
        * tableWidget_offline    (오프라인 카트 목록)
        * Battery_state_1/2      (카트 배터리 전압)
        * map 위젯(MapWidget)     (POSX, POSY 표시)
    - “새로고침” 버튼 : 물품 정보 재요청(FID 2, Length=0)
    - “수정” 버튼    : 선택 물품 QTY 수정(FID 2, Length=64, int*16 전송)
    - “로그아웃” 버튼 : 로그인 화면으로 복귀
    """

    #SERVER_IP = "192.168.0.180"
    SERVER_IP = "127.0.0.1"
    SERVER_PORT = 7001  # 로그인 및 FID2 요청 포트

    ROBOT_STATUS_MAP = {
        0: "CART_INIT", 
        1: "CHARGE_STBY", 
        2: "TASK_STBY",
        3: "ONLINE_DRIVE", 
        4: "ONLINE_PICKUP", 
        5: "ONLINE_STBY",
        6: "ONLINE_PACKING", 
        7: "ONLINE_END",
        33: "OFFLINE_IDCHECK", 
        34: "OFFLINE_USERCHECK",
        35: "OFFLINE_STBY", 
        36: "OFFLINE_GUI", 
        37: "OFFLINE_VOICE",
        38: "OFFLINE_CONFIRM", 
        39: "OFFLINE_FOLLOW",
        40: "OFFLINE_DRIVE", 
        41: "OFFLINE_PACKING", 
        42: "OFFLINE_END",
        255: "RETURN", 
        99: "test"
    }

    ERROR_STATUS_MAP = {
        0: "low_battery",
        1: "location_lost",
        2: "camera_Nwork",
        99: "test"
    }

    def __init__(self, manager=None):
        super().__init__()
        self.manager = manager

        ui_path = os.path.join(os.path.dirname(__file__), "admin_manage.ui")
        uic.loadUi(ui_path, self)

        # -------------------- 위젯 참조 --------------------
        self.table_item = self.findChild(QTableWidget, "tableWidget_item")
        self.table_cart = self.findChild(QTableWidget, "tableWidget_cartstate")
        self.table_error = self.findChild(QTableWidget, "tableWidget_error")
        self.table_online = self.findChild(QTableWidget, "tableWidget_online")
        self.table_offline = self.findChild(QTableWidget, "tableWidget_offline")

        self.btn_refresh = self.findChild(QPushButton, "pushButton_item_refresh")
        self.btn_update = self.findChild(QPushButton, "pushButton_update")
        self.btn_logout = self.findChild(QPushButton, "pushButton_logout")

        self.combo_item = self.findChild(QComboBox, "comboBox_itemlist")
        self.edit_qty = self.findChild(QLineEdit, "lineEdit_QTY")

        self.battery1 = self.findChild(QProgressBar, "Battery_state_1")
        self.battery2 = self.findChild(QProgressBar, "Battery_state_2")

        # ---- MapWidget 장착 ----
        container = self.findChild(QWidget, "map")
        self.map_widget = None
        if container:
            layout = QVBoxLayout(container)
            layout.setContentsMargins(0, 0, 0, 0)
            self.map_widget = MapWidget(container)
            layout.addWidget(self.map_widget)

        # 내부 재고 버퍼
        self.qty_list = [0] * 16
        self.item_names = [
            "소주", "맥주", "케첩", "마요네즈",
            "과자_주황색", "과자_노란색",
            "초밥", "피자",
            "후라이팬", "냄비",
            "딸기", "수박", "포도",
            "장식_나무", "장식_산타", "장식_고리"
        ]

        self._setup_tables()

        # 버튼 연결
        self.btn_refresh.clicked.connect(self.request_item_list)
        self.btn_update.clicked.connect(self.update_qty)
        self.btn_logout.clicked.connect(self.logout)

        self._first_show = False

    # -----------------------------------------------------
    def showEvent(self, event):
        super().showEvent(event)
        if not self._first_show:
            self._first_show = True
            self.request_item_list()

    # -----------------------------------------------------
    def _setup_tables(self):
        self.table_item.setRowCount(16)
        for i in range(16):
            self.table_item.setItem(i, 0, QTableWidgetItem(str(i)))
            self.table_item.setItem(i, 1, QTableWidgetItem(self.item_names[i]))
            self.table_item.setItem(i, 2, QTableWidgetItem("0"))

        self.table_cart.setRowCount(0)
        self.table_error.setRowCount(0)
        self.table_online.setRowCount(0)
        self.table_offline.setRowCount(0)

    # -----------------------------------------------------
    # 패킷 송수신
    # -----------------------------------------------------
    def send_packet(self, payload):
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.settimeout(1)
                s.connect((self.SERVER_IP, self.SERVER_PORT))
                s.sendall(payload)
                s.sendall(b"DONE")
                return s.recv(4096)
        except Exception as e:
            print("[AdminManage] TCP Error:", e)
            return None

    # -----------------------------------------------------
    # FID2 - 재고 요청 패킷
    # -----------------------------------------------------
    def request_item_list(self):
        packet = struct.pack("<iii", 1, 0, 2)
        resp = self.send_packet(packet)

        if not resp or len(resp) < 80:
            print("[AdminManage] Invalid FID102 Response")
            return

        _, length, fid = struct.unpack("<iii", resp[:12])
        if fid != 102:
            return

        admin_id = struct.unpack("<i", resp[12:16])[0]
        self.qty_list = list(struct.unpack("<16i", resp[16:16+64]))
        self.update_item_table()

    def update_item_table(self):
        for i, qty in enumerate(self.qty_list):
            self.table_item.setItem(i, 2, QTableWidgetItem(str(qty)))

    # -----------------------------------------------------
    # 수량 수정 FID2 (length=64)
    # -----------------------------------------------------
    def update_qty(self):
        idx = self.combo_item.currentIndex()
        text = self.edit_qty.text().strip()

        if not text.isdigit():
            QMessageBox.warning(self, "오류", "수량은 숫자만 입력하세요.")
            return

        qty = int(text)
        self.qty_list[idx] = qty

        packet = struct.pack("<iii16i", 1, 64, 2, *self.qty_list)
        self.send_packet(packet)
        self.update_item_table()

    # -----------------------------------------------------
    # FID103 처리(실시간)
    # -----------------------------------------------------
    def handle_cart_state_packet(self, body: bytes):
        if len(body) < 32:
            return

        cid, user, stid, bat, estid, prog, x, y = struct.unpack(
            "<iii f i i f f", body[:32]
        )

        # 배터리
        if cid == 1:
            #self.battery1.setValue(int(bat))
            scaled = int(bat * 10)        
            self.battery1.setMaximum(1000)   # 100.0%까지 가능
            self.battery1.setValue(scaled)

            self.battery1.setFormat(f"{bat:.1f}%")            
        elif cid == 2:
            #self.battery2.setValue(int(bat))
            scaled = int(bat * 10)        
            self.battery2.setMaximum(1000)   # 100.0%까지 가능
            self.battery2.setValue(scaled)

            self.battery2.setFormat(f"{bat:.1f}%")
        # cart 상태 테이블 업데이트
        self._update_cart_table(cid, stid)

        # error 테이블
        if estid != 0:
            self._update_error_table(cid, estid)

        # online/offline
        if user >= 0:
            self._update_online_table(cid, user, stid)
        else:
            self._update_offline_table(cid, user, stid)

        # 지도 업데이트
        if self.map_widget:
            self.map_widget.update_point(cid, x, y)
        # 배터리 ProgressBar 색상 적용
        if self.map_widget:
            color = self.map_widget.cart_colors.get(cid, QColor(200, 200, 200))
            r, g, b = color.red(), color.green(), color.blue()

            style = f"""
                QProgressBar::chunk {{
                    background-color: rgb({r}, {g}, {b});
                }}
                QProgressBar {{
                    border: 1px solid gray;
                    border-radius: 5px;
                    text-align: center;
                    color: rgb(0, 0, 0);         /* 검정 글씨 */
                    font-weight: bold;           /* 굵게 */
                    font-size: 14px;             /* 글씨 크기 */
                }}
            """

            if cid == 1:
                self.battery1.setStyleSheet(style)
            elif cid == 2:
                self.battery2.setStyleSheet(style)

    # -----------------------------------------------------
    def _update_cart_table(self, cid, stid):
        state = self.ROBOT_STATUS_MAP.get(stid, f"ST:{stid}")

        row = self._find(self.table_cart, cid)
        if row == -1:
            row = self.table_cart.rowCount()
            self.table_cart.insertRow(row)

        self.table_cart.setItem(row, 0, QTableWidgetItem(str(cid)))
        self.table_cart.setItem(row, 1, QTableWidgetItem(state))
        self.table_cart.setItem(row, 2, QTableWidgetItem("-"))

    def _update_error_table(self, cid: int, estid: int):
            if self.table_error is None:
                return

            err_str = self.ERROR_STATUS_MAP.get(estid, f"ESTID:{estid}")

            # 이미 같은 Cart ID row가 있으면 덮어쓰기
            row = self._find_row_by_cart_id(self.table_error, cid)
            if row == -1:
                row = self.table_error.rowCount()
                self.table_error.insertRow(row)

            self.table_error.setItem(row, 0, QTableWidgetItem(str(cid)))
            self.table_error.setItem(row, 1, QTableWidgetItem(err_str))
            self.table_error.setItem(row, 2, QTableWidgetItem("-"))

    def _update_online_table(self, cid, user, stid):
        row = self._find(self.table_online, cid)
        if row == -1:
            row = self.table_online.rowCount()
            self.table_online.insertRow(row)

        st = self.ROBOT_STATUS_MAP.get(stid, f"ST:{stid}")
        self.table_online.setItem(row, 0, QTableWidgetItem(str(cid)))
        self.table_online.setItem(row, 1, QTableWidgetItem(str(user)))
        self.table_online.setItem(row, 2, QTableWidgetItem(st))

    def _update_offline_table(self, cid, user, stid):
        row = self._find(self.table_offline, cid)
        if row == -1:
            row = self.table_offline.rowCount()
            self.table_offline.insertRow(row)

        st = self.ROBOT_STATUS_MAP.get(stid, f"ST:{stid}")

        # 🔥 user가 -1이면 1로 표시
        display_user = abs(user) if user < 0 else user

        self.table_offline.setItem(row, 0, QTableWidgetItem(str(cid)))
        self.table_offline.setItem(row, 1, QTableWidgetItem(str(display_user)))
        self.table_offline.setItem(row, 2, QTableWidgetItem(st))

    # -----------------------------------------------------
    @staticmethod
    def _find(table, cid):
        for r in range(table.rowCount()):
            if table.item(r, 0) and table.item(r, 0).text() == str(cid):
                return r
        return -1

    def _find_row_by_cart_id(self, table, cid: int) -> int:
        if table is None:
            return -1

        for row in range(table.rowCount()):
            item = table.item(row, 0)
            if item and item.text() == str(cid):
                return row

        return -1

    # -----------------------------------------------------
    def parse_item_update(self, packet: bytes):
        """
        Function ID 102 패킷 수신 (push_server 또는 request 응답)
        포맷:
          [tid(int), length(int), fid(int=102), admin_id(int), qty(16*int)]
        """
        if len(packet) < 12 + 68:
            print("[AdminManage] Invalid FID102 packet length")
            return

        tid, length, fid = struct.unpack("<iii", packet[:12])
        if fid != 102:
            print("[AdminManage] Invalid FID102 Response")
            return

        admin_id = struct.unpack("<i", packet[12:16])[0]
        qty_values = list(struct.unpack("<16i", packet[16:16 + 64]))

        self.qty_list = qty_values
        self.update_item_table()
    # -----------------------------------------------------
    def logout(self):
        QMessageBox.information(self, "로그아웃", "로그아웃 완료!")
        self.manager.show_page("AdminLoginWindow")
