# pages/admin_orderstate.py
import os
import struct

from PyQt6 import uic
from PyQt6.QtWidgets import (
    QDialog, QTableWidget, QTableWidgetItem,
    QPushButton, QWidget, QVBoxLayout
)

from .map_widget import MapWidget


class AdminOrderStateWindow(QDialog):
    """
    주문 상태 화면.
    - 온라인 / 오프라인 카트 목록
    - 맵 위젯에 카트 위치 표시 (POSX, POSY)
    """

    def __init__(self, manager=None):
        super().__init__()
        self.manager = manager

        ui_path = os.path.join(os.path.dirname(__file__), "admin_orderstate.ui")
        uic.loadUi(ui_path, self)

        # 위젯 참조
        self.table_online: QTableWidget = self.findChild(QTableWidget, "tableWidget_online")
        self.table_offline: QTableWidget = self.findChild(QTableWidget, "tableWidget_offline")
        self.btn_back: QPushButton = self.findChild(QPushButton, "pushButton_back")

        # map 컨테이너 위젯에 MapWidget 삽입
        container: QWidget = self.findChild(QWidget, "map")
        self.map_widget = None
        if container is not None:
            layout = QVBoxLayout(container)
            layout.setContentsMargins(0, 0, 0, 0)
            self.map_widget = MapWidget(container)
            layout.addWidget(self.map_widget)
        else:
            print("[AdminOrderState] map 위젯을 찾을 수 없습니다.")

        # 시그널 연결
        if self.btn_back:
            self.btn_back.clicked.connect(self.go_back)

    # ------------------------------------------------------------------
    # 뒤로가기 → AdminManage 로 이동
    # ------------------------------------------------------------------
    def go_back(self):
        if self.manager is not None:
            self.manager.show_page("AdminManage")

    # ------------------------------------------------------------------
    # 카트 상태 패킷 처리 (Function ID 103 공용 포맷)
    # ------------------------------------------------------------------
    def handle_cart_state_packet(self, payload: bytes):
        """
        MarketCoreManager → AdminGui : Function ID 103
        ByteLength 32, 데이터:
        CID(int), ID(int), STID(int), BAT(float),
        ESTID(int), PROG(int), POSX(float), POSY(float)
        """
        try:
            if len(payload) < 32:
                raise ValueError("카트 상태 데이터 길이 부족")

            cid, user_id, stid, bat, estid, prog, posx, posy = struct.unpack(
                "<iii f i i f f",
                payload[:32],
            )
        except struct.error as e:
            print("[AdminOrderState] 카트 상태 파싱 오류:", e)
            return

        # 맵에 위치 갱신
        if self.map_widget is not None:
            self.map_widget.update_point(cid, posx, posy)

        # 온라인 / 오프라인 분류 (user_id >= 0 이면 온라인)
        if user_id >= 0:
            self._update_online_table(cid, user_id, stid)
        else:
            self._update_offline_table(cid, user_id, stid)



    # ------------------------------------------------------------------
    # 온라인 / 오프라인 테이블 갱신
    # ------------------------------------------------------------------
    def _update_online_table(self, cid: int, user_id: int, stid: int):
        row = self._find_row(self.table_online, cid)
        if row == -1:
            row = self.table_online.rowCount()
            self.table_online.insertRow(row)

        self.table_online.setItem(row, 0, QTableWidgetItem(str(cid)))
        self.table_online.setItem(row, 1, QTableWidgetItem(str(user_id)))
        self.table_online.setItem(row, 2, QTableWidgetItem(str(stid)))

    def _update_offline_table(self, cid: int, user_id: int, stid: int):
        row = self._find_row(self.table_offline, cid)
        if row == -1:
            row = self.table_offline.rowCount()
            self.table_offline.insertRow(row)

        self.table_offline.setItem(row, 0, QTableWidgetItem(str(cid)))
        self.table_offline.setItem(row, 1, QTableWidgetItem(str(user_id)))
        self.table_offline.setItem(row, 2, QTableWidgetItem(str(stid)))

    @staticmethod
    def _find_row(table: QTableWidget, cid: int) -> int:
        for row in range(table.rowCount()):
            item = table.item(row, 0)
            if item and item.text() == str(cid):
                return row
        return -1
