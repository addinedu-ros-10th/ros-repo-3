
import socket
import struct
from PyQt6.QtWidgets import QApplication, QWidget, QPushButton, QVBoxLayout

class ProUpdateClass(QWidget):
    def __init__(self, manager):
        super().__init__()

        self.manager = manager

    # ---------------------------------------------------------
    # ① 모든 물품에 대해 매장 물품 정보 요청
    # ---------------------------------------------------------
    def product_update(self):
        customer_id = 1  # 고객 ID(예: 1)

        # 1~16번까지 모든 물품 선택
        selected_items = list(range(1, 17))

        # 모든 물품의 수량은 0 (요청 목적)
        quantities = {i: 0 for i in selected_items}

        # 패킷 생성 및 전송
        packet = self.make_store_item_request(customer_id, selected_items, quantities)
        self.send_tcp_packet(packet)

    # ---------------------------------------------------------
    # ② 프로토콜 형식에 맞는 패킷 구성
    # ---------------------------------------------------------
    def make_store_item_request(self, customer_id, selected_items, quantities):
        function_id = 1  # 매장 물품 정보 요청

        # -----------------------------------------------------
        # (1) 물품 ID bit flag 생성 (1~16번 전부 활성화)
        # -----------------------------------------------------
        item_flag = 0
        for item in selected_items:
            item_flag |= (1 << (item - 1))  # 모든 비트 켜짐

        # -----------------------------------------------------
        # (2) 수량 리스트 생성 (항상 16개)
        # -----------------------------------------------------
        qty_list = []
        for i in range(1, 17):
            qty = quantities.get(i, 0)  # 없으면 0
            qty_list.append(qty)

        # -----------------------------------------------------
        # (3) body 생성 - 고객ID + 물품ID(bit) + 수량(16개)
        # -----------------------------------------------------
        body = struct.pack(
            "!B I" + "H" * 16,
            customer_id,     # 1바이트
            item_flag,       # 4바이트 물품 비트ID
            *qty_list        # 16개 x 2바이트
        )

        # -----------------------------------------------------
        # (4) 전체 패킷 길이 계산 = header + body
        # -----------------------------------------------------
        packet_length = len(body) + 3  # FunctionID(1) + Length(2)

        header = struct.pack("!BH", function_id, packet_length)
        return header + body

    # ---------------------------------------------------------
    # ③ TCP 패킷 전송
    # ---------------------------------------------------------
    def send_tcp_packet(self, packet):
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
                sock.connect(("127.0.0.1", 9500))
                sock.sendall(packet)

                print("전송된 패킷:", packet)

                response = sock.recv(1024)
                print("서버 응답:", response)

        except Exception as e:
            print("TCP 통신 오류:", e)


if __name__ == "__main__":
    app = QApplication([])
    window = ProUpdateClass()
    window.show()
    app.exec()

