import os
import soundfile as sf
import numpy as np
import whisper
import sounddevice as sd
import socket
import struct

from PyQt6.QtWidgets import (
    QDialog, QPushButton, QMessageBox, QListView
)
from PyQt6.QtCore import QStringListModel
from PyQt6 import uic


RATE = 16000
CHANNELS = 1
DURATION = 5
OUTPUT_FILE = "voice_temp.wav"


class ItemVoicePage(QDialog):

    def __init__(self, manager=None):
        super().__init__()
        self.manager = manager

        # Load UI
        ui_path = os.path.join(os.path.dirname(__file__), "item_voice.ui")
        uic.loadUi(ui_path, self)

        # UI widgets
        self.pushButton_record = self.findChild(QPushButton, "pushButton_record")
        self.pushButton_confirm = self.findChild(QPushButton, "pushButton_confirm")
        self.pushButton_back = self.findChild(QPushButton, "pushButton_back")
        self.pushButton_selectD = self.findChild(QPushButton, "pushButton_selectD")
        self.pushButton_selectV = self.findChild(QPushButton, "pushButton_selectV")
        self.shopping_list = self.findChild(QListView, "shopping_list")

        # List model
        self.result_list_model = QStringListModel()
        self.shopping_list.setModel(self.result_list_model)

        # Whisper model
        self.model = whisper.load_model("base")

        # Button connections
        self.pushButton_record.clicked.connect(self.record_voice)
        self.pushButton_confirm.clicked.connect(self.confirm_selection)
        self.pushButton_back.clicked.connect(self.back_move)
        self.pushButton_selectD.clicked.connect(self.go_radio_mode)
        self.pushButton_selectV.clicked.connect(self.go_voice_mode)


    # ----------------------------------------------------
    # 음성 녹음
    # ----------------------------------------------------
    def record_voice(self):
        QMessageBox.information(self, "녹음 시작", f"{DURATION}초 동안 말하세요...")

        recording = sd.rec(int(DURATION * RATE), samplerate=RATE,
                           channels=CHANNELS, dtype='int16')
        sd.wait()

        QMessageBox.information(self, "녹음 종료", "녹음이 종료되었습니다.")

        # numpy float 변환
        audio_np = recording.astype(np.float32) / 32768.0

        # wav 파일 저장
        sf.write(OUTPUT_FILE, audio_np, RATE)

        # Whisper STT
        result = self.model.transcribe(OUTPUT_FILE, language="ko")
        text = result["text"].strip()

        if text:
            # 리스트뷰에 추가
            current = self.result_list_model.stringList()
            current.append(text)
            self.result_list_model.setStringList(current)

        print("STT:", text)


    # ----------------------------------------------------
    # confirm → 선택된 음성 결과를 서버로 전송
    # ----------------------------------------------------
    def confirm_selection(self):
        selected_index = self.shopping_list.currentIndex()

        if not selected_index.isValid():
            QMessageBox.warning(self, "주의", "목록에서 항목을 선택해주세요.")
            return

        selected_text = self.result_list_model.data(selected_index)
        print("선택된 음성 결과:", selected_text)

        # ↙ 여기에 음성 → item_id 매핑 로직 들어가야 함
        item_id = self.text_to_id(selected_text)
        if item_id is None:
            QMessageBox.warning(self, "오류", "해당 음성 결과를 ID로 변환할 수 없습니다.")
            return

        print("전송할 ID:", item_id)
        self.send_to_server(item_id)


    # ----------------------------------------------------
    # 텍스트 → item_id 매핑 (너가 규칙만 알려주면 자동 생성해줌)
    # ----------------------------------------------------
    def text_to_id(self, text: str):
        mapping = {
            "소주": 0,
            "맥주": 1,
            "케첩": 2,
            "마요네즈": 3,
            "과자_A": 4,
            "과자_B": 5,
            "초밥": 6,
            "피자": 7,
            "후라이팬": 8,
            "냄비": 9,
            "딸기": 10,
            "수박": 11,
            "포도": 12,
            "나무 장식": 13,
            "산타 장식": 14,
            "고리 장식": 15,
            # ... 여기에 너의 모든 항목 매핑 넣어줄 수 있음
        }

        for key in mapping:
            if key in text:
                return mapping[key]

        return None  # 못 찾으면 실패


    # ----------------------------------------------------
    # 서버로 ID 송신
    # ----------------------------------------------------
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

            QMessageBox.information(self, "성공", f"ID {DATA} 서버 전송 완료.")

        except Exception as e:
            QMessageBox.critical(self, "오류", str(e))

        finally:
            sock.close()

        self.manager.show_page("MainFrame")

    # ----------------------------------------------------
    # 페이지 전환
    # ----------------------------------------------------
    def go_radio_mode(self):
        self.manager.show_page("ItemRadio")

    def go_voice_mode(self):
        self.manager.show_page("ItemVoice")

    def back_move(self):
        self.manager.show_page("MainFrame")
