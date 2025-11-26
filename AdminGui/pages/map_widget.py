import os
import math
from PyQt6.QtWidgets import QWidget
from PyQt6.QtGui import QPainter, QColor, QPen, QBrush, QPixmap
from PyQt6.QtCore import Qt

WAYPOINTS = [
    ((2.14, 0.05), (0.3, 0.36)),    #1
    ((2.38, -0.48), (0.7, 0.36)),   #2
    ((2.27, -0.94), (1.12, 0.36)),      #3
    ((2.26, -1.54), (1.65, 0.36)),      #4
    ((2.06, -1.56), (1.65, 0.57)),       #5

    ((1.70, 0.02), (0.3, 0.93)),       #6
    ((1.65, -0.44), (0.7, 0.93)),      #7
    ((1.65, -0.90), (1.12, 0.93)),      #8
    ((1.62, -1.57), (1.65, 0.93)),      #9
    ((1.33, -1.57), (1.65, 1.27)),      #10

    ((0.96, 0.01), (0.3, 1.67)),       #11
    ((0.93, -0.52), (0.7, 1.67)),      #12
    ((0.90, -0.97), (1.12, 1.67)),      #13
    ((1.84, -1.57), (1.65, 1.67)),      #14
    ((0.61, -1.56), (1.65, 2.01)),      #15

    ((0.28, -0.01), (0.3, 2.25)),      #16
    ((0.31, -0.47), (0.7, 2.25)),      #17
    ((0.30, -0.82), (1.12, 2.25)),      #18
    ((0.32, -1.55), (1.65, 2.25)),      #19

    ((0.00, 0.00), (0.33, 2.76)),       #20
    ((-0.21, -1.57), (1.5, 2.76)),     #21
]

def nearest_gui_point(real_x, real_y):
    min_dist = 99999
    best_gui = None

    for (rx, ry), (gx, gy) in WAYPOINTS:
        dist = math.sqrt((real_x - rx)**2 + (real_y - ry)**2)
        if dist < min_dist:
            min_dist = dist
            best_gui = (gx, gy)

    return best_gui

class MapWidget(QWidget):

    def __init__(self, parent=None):
        super().__init__(parent)

        base_dir = os.path.dirname(__file__)
        img_path = os.path.join(base_dir, "admin_map.png")
        self.map_image = QPixmap(img_path)

        self.cart_colors = {
            1: QColor(255, 0, 0),      # 빨강
            2: QColor(255, 255, 0),    # 노랑
        }

        self.points = {}  # cid → (px, py)

    def update_point(self, cid, real_x, real_y):

        gui_x, gui_y = nearest_gui_point(real_x, real_y)

        # gui_x, gui_y는 범위 0~2, 0~3 비율 값이므로
        px = int((gui_x / 2.0) * self.width())
        py = int((gui_y / 3.0) * self.height())

        self.points[cid] = (px, py)
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        w = self.width()
        h = self.height()

        if not self.map_image.isNull():
            painter.drawPixmap(self.rect(), self.map_image)

        for cid, (px, py) in self.points.items():

            color = self.cart_colors.get(cid, QColor(255, 255, 255))

            painter.setPen(QPen(QColor(0, 0, 0), 2))
            painter.setBrush(QBrush(color))

            r = 10
            painter.drawEllipse(px - r, py - r, r*2, r*2)

        painter.end()
