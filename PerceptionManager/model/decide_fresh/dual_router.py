# dual_detector.py
from ultralytics import YOLO
import torch

FRUITS = {"apple","banana","orange","broccoli","carrot"}  # COCO에 있는 과일만
MEATS  = {"meat","beef","pork","steak"}                   # 커스텀 모델 클래스에 맞게

class DualDetector:
    def __init__(self, meat_w, fruit_w="yolov8s.pt", device="cuda", conf=0.2, imgsz=640):
        self.meat = YOLO(meat_w)
        self.fruit = YOLO(fruit_w)
        self.device, self.conf, self.imgsz = device, conf, imgsz

    @torch.no_grad()
    def infer(self, frame):
        out = []

        # 1) 과일: COCO 가중치
        r = self.fruit.predict(frame, conf=self.conf, imgsz=self.imgsz, device=self.device, verbose=False)[0]
        names = r.names
        for b in (r.boxes or []):
            x1,y1,x2,y2 = b.xyxy[0].int().tolist()
            cls = int(b.cls.item()); name = (names[cls] if isinstance(names, dict) else names[cls]).lower()
            if name in FRUITS:
                out.append({"xyxy":[x1,y1,x2,y2], "name":name, "conf":float(b.conf.item()), "kind":"fruit"})

        # 2) 고기: 커스텀 가중치
        r = self.meat.predict(frame, conf=self.conf, imgsz=self.imgsz, device=self.device, verbose=False)[0]
        names = r.names
        for b in (r.boxes or []):
            x1,y1,x2,y2 = b.xyxy[0].int().tolist()
            cls = int(b.cls.item()); name = (names[cls] if isinstance(names, dict) else names[cls]).lower()
            if name in MEATS:
                out.append({"xyxy":[x1,y1,x2,y2], "name":name, "conf":float(b.conf.item()), "kind":"meat"})

        return out
