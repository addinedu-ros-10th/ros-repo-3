# realtime_freshness.py

# python realtime_freshness.py
# --meat_ckpt saved_models/meat_checkpoint.pt
# --fruit_ckpt saved_models/fruit_checkpoint.pt
# --meat_fresh_idxes 0,1
# --meat_spoil_idxes 2
# --imgsz 640 --conf 0.25

import os
import cv2
import time
import argparse
import numpy as np
import torch
import torch.nn.functional as F
from torchvision import transforms as T
from PIL import Image

# 라우터(탐지기) 선택: 필요에 맞게 하나만 import해서 사용
# from yolo_world_router import YOLOWorldDetector
# from yolo_v8_router import YOLOv8Detector
from dual_router import DualDetector

# ------------------------------
# RexNet 체크포인트 로더 (.pt/.pth 모두 지원)
# ------------------------------
def load_rexnet_checkpoint(ckpt_path: str, device: str):
    """
    1) 패키지형(.pt): {'meta':..., 'state_dict':...}
    2) 순수 state_dict(.pth): 자동 감지 (분류기 가중치 out_features 기준)
    """
    import timm

    pkg = torch.load(ckpt_path, map_location=device)

    # state_dict / meta 분리
    if isinstance(pkg, dict) and "state_dict" in pkg:
        state_dict = pkg["state_dict"]
        meta = pkg.get("meta", {})
    else:
        # 순수 state_dict만 있을 수도 있음
        state_dict = pkg
        meta = {}

    # 분류기 가중치 모양으로 out_features 자동 감지
    classifier_keys = ["head.fc.weight", "classifier.weight", "fc.weight"]
    out_features = None
    for k in classifier_keys:
        if isinstance(state_dict, dict) and k in state_dict:
            out_features = state_dict[k].shape[0]
            break
    if out_features is None:
        raise RuntimeError("분류기 가중치를 찾지 못했습니다. (head.fc.weight / classifier.weight / fc.weight)")

    # 클래스 목록
    classes = meta.get("class_names")
    if not classes or len(classes) != out_features:
        classes = [f"class_{i}" for i in range(out_features)]

    model_name = meta.get("model_name", "rexnet_150")
    img_size   = meta.get("img_size",   224)
    mean = meta.get("mean", [0.485, 0.456, 0.406])
    std  = meta.get("std",  [0.229, 0.224, 0.225])

    model = timm.create_model(model_name, pretrained=False, num_classes=out_features).to(device).eval()

    # 일부 저장물은 다시 'state_dict' 안에 포장되어 올 수 있음
    if isinstance(state_dict, dict) and "state_dict" in state_dict:
        state_dict = state_dict["state_dict"]

    model.load_state_dict(state_dict, strict=True)

    preprocess = T.Compose([
        T.Resize((img_size, img_size)),
        T.ToTensor(),
        T.Normalize(mean, std),
    ])
    return model, preprocess, classes, img_size

def suppress_overlaps(dets, iou_thr=0.6):
    """
    dets: [{'xyxy',[x1,y1,x2,y2],'name', 'conf', 'kind'}, ...]
    같은 kind끼리 IoU가 높으면 하나만 남김.
    우선순위: conf 큰 것.
    """
    out = []
    for kind in {"meat","fruit"}:
        group = [d for d in dets if d.get("kind")==kind]
        keep = []
        used = [False]*len(group)
        for i in range(len(group)):
            if used[i]: continue
            best = i
            for j in range(i+1, len(group)):
                if used[j]: continue
                if iou(group[i]["xyxy"], group[j]["xyxy"]) >= iou_thr:
                    # conf 큰 쪽만 남긴다
                    if group[j]["conf"] > group[best]["conf"]:
                        best = j
                    used[j]=True
            used[i]=True
            keep.append(group[best])
        out.extend(keep)
    # kind가 없는 회색 디버그 박스는 그대로
    out.extend([d for d in dets if d.get("kind") not in {"meat","fruit"}])
    return out


# ------------------------------
# EMA 스무더 (ID별)
# ------------------------------
class EmaBank:
    def __init__(self, alpha=0.95):
        self.alpha = alpha
        self.bank = {}  # key -> prob vec (np.array)

    def update(self, key, probs):
        if key not in self.bank:
            self.bank[key] = probs.copy()
        else:
            self.bank[key] = self.alpha * probs + (1 - self.alpha) * self.bank[key]
        return self.bank[key]

# ------------------------------
# 간단 IoU 매칭으로 ID 유지
# ------------------------------
def iou(a, b):
    ax1, ay1, ax2, ay2 = a; bx1, by1, bx2, by2 = b
    iw = max(0, min(ax2, bx2) - max(ax1, bx1))
    ih = max(0, min(ay2, by2) - max(ay1, by1))
    inter = iw * ih
    area_a = (ax2 - ax1) * (ay2 - ay1)
    area_b = (bx2 - bx1) * (by2 - by1)
    union = area_a + area_b - inter + 1e-6
    return inter / union

def match_boxes(prev, curr, thr=0.4):
    """
    prev: {id: xyxy}, curr: list[xyxy]
    return: (curr_idx -> id, new_prev)
    """
    curr_to_id, used = {}, set()
    # greedy match
    for i, c in enumerate(curr):
        best_id, best_iou = None, 0.0
        for pid, pxy in prev.items():
            if pid in used:
                continue
            v = iou(pxy, c)
            if v > best_iou:
                best_iou, best_id = v, pid
        if best_iou >= thr and best_id is not None:
            curr_to_id[i] = best_id
            used.add(best_id)
    # new ids
    next_id = (max(prev.keys()) + 1) if prev else 1
    for i in range(len(curr)):
        if i not in curr_to_id:
            curr_to_id[i] = next_id
            next_id += 1
    # new prev mapping
    new_prev = {curr_to_id[i]: curr[i] for i in range(len(curr))}
    return curr_to_id, new_prev

# ------------------------------
# RexNet 분류기 래퍼
# ------------------------------
class FreshnessClassifier:
    def __init__(self, ckpt_path: str, device: str):
        self.device = device
        self.model, self.preprocess, self.classes, self.img_size = load_rexnet_checkpoint(ckpt_path, device)

    @torch.no_grad()
    def predict(self, frame_bgr, xyxy, pad=8):
        h, w = frame_bgr.shape[:2]
        x1, y1, x2, y2 = xyxy
        x1 = max(0, x1 - pad); y1 = max(0, y1 - pad)
        x2 = min(w, x2 + pad); y2 = min(h, y2 + pad)
        crop = frame_bgr[y1:y2, x1:x2]
        if crop.size == 0:
            return None
        img = Image.fromarray(cv2.cvtColor(crop, cv2.COLOR_BGR2RGB))
        x = self.preprocess(img).unsqueeze(0).to(self.device)
        logits = self.model(x)
        probs = F.softmax(logits, dim=1)[0].cpu().numpy()
        return probs

# ------------------------------
# Fresh / Spoiled 전용 헬퍼 & 렌더러
# ------------------------------
# 기존 extract_fresh_spoiled 를 아래로 교체
FRESH_KEYS   = {"fresh", "good"}
SPOILED_KEYS = {"spoiled", "bad", "rotten", "decayed"}

def _find_index_by_keywords(classes, keys):
    norm = [c.strip().lower() for c in classes]
    for i, name in enumerate(norm):
        for k in keys:
            if k in name:
                return i
    return None

def extract_fresh_spoiled(
    probs: np.ndarray,
    classes: list[str],
    fresh_set: set[int] | None = None,
    spoil_set: set[int] | None = None,
    force_fresh_idx: int | None = None,
    force_spoil_idx: int | None = None,
):
    """
    우선순위:
    1) fresh_set/spoil_set가 주어지면 -> 그 집합의 확률을 합산해 fresh/spoiled 결정
    2) 강제 단일 인덱스(force_*)가 주어지면 -> 해당 인덱스로 계산
    3) 키워드 매칭('fresh','good' / 'spoiled','bad'...)으로 인덱스 찾기
    4) (이진) 0/1 가정, (다중) argmax 휴리스틱
    """
    n = len(probs)
    # 1) 집합 매핑 (다대일)
    if fresh_set is not None and spoil_set is not None:
        fresh_conf   = float(np.sum(probs[list(fresh_set)]))
        spoiled_conf = float(np.sum(probs[list(spoil_set)]))
        pred = "fresh" if fresh_conf >= spoiled_conf else "spoiled"
        return fresh_conf, spoiled_conf, pred

    # 2) 단일 인덱스 강제
    fi = force_fresh_idx
    si = force_spoil_idx

    # 3) 키워드 매칭
    if fi is None: fi = _find_index_by_keywords(classes, FRESH_KEYS)
    if si is None: si = _find_index_by_keywords(classes, SPOILED_KEYS)

    # 4) 폴백
    if fi is None or si is None:
        if n == 2:
            if fi is None and si is None:
                print("[WARN] class names lack keywords; assume idx0=fresh, idx1=spoiled ->", classes)
                fi, si = 0, 1
            elif fi is None:
                fi = 1 - si; print(f"[WARN] fresh idx inferred as {fi} from spoiled idx {si} ->", classes)
            elif si is None:
                si = 1 - fi; print(f"[WARN] spoiled idx inferred as {si} from fresh idx {fi} ->", classes)
        else:
            # 다중인데 키워드도/강제도 없으면 argmax 기준 휴리스틱
            idx = int(np.argmax(probs))
            name = str(classes[idx]).strip().lower()
            guess_fresh = any(k in name for k in FRESH_KEYS)
            fresh_conf   = float(probs[idx]) if guess_fresh else 0.0
            spoiled_conf = 1.0 - fresh_conf
            pred = "fresh" if fresh_conf >= spoiled_conf else "spoiled"
            return fresh_conf, spoiled_conf, pred

    fresh_conf   = float(probs[fi])
    spoiled_conf = float(probs[si])
    pred = "fresh" if fresh_conf >= spoiled_conf else "spoiled"
    return fresh_conf, spoiled_conf, pred

def parse_idxes(s, n_classes):
    if s is None: return None
    out = []
    for tok in s.split(","):
        tok = tok.strip()
        if not tok: continue
        i = int(tok)
        if i < 0 or i >= n_classes:
            raise ValueError(f"index {i} out of range for n_classes={n_classes}")
        out.append(i)
    return set(out) if out else None


def draw_items_by_fresh_rule(frame, items, best_title):
    """
    items: [{ 'box':(x1,y1,x2,y2), 'fresh':float, 'spoiled':float, 'pred':str }, ...]
    규칙:
      - pred == 'fresh' 인 것들 중 fresh가 가장 큰 1개만 best(초록)
      - pred == 'fresh' 이지만 best가 아닌 것: 하양
      - pred != 'fresh' (spoiled 등): 빨강
    """
    if not items:
        return
    # best 후보: 'fresh'로 예측된 박스만
    cand_idx = [i for i, it in enumerate(items) if str(it["pred"]).strip().lower() == "fresh"]
    best_i = None
    if cand_idx:
        best_i = max(cand_idx, key=lambda i: items[i]["fresh"])

    for i, it in enumerate(items):
        x1, y1, x2, y2 = it["box"]
        pred = str(it["pred"]).strip().lower()
        fresh_c = it["fresh"]

        if best_i is not None and i == best_i:
            color = (0, 255, 0)   # 초록
            tag = f"[{best_title}] fresh {fresh_c:.2f}"
            th = 3
        elif pred == "fresh":
            color = (255, 255, 255)  # 하양
            tag = f"fresh {fresh_c:.2f}"
            th = 2
        else:
            color = (0, 0, 255)      # 빨강
            tag = f"spoiled"
            th = 2

        cv2.rectangle(frame, (x1,y1), (x2,y2), color, th)
        cv2.putText(frame, tag, (x1, max(0, y1-6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

# ------------------------------
# 메인
# ------------------------------
def main():
    ap = argparse.ArgumentParser(description="YOLO(detector) → RexNet(classifier) | meat & fruit freshness")
    ap.add_argument("--meat_ckpt",  required=True,  help="RexNet checkpoint (.pt or .pth) for meat")
    ap.add_argument("--fruit_ckpt", required=False, help="RexNet checkpoint (.pt or .pth) for fruit")
    ap.add_argument("--device", default="cuda")
    ap.add_argument("--cam", type=int, default=0)
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--conf", type=float, default=0.25)
    ap.add_argument("--ema", type=float, default=0.85)
    # argparse 추가
    ap.add_argument("--meat_fresh_idxes",  type=str, default=None, help="예: 0,1  (good,normal을 fresh로 묶기)")
    ap.add_argument("--meat_spoil_idxes",  type=str, default=None, help="예: 2    (bad를 spoiled로)")
    ap.add_argument("--fruit_fresh_idxes", type=str, default=None)
    ap.add_argument("--fruit_spoil_idxes", type=str, default=None)


    args = ap.parse_args()

    device = args.device if torch.cuda.is_available() and args.device != "cpu" else "cpu"

    # 1) 탐지기 (듀얼: 고기=커스텀, 과일=COCO)
    detector = DualDetector(
        meat_w="runs/detect/train/weights/best.pt",  # 고기 커스텀 가중치
        fruit_w="yolov8s.pt",                        # 과일 COCO 가중치
        device=device, conf=args.conf, imgsz=args.imgsz
    )

    # 2) 분류기
    meat_cls  = FreshnessClassifier(args.meat_ckpt,  device)
    fruit_cls = FreshnessClassifier(args.fruit_ckpt, device) if args.fruit_ckpt else None

    meat_fresh_set  = parse_idxes(args.meat_fresh_idxes,  len(meat_cls.classes))
    meat_spoil_set  = parse_idxes(args.meat_spoil_idxes,  len(meat_cls.classes))
    fruit_fresh_set = parse_idxes(args.fruit_fresh_idxes, len(fruit_cls.classes)) if fruit_cls else None
    fruit_spoil_set = parse_idxes(args.fruit_spoil_idxes, len(fruit_cls.classes)) if fruit_cls else None
    
    
    # 3) 상태 (EMA / ID)
    ema = EmaBank(alpha=args.ema)
    prev_boxes = {}  # id -> xyxy

    # 4) 웹캠/윈도우
    cap = cv2.VideoCapture(args.cam)
    window_name = "Freshness (meat & fruit)"
    try:
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    except:
        print("[WARN] Could not create display window (headless mode)")

    if not cap.isOpened():
        raise RuntimeError("Cannot open camera")

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            t0 = time.time()

            # 탐지
            # 사용 위치:
            dets = detector.infer(frame)
            dets = suppress_overlaps(dets, iou_thr=0.6)
            # [{'xyxy','name','conf','kind'}, ...]
            xyxys = [d["xyxy"] for d in dets]
            id_map, prev_boxes = match_boxes(prev_boxes, xyxys, thr=0.4)

            # 리스트 초기화
            meat_rank  = []  # [{box,fresh,spoiled,pred}, ...]
            fruit_rank = []  # [{box,fresh,spoiled,pred}, ...]

            # 라우팅 → 분류 → EMA → 저장
            for i, d in enumerate(dets):
                box = d["xyxy"]; kind = d["kind"]
                obj_id = id_map.get(i, -1)

                if kind == "meat":
                    probs = meat_cls.predict(frame, box)
                    if probs is None:
                        continue
                    probs = ema.update(("meat", obj_id), probs)
                    # meat
                    fresh_c, spoiled_c, pred_label = extract_fresh_spoiled(
                        probs, meat_cls.classes,
                        fresh_set=meat_fresh_set, spoil_set=meat_spoil_set,
                        # 단일 인덱스 강제도 함께 전달 가능(둘 다 주면 fresh_set/spoil_set 우선)
                        force_fresh_idx=args.meat_fresh_idxes, force_spoil_idx=args.meat_spoil_idxes
                    )
                    print("[MEAT]", meat_cls.classes, "fresh:", round(fresh_c,3), "spoiled:", round(spoiled_c,3), "pred:", pred_label)


                    meat_rank.append({"box": tuple(box), "fresh": fresh_c, "spoiled": spoiled_c, "pred": pred_label})

                elif kind == "fruit" and fruit_cls is not None:
                    probs = fruit_cls.predict(frame, box)
                    if probs is None:
                        continue
                    probs = ema.update(("fruit", obj_id), probs)
                    # fruit
                    fresh_c, spoiled_c, pred_label = extract_fresh_spoiled(
                        probs, fruit_cls.classes,
                        fresh_set=fruit_fresh_set, spoil_set=fruit_spoil_set,
                        force_fresh_idx=args.fruit_fresh_idxes, force_spoil_idx=args.fruit_spoil_idxes
                    )
                    print("[FRUIT]", fruit_cls.classes, "fresh:", round(fresh_c,3), "spoiled:", round(spoiled_c,3), "pred:", pred_label)
                    fruit_rank.append({"box": tuple(box), "fresh": fresh_c, "spoiled": spoiled_c, "pred": pred_label})

                else:
                    # 분류 대상 아니면 회색 얇은 박스 (디버그용)
                    x1, y1, x2, y2 = box
                    cv2.rectangle(frame, (x1,y1), (x2,y2), (140,140,140), 1)

            # ----- 표시: 규칙 기반 렌더링 -----
            draw_items_by_fresh_rule(frame, meat_rank,  best_title="BEST MEAT")
            draw_items_by_fresh_rule(frame, fruit_rank, best_title="BEST FRUIT")

            # FPS 표기
            ms = (time.time() - t0) * 1000
            cv2.putText(frame, f"YOLO + RexNet | {ms:.1f} ms",
                        (8,24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (30,200,255), 2)

            # 화면 표시 (헤드리스 환경에서는 자동으로 건너뜀)
            try:
                cv2.imshow(window_name, frame)
                key = cv2.waitKey(1) & 0xFF
                if key == 27:  # ESC
                    break
            except:
                pass

    finally:
        cap.release()
        try:
            cv2.destroyAllWindows()
        except:
            pass

if __name__ == "__main__":
    main()
