# hand_gesture.py
import cv2
import numpy as np
import mediapipe as mp
from collections import deque, defaultdict

class HandGestureRecognizer:
    """
    가벼운 MediaPipe Hands 기반 제스처 인식기.
    - 입력: 상반신 ROI(BGR), person_id
    - 출력: 확정된 제스처 라벨 (또는 None)
    제스처 라벨: "OPEN" (손바닥), "FIST"(주먹), "THUMBS_UP"(엄지척), "V_SIGN"(V사인), "UNKNOWN"
    """
    def __init__(self,
                 max_hands=1,
                 det_conf=0.6,
                 track_conf=0.5,
                 smooth_window=5,
                 hold_frames=3,
                 process_every_n=3):
        self.hands = mp.solutions.hands.Hands(
            max_num_hands=max_hands,
            min_detection_confidence=det_conf,
            min_tracking_confidence=track_conf
        )
        self.process_every_n = process_every_n
        self.frame_skip = 0

        # 사람별 최근 제스처 히스토리(다수결) & 디바운스
        self.history = defaultdict(lambda: deque(maxlen=smooth_window))
        self.hold_frames = hold_frames

        # MediaPipe landmark 인덱스
        self.TIPS = [4, 8, 12, 16, 20]  # thumb, index, middle, ring, pinky tips
        self.PIPS = [2, 6, 10, 14, 18]
        self.margin_ratio = 0.02  # ROI 크기의 2%를 여유 마진으로 사용


    def _np_landmarks(self, hand_landmarks, w, h):
        pts = []
        for lm in hand_landmarks.landmark:
            pts.append([lm.x * w, lm.y * h, lm.z])
        return np.array(pts, dtype=np.float32)

    def _is_finger_open(self, pts, tip, pip, h):
        # tip이 pip보다 '여유 마진'만큼 위에 있어야 펼침으로 간주
        margin_y = self.margin_ratio * h
        return pts[tip, 1] < pts[pip, 1] - margin_y


    def _gesture_from_landmarks(self, pts, handedness_label, w, h):
        # 3-1) 손가락 펼침 판정(ROI 비율 마진 적용)
        fingers_open = []
        for tip, pip in zip(self.TIPS[1:], self.PIPS[1:]):  # index~pinky
            fingers_open.append(self._is_finger_open(pts, tip, pip, h))

        # 3-2) 엄지 펼침(좌우 + 약간의 수직 마진 보강)
        margin_x = self.margin_ratio * w
        margin_y = self.margin_ratio * h
        if handedness_label == 'Right':
            thumb_open = (pts[4, 0] < pts[3, 0] - margin_x)  # 오른손: 왼쪽으로 벌어지면 펼침
        else:
            thumb_open = (pts[4, 0] > pts[3, 0] + margin_x)  # 왼손: 오른쪽으로 벌어지면 펼침

        open_count = sum(fingers_open) + int(thumb_open)

        # 3-3) 빠른 결정: FIST / OPEN
        if open_count <= 1:
            return "FIST"
        if open_count >= 4:
            # FOUR도 여기서 커버되지만, 명시 라벨이 필요하면 아래에 분기 추가 가능
            return "OPEN"

        # 3-4) THUMBS_UP / DOWN (수직 성분으로 보강)
        wrist = pts[0]
        thumb_vec_y = pts[4, 1] - wrist[1]  # 음수=위, 양수=아래 (영상 좌표)
        non_thumb_open = sum(fingers_open)  # index~pinky
        if thumb_open and non_thumb_open <= 1:
            if thumb_vec_y < - (2.5 * margin_y):   # 충분히 '위'면 UP
                return "Shaka"
            if thumb_vec_y >   (2.5 * margin_y):   # 충분히 '아래'면 DOWN
                return "THUMBS_DOWN"

        # 3-5) V_SIGN (기존 유지)
        if fingers_open[0] and fingers_open[1] and (not fingers_open[2]) and (not fingers_open[3]):
            return "V_SIGN"

        # 3-6) THREE (엄지 제외: index/middle/ring=열림, pinky=닫힘)
        if fingers_open[0] and fingers_open[1] and fingers_open[2] and (not fingers_open[3]):
            return "THREE"

        # 3-7) FOUR (엄지 제외: index~pinky 모두 열림)
        if all(fingers_open):
            return "FOUR"

        return "UNKNOWN"


    def infer(self, roi_bgr, person_id):
        """
        상반신 ROI(BGR) → 제스처 라벨(확정 시) 또는 None
        - 내부적으로 프레임 스킵 + 다수결 + 디바운스 적용
        """
        if roi_bgr is None or roi_bgr.size == 0:
            return None

        # 연산 절약: N프레임마다만 처리
        if self.frame_skip < self.process_every_n - 1:
            self.frame_skip += 1
            # 최근 다수결 반환(없으면 None)
            hist = self.history[person_id]
            if len(hist) == 0:
                return None
            vals, cnts = np.unique(hist, return_counts=True)
            return vals[np.argmax(cnts)]
        self.frame_skip = 0

        img_rgb = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2RGB)
        res = self.hands.process(img_rgb)
        h, w = roi_bgr.shape[:2]
        curr_label = None

        if res.multi_hand_landmarks and res.multi_handedness:
            # 가장 확실한 한 손만 사용
            hand_lms = res.multi_hand_landmarks[0]
            handed = res.multi_handedness[0].classification[0].label  # 'Left'/'Right'
            pts = self._np_landmarks(hand_lms, w, h)
            curr_label = self._gesture_from_landmarks(pts, handed, w, h)


        if curr_label:
            self.history[person_id].append(curr_label)

        hist = self.history[person_id]
        if len(hist) == 0:
            return None

        if res.multi_hand_landmarks and res.multi_handedness:
            hand_lms = res.multi_hand_landmarks[0]
            handed = res.multi_handedness[0].classification[0].label
            pts = self._np_landmarks(hand_lms, w, h)
            curr_label = self._gesture_from_landmarks(pts, handed, w, h)

            # ✅ 추가: ROI 이미지(roi_bgr)에 '모든' 손 랜드마크를 직접 그리기
            for hlm in res.multi_hand_landmarks:
                mp.solutions.drawing_utils.draw_landmarks(
                    roi_bgr, hlm, mp.solutions.hands.HAND_CONNECTIONS
                )


        # 다수결
        vals, cnts = np.unique(hist, return_counts=True)
        voted = vals[np.argmax(cnts)]

        # 디바운스: 같은 라벨이 hold_frames 연속이면 확정
        if len(hist) >= self.hold_frames and all(
            l == voted for l in list(hist)[-self.hold_frames:]
        ):
            return voted
        return None

