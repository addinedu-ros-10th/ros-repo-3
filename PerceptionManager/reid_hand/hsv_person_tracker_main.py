# hsv_person_tracker.py
import cv2
import numpy as np
from ultralytics import YOLO
import os
from datetime import datetime
import time

from hand_gesture import HandGestureRecognizer
from frame_grabber import FrameGrabber
import tcp_client   # 새로 분리한 TCP 모듈

# Qt 플랫폼 플러그인 오류 방지
os.environ['QT_QPA_PLATFORM'] = 'xcb'
os.environ['QT_DEBUG_PLUGINS'] = '0'
os.environ['QT_AUTO_SCREEN_SCALE_FACTOR'] = '0'
os.environ['QT_SCALE_FACTOR'] = '1'


# --- HSV 기반 ReID + 제스처 인식 ---
class HSVAnalyzer:
    def __init__(self):
        # YOLO 세그멘테이션(사람 클래스)
        self.model = YOLO('yolov8s-seg.pt')

        # 사람별 HSV 히스토그램 / bbox 저장
        # people_data[pid] = {'histograms': [np.array], 'bboxes': [(x1,y1,x2,y2), ...]}
        self.people_data = {}
        self.next_id = 0

        # 매칭 관련 설정
        self.match_threshold = 0.35           # ReID 점수 임계
        self.min_detection_confidence = 0.6   # YOLO confidence
        self.min_person_area = 5000           # 너무 작은 사람은 무시
        self.max_histograms_per_person = 13   # 한 사람당 히스토그램 기억 개수

        # 제스처 인식기 (MediaPipe 유지)
        self.gesture = HandGestureRecognizer(
            max_hands=1, det_conf=0.6, track_conf=0.5,
            smooth_window=5, hold_frames=3, process_every_n=3
        )
        # 사람별 손 제스처 상태
        # person_action_state[pid] = {'gesture': str, 'last_action': str, 'ts': float}
        self.person_action_state = {}

        # 제스처 처리 조건
        self.upper_body_ratio = 0.6       # bbox 상단 60%만 상반신 ROI
        self.gesture_every_n_frames = 3   # 제스처 추론 주기
        self._gesture_frame_mod = 0

        # 제스처 인식 대상으로 고정할 사람 ID
        self.target_person_id = "Person_0"

        # Person_0 템플릿(EMA)
        self.target_template = {
            "hist": None,   # np.ndarray(48,) - HSV 히스토그램
            "bbox": None,   # 마지막 bbox
        }
        self.template_alpha = 0.2          # EMA 가중(0.1~0.3)
        self.template_match_boost = 0.15   # Person_0일 때 점수 보정

        # 호출 상태
        self.call_state = False
        self.action = None

        self.recognition = True

    # --- 히스토그램 & 유사도 ---
    def extract_histogram(self, img, mask, bins=16):
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        h_hist = cv2.calcHist([hsv], [0], mask, [bins], [0, 180])
        s_hist = cv2.calcHist([hsv], [1], mask, [bins], [0, 256])
        v_hist = cv2.calcHist([hsv], [2], mask, [bins], [0, 256])

        h_hist = cv2.normalize(h_hist, h_hist).flatten()
        s_hist = cv2.normalize(s_hist, s_hist).flatten()
        v_hist = cv2.normalize(v_hist, v_hist).flatten()

        combined_hist = np.concatenate([h_hist, s_hist, v_hist])
        return combined_hist

    def calculate_similarity_metrics(self, hist1, hist2):
        h1 = hist1.astype(np.float32)
        h2 = hist2.astype(np.float32)

        bhatt_dist = cv2.compareHist(h1, h2, cv2.HISTCMP_BHATTACHARYYA)
        dot_product = float(np.dot(h1, h2))
        norm1 = float(np.linalg.norm(h1))
        norm2 = float(np.linalg.norm(h2))
        cosine_sim = dot_product / (norm1 * norm2) if norm1 > 0 and norm2 > 0 else 0.0
        return bhatt_dist, cosine_sim

    def _hist_similarity(self, h1, h2):
        bhatt, cos = self.calculate_similarity_metrics(h1, h2)
        return max(1.0 - bhatt, cos)

    # --- Person_0 템플릿 업데이트 ---
    def _update_target_template(self, pid, hist, bbox):
        if pid != self.target_person_id:
            return
        if self.target_template["hist"] is None:
            self.target_template["hist"] = hist.copy()
        else:
            self.target_template["hist"] = (
                (1 - self.template_alpha) * self.target_template["hist"]
                + self.template_alpha * hist
            )
        self.target_template["bbox"] = bbox

    # --- ReID 매칭 ---
    def find_best_match(self, current_hist, current_bbox, used_ids):
        best_match_id = None
        best_score = 0.0

        x1, y1, x2, y2 = current_bbox
        current_center = ((x1 + x2) // 2, (y1 + y2) // 2)

        for pid, pdata in self.people_data.items():
            if pid in used_ids:
                continue
            if len(pdata['histograms']) == 0:
                continue

            # 최근 히스토그램들 중에서 최고 유사도
            hist_scores = []
            for stored_hist in pdata['histograms'][-self.max_histograms_per_person:]:
                sim = self._hist_similarity(current_hist, stored_hist)
                hist_scores.append(sim)
            best_hist_score = max(hist_scores) if hist_scores else 0.0

            # 화면 위치(중심점 거리) 점수
            latest_bbox = pdata['bboxes'][-1]
            sx1, sy1, sx2, sy2 = latest_bbox
            stored_center = ((sx1 + sx2) // 2, (sy1 + sy2) // 2)
            center_distance = np.sqrt(
                (current_center[0] - stored_center[0]) ** 2 +
                (current_center[1] - stored_center[1]) ** 2
            )
            max_distance = np.sqrt(640 ** 2 + 480 ** 2)
            spatial_score = 1.0 - (center_distance / max_distance)

            total_score = 0.9 * best_hist_score + 0.1 * spatial_score

            # Person_0이면 템플릿 유사도로 추가 가산점
            if self.target_template["hist"] is not None and pid == self.target_person_id:
                template_sim = self._hist_similarity(current_hist, self.target_template["hist"])
                total_score += self.template_match_boost * template_sim

            if total_score > best_score:
                best_score = total_score
                best_match_id = pid

        return best_match_id, best_score

    # --- 메인 루프 ---
    def run_analysis(self):
        grabber = FrameGrabber(src=0, width=960, height=540, fps=30, fourcc='MJPG').start()
        if grabber.read() is None:
            print("❌ 카메라 연결 실패")
            grabber.stop()
            return

        frame_count = 0
        send = 0
        start_time = datetime.now()
        frame_skip_counter = 0
        process_every_n_frames = 3

        try:
            while True:
                frame = grabber.read()
                if frame is None:
                    time.sleep(0.005)
                    continue

                frame_count += 1
                elapsed_time = (datetime.now() - start_time).total_seconds()

                # 프레임 스킵으로 YOLO 부하 감소
                if frame_skip_counter < process_every_n_frames - 1:
                    frame_skip_counter += 1
                    continue
                frame_skip_counter = 0

                annotated = frame.copy()

                # --- 사람 감지 (세그멘테이션) ---
                results = self.model(annotated, classes=[0], imgsz=480, verbose=False)
                current_detections = []

                for result in results:
                    if result.masks is None or result.boxes is None:
                        continue

                    masks = result.masks.data
                    boxes = result.boxes

                    for i in range(len(boxes)):
                        box = boxes[i]
                        confidence = float(box.conf[0].item())
                        if confidence < self.min_detection_confidence:
                            continue

                        seg = masks[i]
                        mask = seg.cpu().numpy().astype(np.uint8) * 255
                        mask_resized = cv2.resize(
                            mask,
                            (frame.shape[1], frame.shape[0]),
                            interpolation=cv2.INTER_NEAREST
                        )

                        # 모폴로지: 큰 대상만 한 번 정리
                        person_pixels = cv2.countNonZero(mask_resized)
                        if person_pixels > 3000:
                            kernel = np.ones((3, 3), np.uint8)
                            mask_cleaned = cv2.morphologyEx(
                                mask_resized, cv2.MORPH_CLOSE, kernel, iterations=1
                            )
                        else:
                            mask_cleaned = mask_resized

                        # HSV 히스토그램 추출
                        combined_hist = self.extract_histogram(frame, mask_cleaned)

                        bbox = box.xyxy[0].cpu().numpy()
                        x1, y1, x2, y2 = bbox
                        area = (x2 - x1) * (y2 - y1)
                        if area < self.min_person_area:
                            continue

                        current_detections.append({
                            'hist': combined_hist,
                            'bbox': bbox,
                            'mask': mask_cleaned,
                            'confidence': confidence,
                            'area': area,
                        })

                # 큰 사람부터 처리
                current_detections.sort(key=lambda x: x['area'], reverse=True)
                used_ids = set()

                for detection in current_detections:
                    combined_hist = detection['hist']
                    bbox = detection['bbox']

                    matched_id, match_score = self.find_best_match(
                        combined_hist, bbox, used_ids
                    )

                    # 매칭 성공
                    if matched_id is not None and match_score > self.match_threshold:
                        pdata = self.people_data[matched_id]
                        pdata['histograms'].append(combined_hist)
                        pdata['bboxes'].append(bbox)
                        used_ids.add(matched_id)

                        # 오래된 기록 삭제
                        if len(pdata['histograms']) > self.max_histograms_per_person:
                            pdata['histograms'].pop(0)
                            pdata['bboxes'].pop(0)

                        pid = matched_id
                        color = (0, 255, 0)
                    else:
                        # 새 사람 등록
                        pid = f"Person_{self.next_id}"
                        self.people_data[pid] = {
                            'histograms': [combined_hist],
                            'bboxes': [bbox],
                        }

                        if self.recognition:
                            # 🔹 TCP 모듈로 분리된 함수 호출
                            tcp_client.user_result()

                        self.recognition = False
                        self.next_id += 1
                        used_ids.add(pid)
                        color = (0, 0, 255)

                    # Person_0 템플릿 업데이트
                    self._update_target_template(pid, combined_hist, bbox)

                    x1, y1, x2, y2 = map(int, bbox)
                    cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)
                    cv2.putText(
                        annotated, pid,
                        (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2
                    )

                    # ---------- 제스처 인식 (Person_0만) ----------
                    self._gesture_frame_mod = (self._gesture_frame_mod + 1) % self.gesture_every_n_frames
                    if pid != self.target_person_id:
                        continue  # 타깃이 아니면 제스처 스킵

                    do_gesture_now = (self._gesture_frame_mod == 0)
                    if do_gesture_now:
                        roi_top = y1
                        roi_bottom = y1 + int((y2 - y1) * self.upper_body_ratio)
                        roi_left, roi_right = x1, x2

                        roi_top = max(0, roi_top)
                        roi_bottom = min(frame.shape[0], roi_bottom)
                        roi_left = max(0, roi_left)
                        roi_right = min(frame.shape[1], roi_right)

                        upper_roi = annotated[roi_top:roi_bottom, roi_left:roi_right]

                        label = self.gesture.infer(upper_roi, pid)
                        if label:
                            prev = self.person_action_state.get(
                                pid, {'gesture': None, 'last_action': None, 'ts': 0.0}
                            )

                            # ① 호출 상태가 아닌데 'FARTHEST'이면 → 호출 시작
                            if not self.call_state and label == "Shaka":
                                self.action = "Call"
                                self.call_state = True
                                send = 1
                                print("[CALL MODE ON] 호출됨")

                            # ② 호출 상태일 때만 다른 제스처를 유효하게 해석
                            if self.call_state:
                                if label == "OK_SIGN":
                                    self.action = "RESTART";   self.call_state = False
                                elif label == "V_SIGN":
                                    self.action = "FOLLOWING";   self.call_state = False
                                    send = 3;  tcp_client.send_detection_result(2, 3, send, 0, 0, 0)
                                elif label == "THREE":
                                    self.action = "GUIDING";     self.call_state = False
                                    send = 4;  tcp_client.send_detection_result(2, 3, send, 0, 0, 0)
                                elif label == "OPEN":
                                    self.action = "QUIT";      self.call_state = False
                                    send = 5;  tcp_client.send_detection_result(2, 3, send, 0, 0, 0)
                                elif label == "FIST":
                                    self.action = "STOP";      self.call_state = False
                                    send = 2;  tcp_client.send_detection_result(2, 3, send, 0, 0, 0)

                            new_gesture = label
                            new_action = self.action

                            if prev['gesture'] != new_gesture or prev['last_action'] != new_action:
                                self.person_action_state[pid] = {
                                    'gesture': new_gesture,
                                    'last_action': new_action,
                                    'ts': elapsed_time,
                                }

                    # --- 텍스트 표시 (항상 마지막 상태 기준) ---
                    state = self.person_action_state.get(pid)
                    if state and state.get('last_action') is not None:
                        cv2.putText(
                            annotated,
                            f"Hand: {state['gesture']}, CALL: {self.call_state}",
                            (x1, y1 - 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                            (0, 255, 255), 2
                        )
                        cv2.putText(
                            annotated,
                            f"Action: {state['last_action']}",
                            (x1, y1 - 50),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.9,
                            (0, 0, 255), 2
                        )

                # --- 화면 정보 표시 ---
                info_text = f"People:{len(self.people_data)} | Frame:{frame_count} | Time:{elapsed_time:.1f}s"
                cv2.putText(
                    annotated, info_text,
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                    (0, 255, 255), 2
                )

                # 표시(부하 줄이려면 resize 해서 보여줘도 됨)
                if frame_count % 5 == 0:
                    cv2.imshow("HSV ReID + Person_0 Gesture", annotated)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break

        finally:
            grabber.stop()
            cv2.destroyAllWindows()

        print(f"\n📊 종료! 사람 수: {len(self.people_data)}")


if __name__ == "__main__":
    # 1) TCP 연결 스레드 시작
    tcp_client.start_tcp_thread()

    # 2) 분석 시작
    analyzer = HSVAnalyzer()
    analyzer.run_analysis()
