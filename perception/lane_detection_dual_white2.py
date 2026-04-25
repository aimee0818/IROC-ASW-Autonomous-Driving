# lane_detection_dual_white2.py

import cv2
import numpy as np
import collections
from dataclasses import dataclass

"""
Dual-White Lane Detection for track with white dotted lanes
- 양쪽 흰색 점선/실선 기반 주행 전용 버전
- 좌/우 흰색 차선을 각각 검출해서 두 선의 중간을 주행 중심으로 설정
- 한쪽만 보이거나 점선이 끊겨도 최근 중심/폭을 이용해 보정
인터페이스: drive_with_dual_white(image, pid_control, show_debug)
"""

# --- 최근 중심 및 차로폭 이력 (평활화용) ---
target_cx_history = collections.deque([320]*7, maxlen=7)
lane_width_history = collections.deque([], maxlen=10)

@dataclass
class Params:
    # ROI (하단 일부만 사용)
    # 원래 0.60~0.92 이었는데, 반사/노이즈 줄이려고 더 아래만 보도록 올림
    y1_ratio: float = 0.62
    y2_ratio: float = 0.92

    # 모폴로지 커널 (노이즈 제거)
    # 커널이 너무 크면 점선조각들이 전부 이어져서
    # 중앙에 큰 흰 blob처럼 나오는 문제 -> 3으로 축소
    morph_kernel: int = 3
    morph_iters: int = 1

    # 흰색 HSV 범위 (엄격하게)
    # - 채도(S) 낮아야 함 (차선색은 거의 무채색)
    # - 밝기(V) 높아야 함 (바닥보다 확실히 밝음)
    # - V를 더 높게, S를 더 낮게
    white_h_low: int = 0
    white_h_high: int = 180
    white_s_min: int = 0      # 기존 70 → 40으로 더 엄격
    white_v_min: int = 190   # 기존 160 → 210으로 상향

    # 동적 밝기(환경광 따라 적응)
    use_adaptive_vmin: bool = False
    vmin_percentile: int = 65  # ROI 밝기 중 상위 구간만 사용

    # 크기 조정 (성능용)
    resize_width: int | None = 640

    # 중앙 쓰레기 억제 파라미터
    # prev_target 근처(차 전체 중앙 근처)에 뜬 하얀 반사를 차선으로 착각하지 않기 위한 margin
    center_margin_ratio: float = 0.12  # 화면 폭의 약 12% 정도는 "중앙 금지 구간"으로 취급

P = Params()


# --- 유틸 ---
def _maybe_resize(img):
    if P.resize_width is None:
        return img, 1.0
    h, w = img.shape[:2]
    if w == P.resize_width:
        return img, 1.0
    scale = P.resize_width / float(w)
    resized = cv2.resize(img, (P.resize_width, int(h * scale)), interpolation=cv2.INTER_AREA)
    return resized, scale


def _roi(hsv):
    h = hsv.shape[0]
    y1 = int(h * P.y1_ratio)
    y2 = int(h * P.y2_ratio)
    return hsv[y1:y2, :]


def _white_mask(roi_hsv):
    """밝은 흰색 계열 차선 검출 (dual_yellow2 기반)"""
    lower = np.array([P.white_h_low, P.white_s_min, P.white_v_min], dtype=np.uint8)
    upper = np.array([P.white_h_high, 20, 255], dtype=np.uint8)
    mask = cv2.inRange(roi_hsv, lower, upper)

    if P.use_adaptive_vmin:
        V = roi_hsv[..., 2]
        dyn_v = np.percentile(V, P.vmin_percentile)
        mask = cv2.bitwise_and(mask, (V >= max(P.white_v_min, int(dyn_v))).astype(np.uint8) * 255)

    # 모폴로지 정제
    k = P.morph_kernel if P.morph_kernel % 2 == 1 else P.morph_kernel + 1
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (k, k))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=P.morph_iters)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=P.morph_iters)
    return mask


def _center_from_mask(mask):
    """
    세로로 projection해서 가장 강한 열(column)을 반환.
    """
    col_sum = mask.sum(axis=0)
    if col_sum.max() == 0:
        return None
    return int(np.argmax(col_sum))


def _split_centers(mask, prev_target):
    h, w = mask.shape[:2]
    half = w // 2
    left = _center_from_mask(mask[:, :half])
    right = _center_from_mask(mask[:, half:])
    if right is not None:
        right += half

    if left is not None and left > prev_target:
        left = None
    if right is not None and right < prev_target:
        right = None
    return left, right


def drive_with_dual_white(image, pid_control, show_debug=False):
    """양쪽 흰색 차선을 기반으로 주행 중심 계산"""
    global target_cx_history, lane_width_history

    # 1️⃣ 전처리
    img, scale = _maybe_resize(image)
    h, w = img.shape[:2]
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    roi = _roi(hsv)

    # 2️⃣ 흰색 마스크
    mask = _white_mask(roi)

    # 3️⃣ 이전 중심
    prev_target = int(target_cx_history[-1] * scale)

    # 4️⃣ 좌/우 차선 검출
    left_x, right_x = _split_centers(mask, prev_target)

    # 5️⃣ 중심 계산
    mode = "normal"
    if left_x is not None and right_x is not None:
        target_cx = (left_x + right_x) // 2
        lane_width_history.append(right_x - left_x)
    elif left_x is not None:
        mode = "augmented"
        if lane_width_history:
            target_cx = left_x + int(np.median(lane_width_history) // 2)
        else:
            target_cx = prev_target
    elif right_x is not None:
        mode = "augmented"
        if lane_width_history:
            target_cx = right_x - int(np.median(lane_width_history) // 2)
        else:
            target_cx = prev_target
    else:
        mode = "augmented"
        target_cx = prev_target

    # 6️⃣ PID 제어
    target_cx_history.append(int(target_cx))
    angle, speed = pid_control(int(target_cx), w, mode=mode)

    # 7️⃣ 디버그 시각화
    if show_debug:
        dbg_h, dbg_w = mask.shape[:2]
        debug = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        midy = dbg_h // 2

        if left_x is not None:
            cv2.circle(debug, (left_x, midy), 7, (255,255,255), -1)
            cv2.putText(debug, "LW", (left_x - 20, midy - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)
        if right_x is not None:
            cv2.circle(debug, (right_x, midy), 7, (255,255,255), -1)
            cv2.putText(debug, "RW", (right_x - 20, midy - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)
        cv2.circle(debug, (int(target_cx), midy), 10, (255,0,0), 2)
        cv2.putText(debug, f"Mode:{mode}", (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255), 2)
        cv2.imshow("Lane Debug (Dual-White unified)", debug)
        cv2.waitKey(1)

    # 8️⃣ 결과 반환
    return (angle, speed), int(target_cx/scale), 1


# --- Standalone test ---
if __name__ == "__main__":
    def _dummy_pid(cx, width, mode="normal"):
        err = cx - width // 2
        steer = 90 + err * 0.05
        steer = max(45, min(135, steer))
        return int(steer), 50

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("❌ Camera open failed")
        raise SystemExit

    print("Dual-White lane test. Press 'q' to quit.")
    while True:
        ok, frame = cap.read()
        if not ok:
            break

        (_, _), target, _ = drive_with_dual_white(frame, _dummy_pid, show_debug=True)

        if 0 <= target < frame.shape[1]:
            cv2.line(frame,
                     (int(target), 0),
                     (int(target), frame.shape[0]),
                     (255, 0, 255), 2)

        cv2.imshow("Camera", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
