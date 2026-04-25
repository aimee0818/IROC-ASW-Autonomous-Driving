# lane_detection_dual_yellow2.py
import cv2
import numpy as np
import collections
from dataclasses import dataclass


# 최근 목표 중심 / 차로 폭 이력(평활화용)
target_cx_history = collections.deque([320]*7, maxlen=7)
lane_width_history = collections.deque([], maxlen=10)  # 최근 좌/우 간격(px)


@dataclass
class Params:
    # ROI 비율(세로 방향): 노면 하부만 사용해 노이즈/연산량 감소
    y1_ratio: float = 0.62
    y2_ratio: float = 0.92

    # 모폴로지(노이즈 제거)
    morph_kernel: int = 3
    morph_iters: int = 1

    # 노란색 HSV 범위(테이프/바닥 조명에 맞춰 조정)
    yellow_h_low: int = 0
    yellow_h_high: int = 60
    yellow_s_min: int = 0
    yellow_v_min: int = 180

    # 동적 임계: 조도 변화가 크면 사용
    use_adaptive_vmin: bool = False
    vmin_percentile: int = 60  # ROI V의 60% 백분위 이상만 유효

    # 성능 옵션
    resize_width: int | None = 640  # None이면 원본 유지


P = Params()


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


def _yellow_mask(roi_hsv):
    lower = np.array([P.yellow_h_low, P.yellow_s_min, P.yellow_v_min], dtype=np.uint8)
    upper = np.array([P.yellow_h_high, 255, 255], dtype=np.uint8)
    mask = cv2.inRange(roi_hsv, lower, upper)

    if P.use_adaptive_vmin:
        # V 60% 백분위 아래는 제거 → 어두운 바닥/반사 제거
        V = roi_hsv[..., 2]
        dyn_v = np.percentile(V, P.vmin_percentile)
        mask = cv2.bitwise_and(mask, (V >= max(P.yellow_v_min, int(dyn_v))).astype(np.uint8) * 255)

    # 모폴로지로 잡음 제거
    k = P.morph_kernel if P.morph_kernel % 2 == 1 else P.morph_kernel + 1
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (k, k))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=P.morph_iters)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=P.morph_iters)
    return mask


def _center_from_mask(mask):
    col = mask.sum(axis=0)
    if col.max() == 0:
        return None
    return int(np.argmax(col))


def _split_centers(y_mask, prev_target):
    h, w = y_mask.shape[:2]
    half = w // 2
    left = _center_from_mask(y_mask[:, :half])
    right = _center_from_mask(y_mask[:, half:])
    if right is not None:
        right += half

    # 무효화 규칙(이전 중심 대비 좌/우 일관성)
    if left is not None and left > prev_target:
        left = None
    if right is not None and right < prev_target:
        right = None
    return left, right


def drive_with_dual_yellow(image, pid_control, show_debug=False):
    global target_cx_history, lane_width_history

    # 1) 전처리
    img, scale = _maybe_resize(image)
    h, w = img.shape[:2]
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    roi = _roi(hsv)

    # 2) 마스크 & 중심
    y_mask = _yellow_mask(roi)
    prev_target = int(target_cx_history[-1] * scale)
    left_y, right_y = _split_centers(y_mask, prev_target)

    # 3) 타겟 결정
    mode = "normal"
    if left_x is not None and right_x is not None:
        target_cx = (left_x + right_x) // 2
        lane_width_history.append(right_x - left_x)
    elif left_x is not None:
        mode = "augmented"
        if lane_width_history:
            est = left_x + int(np.median(lane_width_history) // 2)
            target_cx = est
        else:
            target_cx = prev_target
    elif right_x is not None:
        mode = "augmented"
        if lane_width_history:
            est = right_x - int(np.median(lane_width_history) // 2)
            target_cx = est
        else:
            target_cx = prev_target
    else:
        mode = "augmented"
        target_cx = prev_target

    # 4) 히스토리 갱신 & PID 호출
    target_cx_history.append(int(target_cx))
    angle, speed = pid_control(int(target_cx), w, mode=mode)

    # 5)디버그 시각화
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

        cv2.circle(debug, (int(target_cx), midy), 10, (255, 0, 0), 2)
        cv2.putText(debug, f"Mode:{mode}", (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255), 2)
        cv2.imshow("Lane Debug (Dual-Yellow unified)", debug)
        cv2.waitKey(1)

    # 6 리턴
    return (angle, speed), int(target_cx/scale), 1

    # 원본 해상도로 환산
    # ret_target = int(target_cx / scale)
    # current_lane = 1  # 1/2 구분 의미가 약해, 더미로 1
    # return (angle, speed), ret_target, current_lane


# --- Standalone demo ---------------------------------------------------------
# 카메라 테스트만 하고 싶을 때: python3 lane_detection_dual_yellow.py
# 'q' 로 종료
if __name__ == "__main__":
    def _dummy_pid(cx, width, mode="normal"):
        err = cx - width // 2
        steer = 90 + err * 0.05
        steer = max(45, min(135, steer))
        return int(steer), 50

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("카메라를 열 수 없습니다.")
        raise SystemExit

    print("Dual-Lane (single_right unified) test. Press 'q' to quit.")
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        (_, _), target, _ = drive_with_dual_yellow(frame, _dummy_pid, show_debug=True)
        if 0 <= target < frame.shape[1]:
            cv2.line(frame, (int(target), 0), (int(target), frame.shape[0]), (255, 0, 255), 2)
        cv2.imshow("Camera", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()
