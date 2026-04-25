# lane_drive_single.py — 완전 자동형 단일 차선 주행 (Left / Right)
import cv2, numpy as np, collections
from dataclasses import dataclass


# ====== 기본 설정 ======
BASE_SPEED = 50
STRAIGHT_ANGLE = 90
TURN_GAIN = 0.4

# PID 내부 상태
_prev_error, _integral = 0, 0
lost_counter = 0
target_cx_history = collections.deque([320]*5, maxlen=7)

@dataclass
class Params:
    y1_ratio: float = 0.67
    y2_ratio: float = 0.92
    morph_kernel: int = 3
    morph_iters: int = 1
    yellow_h_low: int = 0
    yellow_h_high: int = 60
    yellow_s_min: int = 0
    yellow_v_min: int = 190
    use_adaptive_vmin: bool = False
    vmin_percentile: int = 60
    resize_width: int | None = 640
    offset_ratio: float = 0.48

P = Params()

# ===== 유틸 =====
def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def send_values(ser, L, R, angle):
    """시리얼 전송"""
    try:
        ser.write(f"{L},{R},{angle}\n".encode())
        ser.flush()
    except Exception as e:
        print(f"⚠️ Serial send error: {e}")

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
    return hsv[int(h*P.y1_ratio):int(h*P.y2_ratio), :]

def _yellow_mask(hsv):
    """
    [3] HSV 색공간에서 노란색(또는 지정색상) 차선을 검출.
    - 조명 변화 대응을 위해 adaptive threshold 적용.
    - 모폴로지 연산으로 작은 노이즈 제거 및 끊긴 선 보정.
    """
    # 1) 색상 범위로 마스크 생성
    lower = np.array([P.yellow_h_low, P.yellow_s_min, P.yellow_v_min], dtype=np.uint8)
    upper = np.array([P.yellow_h_high, 255, 255], dtype=np.uint8)
    mask = cv2.inRange(hsv, lower, upper)

    # 2) 동적 밝기 필터링 (ROI의 밝기 백분위 기반)
    if P.use_adaptive_vmin:
        V = hsv[...,2]
        dyn_v = np.percentile(V, P.vmin_percentile)
        mask = cv2.bitwise_and(mask, (V >= max(P.yellow_v_min, int(dyn_v))).astype(np.uint8)*255)

    # 3) 모폴로지 정제 (열고 닫기 연산)
    k = P.morph_kernel if P.morph_kernel % 2 == 1 else P.morph_kernel + 1
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (k, k))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=P.morph_iters)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=P.morph_iters)
    return mask

def _find_left_lane(mask):
    h, w = mask.shape[:2]
    col_sum = mask.sum(axis=0)
    half = w // 2
    left_region = col_sum[:half]
    if left_region.max() == 0:
        return None
    return int(np.argmax(left_region))

def _find_right_lane(mask):
    h, w = mask.shape[:2]
    col_sum = mask.sum(axis=0)
    half = w // 2
    right_region = col_sum[half:]
    if right_region.max() == 0:
        return None
    return int(np.argmax(right_region) + half)

def pid_control(cx, width, mode="normal"):
    """차선 중심 기반 PID 조향"""
    global _prev_error, _integral
    center = width // 2
    error = cx - center
    Kp, Kd, Ki = 0.41, 0.14, 0.0
    _integral += error
    derivative = error - _prev_error
    _prev_error = error
    steer = 90 + (Kp * error + Kd * derivative + Ki * _integral)
    return int(clamp(steer, 45, 135)), BASE_SPEED

# =====================================================
# 왼쪽 차선 기반 주행
# =====================================================
def drive_with_single_left(ser, frame, show_debug=False):
    global target_cx_history, lane_width_history, lost_counter
    img, scale = _maybe_resize(frame)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    roi = _roi(hsv)
    mask = _yellow_mask(roi)
    w = img.shape[1]
    prev_cx = int(target_cx_history[-1] * scale)
    left_x = _find_left_lane(mask)

    mode = "normal"
    if left_x is not None:
        # 왼쪽 차선이 보일 경우 → 정상 주행
        target_cx = int(left_x + w * P.offset_ratio)
        lost_counter = 0  # 라인 탐색 상태 리셋
    else:
        # 왼쪽 차선이 안보일 경우 → 재탐색 모드
        mode = "augmented"

        # 라인 탐색 카운터를 증가시켜서 일정 시간동안만 탐색 동작 수행
        lost_counter += 1

        if lost_counter < 300:
            # 최근 중심에서 살짝 왼쪽으로 회전하도록 오프셋 보정
            # offset 크기는 화면 폭의 5~8% 정도가 적당
            target_cx = int(prev_cx - w * 0.002)
        else:
            # 너무 오래 안보이면 다시 중앙 복귀
            target_cx = prev_cx

    target_cx_history.append(int(target_cx))
    angle, speed = pid_control(target_cx, w, mode=mode)

    # PWM 계산 및 전송
    steer_offset = angle - 90
    steer_ratio = steer_offset / 45.0
    L_pwm = int(speed * (1.0 - TURN_GAIN * steer_ratio))
    R_pwm = int(speed * (1.0 + TURN_GAIN * steer_ratio))
    send_values(ser, clamp(L_pwm, -255, 255), clamp(R_pwm, -255, 255), angle)

    if show_debug:
        debug = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        midy = debug.shape[0]//2

        if left_x is not None:
            cv2.circle(debug, (left_x, midy), 6, (0,255,255), -1)
            cv2.putText(debug, "LW", (left_x-25, midy-15), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255),2)

        cv2.circle(debug, (int(target_cx), midy), 10, (255,0,0),2)
        cv2.putText(debug, f"Mode:{mode}", (10,25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255),2)
        cv2.imshow("Left-Line Debug", debug)
        cv2.waitKey(1)

    # 10️⃣ 결과 반환 (원본 스케일로 환산)
    return #(angle, speed), int(target_cx/scale), 1  # lane_type=1(dummy)

# =====================================================
# 오른쪽 차선 기반 주행
# =====================================================
def drive_with_single_right(ser, frame, show_debug=False):
    global target_cx_history, lane_width_history, lost_counter
    img, scale = _maybe_resize(frame)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    roi = _roi(hsv)
    mask = _yellow_mask(roi)
    w = img.shape[1]
    prev_cx = int(target_cx_history[-1] * scale)
    right_x = _find_right_lane(mask)

    mode = "normal"
    if right_x is not None:
        target_cx = int(right_x - w * P.offset_ratio)
        lost_counter = 0
    else:
        mode = "augmented"
        lost_counter += 1

        if lost_counter < 300:
            # 오른쪽 차선이 사라졌을 때 → 오른쪽으로 살짝 보정
            target_cx = int(prev_cx + w * 0.002)
        else:
            target_cx = prev_cx

    target_cx_history.append(int(target_cx))
    angle, speed = pid_control(target_cx, w, mode=mode)

    # PWM 계산 및 전송
    steer_offset = angle - 90
    steer_ratio = steer_offset / 45.0
    L_pwm = int(speed * (1.0 - TURN_GAIN * steer_ratio))
    R_pwm = int(speed * (1.0 + TURN_GAIN * steer_ratio))
    send_values(ser, clamp(L_pwm, -255, 255), clamp(R_pwm, -255, 255), angle)

    # 9️⃣ 디버그 시각화
    if show_debug:
        debug = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        midy = debug.shape[0]//2

        if right_x is not None:
            cv2.circle(debug, (right_x, midy), 6, (0,255,255), -1)
            cv2.putText(debug, "RW", (right_x-25, midy-15), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255),2)

        cv2.circle(debug, (int(target_cx), midy), 10, (255,0,0),2)
        cv2.putText(debug, f"Mode:{mode}", (10,25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255),2)
        cv2.imshow("Right-Line Debug", debug)
        cv2.waitKey(1)

    return #(angle, speed), int(target_cx/scale), 2  # lane_type=2(dummy)