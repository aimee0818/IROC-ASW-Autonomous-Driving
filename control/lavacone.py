# lavacone.py — LiDAR corridor-style PID with robust should_stop
import time
import math
import numpy as np

# ===== LiDAR 설정 =====
LIDAR_MIRROR       = True
LIDAR_YAW_OFFSET   = 180.0
LIDAR_SECTOR_FRONT = (-15.0, +15.0)
LIDAR_SECTOR_LEFT  = (-70.0, -25.0)
LIDAR_SECTOR_RIGHT = (+25.0, +70.0)
LIDAR_MAX_RANGE    = 15.0

# ===== PID 파라미터 =====
LIDAR_KP = 1.2
LIDAR_KD = 0.5

BASE_SPEED      = 50
STRAIGHT_ANGLE  = 90
DEBUG_INTERVAL  = 0.3

# ===== 내부 상태 =====
LIDAR_PREV_ERR = 0.0

# ===== 유틸 =====
def _wrap_deg(a):
    return (a + 180.0) % 360.0 - 180.0

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def as_valid_dist(d, fallback):
    return d if np.isfinite(d) else fallback

def min_in_sector(scan, deg_lo, deg_hi,
                  mirror=LIDAR_MIRROR, yaw_off=LIDAR_YAW_OFFSET,
                  max_range=LIDAR_MAX_RANGE):
    if scan is None or not getattr(scan, "ranges", None):
        return float('inf')
    a_min, da = scan.angle_min, scan.angle_increment
    lo, hi = (deg_lo, deg_hi) if deg_lo <= deg_hi else (deg_hi, deg_lo)
    best = float('inf')
    for i, d in enumerate(scan.ranges):
        if not (0.01 < d <= max_range):
            continue
        ang = math.degrees(a_min + i * da)
        if mirror:
            ang = -ang
        ang = _wrap_deg(ang + yaw_off)
        if lo <= ang <= hi and d < best:
            best = d
    return best

def mean_in_sector(scan, deg_lo, deg_hi,
                   mirror=LIDAR_MIRROR, yaw_off=LIDAR_YAW_OFFSET,
                   max_range=LIDAR_MAX_RANGE):
    if scan is None or not getattr(scan, "ranges", None):
        return float('inf')
    a_min, da = scan.angle_min, scan.angle_increment
    lo, hi = (deg_lo, deg_hi) if deg_lo <= deg_hi else (deg_hi, deg_lo)
    vals = []
    for i, d in enumerate(scan.ranges):
        if not (0.01 < d <= max_range):
            continue
        ang = math.degrees(a_min + i * da)
        if mirror:
            ang = -ang
        ang = _wrap_deg(ang + yaw_off)
        if lo <= ang <= hi:
            vals.append(d)
    return float(np.mean(vals)) if vals else float('inf')

def mirror_angle(angle, center=90, lo=45, hi=135, margin_if_oob=10):
    was_oob = (angle < lo) or (angle > hi)
    a_clamped = max(lo, min(hi, angle))
    if was_oob and margin_if_oob > 0:
        a_eff = min(hi - margin_if_oob, max(lo + margin_if_oob, a_clamped))
    else:
        a_eff = a_clamped
    mirrored = 2 * center - a_eff
    mirrored = max(lo, min(hi, mirrored))
    return int(round(mirrored))

def send_values(ser, L, R, angle):
    try:
        ser.write(f"{L},{R},{angle}\n".encode())
        ser.flush()
    except Exception as e:
        print(f"[lavacone] Serial send error: {e}")

# ============================================================
# 라바콘 복도 주행 (with EXIT 연동)
# ============================================================
def run(ser, lidar=None, show_debug=False, should_stop=None):
    """
    라바콘 복도 구간 주행
    - LiDAR 좌/우 거리 차이를 PID로 보정
    - should_stop()이 True(또는 'EXIT' 판단)일 때까지 유지
    """
    global LIDAR_PREV_ERR
    last_send = 0.0
    last_dbg = 0.0
    new_angle = STRAIGHT_ANGLE

    print("🟠 [LAVACONE] 라바콘 복도 주행 시작")

    while True:
        now = time.time()

        # ---- 종료 조건: EXIT 트리거 ----
        if callable(should_stop):
            try:
                stop_flag = bool(should_stop())
            except TypeError:
                # 혹시 인자를 기대하는 콜러블이면 lidar를 넘겨 재시도
                try:
                    stop_flag = bool(should_stop(lidar))
                except Exception:
                    stop_flag = False
            if stop_flag:
                print("✅ EXIT 감지 — 라바콘 주행 종료")
                break

        # ---- LiDAR 스캔 처리 ----
        scan = lidar.latest_scan if lidar is not None else None
        if scan is None:
            time.sleep(0.05)
            continue

        left_d  = min_in_sector(scan, *LIDAR_SECTOR_LEFT)
        right_d = min_in_sector(scan, *LIDAR_SECTOR_RIGHT)
        front_d = min_in_sector(scan, *LIDAR_SECTOR_FRONT)
        back_angle = mirror_angle(new_angle)

        left_d  = as_valid_dist(left_d,  0.1)
        right_d = as_valid_dist(right_d, 0.1)
        front_d = as_valid_dist(front_d, 0.15)

        # ---- PID 제어 ----
        err = left_d - right_d
        steer_delta = (LIDAR_KP * err) + (LIDAR_KD * (err - LIDAR_PREV_ERR))
        LIDAR_PREV_ERR = err
        new_angle = int(clamp(STRAIGHT_ANGLE - steer_delta * 70.0, 45, 135))

        # ---- 좌/우 속도 보정 ----
        pwm = BASE_SPEED
        steer_offset = new_angle - 90
        steer_ratio = steer_offset / 45.0
        turn_gain = 0.4
        L_pwm = int(clamp(pwm * (1.0 - turn_gain * steer_ratio), -255, 255))
        R_pwm = int(clamp(pwm * (1.0 + turn_gain * steer_ratio), -255, 255))

        # ---- 전방 너무 가까우면 후진 ----
        if front_d < 0.25:
            send_values(ser, 0, 0, new_angle)
            time.sleep(0.5)
            send_values(ser, 0, 0, back_angle)
            time.sleep(0.5)
            send_values(ser, -L_pwm, -R_pwm, back_angle)
            time.sleep(0.5)
            send_values(ser, 0, 0, new_angle)
            time.sleep(0.5)
            send_values(ser, L_pwm, R_pwm, new_angle)
            time.sleep(0.5)

        # ---- 명령 전송 ----
        if now - last_send > 0.05:
            send_values(ser, L_pwm, R_pwm, new_angle)
            last_send = now

        # ---- 디버그 출력 ----
        if show_debug and now - last_dbg > DEBUG_INTERVAL:
            print(f"[LAVACONE] L={L_pwm} R={R_pwm} angle={new_angle} | "
                  f"L={left_d:.2f} R={right_d:.2f} F={front_d:.2f} err={err:.2f}")
            last_dbg = now

        time.sleep(0.05)

    # ---- 종료 시 안전 정지 ----
    send_values(ser, 0, 0, STRAIGHT_ANGLE)
    print("✅ [LAVACONE] 라바콘 복도 주행 완료")
    return True

