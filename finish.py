# finish.py
import time
import numpy as np

# ===== 기본 파라미터 =====
STRAIGHT_ANGLE   = 90
RIGHT_TURN_ANGLE = 115
LEFT_TURN_ANGLE  = 65
MISSION_SPEED    = 60

# 라이다 스캔 파라미터
SCAN_DURATION        = 0.8
MANEUVER_DURATION    = 2.5
DEBUG_PRINT_INTERVAL = 0.5

# ===== 내부 유틸 =====
def _wrap_deg(a):
    return (a + 180.0) % 360.0 - 180.0


def min_in_sector(scan, deg_lo, deg_hi,
                  mirror=True, yaw_off=180.0,
                  max_range=3.0):
    """특정 각도 섹터의 최소 거리 계산"""
    if scan is None or not getattr(scan, "ranges", None):
        return float('inf')
    a_min, da = scan.angle_min, scan.angle_increment
    lo, hi = (deg_lo, deg_hi) if deg_lo <= deg_hi else (deg_hi, deg_lo)
    best = float('inf')
    for i, d in enumerate(scan.ranges):
        if not (0.01 < d <= max_range):
            continue
        ang = np.degrees(a_min + i * da)
        if mirror:
            ang = -ang
        ang = _wrap_deg(ang + yaw_off)
        if lo <= ang <= hi and d < best:
            best = d
    return best


def _send_values(ser, L, R, angle):
    """시리얼 전송"""
    try:
        ser.write(f"{L},{R},{angle}\n".encode())
        ser.flush()
    except Exception as e:
        print(f"[finish] Serial error: {e}")


# ===== 메인 실행 함수 =====
def run(ser, lidar, show_debug=True):
    """
    FINISH 시퀀스 실행
    - LiDAR를 기반으로 좌/우 중 더 열린 방향으로 마지막 회전
    - 회전 후 정지
    """
    print("🏁 [FINISH] 종료 시퀀스 시작")

    phase_ts = time.time()
    finish_phase = 0
    last_dbg = 0.0

    while True:
        now = time.time()

        # 1️⃣ 스캔 단계
        if finish_phase == 0:
            if now - phase_ts < SCAN_DURATION:
                _send_values(ser, 0, 0, STRAIGHT_ANGLE)
                continue

            scan_now = lidar.latest_scan
            if scan_now is None:
                time.sleep(0.05)
                continue

            # 전/좌/우 최소 거리 계산
            f = min_in_sector(scan_now, -15.0, +15.0)
            r = min_in_sector(scan_now, +30.0, +70.0)
            l = min_in_sector(scan_now, -70.0, -30.0)

            left_ok  = (np.isfinite(l) and l >= r)
            right_ok = (np.isfinite(r) and r >= l)

            if (not left_ok) and right_ok:
                decision = "RIGHT"
                maneuver_angle = RIGHT_TURN_ANGLE
            elif left_ok and (not right_ok):
                decision = "LEFT"
                maneuver_angle = LEFT_TURN_ANGLE
            else:
                decision = "BLOCKED"

            if show_debug and now - last_dbg > DEBUG_PRINT_INTERVAL:
                def fmt(x): return f"{x:.2f}m" if np.isfinite(x) else "inf"
                print(f"[FINISH] Front:{fmt(f)}  Right:{fmt(r)}  Left:{fmt(l)}  => {decision}")
                last_dbg = now

            if decision == "BLOCKED":
                phase_ts = now  # 다시 스캔
                continue
            else:
                finish_phase = 1
                phase_ts = now

        # 2️⃣ 회전/전진 단계
        elif finish_phase == 1:
            if now - phase_ts < MANEUVER_DURATION:
                _send_values(ser, MISSION_SPEED, MISSION_SPEED, maneuver_angle)
            else:
                _send_values(ser, 0, 0, STRAIGHT_ANGLE)
                print("✅ [FINISH] 종료 시퀀스 완료 — 차량 정지")
                return True

        time.sleep(0.05)
