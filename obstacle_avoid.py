# obstacle_avoid.py (post_avoid_dir 반환 버전)
import time, math, numpy as np

BASE_SPEED = 55
MISSION_SPEED = 60
RIGHT_TURN_ANGLE = 114
STRAIGHT_ANGLE = 89
LEFT_TURN_ANGLE = 65

# --- LIDAR 설정값 ---
LIDAR_MIRROR = True
LIDAR_YAW_OFFSET = 180.0
LIDAR_SECTOR_FRONT = (-15.0, +15.0)
LIDAR_SECTOR_RIGHT = (+30.0, +70.0)
LIDAR_SECTOR_LEFT = (-70.0, -30.0)
LIDAR_MAX_RANGE = 15
DEBUG_PRINT_INTERVAL = 0.5

SCAN_DURATION = 0.8
MANEUVER_DURATION = 3.5

# --- 내부 상태 ---
avoid_phase = 0
phase_ts = time.time()
last_dbg = 0.0
maneuver_angle = STRAIGHT_ANGLE
Mission = 0
post_avoid_dir = None  # ✅ 회피 방향 기록용
decision = None        # ✅ 0단계에서 정해서 1단계에서 다시 쓸 값


# ====== 유틸 ======
def _wrap_deg(a):
    return (a + 180.0) % 360.0 - 180.0


def send_values(ser, L, R, angle):
    """모터/조향 명령 전송"""
    cmd = f"{L},{R},{angle}\n"
    try:
        ser.write(cmd.encode())
        ser.flush()
    except Exception as e:
        print(f"[obstacle_avoid] ⚠️ Serial error: {e}")


def min_in_sector(scan, deg_lo, deg_hi,
                  mirror=LIDAR_MIRROR, yaw_off=LIDAR_YAW_OFFSET,
                  max_range=LIDAR_MAX_RANGE):
    """특정 각도 섹터의 최소 거리 계산"""
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


# ===== 상태 초기화 =====
def reset():
    global avoid_phase, phase_ts, last_dbg, maneuver_angle, Mission, decision, post_avoid_dir
    avoid_phase = 0
    phase_ts = time.time()
    last_dbg = 0.0
    maneuver_angle = STRAIGHT_ANGLE
    Mission = 0
    decision = None
    post_avoid_dir = None
    print("[obstacle_avoid] 상태 초기화 완료")


# ====== 메인 함수 ======
def run(ser, lidar_obj, show_debug=False):
    """
    외부에서 lidar 객체를 전달받아 장애물 회피를 수행
    회피 완료 시 (done, post_avoid_dir) 반환
    """
    global avoid_phase, phase_ts, last_dbg, maneuver_angle, Mission, post_avoid_dir, decision

    now = time.time()

    # 0️⃣ 스캔 단계
    if avoid_phase == 0:
        # 스캔 시간 동안은 정지
        if now - phase_ts < SCAN_DURATION:
            send_values(ser, 0, 0, STRAIGHT_ANGLE)
            return False, None

        scan_now = lidar_obj.latest_scan
        f = min_in_sector(scan_now, *LIDAR_SECTOR_FRONT)
        r = min_in_sector(scan_now, *LIDAR_SECTOR_RIGHT)
        l = min_in_sector(scan_now, *LIDAR_SECTOR_LEFT)

        front_ok = (np.isfinite(f) and f >= 0.8)
        right_ok = (np.isfinite(f) and 0.8 >= f)

        # ===== 방향 결정 =====
        if (not front_ok) and right_ok:
            decision = "RIGHT"
            maneuver_angle = RIGHT_TURN_ANGLE
            post_avoid_dir = "RIGHT"
        elif front_ok and (not right_ok):
            decision = "STRAIGHT"
            maneuver_angle = STRAIGHT_ANGLE
            post_avoid_dir = "STRAIGHT"
        else:
            decision = "BLOCKED"

        if show_debug and now - last_dbg > DEBUG_PRINT_INTERVAL:
            def fmt(x): return f"{x:.2f}m" if np.isfinite(x) else "inf"
            print(f"[LIDAR] F:{fmt(f)}  R:{fmt(r)}  L:{fmt(l)}  => {decision}")
            last_dbg = now

        # BLOCKED면 다시 스캔
        if decision == "BLOCKED":
            phase_ts = now  # 다시 스캔 시작
        else:
            # 다음 단계로 넘어감
            avoid_phase = 1
            phase_ts = now

        return False, None

    # 1️ 회피 단계
    else:
        # 1) 오른쪽으로 도는 경우
        if decision == "RIGHT":
            if now - phase_ts < MANEUVER_DURATION:
                send_values(ser, MISSION_SPEED, MISSION_SPEED, maneuver_angle)
                return False, None  # 아직 끝 아님
            else:
                Mission = 1
                if show_debug:
                    print("✅ 장애물 회피 완료 (RIGHT)")
                avoid_phase = 0
                return True, post_avoid_dir

        # 2) 직진으로 빠지는 경우
        elif decision == "STRAIGHT":
            if now - phase_ts < 1.0:   # 직진은 짧게
                send_values(ser, MISSION_SPEED, MISSION_SPEED, maneuver_angle)
                return False, None
            else:
                Mission = 1
                if show_debug:
                    print("✅ 장애물 회피 완료 (STRAIGHT)")
                avoid_phase = 0
                return True, post_avoid_dir

        # 3) 예외 처리
        else:  # decision == "BLOCKED" or None
            Mission = 1
            if show_debug:
                print("⚠️ 결정 불명확 → 차선주행으로 복귀")
            avoid_phase = 0
            return True, post_avoid_dir
