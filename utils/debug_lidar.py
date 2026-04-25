# debug_lidar.py
# 🚗 LiDAR 실시간 감시 모듈 (main.py에서 lidar 객체를 전달받음)
import time, math, threading, numpy as np

# ===== 설정 =====
LIDAR_MIRROR       = True
LIDAR_YAW_OFFSET   = 180.0
LIDAR_MAX_RANGE    = 4.3

LEFT_SECTOR        = (-55.0, -45.0)
MID_SECTOR         = (-10.0, +10.0)
RIGHT_SECTOR       = (+45.0, +55.0)

SWITCH_DETECT_RANGE = 1.0
SWITCH_SECTOR_FRONT = (-30.0, +30.0)

ENGAGE_DETECT_RANGE = 0.5
ENGAGE_SECTOR       = (-8.0, +8.0)


DEBUG_INTERVAL = 0.3


# ===== 유틸 =====
def _wrap_deg(a):
    return (a + 180.0) % 360.0 - 180.0


def min_in_sector(scan, deg_lo, deg_hi,
                  mirror=LIDAR_MIRROR,
                  yaw_off=LIDAR_YAW_OFFSET,
                  max_range=LIDAR_MAX_RANGE):
    """지정 각도 구간 내 최소 거리 계산"""
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


def detect_open_lane(scan):
    """LEFT / MID / RIGHT 거리 비교를 통해 개방된 방향 판단"""
    left_d  = min_in_sector(scan, *LEFT_SECTOR)
    mid_d   = min_in_sector(scan, *MID_SECTOR)
    right_d = min_in_sector(scan, *RIGHT_SECTOR)

    # 유효값 정리
    left_d  = float(left_d)  if np.isfinite(left_d)  else LIDAR_MAX_RANGE
    mid_d   = float(mid_d)   if np.isfinite(mid_d)   else LIDAR_MAX_RANGE
    right_d = float(right_d) if np.isfinite(right_d) else LIDAR_MAX_RANGE

    # 열린(open) 방향은 가장 멀리 뚫린 곳
    dists = {"LEFT": left_d, "MID": mid_d, "RIGHT": right_d}
    open_dir = max(dists, key=lambda k: dists[k])

    return left_d, mid_d, right_d, open_dir


def switch_detect(scan):
    """SWITCH 감지 여부 판단"""
    front_d = min_in_sector(scan, *SWITCH_SECTOR_FRONT)
    if np.isfinite(front_d) and front_d < SWITCH_DETECT_RANGE:
        return True, front_d
    else:
        return False, front_d
    
    

def engage_detect(scan):
    """ENGAGE 감지 여부 판단"""
    front_d = min_in_sector(scan, *ENGAGE_SECTOR)
    if np.isfinite(front_d) and front_d < ENGAGE_DETECT_RANGE:
        return True, front_d
    else:
        return False, front_d



# ===== 클래스 =====
class DebugLidar:
    """
    LiDAR 실시간 감시 클래스
    - main.py에서 lidar 객체를 전달받음 (LidarSubscriber 인스턴스)
    - open_lane: "LEFT" / "MID" / "RIGHT"
    - switch: True / False
    """

    def __init__(self, lidar_obj):
        self.lidar = lidar_obj
        self.open_lane = None
        self.switch = False
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        print("📡 [DEBUG_LIDAR] LiDAR thread started (external lidar injected)")

    def _loop(self):
        last_dbg = 0
        while self._running:
            scan = getattr(self.lidar, "latest_scan", None)
            if scan is None:
                time.sleep(0.05)
                continue


            # --- 거리 계산 ---
            left_d, mid_d, right_d, open_dir = detect_open_lane(scan)
            switch_state, front_d_switch = switch_detect(scan)
            engage_state, front_d_engage = engage_detect(scan)

            # --- 상태 업데이트 ---
            self.open_lane = open_dir
            self.switch = switch_state
            self.engage = engage_state

            # 상태 업데이트
            self.open_lane = open_dir
            self.switch = switch_state

            # 디버깅 출력
            now = time.time()
            if now - last_dbg > DEBUG_INTERVAL:
                print(
                    f"[DEBUG_LIDAR] "
                    f"L:{left_d:.2f} M:{mid_d:.2f} R:{right_d:.2f} → "
                    f"OPEN:{open_dir:<5} | "
                    f"ENGAGE:{'TRUE' if engage_state else 'FALSE'} ({front_d_engage:.2f}m) | "
                    f"SWITCH:{'TRUE' if switch_state else 'FALSE'} ({front_d_switch:.2f}m)"
                )
                last_dbg = now

            time.sleep(0.05)

    def stop(self):
        """스레드 종료"""
        self._running = False
        print("🧹 [DEBUG_LIDAR] LiDAR thread stopped")
