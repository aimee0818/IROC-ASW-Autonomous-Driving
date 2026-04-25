# lavacone_2_trigger.py (refactored)
import math
import numpy as np
import time

class LavaCone_2Trigger:
    """
    LiDAR 기반 복도(라바콘) 진입/이탈 감지기
    - main에서 lidar 객체를 받아 사용
    """

    def __init__(self,
                 mirror=True,
                 yaw_offset=180.0,
                 left_sector=(-80.0, -30.0),
                 right_sector=(30.0, 80.0),
                 front_sector=(-15.0, 15.0),
                 max_range=15.0,
                 band_min=0.20,
                 band_max=0.80,
                 enter_frames=10,
                 exit_frames=3):

        # LiDAR 세팅
        self.mirror = mirror
        self.yaw_offset = yaw_offset
        self.left_sector = left_sector
        self.right_sector = right_sector
        self.front_sector = front_sector
        self.max_range = max_range

        # 복도 조건 파라미터
        self.band_min = band_min
        self.band_max = band_max
        self.enter_frames = enter_frames
        self.exit_frames = exit_frames

        # 내부 상태
        self.corridor_hold = 0
        self.corridor_release = 0
        self.in_corridor = False
        self._last_dbg = 0.0
        self.DEBUG_INTERVAL = 0.5

    # ========================
    # LiDAR 도우미
    # ========================
    def _wrap_deg(self, a):
        return (a + 180.0) % 360.0 - 180.0
    
    def min_in_sector(self, scan, deg_lo, deg_hi):
        """지정된 각도 구간의 최소 거리 계산. 유효 값이 하나도 없으면 inf."""
        if scan is None or not getattr(scan, "ranges", None):
            return float('inf')

        a_min, da = scan.angle_min, scan.angle_increment
        lo, hi = (deg_lo, deg_hi) if deg_lo <= deg_hi else (deg_hi, deg_lo)
        best = float('inf')

        for i, d in enumerate(scan.ranges):
            # 거리 유효성 체크
            if not (0.01 < d <= self.max_range):
                continue

            # 현재 빔 각도 -> 도(deg)
            ang = math.degrees(a_min + i * da)

            # 라이다를 거꾸로 달았으면 좌우 반전
            if self.mirror:
                ang = -ang

            # 로봇 기준으로 회전 보정
            ang = self._wrap_deg(ang + self.yaw_offset)

            # 원하는 섹터 안에 있으면 최소값 갱신
            if lo <= ang <= hi and d < best:
                best = d

        return best


    def mean_in_sector(self, scan, deg_lo, deg_hi):
        """지정된 각도 구간의 평균 거리 계산"""
        if scan is None or not getattr(scan, "ranges", None):
            return float('inf')

        a_min, da = scan.angle_min, scan.angle_increment
        lo, hi = (deg_lo, deg_hi) if deg_lo <= deg_hi else (deg_hi, deg_lo)
        vals = []

        for i, d in enumerate(scan.ranges):
            if not (0.01 < d <= self.max_range):
                continue
            ang = math.degrees(a_min + i * da)
            if self.mirror:
                ang = -ang
            ang = self._wrap_deg(ang + self.yaw_offset)
            if lo <= ang <= hi:
                vals.append(d)

        return float(np.mean(vals)) if vals else float('inf')

    # ========================
    # 메인 감지 함수
    # ========================
    def detect(self, lidar_obj, show_debug=False):
        """
        외부에서 전달받은 lidar 객체를 사용하여 복도 진입/이탈을 판단한다.
        lidar_obj.latest_scan을 기반으로 계산
        """
        scan = lidar_obj.latest_scan
        if scan is None:
            return None

        left_d = self.min_in_sector(scan, *self.left_sector)
        right_d = self.min_in_sector(scan, *self.right_sector)
        front_d = self.min_in_sector(scan, *self.front_sector)

        in_band_1 = (
            (self.band_min < left_d  < self.band_max) and
            (self.band_min < right_d < self.band_max)
        )
        in_band_2 = (
            (self.band_min < left_d  < 2.3) and
            (self.band_min < right_d < 2.3) and
            (self.band_min < front_d < 2.3)
        )

        # 상태 카운트
        if not self.in_corridor and in_band_1:
            self.corridor_hold += 1
        else:
            self.corridor_hold = 0

        if self.in_corridor and not in_band_2:
            self.corridor_release += 1
        else:
            self.corridor_release = 0

        # 디버그 출력
        now = time.time()
        if show_debug and now - self._last_dbg > self.DEBUG_INTERVAL:
            def fmt(x): return f"{x:.2f}m" if np.isfinite(x) else "inf"
            print(f"[LAVACONE_2_TRIGGER] L:{fmt(left_d)} R:{fmt(right_d)} "
                  f"| hold:{self.corridor_hold} rel:{self.corridor_release} ")
            self._last_dbg = now

        # ENTER 조
        if not self.in_corridor and self.corridor_hold > self.enter_frames:
            self.in_corridor = True
            print("🟠 [LAVACONE_2_TRIGGER] 복도 진입 감지 — 라바콘 모드 시작")
            return "ENTER"

        # EXIT 조건
        if self.in_corridor and self.corridor_release > self.exit_frames:
            self.in_corridor = False
            print("⚠️ [LAVACONE_2_TRIGGER] 복도 이탈 감지 — 모드 종료")
            return "EXIT"

        return None
