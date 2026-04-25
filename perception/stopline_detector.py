# stopline_detector.py
import cv2
import numpy as np
import time

class StoplineDetector:
    """
    하단 ROI에서 밝은 가로띠(정지선) 검출.
    모든 파라미터는 생성자에서 메인이 넘겨줍니다.
    """
    def __init__(self,
                 confirm_frames=2,
                 roi_ratio=0.125,
                 adaptive_block=31,
                 adaptive_C=-10,
                 close_kernel=(15,3),
                 min_width_ratio=0.60,
                 min_aspect_ratio=6.0,
                 min_area=500,
                 ema_decay=0.7,
                 ema_threshold=0.6,
                 use_color_mask=True,
                 white_hsv_low=(0, 0, 190),      # 흰색: S 낮고 V 높은 영역
                 white_hsv_high=(180, 20, 255),
                 yellow_hsv_low=(0, 0, 190),   # 노란색: H≈15~40, S/V 중~높음
                 yellow_hsv_high=(180, 20, 255)):
        
        self.confirm_frames   = confirm_frames
        self.roi_ratio        = roi_ratio
        self.adaptive_block   = adaptive_block
        self.adaptive_C       = adaptive_C
        self.close_kernel     = close_kernel
        self.min_width_ratio  = min_width_ratio
        self.min_aspect_ratio = min_aspect_ratio
        self.min_area         = min_area
        self.ema_decay        = ema_decay
        self.ema_threshold    = ema_threshold

        self.use_color_mask = use_color_mask
        self.white_hsv_low  = np.array(white_hsv_low,  dtype=np.uint8)
        self.white_hsv_high = np.array(white_hsv_high, dtype=np.uint8)
        self.yellow_hsv_low  = np.array(yellow_hsv_low, dtype=np.uint8)
        self.yellow_hsv_high = np.array(yellow_hsv_high, dtype=np.uint8)

        self._hits = 0
        self._ema  = 0.0
        self.count = 0               # 지금까지 '유효 정지선'을 몇 번 만났는지
        self._last_fire_ts = 0.0     # 최근 트리거 시간
        self._armed = True           # 릴레이징용 플래그

    def detect(self, frame) -> bool:
        h, w = frame.shape[:2]
        roi_h = max(30, int(h * self.roi_ratio))
        roi = frame[h - roi_h : h, :]

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        v = hsv[..., 2]

        bin_ = cv2.adaptiveThreshold(
            v, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY, self.adaptive_block, self.adaptive_C
        )

        if self.use_color_mask:
         mask_white  = cv2.inRange(hsv, self.white_hsv_low,  self.white_hsv_high)
         mask_yellow = cv2.inRange(hsv, self.yellow_hsv_low, self.yellow_hsv_high)
         color_mask  = cv2.bitwise_or(mask_white, mask_yellow)
         # 적응 이진화 결과와 결합(OR)
         bin_ = cv2.bitwise_or(bin_, color_mask)

        k = cv2.getStructuringElement(cv2.MORPH_RECT, self.close_kernel)
        bin_ = cv2.morphologyEx(bin_, cv2.MORPH_CLOSE, k, iterations=2)

        contours, _ = cv2.findContours(bin_, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        detected = False
        for c in contours:
            x, y, cw, ch = cv2.boundingRect(c)
            area = cw * ch
            ar = cw / max(1, ch)
            if cw > int(self.min_width_ratio * w) and ar > self.min_aspect_ratio and area > self.min_area:
                detected = True
                break

        # 안정화(EMA + 연속 프레임)
        self._ema = self.ema_decay * self._ema + (1 - self.ema_decay) * (1.0 if detected else 0.0)
        stable = self._ema > self.ema_threshold
        self._hits = self._hits + 1 if stable else 0
        return self._hits >= self.confirm_frames
    
    def detect_once(self, frame, cooldown: float = 1.5) -> bool:
        """
        정지선이 '새롭게' 검출됐을 때만 True를 한 번 반환.
        - cooldown: 같은 정지선에서 중복 트리거 방지 시간(초)
        """
        now = time.time()
        is_stop = self.detect(frame)
        if is_stop and self._armed and (now - self._last_fire_ts) > cooldown:
            self.count += 1
            self._last_fire_ts = now
            self._armed = False
            return True
        if not is_stop:
            # 정지선 상태에서 벗어나면 다시 무장
            self._armed = True
        return False
