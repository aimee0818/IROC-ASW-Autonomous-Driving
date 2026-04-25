# main.py
import cv2, time, serial, math, numpy as np
import lane_drive, obstacle_avoid, t_mode, car_avoid, lavacone, parking, finish
from stopline_detector import StoplineDetector
from lidar_subscriber import LidarSubscriber
from car_avoid_trigger import CarAvoidTrigger
from lavacone_trigger import LavaConeTrigger

car_avoid._debug_active = False

# ===== 시리얼 통신 =====
SERIAL_PORT = "/dev/ttyACM0"
SERIAL_BAUD = 115200

# ===== 정지선(StopLine) =====
STOP_HOLD_SEC   = 1.0   # 정지선에서 멈추는 시간
STOP_COOLDOWN   = 1.5   # 같은 정지선 중복 방지 쿨다운
STOP_CONFIRM_FRAMES = 2
STOP_ROI_RATIO      = 0.125    # 하단 12.5%
STOP_ADAPT_BLOCK    = 31
STOP_ADAPT_C        = -10
STOP_CLOSE_KERNEL   = (15, 3)
STOP_MIN_WIDTH_RAT  = 0.60
STOP_MIN_AR         = 6.0
STOP_MIN_AREA       = 500
STOP_EMA_DECAY      = 0.7
STOP_EMA_THRESH     = 0.6
STOP_COAST_TIME     = 0.3


# ===== 시간저장/트리거/플래그 =====
start_pass_time = 0
delay_time = 0  
cross_pass_time = 0
oneline_pass_time = 0
cross_start = None

flag_stop = 0
flag_start = 0
flag_obstacle = 0 
flag_tmode = 0
flag_car_avoid = 0
sequence = 0
success = 0
car_avoid_entered = 0
lavacone_entered = 0
parking_entered = 0
flag_lavacone = 0 
flag_parking = 0
flag_corner = 0
flag_finish = 0

obstacle_reset = True
post_avoid_dir = None

car_trigger = CarAvoidTrigger()
lavacone_trigger = LavaConeTrigger()

def min_in_sector(scan, deg_lo, deg_hi, mirror=True, yaw_off=180.0, max_range=3.0):
    """특정 각도 구간 내 최소 거리 계산"""
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
        ang = (ang + yaw_off + 180) % 360 - 180
        if lo <= ang <= hi and d < best:
            best = d
    return best


def emergency_stop(ser, lidar, threshold=0.30, hold_time=3.0, show_debug=True):
    """전방 장애물 감지 시 3초간 정지"""
    scan = lidar.latest_scan
    if scan is None:
        return False

    front_d = min_in_sector(scan, -15.0, +15.0)
    if np.isfinite(front_d) and front_d < threshold:
        print(f"🚨 [EMERGENCY] 전방 거리 {front_d:.2f}m — 긴급 정지 발동")
        lane_drive.stop_motors(ser)
        time.sleep(hold_time)
        print("✅ [EMERGENCY] 긴급 정지 해제 — 주행 재개")
        return True
    return False




try:    
    # ===== 시리얼(아두이노) =====
    ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=0.2)
    time.sleep(2.0)
    
    # ===== 카메라 =====
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        raise SystemExit("카메라를 열 수 없습니다.")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    # ===== 라이다 =====
    lidar = LidarSubscriber("/scan")
    
    # ===== 정지선 감지기 =====
    stopdet = StoplineDetector(
        confirm_frames   = STOP_CONFIRM_FRAMES,
        roi_ratio        = STOP_ROI_RATIO,
        adaptive_block   = STOP_ADAPT_BLOCK,
        adaptive_C       = STOP_ADAPT_C,
        close_kernel     = STOP_CLOSE_KERNEL,
        min_width_ratio  = STOP_MIN_WIDTH_RAT,
        min_aspect_ratio = STOP_MIN_AR,
        min_area         = STOP_MIN_AREA,
        ema_decay        = STOP_EMA_DECAY,
        ema_threshold    = STOP_EMA_THRESH,
        )

    while True:
        
        # ===== frame 뿌려주기  =====
        ret, frame = cap.read()
        if not ret:
            continue
        
        # ===== 🚨 긴급 정지 검사 =====
        if emergency_stop(ser, lidar, threshold=0.30, hold_time=3.0, show_debug=True) and lavacone_entered == 0 and car_avoid_entered == 0 :
            continue  # 긴급정지 후 다음 루프로 넘어감

        # ===== 트리거 활성화 =====
        car_state = car_trigger.detect(frame, show_debug=True)
        lavacone_state = lavacone_trigger.detect(lidar, show_debug=True) 
        
        # ===== 1️ 출발선 감지 =====
        if flag_start == 0:  # 1번조건: 출발선을 지나지 않음
            lane_drive.gostraight(ser)
            
            if flag_stop == 0 and stopdet.detect_once(frame, cooldown=STOP_COOLDOWN): # 2번조건: 출발선에 도달
                end_t = time.time() + STOP_COAST_TIME
                while time.time() < end_t:
                # 직진 각도 유지, 저속으로 살짝 굴림
                    lane_drive.gostraight(ser)
                    time.sleep(0.02)
                print("🛑 출발선 감지")
                lane_drive.stop_motors(ser)
                time.sleep(STOP_HOLD_SEC)
                flag_start = 1  # 1번조건 끝
                flag_stop = 1  # 2번조건 끝, 3번조건 발동
                

        # ===== 2️ 첫 정지선 (출발선) 감지 =====
        if flag_stop == 1:      # 3번조건: 첫번째 정지선을 지나지 않음
            print("✅ 출발선 통과 — 차선 주행 시작")
            lane_drive.send_value(ser, frame)
            
            if stopdet.detect_once(frame, cooldown=STOP_COOLDOWN):    # 4번조건: 첫번째 정지선에 도달
                end_t = time.time() + STOP_COAST_TIME
                while time.time() < end_t:
                # 직진 각도 유지, 저속으로 살짝 굴림
                    lane_drive.gostraight(ser)
                    time.sleep(0.02)
                print("🛑 정지선 감지 — 정지합니다.")
                lane_drive.stop_motors(ser)
                time.sleep(STOP_HOLD_SEC)
                flag_stop = 2       # 3, 4번조건 끝, 5번조건 발동
              
            
        # ===== 3️ 장애물 미션  =====
        if flag_stop == 2 and flag_obstacle == 0: # 5번조건: obstacle_avoid 시작
            if obstacle_reset:  # (직전 단계에서 막 진입했을 때만)
                print("✅ 정지 완료 🚧 장애물 회피 시작")
                obstacle_avoid.reset()   # 딱 한 번 초기화
                obstacle_reset = False

            done, post_avoid_dir = obstacle_avoid.run(ser, lidar, show_debug=True)
            if done:
                flag_obstacle = 1 # 5번조건 끝, 6번조건 발동 
                
                
        # ===== 4 교차로 통과  =====
        if flag_stop == 2 and flag_obstacle == 1:  # 6번조건: 차선주행으로 복귀
            print("✅ 장애물 통과 — 차선 주행 시작")
            lane_drive.send_value(ser, frame)
            if stopdet.detect_once(frame, cooldown=STOP_COOLDOWN):               # 7번조건: Tmode 수행
                print("🛑 교차로 내 정지선 감지 — T-Mode 수행")
                success = t_mode.run(ser, post_avoid_dir, show_debug=True)
                if success:
                    print(" T-Mode 완료 — 차선 주행 복귀")
                    flag_tmode = 1
                    flag_stop = 3       # 6, 7번조건 끝, 9번조건 발동
            else:                                   # 8번조건: 그냥 통과
                lane_drive.send_value(ser, frame)
                if cross_start is None:
                    cross_start = time.time()
                    print("🚗 교차로 진입 — 타이머 시작")
                # 5초 경과 확인
                if time.time() - cross_start > 30:
                    print("⏩ 교차로 통과 완료 (정지선 없음)")
                    flag_stop = 3       # 6, 7번조건 끝, 9번조건 발동
                    
            
        # ===== 5 추월 미션  =====
        if flag_stop == 3:
            lane_drive.send_value(ser, frame)
            if stopdet.detect_once(frame, cooldown=STOP_COOLDOWN):    # 9번조건: 정지선 감지, car_avoid 진입
                end_t = time.time() + STOP_COAST_TIME
                while time.time() < end_t:
                # 직진 각도 유지, 저속으로 살짝 굴림
                        lane_drive.gostraight(ser)
                        time.sleep(0.02)
                print(" 정지선 감지, 🚗 Car Avoid Mode 진입")
                lane_drive.stop_motors(ser)
                time.sleep(STOP_HOLD_SEC)
                flag_stop = 4 # 9번조건 끝, 10번조건 진입
                car_avoid_entered = 1
            
        if flag_stop == 4 and car_avoid_entered == 1: # 10번조건 추월구간 주파
            car_avoid.run(ser, lidar, frame, show_debug=True)
            if stopdet.detect_once(frame, cooldown=STOP_COOLDOWN):
                end_t = time.time() + STOP_COAST_TIME
                while time.time() < end_t:
                    # 직진 각도 유지, 저속으로 살짝 굴림
                        lane_drive.gostraight(ser)
                        time.sleep(0.02)
                print(" 정지선 감지, 🚗 LAVACONE 모드 진입")
                lane_drive.stop_motors(ser)
                time.sleep(STOP_HOLD_SEC)
                car_avoid_enetered = 0
                flag_stop = 5 # 10번 조건 끝, 11번 조건 발동
                
        
        # ===== 6 라이다 미션  =====
        if flag_stop == 5 and flag_lavacone == 0:  # 11번조건: 라바콘 주행 시작
            print("🚗 lavacone.run() 실행")
            lavacone_entered = 1
            lavacone.run(ser, lidar, show_debug=True, should_stop=lambda: (lavacone_trigger.detect(lidar) == "EXIT"))
            
        if flag_stop == 5 and lavacone_state == "EXIT": # 12번조건 : 라바콘 탈출
            print("🚗 라바콘 모드 종료, 차선 주행 시작")
            lavacone_enetered = 0 
            flag_lavacone = 1   # 11번조건 끝, 13번 조건 발동
            lane_drive.send_value(ser, frame)

          
        # ===== 7 주차 미션 =====
        if flag_lavacone == 1 and stopdet.detect_once(frame, cooldown=STOP_COOLDOWN) and parking_entered == 0:    # 13번 조건: 정지선 도달, 주차구역 이동 시작
            flag_stop = 6 # 11, 12번 조건 끝, 14번 조건 발동
            print("🚗 라바콘 미션 종료 — 주차 구역으로 이동합니다.")
            end_t = time.time() + STOP_COAST_TIME
            while time.time() < end_t:
                # 직진 각도 유지, 저속으로 살짝 굴림
                    lane_drive.gostraight(ser)
                    time.sleep(0.02)
            lane_drive.stop_motors(ser)
            time.sleep(STOP_HOLD_SEC)
            parking_entered = 1 # 13번 조건 끝, 14번 조건 발동
            
        if flag_stop == 6 and parking_entered == 1:  # 14번 조건: 정지 후 오른쪽 차선 주행 시작
            lane_drive.oneline_drive(ser, frame, offset=150, side='right', show_debug=True)
            if (time.time() - oneline_pass_time > delay_time) and stopdet.detect_once(frame, cooldown=STOP_COOLDOWN):   # 15번 조건: 정지 후 파킹 주행 시작
                end_t = time.time() + STOP_COAST_TIME
                while time.time() < end_t:
                # 직진 각도 유지, 저속으로 살짝 굴림
                    lane_drive.gostraight(ser)
                    time.sleep(0.02)
                lane_drive.stop_motors(ser)
                time.sleep(STOP_HOLD_SEC)
                flag_stop = 6 # 14, 15번 조건 끝, 16번 조건 발동
        
        if flag_stop == 6 and flag_parking == 0 and success == 0: # 16번 조건: 차선 주행으로 지정위치까지 이동
            lane_drive.send_value(ser, frame)
            if stopdet.detect_once(frame, cooldown=STOP_COOLDOWN):
                flag_parking = 1 # 16번 조건 끝, 17번 조건 발동
                
        if flag_parking == 1 and success == 0:  # 17번 조건: 파킹 시퀀스 실행
            done_parking = parking.run(ser, show_debug=True)
            if done_parking:
                print("✅ 주차 완료 — 차선 주행 복귀")
                success = 1  # 17번 조건 끝, 18번 조건 발동
                
        if flag_stop == 6 and success == 1 and flag_corner == 0: # 18번 조건: 오른쪽 차선 주행 시작
            lane_drive.oneline_drive(ser, frame, offset=150, side='right', show_debug=True)
           
            
        # ===== 8 코너 미션 =====
            if lavacone_state == "ENTER":
                print(" 코너 플래그 ON — lavacone.run() 실행")
                lavacone.run(ser, lidar, show_debug=True, should_stop=lambda: (lavacone_trigger.detect(frame) == "EXIT"))
                if lavacone_state == "EXIT":
                    flag_corner = 1    # 18번 조건 끝, 19번 조건 발동
                    
                    
        # ===== 9 완주 미션 =====
        if flag_corner == 1 and flag_finish == 0: # 19번 조건: 완주를 향해 주행
                lane_drive.send_value(ser, frame)
                if stopdet.detect_once(frame, cooldown=STOP_COOLDOWN):
                    finish.run(ser, lidar)
                    flag_finish = 1 # 19번 조건 끝, 20번 조건 발동

        if flag_finish == 1:
            break
                
except KeyboardInterrupt:
    print("⏹️ 강제 종료")
finally:
    print("🧹 리소스 정리 중...")
    try:
        lane_drive.stop_motors(ser)  # ser 전달
    except Exception:
        pass
    try:
        if 'cap' in locals():
            cap.release()
    except Exception:
        pass
    try:
        if 'ser' in locals() and ser and ser.is_open:
            ser.close()
    except Exception:
        pass
    try:
        if 'lidar' in locals() and lidar:
            lidar.shutdown()
    except Exception:
        pass
    cv2.destroyAllWindows()
    print("✅ 안전 종료 완료")
