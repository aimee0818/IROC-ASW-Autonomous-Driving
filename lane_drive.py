# lane_drive.py

from lane_detection_dual_yellow2 import drive_with_dual_yellow
from lane_detection_dual_white2 import drive_with_dual_white


# ====== 기본 설정 ======
BASE_SPEED = 50
STRAIGHT_ANGLE = 90
TURN_GAIN = 0.4

# 후륜보조 (옵션)
TURN_THRESHOLD = 25
ASSIST_GAIN = 0.6
ASSIST_DURATION = 0.3

# 내부 PID 상태
_prev_error, _integral = 0, 0


# ====== 유틸 ======
def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def pid_control(cx, width, mode="normal"):
    """차선 중심 기반 PID 제어"""
    global _prev_error, _integral
    center = width // 2
    error = cx - center
    Kp, Kd, Ki = 0.41, 0.14, 0.0     #월래 기본 2-line(0.4, 0.15, 0.0)
    _integral += error
    derivative = error - _prev_error
    _prev_error = error
    steer = 90 + (Kp * error + Kd * derivative + Ki * _integral)
    steer = max(45, min(135, steer))
    return int(steer), BASE_SPEED


# ====== 기본 주행 ======
def send_value(ser, frame, show_debug=False):
    """프레임 기반 차선 주행"""
    if frame is None:
        return None

    (angle, speed), _, _ = drive_with_dual_yellow(
        frame, pid_control=pid_control, show_debug=show_debug
    )

    steer_offset = angle - 90
    steer_ratio = steer_offset / 45.0
    L_pwm = int(speed * (1.0 - TURN_GAIN * steer_ratio))
    R_pwm = int(speed * (1.0 + TURN_GAIN * steer_ratio))
    # 안전 범위 제한
    L_pwm = clamp(L_pwm, -255, 255)
    R_pwm = clamp(R_pwm, -255, 255)

    # 일반 PID 전송
    send_serial(ser, L_pwm, R_pwm, angle)
    if show_debug:
        print(f"[lane_drive] L={L_pwm}, R={R_pwm}, Angle={angle}")

def white_value(ser, frame, show_debug=False):
    """흰색 점선 기반 주행 (차량회피, 복귀 구간 등에서 사용)"""
    if frame is None:
        return None

    (angle, base_speed), _, _ = drive_with_dual_white(
        frame, pid_control=pid_control, show_debug=show_debug
    )

    steer_offset = angle - 90
    steer_ratio = steer_offset / 45.0
    L_pwm = int(base_speed * (1.0 - TURN_GAIN * steer_ratio))
    R_pwm = int(base_speed * (1.0 + TURN_GAIN * steer_ratio))
    L_pwm = clamp(L_pwm, -255, 255)
    R_pwm = clamp(R_pwm, -255, 255)

    send_serial(ser, L_pwm, R_pwm, angle)
    if show_debug:
        print(f"[lane_drive:WHITE] L={L_pwm}, R={R_pwm}, Angle={angle}")




def send_serial(ser, L, R, angle):
    try:
        ser.write(f"{L},{R},{angle}\n".encode())
    except Exception as e:
        print(f"[ERROR] 시리얼 전송 실패: {e}")


def stop_motors(ser):
    send_serial(ser, 0, 0, 90)
    print("🛑 모터 정지")
    

def gostraight(ser):
    send_serial(ser, 50, 50, 90)
    print("직진합니다")


def reset_pid():
    global _prev_error, _integral
    _prev_error = 0
    _integral = 0