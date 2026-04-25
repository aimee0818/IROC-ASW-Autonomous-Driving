# t_mode.py — LEFT/RIGHT 인자 기반 T모드 시퀀스
import time

def run(ser, post_avoid_dir="LEFT", show_debug=True):
    """
    T-Mode 시퀀스 실행 함수
    - post_avoid_dir: 회피 방향 ("LEFT" 또는 "RIGHT")
    - ser: 시리얼 객체
    - show_debug: True면 단계별 로그 출력
    """
    STRAIGHT_ANGLE = 90

    # ==============================
    # 왼쪽 방향 회피 (LEFT)
    # ==============================
    if post_avoid_dir == "LEFT":
        if show_debug: print("🟢 [T-MODE] LEFT 방향 T모드 시작")

        # 1️⃣ 정지 후 좌측 조향
        _send_values(ser, 0, 0, 135)
        time.sleep(1.0)

        # 2️⃣ 좌측으로 후진
        _send_values(ser, -50, -50, 135)
        time.sleep(1.0)

        # 3️⃣ 전진하며 반대편(우측)으로 복귀
        _send_values(ser, 50, 50, 45)
        time.sleep(1.0)

        if show_debug: print("✅ [T-MODE] LEFT 방향 완료 — 차선주행 복귀")
        _send_values(ser, 0, 0, STRAIGHT_ANGLE)
        return True

    # ==============================
    # 오른쪽 방향 회피 (RIGHT)
    # ==============================
    elif post_avoid_dir == "RIGHT":
        if show_debug: print("🟢 [T-MODE] RIGHT 방향 T모드 시작")

        # 1️⃣ 정지 후 우측 조향
        _send_values(ser, 0, 0, 45)
        time.sleep(1.0)

        # 2️⃣ 우측으로 후진
        _send_values(ser, -50, -50, 45)
        time.sleep(1.0)

        # 3️⃣ 전진하며 반대편(좌측)으로 복귀
        _send_values(ser, 50, 50, 135)
        time.sleep(1.0)

        if show_debug: print("✅ [T-MODE] RIGHT 방향 완료 — 차선주행 복귀")
        _send_values(ser, 0, 0, STRAIGHT_ANGLE)
        return True

    # ==============================
    # 잘못된 인자 처리
    # ==============================
    else:
        print(f"[T-MODE] ⚠️ post_avoid_dir 값이 잘못됨: {post_avoid_dir} (LEFT/RIGHT만 허용)")
        return False


def _send_values(ser, L, R, angle):
    """모터 명령 전송"""
    try:
        ser.write(f"{L},{R},{angle}\n".encode())
        ser.flush()
    except Exception as e:
        print(f"[T-MODE] Serial error: {e}")
