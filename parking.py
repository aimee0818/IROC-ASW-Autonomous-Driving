# parking.py
import time

def run(ser, show_debug=True):
    """
    주차 시퀀스 실행 함수
    - main.py에서 ser을 전달받아 사용
    - 주차 완료 후 True 반환
    """


    def _send_values(L, R, angle, duration=None, desc=""):
        """시리얼 전송 + 대기"""
        try:
            ser.write(f"{L},{R},{angle}\n".encode())
            ser.flush()
        except Exception as e:
            print(f"[parking] Serial error: {e}")
            return
        if show_debug:
            print(f"[parking] {desc}: L={L}, R={R}, A={angle}")
        if duration:
            time.sleep(duration)

    print("🅿️ [PARKING] 주차 시퀀스 시작")

    seq = [
        (0,   0,   90, 5.0,  "정지 대기"),
        (0,   0,   55, 2.0,  "왼쪽 각도 정렬"),
        (50,  50,  55, 0.5,  "↙️ 왼쪽 각도 틀기"),
        (0,   0,  135, 2.5,  "조향 전환"),
        (-50, -50, 135, 2.8, "🔙 왼쪽 후진"),
        (0,   0,   65, 1.5,  "좌정렬"),
        (50,  50,   65, 1.5, "전진 정렬"),
        (0,   0,   95, 1.5,  "우정렬"),
        (-50, -50, 95, 2.0,  "↘️ 정렬 후진"),
        (0,   0,   90, 2.0,  "🅿️ 주차 완료, 정지"),
        (50,  50,   90, 0.5, "마지막 위치 보정"),
        (0,   0,   45, 1.5,  "좌측 회전 준비"),
        (50,  50,   45, 2.5, "좌측 전진 탈출"),
        (0,   0,   90, 1.5,  "직진 정렬"),
        (-50, -50, 90, 1.0,  "마지막 후진 정렬"),
    ]

    for L, R, A, T, D in seq:
        _send_values(L, R, A, T, D)

    print("✅ [PARKING] 주차 완료 → 차선주행 복귀")
    return True
