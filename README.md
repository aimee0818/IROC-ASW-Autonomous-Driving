# 🚗 IROC ASW — AI Autonomous Driving System

> **International Robot Olympiad (IROC) 2025** | ASW Division | 🏆 4th Place (Korea Robot Industry Promotion Agency Director Award)

ROS2 기반 Jetson Orin 자율주행 시스템으로, 카메라 비전 + LiDAR 센서 융합을 통해 9개 미션 시퀀스를 자율 수행합니다.

---

## 🧠 System Architecture

```
┌─────────────────────────────────────────────────────┐
│                     main.py                         │
│              (Mission State Machine)                │
└────────┬────────────┬──────────────┬────────────────┘
         │            │              │
   ┌─────▼────┐ ┌─────▼─────┐ ┌────▼──────────┐
   │  Camera  │ │   LiDAR   │ │ Serial (Arduino)│
   │  (OpenCV)│ │  (ROS2)   │ │  L/R PWM + Angle│
   └─────┬────┘ └─────┬─────┘ └────────────────┘
         │            │
   ┌─────▼────────────▼──────────────────────────┐
   │              Perception Layer                │
   │  lane_detection_dual_yellow2.py  (Yellow)   │
   │  lane_detection_dual_white2.py   (White)    │
   │  stopline_detector.py  (EMA + Adaptive)     │
   │  lavacone_trigger.py   (LiDAR Corridor)     │
   └─────────────────┬───────────────────────────┘
                     │
   ┌─────────────────▼───────────────────────────┐
   │               Control Layer                  │
   │  lane_drive.py        (PID Steering)         │
   │  obstacle_avoid.py    (LiDAR-based)          │
   │  lavacone.py          (Corridor PID)         │
   │  t_mode.py            (T-intersection)       │
   │  parking.py           (Sequence Control)     │
   │  finish.py            (Final Maneuver)       │
   └─────────────────────────────────────────────┘
```

---

## 🏁 Mission Sequence

| # | Mission | Method |
|---|---------|--------|
| 1 | 출발선 감지 | Stopline Detector (Adaptive Threshold + EMA) |
| 2 | 차선 주행 | Dual Yellow Lane Detection + PID |
| 3 | 장애물 회피 | LiDAR Sector Scan → 방향 결정 |
| 4 | 교차로 통과 | T-Mode (방향별 후진/전진 시퀀스) |
| 5 | 차량 추월 | Camera-based Car Avoidance |
| 6 | 라바콘 복도 | LiDAR Corridor PID (좌우 거리 균형) |
| 7 | 주차 | 시퀀스 제어 (각도 + 시간 기반) |
| 8 | 코너 미션 | LiDAR ENTER/EXIT Trigger |
| 9 | 완주 | LiDAR 기반 최종 방향 판단 후 정지 |

---

## 🔧 Key Technical Features

### 📷 Vision-based Lane Detection
- HSV 색공간 기반 노란색/흰색 차선 분리 검출
- ROI 기반 연산량 최적화 (하단 30% 영역만 처리)
- 양쪽 차선 소실 시 **이력 기반 중심 추정** (lane_width_history)
- Morphology 연산으로 노이즈 제거 및 끊긴 선 보정

### 📡 LiDAR Sensor Fusion
- 섹터별(전/좌/우) 최소/평균 거리 계산
- **복도 진입/이탈 자동 감지** (ENTER/EXIT 상태머신)
- Kalman Filter 기반 센서 융합

### 🔄 PID Steering Control
```
Kp=0.41, Kd=0.14, Ki=0.0
steer = 90 + (Kp * error + Kd * derivative + Ki * integral)
angle range: [45°, 135°]
```

### 🛑 Stopline Detection
- Adaptive Threshold + EMA (Exponential Moving Average) 안정화
- 쿨다운 기반 중복 트리거 방지
- 컬러 마스크(흰색/노란색) 결합으로 검출 정확도 향상

---

## 🛠️ Tech Stack

| Category | Details |
|----------|---------|
| **Platform** | NVIDIA Jetson Orin |
| **Framework** | ROS2, AutoSAR |
| **Language** | Python 3, C/C++ |
| **Vision** | OpenCV, HSV Color Segmentation |
| **LiDAR** | RPLiDAR, ROS2 `/scan` topic |
| **Control** | PID, PWM Serial (Arduino) |
| **Hardware** | Ackermann Steering, SLA 3D Printed Frame |

---

## 📁 File Structure

```
IROC-ASW-Autonomous-Driving/
│
├── main.py                        # 메인 미션 상태머신
│
├── perception/
│   ├── lane_detection_dual_yellow2.py  # 노란 차선 검출
│   ├── lane_detection_dual_white2.py   # 흰색 차선 검출
│   ├── lane_drive_single.py            # 단일 차선 주행
│   ├── stopline_detector.py            # 정지선 감지
│   ├── lavacone_trigger.py             # 라바콘 진입/이탈 감지
│   ├── lavacone_trigger_2.py           # 라바콘 감지 v2
│   └── lidar_subscriber.py             # ROS2 LiDAR 구독
│
├── control/
│   ├── lane_drive.py                   # PID 차선 주행
│   ├── obstacle_avoid.py               # 장애물 회피
│   ├── lavacone.py                     # 라바콘 복도 주행
│   ├── lavacone_2.py                   # 라바콘 주행 v2
│   ├── t_mode.py                       # T 교차로 처리
│   ├── parking.py                      # 주차 시퀀스
│   └── finish.py                       # 완주 시퀀스
│
└── utils/
    └── debug_lidar.py                  # LiDAR 실시간 디버그
```

---

## 🏆 Result

**Korea Robot Industry Promotion Agency Director Award — 4th Place**  
International Robot Olympiad 2025, AI Autonomous Driving (ASW Division)

---

## 👤 Author

**Yeim Kim** | Mechatronics Engineering, Chungnam National University  
📧 yeim0128@gmail.com# IROC-ASW-Autonomous-Driving
ROS2-based autonomous driving system for IROC ASW competition (Jetson Orin)
