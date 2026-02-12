# NTRexLAB MW-AHRS ROS2 Driver (stella_ahrs)

ROS2 Humble 호환 AHRS IMU 드라이버 패키지

NTRexLAB MW-AHRSv1 센서용 ROS2 드라이버입니다. Raspberry Pi, Jetson Nano, Desktop(amd64)에서 호환됩니다.

## 개요

| 항목 | 내용 |
|------|------|
| 센서 | NTRexLAB MW-AHRSv1 |
| ROS2 버전 | Humble Hawksbill |
| 출력 토픽 | `/imu/data`, `/imu/data_raw`, `/imu/mag` |
| 프레임 | `imu_link` |
| 통신 | UART (115200 baud) |

## 의존 레포지토리

| 레포지토리 | 설명 | 링크 |
|-----------|------|------|
| **2th_NtrexAHRS_lib_ROS_Sendbooster** | AHRS 드라이버 (이 패키지) | [GitHub](https://github.com/SeongminJaden/2th_NtrexAHRS_lib_ROS_Sendbooster) |
| **sendbooster_agv_bringup** | 실제 로봇 통합 런치 | [GitHub](https://github.com/SeongminJaden/sendbooster_agv_bringup) |
| **sendbooster_agv_simulation** | 시뮬레이션 + URDF 모델 | [GitHub](https://github.com/SeongminJaden/sendbooster_agv_simulation) |

## 패키지 구조

```
2th_NtrexAHRS_lib_ROS/
├── README.md
├── lib_aarch64/MW_AHRS_aarch64.a       # Jetson Xavier/Orin
├── lib_amd64/MW_AHRS_amd64.a           # Desktop (x86_64)
├── lib_armv7l/MW_AHRS_armv7l.a         # Raspberry Pi
└── ros2_example/stella_ahrs/           # ROS2 패키지
    ├── CMakeLists.txt
    ├── package.xml
    ├── config/config.yaml              # ROS 파라미터
    ├── include/mw/
    │   ├── mw_ahrs.hpp                 # 메인 드라이버 헤더
    │   ├── mw_ahrsX1_def.hpp           # X1 모델 데이터 테이블
    │   └── mw_serial.hpp               # 시리얼 통신 헤더
    ├── include/serial/                 # 시리얼 라이브러리
    ├── mw/mw_ahrs.cpp                  # 메인 드라이버 구현
    ├── src/listener.cpp                # 노드 엔트리포인트
    ├── launch/stella_ahrs_launch.py    # 런치 파일
    └── rviz/imu_test.rviz             # Rviz 설정
```

## 설치

### 1. 클론

```bash
cd ~/ros2_ws/src
git clone -b ver_2.0 https://github.com/SeongminJaden/2th_NtrexAHRS_lib_ROS_Sendbooster.git
```

### 2. 빌드

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select stella_ahrs
source install/setup.bash
```

## 사전 점검

### 시리얼 포트 확인

```bash
# 연결된 USB 장치 확인
ls /dev/ttyUSB*

# 장치 정보 확인
udevadm info -a /dev/ttyUSB0 | grep -E "idVendor|idProduct"
```

### 시리얼 권한

```bash
# 영구적 (재로그인 필요)
sudo usermod -aG dialout $USER

# 임시
sudo chmod 666 /dev/ttyUSB0
```

### udev 규칙 (포트 고정, 선택사항)

```bash
sudo nano /etc/udev/rules.d/99-ahrs.rules
```

```
SUBSYSTEM=="tty", ATTRS{idVendor}=="YOUR_VID", ATTRS{idProduct}=="YOUR_PID", SYMLINK+="ahrs_imu", MODE="0666"
```

```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### IMU 초기화

센서를 **평평한 바닥에 정지 상태**로 놓고 시작하세요. 기울어진 상태에서 시작하면 roll/pitch 오프셋이 발생합니다.

## 사용법

### 단독 실행

```bash
# 기본 실행 (config.yaml 사용)
ros2 launch stella_ahrs stella_ahrs_launch.py

# 파라미터 지정
ros2 run stella_ahrs stella_ahrs_node --ros-args \
  -p port:=/dev/ttyUSB0 \
  -p baud_rate:=115200 \
  -p publish_tf:=false \
  -p frame_id:=imu_link \
  -p parent_frame_id:=base_link
```

### Sendbooster AGV와 통합 실행

bringup 패키지에 포함되어 있으므로 별도 실행이 필요 없습니다:

```bash
# 모터 드라이버 + AHRS + EKF + LiDAR 한 번에 실행
ros2 launch sendbooster_agv_bringup bringup.launch.py
```

### 데이터 확인

```bash
# IMU 데이터 확인
ros2 topic echo /imu/data --once

# 발행 주파수 확인
ros2 topic hz /imu/data

# Rviz에서 시각화
ros2 launch stella_ahrs stella_ahrs_launch.py
```

## ROS 파라미터

| 파라미터 | 기본값 | 설명 |
|---------|--------|------|
| `port` | `/dev/ttyUSB0` | 시리얼 포트 |
| `baud_rate` | `115200` | 통신 속도 |
| `publish_tf` | `false` | TF 발행 여부 (EKF 사용 시 false) |
| `frame_id` | `imu_link` | IMU 프레임 ID |
| `parent_frame_id` | `base_link` | 부모 프레임 ID |

## ROS2 토픽

### Published Topics

| Topic | Type | 설명 |
|-------|------|------|
| `/imu/data` | sensor_msgs/Imu | 필터링된 IMU (orientation + angular_velocity + linear_acceleration) |
| `/imu/data_raw` | sensor_msgs/Imu | Raw IMU (angular_velocity + linear_acceleration) |
| `/imu/mag` | sensor_msgs/MagneticField | 지자기 데이터 |

## EKF 센서 융합

이 AHRS를 휠 오도메트리와 결합하여 더 정확한 위치추정이 가능합니다.

```
/odom (wheel encoder) ──┐
                        ├──→ EKF ──→ /odometry/filtered
/imu/data (AHRS)    ───┘
```

EKF 설정은 [sendbooster_agv_bringup](https://github.com/SeongminJaden/sendbooster_agv_bringup)의 `config/ekf.yaml`을 참고하세요.

## 지원 플랫폼

| 플랫폼 | 라이브러리 | 테스트 환경 |
|--------|-----------|------------|
| aarch64 | `lib_aarch64/MW_AHRS_aarch64.a` | Jetson Xavier |
| amd64 | `lib_amd64/MW_AHRS_amd64.a` | Desktop / Notebook |
| armv7l | `lib_armv7l/MW_AHRS_armv7l.a` | Raspberry Pi |

## AHRS 데이터 테이블 커스터마이징

다른 AHRS 모델 사용 시 데이터 테이블을 수정하세요:

```
ros2_example/stella_ahrs/include/mw/mw_ahrsX1_def.hpp
```

X1 기준으로 작성되어 있으며, 본인의 AHRS 모델에 맞게 레지스터 주소와 데이터 구조를 변경하면 됩니다.

## Ver 2.0 변경사항 (ROS2 Humble 호환)

- **스레드 안전성**: `std::lock_guard` + `std::mutex`로 데이터 레이스 방지
- **Publish 버그 수정**: `std::move` 대신 copy-then-publish 패턴 적용
- **API 현대화**: `declare_parameter()` Humble API 호환, `LifecycleNode` → `Node`
- **파라미터 추가**: port, baud_rate, publish_tf, frame_id, parent_frame_id
- **ODR 위반 수정**: 헤더의 `static float` 전역 배열 → cpp 지역변수로 이동
- **include guard**: 모든 헤더에 `#pragma once` 추가
- **C++17**: `constexpr` 상수, `std::atomic<bool>` 사용
- **런치 파일**: Humble deprecated API 수정 (`node_executable` → `executable` 등)

## 참고 자료

- [NTRexLAB 원본 레포](https://github.com/ntrexlab/2th_NtrexAHRS_lib_ROS)
- [robot_localization (EKF)](https://docs.ros.org/en/humble/p/robot_localization/)

## 라이선스

Apache-2.0
