# Neuver Robot — ROS2 워크스페이스

2026 kusbeoms

4족 휠-레그 로봇 제어 시스템. Raspberry Pi에서 동작하며 MPU6050 IMU와 4개의 Arduino 다리 컨트롤러를 통합한다.

---

## 패키지 구성

```
ros2_ws/
└── src/
    ├── mpu6050_driver/          # ROS2 C++ 패키지 — IMU 드라이버
    │   ├── include/mpu6050_driver/mpu6050_driver.hpp
    │   ├── src/mpu6050_driver.cpp
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   └── SETTING.md
    └── movement/                # Python — 보행 제어 스택
        └── src/
            ├── main.py              # 독립 실행 엔트리포인트
            ├── robot_server.py      # ROS2 연동 TCP 서버 엔트리포인트
            ├── robot_node.py        # I2C 마스터 (Arduino × 4 제어)
            ├── gait_controller.py   # Trot gait 보행 패턴 생성기
            └── inverse_kinematics.py # FK / IK 계산
```

---

## 시스템 아키텍처

```
[MPU6050]
    │ I2C (0x68)
    ▼
[mpu6050_driver] ─── /imu/data (sensor_msgs/Imu) ──► [robot_server.py]
                                                            │
[외부 클라이언트] ─── TCP JSON (port 9000) ────────────────►│
                                                            │
                                                      GaitController
                                                            │
                                                       RobotNode
                                                            │ I2C
                                          ┌─────────────────┼─────────────────┐
                                       0x08              0x09              0x0A              0x0B
                                   [front_left]       [rear_left]       [front_right]     [rear_right]
                                    Arduino             Arduino           Arduino           Arduino
                                   (모터 1~4)          (모터 1~4)        (모터 1~4)        (모터 1~4)
```

---

## 패키지 상세

### mpu6050_driver

ROS2 C++ 노드. MPU6050에서 가속도·자이로 데이터를 100Hz로 읽어 Madgwick 필터(6-DOF)로 쿼터니언을 추정하고 `/imu/data` 토픽으로 퍼블리시한다.

| 항목 | 값 |
|------|-----|
| I2C 주소 | 0x68 |
| 업데이트 주기 | 10ms (100Hz) |
| 출력 토픽 | `/imu/data` (`sensor_msgs/Imu`) |
| 필터 | Madgwick 6-DOF (beta=0.1, zeta=0.01) |

빌드 및 실행:
```bash
cd ~/ros2_ws
colcon build --packages-select mpu6050_driver
source install/setup.bash
ros2 run mpu6050_driver mpu6050_node
```

---

### movement

Python 보행 제어 스택. 4개 파일로 구성된다.

#### `inverse_kinematics.py` — FK / IK

1번 모터 축 중심 → 바퀴 지면 접촉점 사이의 좌표 변환.

| 함수 | 설명 |
|------|------|
| `forward_kinematics(m1, m2, m3, right_leg)` | 모터 각도 → (x, y, z) |
| `inverse_kinematics(x, y, z, right_leg)` | (x, y, z) → 모터 각도 dict |

**링크 파라미터 (오른쪽 다리 기준)**

| 구간 | 거리 | 기울기 |
|------|------|--------|
| 1번→2번 모터 | +x 70mm | x축 기준 y방향 +15° |
| 2번→3번 모터 | -z 157.3mm | z축 기준 y방향 -12.2° |
| 3번→4번 모터 | +z 158.3mm | z축 기준 y방향 -5.1° |
| 4번→바퀴 중심 | -x 28mm | — |
| 바퀴 반지름 | 25mm | yz 평면 2D 원 |

**모터 정의**

| ID | 역할 | 입력 범위 | 비고 |
|----|------|----------|------|
| 1 | z축 회전 (벌림/오므림) | 0~28° | real = input - 15° |
| 2 | x축 회전 (무릎 앞뒤) | 0~90° | 0°=다리 뒤(-z) |
| 3 | x축 회전 (발목 앞뒤) | 0~90° | 0°=다리 앞(+z) |
| 4 | 바퀴 구동 | dir + speed | dir=false → 앞으로 |

**구조적 한계**

- x 도달 범위: 40.2 ~ 42.0mm (1번 모터 범위가 좁아 x stride 불가)
- 최대 y 도달: ~284mm (2+3번 링크 합 315.6mm, 기울기 손실 포함)

---

#### `robot_node.py` — I2C 마스터

목표 좌표가 변경되면 IK를 계산하고 I2C로 해당 Arduino에 즉시 전송한다. 백그라운드 스레드가 주기적으로 모든 Arduino 상태를 폴링한다.

| I2C 주소 | 다리 |
|----------|------|
| 0x08 | front_left |
| 0x09 | rear_left |
| 0x0A | front_right |
| 0x0B | rear_right |

**송신 패킷 (4바이트)**

| 바이트 | 내용 |
|--------|------|
| 0 | 모터 ID (1~4) |
| 1 | 타입: 0=서보위치, 1=모터회전 |
| 2~3 | 값 (little-endian uint16) |

**수신 패킷 (20바이트)**

| 바이트 | 내용 |
|--------|------|
| 0~7 | 모터 현재 위치 × 4 (int16 × 4) |
| 8~9 | 전류 (uint16) |
| 10~11 | 가속도 합계 — 디버그 (int16) |
| 12~19 | 다음 목표위치 × 4 — 디버그 (int16 × 4) |

주요 API:
```python
node = RobotNode(poll_interval_s=0.05, mock=False)
node.start()
node.set_target("front_right", x=41.0, y=170.0, z=0.0)  # IK + I2C 자동
node.set_wheel("front_right", rotate_dir=False, speed=50)
node.stop()
```

---

#### `gait_controller.py` — Trot Gait

방향각(rad)과 크기(0~1)를 입력받아 4다리 trot 보행 패턴을 50Hz로 생성한다.

**다리 위상 (Trot)**

| Group A (phase=0) | Group B (phase=π) |
|-------------------|-------------------|
| front_right, rear_left | front_left, rear_right |

**보행 파라미터**

| 파라미터 | 기본값 |
|----------|--------|
| 한 걸음 z 이동거리 | 40mm |
| 발 드는 높이 | 30mm |
| 사이클 주기 | 0.6초 |
| 제어 루프 | 50Hz |

IMU pitch 보정: pitch 1°당 약 3.3mm y 보정 (최대 ±30°에서 ±20mm).

```python
gait = GaitController(node)
gait.start()
gait.set_command(angle=0.0, magnitude=0.8)   # 전진
gait.set_command(angle=math.pi/2, magnitude=0.6)  # 오른쪽 대각
gait.stop()
```

---

#### `robot_server.py` — TCP 서버 (ROS2 연동)

ROS2 `/imu/data`를 구독하면서 TCP 포트 9000으로 JSON 명령을 수신한다. `main.py` 대신 ROS2 환경에서 사용하는 엔트리포인트.

```bash
# 터미널 1: IMU 노드
ros2 run mpu6050_driver mpu6050_node

# 터미널 2: 로봇 서버
python robot_server.py
```

**명령 프로토콜 (JSON + 줄바꿈)**

```json
{"cmd": "move", "angle": 0.0, "magnitude": 0.8}
{"cmd": "stop"}
{"cmd": "status"}
{"cmd": "quit"}
```

#### `main.py` — 독립 실행

ROS2 없이 `imu_reader.py`(별도)를 사용하는 독립 실행 엔트리포인트. `MOCK=True`로 설정하면 실제 하드웨어 없이 테스트 가능.

```bash
python main.py
```

---

## 좌표계

```
        +z (앞)
         │
    +y   │
    (위) │
         └──────── +x (오른쪽)
```

- 원점: 각 다리 1번 모터 축 중심
- 좌표값: 원점 → 바퀴 지면 접촉점 벡터 (mm)
- 왼쪽 다리는 오른쪽 다리의 yz 평면 대칭 (x 부호 반전)

---

## 의존성

```bash
# Python
pip install smbus2 numpy

# ROS2 (Humble 이상)
sudo apt install ros-<distro>-rclpy ros-<distro>-sensor-msgs

# C++ 빌드
sudo apt install libi2c-dev
```
