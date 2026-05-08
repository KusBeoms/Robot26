# Neuver Robot — Python IK & I2C Interface

2026 kusbeoms

---

## 파일 구성

```
kusbeoms_ik_fixed.py   역기구학 (FK / IK)
sendpacket.py          라즈베리파이 ↔ Arduino I2C 통신
example.py             사용 예시
README.md              이 파일
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

- **x축** : 로봇 왼쪽 → 오른쪽
- **y축** : 아래 → 위 (지면 방향이 -y)
- **z축** : 뒤 → 앞
- **원점** : 1번 모터 축 중심
- **좌표값** : 원점 → 바퀴 지면 접촉점까지의 벡터 (mm)

---

## 링크 파라미터

| 구간 | 거리 | 기울기 |
|------|------|--------|
| 1번→2번 모터 | +x 방향 70mm | x축 기준 y방향 +15° |
| 2번→3번 모터 | -z 방향 157.3mm | z축 기준 y방향 -12.2° |
| 3번→4번 모터 | +z 방향 158.3mm | z축 기준 y방향 -5.1° |
| 4번→바퀴 중심 | -x 방향 28mm | — |
| 바퀴 반지름 | 25mm | yz평면 2D 원 |

---

## 모터 정의 (오른쪽 다리 기준, 왼쪽은 yz평면 대칭)

| ID | 역할 | 범위 | 0도 | 최대도 |
|----|------|------|-----|--------|
| 1 | z축 회전 (벌림/오므림) | 0~28도 | 실제 -15도 (바깥) | 실제 +13도 (안쪽) |
| 2 | x축 회전 (무릎 앞뒤) | 0~90도 | 다리 뒤(-z) | 다리 위(+y) |
| 3 | x축 회전 (발목 앞뒤) | 0~90도 | 다리 앞(+z) | 다리 위(+y) |
| 4 | 바퀴 | — | — | dir=false → 앞으로 |

- `m1_real = m1_input - 15`  (실제 회전각)
- 무릎(3번모터)은 뒤로 나오는 구조 (p3.z < 0)
- 모터단위 변환: `step = round(degree × 4.16667)`  (0~240도 → 0~1000)

---

## I2C 배치

| I2C 주소 | 다리 | 방향 |
|----------|------|------|
| `0x08` | front_left  | 왼쪽 |
| `0x09` | rear_left   | 왼쪽 |
| `0x0A` | front_right | 오른쪽 |
| `0x0B` | rear_right  | 오른쪽 |

### 송신 패킷 (4바이트)

| 바이트 | 내용 |
|--------|------|
| 0 | 모터 ID (1~4) |
| 1 | 타입: 0=서보위치, 1=모터회전 |
| 2 | 값 low byte |
| 3 | 값 high byte |

### 수신 패킷 (20바이트)

| 바이트 | 내용 |
|--------|------|
| 0~7   | 모터 현재 위치 × 4 (int16 × 4) |
| 8~9   | 전류 (uint16) |
| 10~11 | 가속도 합계 — 디버그 (int16) |
| 12~19 | 다음 목표위치 × 4 — 디버그 (int16 × 4) |

---

## 구조적 한계

- **최대 도달 x** : 42.0mm (m1=15, real=0°일 때)
- **최소 도달 x** : 40.2mm (m1=28, real=13°일 때)
- **x=0은 도달 불가** (1번 모터 범위 초과)
- **2+3번 링크 합** : 315.6mm (최대 y 도달 거리)
- **y > ~284mm** : 도달 불가 (링크가 완전히 펴져도 닿지 않음)

---

## 사용법

### FK (각도 → 좌표)
```python
from kusbeoms_ik_fixed import forward_kinematics

x, y, z = forward_kinematics(m1_deg=15, m2_deg=45, m3_deg=45, right_leg=True)
```

### IK (좌표 → 각도)
```python
from kusbeoms_ik_fixed import inverse_kinematics

result = inverse_kinematics(41.0, 170.0, 0.0, right_leg=True)
if result["valid"]:
    print(result["m1"], result["m2"], result["m3"])
```

### I2C 송신
```python
import smbus2
from sendpacket import move_leg_ik, send_wheel, print_memory

bus = smbus2.SMBus(1)

# IK 계산 후 바로 송신
move_leg_ik(bus, "front_right", x=41.0, y=170.0, z=0.0)

# 바퀴
send_wheel(bus, "front_right", rotate_dir=False, speed=50)

# 메모리 확인
print_memory("front_right")   # 특정 다리
print_memory()                 # 전체
```

---

## 의존성

```
pip install smbus2 numpy
```