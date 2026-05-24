'''
2026 kusbeoms

로봇의 정면 방향을 z축의 방향과 평행한다고 가정.
로봇을 위에서 바라보았을때의 평면에서 z축을 90도만큼 오른쪽으로 회전하였을때를 x축의 방향과 평행한다고 가정.
로봇을 x축의 진행이 안보이는 방향, 즉 오른쪽에서 바라보았을때의 평면에서 z축을 90도만큼 왼쪽으로 회전하였을때를 y축의 방향과 평행한다고 가정.

x축은 로봇의 왼쪽에서 부터 오른쪽을 가리킴.
y축은 로봇의 아래쪽에서 부터 위쪽을 가리킴.
z축은 로봇의 뒤부터 앞까지를 가리킴.

지금부터 모터는 id로 설명함. (idx = id + 1), 각도는 모터단위 0 ~ 1000을 0 ~ 240으로 매핑한것임.
또한 2번 3번 4번 모터는 1번 모터가 15도인 상황(2번 3번 모터가 yz평면에서만 이동하는 상황)으로 설명함
    1번 모터는 z축 회전을 담당함. 0 ~ 28도, 0도를 입력하면 15도 바깥쪽으로 벌어짐(-15도, 벌어지므로), 90도일경우 실제로는 75도.
    2번 모터는 x축 회전을 담당함. 0 ~ 90도, 0도시 다리가 뒤를 향함. 90도시 다리가 y축과 평행하게 펴짐.
    3번 모터는 x축 회전을 담당함. 0 ~ 90도, 0도시 다리가 앞을 향함. 90도시 다리가 y축과 평행하게 펴짐.
    4번 모터는 바퀴를 담당함. dir == false 회전시 앞으로 굴러감

    *각도는 오른쪽 다리로 설명함. 왼쪽 다리는 yz평면 대칭
    1번 모터와 2번 모터의 축의 중앙끼리의 거리 : +x축방향으로 70mm
    1번 모터와 2번 모터의 축의 중앙끼리의 각도 : x축 방향을 기준으로 y축 방향으로 15도 만큼 기울어짐

    2번 모터와 3번 모터의 축의 중앙끼리의 거리 : -z축방향으로 157.3mm
    2번 모터와 3번 모터의 축의 중앙끼리의 각도 : z축 방향을 기준으로 y축 방향으로 -12.2도 만큼 기울어짐
    3번 모터와 4번 모터의 축의 중앙끼리의 거리 : +z축방향으로 158.3mm
    3번 모터와 4번 모터의 축의 중앙끼리의 각도 : z축 방향을 기준으로 y축 방향으로 -5.1도 만큼 기울어짐
    4번 모터의 축의 중앙과 바퀴의 거리 : -x축방향으로 28mm
    바퀴의 지름(바퀴는 yz평면에서의 2차원 원이라 가정) : 50mm

좌표 : 1번모터의 축의 중심부터 바퀴가 지면에 닿는 점까지의 거리
'''


'''
역기구학 (Inverse Kinematics)
좌표 (x, y, z) : 1번 모터 축 중심 → 바퀴 지면 접촉점까지의 벡터
반환 : 모터 1, 2, 3번의 각도 (degree)

좌표계:
    x축: 로봇의 왼쪽 → 오른쪽
    y축: 로봇의 아래 → 위  (지면 방향이 -y)
    z축: 로봇의 뒤  → 앞

링크 파라미터 (오른쪽 다리 기준):
    1번 모터 → 2번 모터: +x 방향 70mm, x축 기준 y방향으로 15도 기울어짐
    2번 모터 → 3번 모터: -z 방향 157.3mm, z축 기준 y방향으로 -12.2도 기울어짐
    3번 모터 → 4번 모터: +z 방향 158.3mm, z축 기준 y방향으로 -5.1도 기울어짐
    4번 모터 → 바퀴 중심: -x 방향 28mm
    바퀴 반지름: 25mm

모터 각도 정의:
    1번: z축 회전. 입력 0 → 실제 -15도(바깥). 입력 28 → 실제 13도(안쪽).
         범위: 0~28도 입력 → real -15°~+13°
         m1_real = m1_input - 15  (바깥 방향이 음수)
         p2 = (side*L01*cos(m1_real), L01*sin(m1_real), 0)
         ※ 실용 범위: m1_input ≥ 15 (real ≥ 0°, 2번모터가 위쪽에 위치)
            m1_input < 15 구간은 2번모터가 아래(-y)로 내려가는 비정상 자세

    2번: x축 회전. 0도=다리 뒤(-z). 90도=다리 위(+y).
         FK: link2 = (0, L12*sin(m2_rad + tilt12), -L12*cos(m2_rad + tilt12))
    3번: x축 회전. 0도=다리 앞(+z). 90도=다리 위(+y).
         FK: link3 = (0, L23*sin(m3_rad + tilt23), +L23*cos(m3_rad + tilt23))

왼쪽 다리: yz평면 대칭 (x부호 반전, side=-1)

[수정 내역 vs 원본]
    버그1 FK - 1번 모터 실제각도 공식:
        원본: m1_real = m1_input + 15  → 입력 90 시 실제 105도 (조건 불일치)
        수정: m1_real = m1_input - 15  → 입력 0 시 -15도, 입력 90 시 75도

    버그2 FK - 1번 모터 이중 회전:
        원본: (ox,oy) 벡터를 a1=side*m1_real 만큼 다시 z축 회전
              ox*cos(a1)-oy*sin(a1) = L01*cos(15°+m1_real) → 15도가 두 번 적용
        수정: p2 = (side*L01*cos(m1_real), L01*sin(m1_real), 0)
              m1_real이 곧 x-y 평면에서 x축으로부터의 각도

    버그3 IK - 2번 모터 atan2 기준점 부호:
        원본: phi = atan2(dy, dz)  → +z 기준 (FK의 -z 기준과 180° 불일치)
        수정: phi = atan2(dy, -dz) → -z 기준 (FK 정의와 일치)

    버그4 IK - 1번 모터 역산 (cos 부호 모호성):
        cos(m1_real) = p4.x/(side*L01) 에서 acos는 항상 양수 반환
        → m1_real < 0 구간에서 대칭해(양수) 반환
        수정: acos 결과(양수)를 먼저 시도, 안되면 음수 후보도 시도
              실용 범위(m1≥15, real≥0)에서는 항상 올바른 해 반환

    버그5 - 도달 불가 테스트 케이스:
        원본: x=70mm (구조적 한계 ≈39.6mm 초과)
        수정: FK로 실제 좌표 생성 후 왕복 검증
'''

import math
import numpy as np


# ──────────────────────────────────────────────
# 링크 상수
# ──────────────────────────────────────────────
L01_dist = 70.0                      # 1번→2번 모터 거리 (mm)
L12_dist = 157.3                     # 2번→3번 모터 거리 (mm)
L12_tilt = math.radians(-12.2)       # z축 기준 y방향 -12.2도
L23_dist = 158.3                     # 3번→4번 모터 거리 (mm)
L23_tilt = math.radians(-5.1)        # z축 기준 y방향 -5.1도
L34_dist = 28.0                      # 4번 모터→바퀴 중심 (-x 방향, mm)
WHEEL_R  = 25.0                      # 바퀴 반지름 (mm)


# ──────────────────────────────────────────────
# 정기구학 (Forward Kinematics)
# ──────────────────────────────────────────────
def forward_kinematics(m1_deg, m2_deg, m3_deg, right_leg=True):
    '''
    모터 각도 → 바퀴 지면 접촉점 좌표 (1번 모터 축 기준)

    Args:
        m1_deg: 1번 모터 입력 각도 (0~28)
        m2_deg: 2번 모터 입력 각도 (0~90)
        m3_deg: 3번 모터 입력 각도 (0~90)
        right_leg: True=오른쪽, False=왼쪽
    Returns:
        (x, y, z) mm
    '''
    side = 1.0 if right_leg else -1.0

    # 1번 모터: z축 회전, m1_real = input - 15도
    # x-y 평면에서 x축으로부터의 각도
    m1_real = math.radians(m1_deg - 15.0)
    p2 = np.array([
        side * L01_dist * math.cos(m1_real),
        L01_dist * math.sin(m1_real),
        0.0
    ])

    # 2번 모터: x축 회전, -z 기준 0도
    a2 = math.radians(m2_deg) + L12_tilt
    p3 = p2 + np.array([
        0.0,
        L12_dist * math.sin(a2),
        -L12_dist * math.cos(a2)
    ])

    # 3번 모터: x축 회전, +z 기준 0도
    a3 = math.radians(m3_deg) + L23_tilt
    p4 = p3 + np.array([
        0.0,
        L23_dist * math.sin(a3),
        L23_dist * math.cos(a3)
    ])

    # 4번 모터 → 바퀴 중심 (-x), 바퀴 → 지면 접촉 (-y)
    contact = p4 + np.array([-side * L34_dist, -WHEEL_R, 0.0])
    return tuple(contact)


# ──────────────────────────────────────────────
# 역기구학 (Inverse Kinematics)
# ──────────────────────────────────────────────
def inverse_kinematics(target_x, target_y, target_z, right_leg=True):
    '''
    바퀴 지면 접촉점 좌표 → 모터 입력 각도

    Args:
        target_x, target_y, target_z: 목표 좌표 (mm), 1번 모터 축 기준
        right_leg: True=오른쪽, False=왼쪽
    Returns:
        dict { "m1", "m2", "m3": float (도), "valid": bool, "error": str|None }

    Note:
        1번 모터의 cos(m1_real) = p4.x/(side*L01) 로 역산할 때,
        acos는 항상 양수를 반환하므로 m1_real < 0 구간(input < 15)에서
        대칭된 양수 해가 반환될 수 있습니다.
        실용 운용 범위(m1_input ≥ 15, real ≥ 0°)에서는 항상 정확합니다.
        두 후보(+acos, -acos)를 모두 시도하며 유효한 해를 우선 반환합니다.
    '''
    side = 1.0 if right_leg else -1.0

    # Step 1: 지면 접촉점 → 4번 모터 축 위치
    p4 = np.array([
        target_x + side * L34_dist,
        target_y + WHEEL_R,
        target_z
    ])

    # Step 2: 1번 모터 역산
    # FK: p4.x = p2.x = side*L01*cos(m1_real)  (2,3번은 x축 회전이므로 x 불변)
    # → cos(m1_real) = p4.x / (side * L01)
    cos_m1 = p4[0] / (side * L01_dist)
    cos_m1 = max(-1.0, min(1.0, cos_m1))
    acos_val = math.acos(cos_m1)

    # Step 3: 두 후보 시도 (양수 먼저 → 실용 범위 우선)
    for m1_real in [acos_val, -acos_val]:
        m1_input = math.degrees(m1_real) + 15.0
        if not (0.0 <= m1_input <= 28.0):
            continue

        p2 = np.array([
            side * L01_dist * math.cos(m1_real),
            L01_dist * math.sin(m1_real),
            0.0
        ])

        # 2번·3번 모터: yz 평면 2-link IK
        dp = p4 - p2
        dy, dz = dp[1], dp[2]
        D = math.sqrt(dy**2 + dz**2)
        L1, L2 = L12_dist, L23_dist

        if D > L1 + L2 or D < abs(L1 - L2):
            continue

        cos_alpha = (L1**2 + D**2 - L2**2) / (2 * L1 * D)
        alpha = math.acos(max(-1.0, min(1.0, cos_alpha)))

        # 2번 모터: FK z = -L12*cos(a2) → -z 기준 → phi = atan2(dy, -dz)
        phi = math.atan2(dy, -dz)
        link1_angle = phi - alpha
        m2_input = math.degrees(link1_angle - L12_tilt)

        if not (0.0 <= m2_input <= 90.0):
            continue

        # p3 위치

        p3 = p2 + L1 * np.array([0.0, math.sin(link1_angle), -math.cos(link1_angle)])

        # 3번 모터: FK z = +L23*cos(a3) → +z 기준 → atan2(dy, dz)
        dp34 = p4 - p3
        link2_angle = math.atan2(dp34[1], dp34[2])
        m3_input = math.degrees(link2_angle - L23_tilt)

        if not (0.0 <= m3_input <= 90.0):
            continue

        return {
            "m1": round(m1_input, 2),
            "m2": round(m2_input, 2),
            "m3": round(m3_input, 2),
            "valid": True,
            "error": None
        }

    return {
        "m1": None, "m2": None, "m3": None,
        "valid": False,
        "error": f"도달 불가 또는 각도 범위 초과: target=({target_x:.1f},{target_y:.1f},{target_z:.1f})"
    }


# ──────────────────────────────────────────────
# 테스트
# ──────────────────────────────────────────────
if __name__ == "__main__":
    print("=" * 68)
    print("역기구학 테스트  (FK → IK 왕복 검증)")
    print("=" * 68)

    print(f"\n[구조 정보]")
    print(f"  1번 모터 범위: 0~28도  (real: -15°~+13°)")
    print(f"  최대 도달 x (m1=15, real=0°):   "
          f"{L01_dist - L34_dist:.1f} mm")
    print(f"  최소 도달 x (m1=28, real=13°):  "
          f"{L01_dist*math.cos(math.radians(13)) - L34_dist:.1f} mm")
    print(f"  2+3번 링크 합: {L12_dist+L23_dist:.1f} mm")
    print(f"  2+3번 링크 차: {abs(L12_dist-L23_dist):.1f} mm")
    print(f"  ※ m1_input ≥ 15 (real ≥ 0°) 가 정상 운용 범위\n")

    cases = [
        # (m1, m2, m3, right_leg, 설명)
        (15, 45, 45, True,  "오른쪽 m1=15 기본 직립"),
        (15, 30, 60, True,  "오른쪽 m1=15 무릎 굽힘"),
        (15, 60, 30, True,  "오른쪽 m1=15 무릎 펼침"),
        (20, 45, 45, True,  "오른쪽 m1=20 벌림"),
        (28, 45, 45, True,  "오른쪽 m1=28 최대"),
        (28, 30, 60, True,  "오른쪽 m1=28 무릎굽힘"),
        (15, 45, 45, False, "왼쪽  m1=15 기본 직립"),
        (20, 30, 60, False, "왼쪽  m1=20 무릎굽힘"),
        (28, 60, 30, False, "왼쪽  m1=28 무릎펼침"),
    ]

    print(f"{'케이스':<22} {'입력':>10} {'FK좌표':>32} {'IK역산':>14}  {'오차':>10}  판정")
    print("─" * 106)
    all_pass = True
    for m1, m2, m3, rl, label in cases:
        tx, ty, tz = forward_kinematics(m1, m2, m3, rl)
        res = inverse_kinematics(tx, ty, tz, rl)

        if res["valid"]:
            fx, fy, fz = forward_kinematics(res["m1"], res["m2"], res["m3"], rl)
            err = math.sqrt((fx-tx)**2 + (fy-ty)**2 + (fz-tz)**2)
            ok = err < 0.001
            if not ok: all_pass = False
            angle_match = (abs(res["m1"]-m1)<0.01 and
                           abs(res["m2"]-m2)<0.01 and
                           abs(res["m3"]-m3)<0.01)
            print(f"{label:<22} ({m1:2},{m2:2},{m3:2})"
                  f"  ({tx:7.2f},{ty:7.2f},{tz:7.2f})"
                  f"  ({res['m1']:5.1f},{res['m2']:5.1f},{res['m3']:5.1f})"
                  f"  {err:9.4f}mm  {'✓' if ok else '✗'}"
                  f"{'  각도✓' if angle_match else '  각도⚠(복수해)'}")
        else:
            all_pass = False
            print(f"{label:<22} IK 실패: {res['error']}")

    print("─" * 106)
    print(f"전체 결과: {'전부 통과 ✓' if all_pass else '일부 실패 ✗'}\n")

    print("사용 예시:")
    print("  result = inverse_kinematics(30.0, -160.0, 0.0)")
    print("  set_servo_deg(1, result['m1'])")
    print("  set_servo_deg(2, result['m2'])")
    print("  set_servo_deg(3, result['m3'])")
    print("  communicate()")