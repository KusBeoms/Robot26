#!/usr/bin/env python3
"""
접힌 자세(0,0,0) → 기본자세 → 접힌 자세(0,0,0) 로 천천히 이동하는 테스트.

사용법:
    python3 stretch_legs.py [--duration 초] [--steps 단계수]
"""

import sys
import os
import time
import argparse
import logging

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from robot_node import RobotNode, LEG_ADDR
from inverse_kinematics import inverse_kinematics

# ── 기본 자세 좌표 (1번모터 축 기준, mm)
# 오른쪽 다리: x 양수, 왼쪽 다리: x 음수 (좌우 대칭)
DEFAULT_Y   = 170.0
DEFAULT_Z   = 0.0
DEFAULT_X_R =  41.0   # 오른쪽
DEFAULT_X_L = -41.0   # 왼쪽

# ── 접힌 자세 모터 각도
FOLDED_ANGLES = (0.0, 0.0, 0.0)

RIGHT_LEGS = {"front_right", "rear_right"}

log = logging.getLogger("stretch_legs")


def lerp(a: float, b: float, t: float) -> float:
    return a + (b - a) * t


def send_all(node: RobotNode, angles: dict[str, tuple[float, float, float]]):
    """모든 다리에 동시에 각도 명령 전송."""
    for leg, (m1, m2, m3) in angles.items():
        node._send_leg(leg, m1, m2, m3)


def interpolate(
    node: RobotNode,
    start: dict,
    end: dict,
    steps: int,
    step_delay: float,
    label: str,
):
    """start → end 각도를 steps 단계로 보간하며 전송."""
    print(f"\n{label}  ({steps}단계, {steps * step_delay:.1f}초)")
    for i in range(1, steps + 1):
        t = i / steps
        angles = {
            leg: (
                lerp(start[leg][0], end[leg][0], t),
                lerp(start[leg][1], end[leg][1], t),
                lerp(start[leg][2], end[leg][2], t),
            )
            for leg in LEG_ADDR
        }
        send_all(node, angles)
        print(f"  [{i:3d}/{steps}] t={t:.2f}", end="\r", flush=True)
        time.sleep(step_delay)
    print()


def main():
    parser = argparse.ArgumentParser(description="다리 펴기/접기 테스트")
    parser.add_argument("--duration", type=float, default=3.0,
                        help="펴기/접기 각각 걸리는 시간(초). 기본 3.0")
    parser.add_argument("--steps",    type=int,   default=30,
                        help="보간 단계 수. 기본 30")
    parser.add_argument("--hold",     type=float, default=2.0,
                        help="기본자세 유지 시간(초). 기본 2.0")
    parser.add_argument("--mock",     action="store_true",
                        help="실제 I2C 없이 MockBus 로 동작")
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.WARNING,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
        datefmt="%H:%M:%S",
    )

    step_delay = args.duration / args.steps

    # ── 기본 자세 IK 계산
    print("기본자세 IK 계산...")
    default_angles: dict[str, tuple] = {}
    for leg in LEG_ADDR:
        right = leg in RIGHT_LEGS
        tx = DEFAULT_X_R if right else DEFAULT_X_L
        result = inverse_kinematics(tx, DEFAULT_Y, DEFAULT_Z, right_leg=right)
        if not result["valid"]:
            print(f"[ERROR] {leg} IK 실패: {result['error']}")
            sys.exit(1)
        default_angles[leg] = (result["m1"], result["m2"], result["m3"])
        print(f"  {leg:<14} m1={result['m1']:5.1f}°  m2={result['m2']:5.1f}°  m3={result['m3']:5.1f}°")

    folded_angles = {leg: FOLDED_ANGLES for leg in LEG_ADDR}

    # ── RobotNode 초기화 (폴링 스레드 없이 직접 전송)
    node = RobotNode(mock=args.mock)

    try:
        # 1. 접힌 자세로 초기화
        print("\n[1] 접힌 자세(0°, 0°, 0°)로 이동...")
        send_all(node, folded_angles)
        time.sleep(1.0)

        # 2. 기본자세로 천천히 펴기
        interpolate(
            node, folded_angles, default_angles,
            steps=args.steps, step_delay=step_delay,
            label="[2] 기본자세로 펴기",
        )

        # 3. 기본자세 유지
        print(f"기본자세 유지 {args.hold:.1f}초...")
        time.sleep(args.hold)

        # 4. 접힌 자세로 천천히 접기
        interpolate(
            node, default_angles, folded_angles,
            steps=args.steps, step_delay=step_delay,
            label="[3] 접힌 자세로 접기",
        )

        print("완료.")

    finally:
        node._bus.close()


if __name__ == "__main__":
    main()
