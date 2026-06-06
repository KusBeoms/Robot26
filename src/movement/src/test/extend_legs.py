#!/usr/bin/env python3
"""
접힌 자세(0°,0°,0°) → 펴진 자세 → 접힌 자세 테스트.
앞다리: m1_real=-10°, m2=30°, m3=75° / 뒷다리: m1_real=-10°, m2=35°, m3=80°
IK 없이 직접 각도 지정. 응답 없는 다리는 자동 제외.

사용법:
    python3 extend_legs.py [--duration 초] [--steps 단계수] [--hold 초] [--mock]
"""

import sys
import os
import time
import argparse
import logging

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from robot_node import RobotNode, LEG_ADDR

# m1 real = input - 15
# 앞다리: m1_real=-10° → input=5°,  m2=30°, m3=75°
# 뒷다리: m1_real=-10° → input=5°,  m2=35°, m3=80°
FOLDED_ANGLES    = (0.0,  0.0,  0.0)
FRONT_EXTENDED   = (5.0, 30.0, 75.0)
REAR_EXTENDED    = (5.0, 35.0, 80.0)
FRONT_LEGS       = {"front_left", "front_right"}

log = logging.getLogger("extend_legs")


def lerp(a: float, b: float, t: float) -> float:
    return a + (b - a) * t


def send_all(node: RobotNode, angles: dict[str, tuple[float, float, float]]):
    for leg, (m1, m2, m3) in angles.items():
        node._send_leg(leg, m1, m2, m3)


def interpolate(node: RobotNode, start: dict, end: dict, steps: int, step_delay: float, label: str):
    legs = list(start.keys())
    print(f"\n{label}  ({steps}단계, {steps * step_delay:.1f}초)")
    for i in range(1, steps + 1):
        t = i / steps
        t0 = time.monotonic()
        angles = {
            leg: (
                lerp(start[leg][0], end[leg][0], t),
                lerp(start[leg][1], end[leg][1], t),
                lerp(start[leg][2], end[leg][2], t),
            )
            for leg in legs
        }
        send_all(node, angles)
        elapsed = time.monotonic() - t0
        remaining = step_delay - elapsed
        if remaining > 0:
            time.sleep(remaining)
        print(f"  [{i:3d}/{steps}] t={t:.2f}", end="\r", flush=True)
    print()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=3.0,
                        help="펴기/접기 각각 걸리는 시간(초). 기본 3.0")
    parser.add_argument("--steps",    type=int,   default=200,
                        help="보간 단계 수. 기본 200")
    parser.add_argument("--hold",     type=float, default=2.0,
                        help="펴진 자세 유지 시간(초). 기본 2.0")
    parser.add_argument("--mock",     action="store_true",
                        help="MockBus — 실제 하드웨어 없이 테스트")
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.WARNING,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
        datefmt="%H:%M:%S",
    )

    step_delay = args.duration / args.steps

    node = RobotNode(mock=args.mock)
    active = node.active_legs

    if not active:
        print("[ERROR] 응답하는 다리가 없습니다. 배선/전원을 확인하세요.")
        node._bus.close()
        sys.exit(1)

    print(f"\n활성 다리 : {sorted(active)}")
    dead = set(LEG_ADDR) - active
    if dead:
        print(f"제외된 다리: {sorted(dead)}")

    print(f"\n앞다리 목표: m1_input=5°(real=-10°)  m2=30°  m3=75°")
    print(f"뒷다리 목표: m1_input=5°(real=-10°)  m2=35°  m3=80°")

    folded   = {leg: FOLDED_ANGLES for leg in active}
    extended = {
        leg: (FRONT_EXTENDED if leg in FRONT_LEGS else REAR_EXTENDED)
        for leg in active
    }

    try:
        # 1. 접힌 자세
        print("\n[1] 접힌 자세(0°, 0°, 0°)로 이동...")
        send_all(node, folded)
        time.sleep(1.0)

        # 2. 완전히 펴기
        interpolate(node, folded, extended,
                    steps=args.steps, step_delay=step_delay,
                    label="[2] 다리 펴기 (앞: -10°/30°/75°, 뒤: -10°/35°/80°)")

        print(f"펴진 자세 유지 {args.hold:.1f}초...")
        time.sleep(args.hold)

        # 3. 접힌 자세로 복귀
        interpolate(node, extended, folded,
                    steps=args.steps, step_delay=step_delay,
                    label="[3] 접힌 자세로 복귀")

        print("완료.")

    finally:
        node._bus.close()


if __name__ == "__main__":
    main()
