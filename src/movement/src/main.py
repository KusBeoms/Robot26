"""
2026 kusbeoms

main.py
로봇 메인 루프. 라즈베리파이에서 직접 실행.

구조:
    ImuReader        : MPU6050 100Hz 읽기 + Madgwick 필터
    RobotNode        : I2C 마스터, 4개 Arduino 폴링
    GaitController   : 방향각 + 크기 -> trot gait

실행:
    python main.py

종료:
    Ctrl+C
"""

import math
import time
import logging

from imu_reader import ImuReader
from robot_node import RobotNode
from gait_controller import GaitController

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("main")

# ──────────────────────────────────────────────
# 설정
# ──────────────────────────────────────────────
MOCK = False          # True: smbus2 없이 테스트 / False: 실제 하드웨어
POLL_INTERVAL = 0.05  # RobotNode 폴링 주기 (초, 20Hz)
IMU_INTERVAL  = 0.01  # IMU 읽기 주기 (초, 100Hz)
IMU_UPDATE_RATE = 0.02  # GaitController에 IMU 전달 주기 (초, 50Hz)


def main():
    log.info("Neuver 로봇 시작")

    # ── 초기화 ──────────────────────────────
    imu  = ImuReader(interval=IMU_INTERVAL, mock=MOCK)
    node = RobotNode(poll_interval_s=POLL_INTERVAL, mock=MOCK)
    gait = GaitController(node)

    imu.start()
    node.start()
    gait.start()

    log.info("모든 서비스 시작 완료")
    log.info("Ctrl+C 로 종료")

    # ── 명령 예시 (실제로는 소켓/파이프/조이스틱 등으로 대체) ──
    # 여기서는 간단한 시퀀스 데모
    commands = [
        (0.0,        0.8, 2.0, "전진"),
        (math.pi/4,  0.6, 2.0, "오른쪽 대각"),
        (math.pi,    0.6, 2.0, "후진"),
        (-math.pi/4, 0.6, 2.0, "왼쪽 대각"),
        (0.0,        0.0, 1.5, "정지"),
    ]

    try:
        for angle, mag, duration, label in commands:
            log.info("[명령] %s  angle=%.1fdeg  mag=%.1f  duration=%.1fs",
                     label, math.degrees(angle), mag, duration)
            gait.set_command(angle=angle, magnitude=mag)

            t_end = time.monotonic() + duration
            while time.monotonic() < t_end:
                # IMU -> GaitController 매 루프 전달
                gait.update_imu(*imu.quaternion)

                roll, pitch, yaw = imu.euler_deg
                log.debug("IMU roll=%.1f pitch=%.1f yaw=%.1f", roll, pitch, yaw)

                time.sleep(IMU_UPDATE_RATE)

    except KeyboardInterrupt:
        log.info("Ctrl+C 감지 - 종료 중...")

    finally:
        log.info("정지 명령 전송")
        gait.set_command(angle=0.0, magnitude=0.0)
        time.sleep(0.5)

        gait.stop()
        node.stop()
        imu.stop()
        log.info("종료 완료")


if __name__ == "__main__":
    main()