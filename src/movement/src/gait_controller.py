"""
2026 kusbeoms

gait_controller.py
방향각 + 크기 -> 4다리 trot gait 제어기.

입력:
    angle     : 이동 방향각 (rad, 로봇 전방 = 0, 반시계 양수)
    magnitude : 이동 속도 (0.0 ~ 1.0)

몸통 치수:
    z축 (앞뒤, 1번모터 중심간): 296.5mm
    x축 (좌우, 1번모터 중심간): 126.7mm  (152.5 - 12.9*2)

다리 위상 (Trot):
    Group A: front_right, rear_left   -> phase 0
    Group B: front_left,  rear_right  -> phase pi

주의:
    1번 모터 x 범위 40~42mm로 좁음 -> x stride 불가.
    전방향 이동은 z stride 투영 + 앞뒤 다리 차동으로 처리.
"""

import math
import time
import threading
import logging
from typing import Optional, Callable

from robot_node import RobotNode

log = logging.getLogger("gait")

# ──────────────────────────────────────────────
# 중립 자세 (FK 검증: IK(41,170,0) valid)
# ──────────────────────────────────────────────
NEUTRAL_X = 41.0
NEUTRAL_Y = 170.0
NEUTRAL_Z = 0.0

# ──────────────────────────────────────────────
# 보행 파라미터
# ──────────────────────────────────────────────
STEP_LENGTH  = 40.0   # 한 걸음 최대 z 이동거리 (mm)
STEP_HEIGHT  = 30.0   # 발을 드는 최대 높이 (mm, y 감소량)
CYCLE_PERIOD = 0.6    # 한 사이클 시간 (초)
UPDATE_RATE  = 0.02   # 제어 루프 주기 (초, 50Hz)

# ──────────────────────────────────────────────
# 관성 LPF
# ──────────────────────────────────────────────
LPF_ALPHA       = 0.08
LPF_ANGLE_ALPHA = 0.12
MIN_MAGNITUDE   = 0.02

# ──────────────────────────────────────────────
# 다리 설정
# ──────────────────────────────────────────────
LEG_CONFIG = {
    "front_right": {"phase": 0.0,        "right": True,  "front": True },
    "rear_left":   {"phase": 0.0,        "right": False, "front": False},
    "front_left":  {"phase": math.pi,    "right": False, "front": True },
    "rear_right":  {"phase": math.pi,    "right": True,  "front": False},
}


# ──────────────────────────────────────────────
# 발 궤적
# ──────────────────────────────────────────────
def foot_trajectory(phase_t, step_z, height):
    """
    위상 t (0 ~ 2pi) 에서 발의 (dy, dz) 오프셋 반환.

    Stance (0 ~ pi): 발이 지면에서 뒤로 밀림 (step_z 반대)
    Swing  (pi ~ 2pi): 발이 공중에서 앞으로 이동 + arc

    arc: sin(t_sw * pi) * height -> 0 -> height -> 0
    """
    if phase_t < math.pi:
        t_s = phase_t / math.pi
        dz  = step_z * (0.5 - t_s)
        dy  = 0.0
    else:
        t_sw = (phase_t - math.pi) / math.pi
        dz   = step_z * (t_sw - 0.5)
        dy   = height * math.sin(t_sw * math.pi)
    return dy, dz


# ──────────────────────────────────────────────
# 방향각 LPF
# ──────────────────────────────────────────────
def angle_lpf(current, target, alpha):
    diff = target - current
    while diff >  math.pi: diff -= 2 * math.pi
    while diff < -math.pi: diff += 2 * math.pi
    return current + alpha * diff


# ──────────────────────────────────────────────
# GaitController
# ──────────────────────────────────────────────
class GaitController:
    """
    방향각 + 크기 -> 4다리 trot gait.

    사용법:
        node = RobotNode(poll_interval_s=0.05, mock=True)
        node.start()
        gait = GaitController(node)
        gait.start()
        gait.set_command(angle=0.0, magnitude=0.8)   # 전진
        gait.stop()
        node.stop()
    """

    def __init__(self, node, step_length=STEP_LENGTH,
                 step_height=STEP_HEIGHT, cycle_period=CYCLE_PERIOD):
        self.node         = node
        self.step_length  = step_length
        self.step_height  = step_height
        self.cycle_period = cycle_period

        self._cmd_angle     = 0.0
        self._cmd_magnitude = 0.0
        self._lock          = threading.Lock()

        self._filt_magnitude = 0.0
        self._filt_angle     = 0.0

        self._phase = {leg: cfg["phase"] for leg, cfg in LEG_CONFIG.items()}

        self._running = False
        self._thread  = None
        self._imu_q   = (1.0, 0.0, 0.0, 0.0)

        self.on_step: Optional[Callable] = None

    # ── 외부 인터페이스 ───────────────────────
    def set_command(self, angle, magnitude):
        with self._lock:
            self._cmd_angle     = angle
            self._cmd_magnitude = max(0.0, min(1.0, magnitude))

    def update_imu(self, qw, qx, qy, qz):
        with self._lock:
            self._imu_q = (qw, qx, qy, qz)

    def start(self):
        if self._running:
            return
        self._running = True
        self._thread  = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        log.info("GaitController started")

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        for leg, cfg in LEG_CONFIG.items():
            xs = 1.0 if cfg["right"] else -1.0
            self.node.set_target(leg, xs * NEUTRAL_X, NEUTRAL_Y, NEUTRAL_Z)
        log.info("GaitController stopped")

    # ── 제어 루프 ────────────────────────────
    def _loop(self):
        dt = UPDATE_RATE
        phase_speed = (2 * math.pi) / self.cycle_period

        while self._running:
            t0 = time.monotonic()

            with self._lock:
                cmd_angle = self._cmd_angle
                cmd_mag   = self._cmd_magnitude

            # LPF (관성)
            self._filt_magnitude += LPF_ALPHA * (cmd_mag - self._filt_magnitude)
            self._filt_angle = angle_lpf(self._filt_angle, cmd_angle, LPF_ANGLE_ALPHA)

            moving = self._filt_magnitude > MIN_MAGNITUDE
            dy_imu = self._imu_correction()

            mag_scaled = self._filt_magnitude * self.step_length

            # 전후 stride: z 투영
            step_z = mag_scaled * math.cos(self._filt_angle)

            # 좌우 이동 보조: 앞다리/뒷다리 차동 stride
            # angle=90deg(오른쪽): 앞다리 +z, 뒷다리 -z -> 오른쪽 선회 유사
            side_delta = mag_scaled * math.sin(self._filt_angle) * 0.5

            for leg, cfg in LEG_CONFIG.items():
                xs = 1.0 if cfg["right"] else -1.0

                # pitch 차동 보정: 앞다리 +, 뒷다리 - (앞으로 기울면 앞다리 더 뻗고 뒷다리 줄임)
                imu_corr = dy_imu if cfg["front"] else -dy_imu

                if moving:
                    self._phase[leg] = (
                        self._phase[leg] + phase_speed * dt) % (2 * math.pi)

                    sz = step_z + (side_delta if cfg["front"] else -side_delta)

                    dy_swing, dz = foot_trajectory(self._phase[leg], sz, self.step_height)

                    target_x = xs * NEUTRAL_X
                    target_y = NEUTRAL_Y - dy_swing + imu_corr
                    target_z = NEUTRAL_Z + dz
                else:
                    target_x = xs * NEUTRAL_X
                    target_y = NEUTRAL_Y + imu_corr
                    target_z = NEUTRAL_Z

                self.node.set_target(leg, target_x, target_y, target_z)

            if self.on_step:
                try:
                    self.on_step(self._filt_magnitude, self._filt_angle, dict(self._phase))
                except Exception:
                    pass

            elapsed = time.monotonic() - t0
            sleep_t = dt - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)

    # ── IMU 보정 ─────────────────────────────
    def _imu_correction(self):
        with self._lock:
            qw, qx, qy, qz = self._imu_q
        sinp = 2.0 * (qw * qy - qz * qx)
        sinp = max(-1.0, min(1.0, sinp))
        pitch = math.asin(sinp)
        # pitch 양수 = 앞으로 기울어짐 -> 앞발이 더 내려가야(y 증가 필요)
        # NEUTRAL_Y에서 dy_swing을 빼는 구조이므로 +pitch -> dy_imu 음수 -> y 증가
        # 최대 ±30도(pi/6)에서 ±20mm 보정
        return pitch * 20.0 / (math.pi / 6)


# ──────────────────────────────────────────────
# 직접 실행 데모
# ──────────────────────────────────────────────

if __name__ == "__main__":
    import time as _t
    logging.basicConfig(level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
        datefmt="%H:%M:%S")
    node = RobotNode(poll_interval_s=0.05, mock=True)
    node.start()
    gait = GaitController(node)
    gait.start()
    try:
        print("[1] forward")
        gait.set_command(angle=0.0, magnitude=0.8)
        _t.sleep(2.0)
        print("[2] diagonal")
        gait.set_command(angle=0.7854, magnitude=0.6)
        _t.sleep(2.0)
        print("[3] backward")
        gait.set_command(angle=3.1416, magnitude=0.5)
        _t.sleep(2.0)
        print("[4] stop")
        gait.set_command(angle=0.0, magnitude=0.0)
        _t.sleep(2.0)
    finally:
        gait.stop()
        node.stop()
        print("done.")