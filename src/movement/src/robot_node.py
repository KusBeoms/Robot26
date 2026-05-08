'''
2026 kusbeoms

robot_node.py
라즈베리파이 로봇 제어 노드.

- 목표 좌표(x, y, z)가 set_target()으로 바뀌면 → IK 계산 → 자동 I2C 전송
- 백그라운드 스레드가 주기적으로 모든 Arduino 상태를 폴링(수신)
- 콜백(on_update)을 등록하면 수신 때마다 호출됨

구조:
    RobotNode
    ├── LegNode × 4  (front_left, rear_left, front_right, rear_right)
    │     └── 목표 좌표, 마지막 IK 결과, 바퀴 상태
    └── 폴링 스레드 (poll_interval_s마다 _receive 실행)

I2C 주소:
    0x08 front_left   (왼쪽)
    0x09 rear_left    (왼쪽)
    0x0A front_right  (오른쪽)
    0x0B rear_right   (오른쪽)

의존성:
    pip install smbus2 numpy
'''

import struct
import threading
import time
import logging
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Tuple

try:
    import smbus2
    _SMBUS_AVAILABLE = True
except ImportError:
    _SMBUS_AVAILABLE = False

from inverse_kinematics import inverse_kinematics

# ──────────────────────────────────────────────
# 로깅
# ──────────────────────────────────────────────
log = logging.getLogger("robot_node")

# ──────────────────────────────────────────────
# 상수
# ──────────────────────────────────────────────
I2C_BUS      = 1        # 라즈베리파이 기본 I2C 버스
RESPONSE_LEN = 20       # requestEvent() 응답 바이트 수

LEG_ADDR: Dict[str, int] = {
    "front_left":  0x08,
    "rear_left":   0x09,
    "front_right": 0x0A,
    "rear_right":  0x0B,
}
RIGHT_LEGS = {"front_right", "rear_right"}


# ──────────────────────────────────────────────
# 각도 변환
# ──────────────────────────────────────────────
def degree_to_step(degree: float) -> int:
    '''각도(0~240도) → 모터 단위(0~1000).'''
    degree = max(0.0, min(240.0, float(degree)))
    return int(round(degree * 4.16667))

def step_to_degree(step: int) -> float:
    '''모터 단위(0~1000) → 각도(0~240도).'''
    return step / 4.16667


# ──────────────────────────────────────────────
# 데이터 클래스
# ──────────────────────────────────────────────
@dataclass
class MotorState:
    '''Arduino에서 수신한 모터 상태 (20바이트 파싱 결과).'''
    position: List[int]   = field(default_factory=lambda: [0, 0, 0, 0])   # 현재 위치 (모터단위, int16 × 4)
    current:  int         = 0    # 전류 (uint16, mA 단위 환산 전 raw)
    accel_debug: int      = 0    # 가속도 합계 (디버그, int16)
    next_pos: List[int]   = field(default_factory=lambda: [0, 0, 0, 0])   # 다음 목표위치 (디버그, int16 × 4)

    def position_deg(self) -> List[float]:
        return [step_to_degree(p) for p in self.position]

    def __str__(self) -> str:
        deg = [f"{d:.1f}°" for d in self.position_deg()]
        return (f"pos={self.position}({deg})  "
                f"cur={self.current}  accel={self.accel_debug}  "
                f"next={self.next_pos}")


@dataclass
class LegState:
    '''다리 하나의 전체 상태.'''
    # 목표 좌표 (mm, 1번 모터 축 기준)
    target: Tuple[float, float, float] = (41.0, 170.0, 0.0)
    # 마지막 IK 결과
    ik_result: Optional[dict] = None
    # 바퀴 상태
    wheel_dir: bool  = False   # False=앞, True=뒤
    wheel_speed: int = 0       # 0~100
    # Arduino 수신 상태
    motor: MotorState = field(default_factory=MotorState)


# ──────────────────────────────────────────────
# Mock I2C (smbus2 없을 때 테스트용)
# ──────────────────────────────────────────────
class _MockBus:
    def write_i2c_block_data(self, addr, reg, buf):
        log.debug(f"[MockI2C] TX addr=0x{addr:02X} buf={buf}")

    def read_i2c_block_data(self, addr, reg, length):
        log.debug(f"[MockI2C] RX addr=0x{addr:02X} len={length}")
        return [0] * length

    def close(self):
        pass


# ──────────────────────────────────────────────
# RobotNode
# ──────────────────────────────────────────────
class RobotNode:
    '''
    로봇 제어 노드.

    사용법:
        node = RobotNode(poll_interval_s=0.1)
        node.on_update = lambda leg, state: print(leg, state.motor)
        node.start()

        # 목표 좌표 변경 → 자동 IK + I2C 전송
        node.set_target("front_right", x=41.0, y=160.0, z=10.0)

        # 바퀴 제어
        node.set_wheel("front_right", rotate_dir=False, speed=50)

        # 종료
        node.stop()

    Args:
        poll_interval_s : 상태 폴링 주기 (초). 기본 0.1s (10Hz).
        send_on_set     : set_target() 호출 즉시 I2C 전송 여부. 기본 True.
        bus             : smbus2.SMBus 인스턴스 (None이면 자동 생성).
        mock            : True이면 실제 I2C 없이 MockBus 사용.
    '''

    def __init__(
        self,
        poll_interval_s: float = 0.1,
        send_on_set: bool = True,
        bus=None,
        mock: bool = False,
    ):
        self.poll_interval_s = poll_interval_s
        self.send_on_set     = send_on_set

        # I2C 버스
        if mock or not _SMBUS_AVAILABLE:
            if not mock:
                log.warning("smbus2 없음 → MockBus 사용")
            self._bus = _MockBus()
        else:
            self._bus = bus if bus is not None else smbus2.SMBus(I2C_BUS)

        # 다리 상태
        self._legs: Dict[str, LegState] = {name: LegState() for name in LEG_ADDR}
        self._lock = threading.Lock()

        # 폴링 스레드
        self._running   = False
        self._thread    = None

        # 콜백: on_update(leg_name: str, state: LegState)
        self.on_update: Optional[Callable[[str, LegState], None]] = None

    # ────────────────────────────────
    # 시작 / 종료
    # ────────────────────────────────
    def start(self):
        '''폴링 스레드 시작.'''
        if self._running:
            return
        self._running = True
        self._thread  = threading.Thread(target=self._poll_loop, daemon=True)
        self._thread.start()
        log.info("RobotNode started (poll=%.3fs)", self.poll_interval_s)

    def stop(self):
        '''폴링 스레드 종료 후 I2C 버스 닫기.'''
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        self._bus.close()
        log.info("RobotNode stopped")

    # ────────────────────────────────
    # 목표 설정 (서보)
    # ────────────────────────────────
    def set_target(self, leg: str, x: float, y: float, z: float) -> dict:
        '''
        다리 목표 좌표 설정.
        좌표가 이전과 다를 때만 IK 계산 + I2C 전송.

        Returns:
            IK 결과 dict (valid, m1, m2, m3, error)
        '''
        self._check_leg(leg)
        with self._lock:
            state = self._legs[leg]
            if state.target == (x, y, z):
                return state.ik_result or {"valid": False, "error": "변화 없음"}

            state.target = (x, y, z)
            right = leg in RIGHT_LEGS
            result = inverse_kinematics(x, y, z, right_leg=right)
            state.ik_result = result

        if result["valid"]:
            if self.send_on_set:
                self._send_leg(leg, result["m1"], result["m2"], result["m3"])
            log.debug("[%s] 목표 (%.1f,%.1f,%.1f) → m1=%.1f m2=%.1f m3=%.1f",
                      leg, x, y, z, result["m1"], result["m2"], result["m3"])
        else:
            log.warning("[%s] IK 실패: %s", leg, result["error"])

        return result

    def set_target_all(self, x: float, y: float, z: float):
        '''모든 다리를 같은 목표 좌표로 설정.'''
        for leg in LEG_ADDR:
            self.set_target(leg, x, y, z)

    # ────────────────────────────────
    # 바퀴 제어
    # ────────────────────────────────
    def set_wheel(self, leg: str, rotate_dir: bool, speed: int):
        '''
        바퀴 모터 제어.

        Args:
            leg        : 다리 이름
            rotate_dir : False=앞으로, True=뒤로
            speed      : 0~100
        '''
        self._check_leg(leg)
        with self._lock:
            state = self._legs[leg]
            state.wheel_dir   = rotate_dir
            state.wheel_speed = max(0, min(100, speed))

        self._send_wheel(leg, rotate_dir, speed)
        log.debug("[%s] 바퀴 dir=%s speed=%d", leg, rotate_dir, speed)

    def set_wheel_all(self, rotate_dir: bool, speed: int):
        '''모든 다리 바퀴를 같은 설정으로 제어.'''
        for leg in LEG_ADDR:
            self.set_wheel(leg, rotate_dir, speed)

    # ────────────────────────────────
    # 상태 조회
    # ────────────────────────────────
    def get_state(self, leg: str) -> LegState:
        '''다리 상태 반환 (읽기 전용 복사본).'''
        self._check_leg(leg)
        with self._lock:
            return self._legs[leg]

    def get_all_states(self) -> Dict[str, LegState]:
        '''전체 다리 상태 딕셔너리 반환.'''
        with self._lock:
            return dict(self._legs)

    def print_state(self, leg: str = None):
        '''상태 출력. leg 미지정 시 전체 출력.'''
        legs = [leg] if leg else list(LEG_ADDR.keys())
        with self._lock:
            for lg in legs:
                s = self._legs[lg]
                print(f"[{lg}]")
                print(f"  목표: {s.target}")
                ik = s.ik_result
                if ik and ik.get("valid"):
                    print(f"  IK : m1={ik['m1']}° m2={ik['m2']}° m3={ik['m3']}°")
                print(f"  모터: {s.motor}")

    # ────────────────────────────────
    # 내부: 폴링 루프
    # ────────────────────────────────
    def _poll_loop(self):
        while self._running:
            t0 = time.monotonic()
            for leg in LEG_ADDR:
                self._receive(leg)
            elapsed = time.monotonic() - t0
            sleep_t = self.poll_interval_s - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)

    # ────────────────────────────────
    # 내부: I2C 수신
    # ────────────────────────────────
    def _receive(self, leg: str):
        addr = LEG_ADDR[leg]
        try:
            raw = self._bus.read_i2c_block_data(addr, 0, RESPONSE_LEN)
        except Exception as e:
            log.error("[%s] I2C 수신 오류: %s", leg, e)
            return

        b = bytes(raw)
        with self._lock:
            m = self._legs[leg].motor
            m.position    = list(struct.unpack_from('<4h', b, 0))
            m.current     = struct.unpack_from('<H',  b, 8)[0]
            m.accel_debug = struct.unpack_from('<h',  b, 10)[0]
            m.next_pos    = list(struct.unpack_from('<4h', b, 12))
            state_copy    = self._legs[leg]

        if self.on_update:
            try:
                self.on_update(leg, state_copy)
            except Exception as e:
                log.error("[%s] on_update 콜백 오류: %s", leg, e)

    # ────────────────────────────────
    # 내부: I2C 송신 (서보)
    # ────────────────────────────────
    def _send_servo(self, leg: str, motor_id: int, degree: float):
        addr = LEG_ADDR[leg]
        step = degree_to_step(degree)
        buf  = [motor_id, 0, step & 0xFF, (step >> 8) & 0xFF]
        try:
            self._bus.write_i2c_block_data(addr, 0, buf)
        except Exception as e:
            log.error("[%s] I2C 서보 송신 오류 (id=%d): %s", leg, motor_id, e)
            return
        self._receive(leg)

    def _send_leg(self, leg: str, m1: float, m2: float, m3: float):
        '''1~3번 모터 순차 송신.'''
        self._send_servo(leg, 1, m1)
        self._send_servo(leg, 2, m2)
        self._send_servo(leg, 3, m3)

    # ────────────────────────────────
    # 내부: I2C 송신 (바퀴)
    # ────────────────────────────────
    def _send_wheel(self, leg: str, rotate_dir: bool, speed: int):
        addr      = LEG_ADDR[leg]
        dir_byte  = 0 if not rotate_dir else 1
        spd_byte  = max(0, min(100, speed))
        buf       = [4, 1, dir_byte, spd_byte]
        try:
            self._bus.write_i2c_block_data(addr, 0, buf)
        except Exception as e:
            log.error("[%s] I2C 바퀴 송신 오류: %s", leg, e)
            return
        self._receive(leg)

    # ────────────────────────────────
    # 내부: 유효성 검사
    # ────────────────────────────────
    @staticmethod
    def _check_leg(leg: str):
        if leg not in LEG_ADDR:
            raise ValueError(f"알 수 없는 다리: '{leg}'. "
                             f"가능한 값: {list(LEG_ADDR.keys())}")


# ──────────────────────────────────────────────
# 직접 실행 시 데모
# ──────────────────────────────────────────────
if __name__ == "__main__":
    logging.basicConfig(
        level=logging.DEBUG,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
        datefmt="%H:%M:%S",
    )

    print("=" * 60)
    print("RobotNode 데모 (mock=True, smbus2 불필요)")
    print("=" * 60)

    # ── 콜백 정의
    def on_motor_update(leg: str, state: LegState):
        # 폴링 때마다 호출됨 (필요한 처리만 추가)
        pass  # 여기서 UI 갱신, 로깅 등 가능

    node = RobotNode(poll_interval_s=0.2, mock=True)
    node.on_update = on_motor_update
    node.start()

    time.sleep(0.1)

    # ── 목표 좌표 변경 → 자동 전송
    print("\n[1] 모든 다리 기본 자세로")
    node.set_target_all(x=41.0, y=170.0, z=0.0)

    time.sleep(0.5)

    print("\n[2] front_right만 z=30 앞으로")
    result = node.set_target("front_right", x=41.0, y=170.0, z=30.0)
    print(f"    IK 결과: valid={result['valid']}  "
          f"m1={result['m1']} m2={result['m2']} m3={result['m3']}")

    time.sleep(0.5)

    print("\n[3] 도달 불가 좌표 테스트")
    result = node.set_target("front_right", x=41.0, y=300.0, z=0.0)
    print(f"    IK 결과: valid={result['valid']}  error={result['error']}")

    time.sleep(0.5)

    print("\n[4] 바퀴 회전 (front_right, 앞으로, speed=50)")
    node.set_wheel("front_right", rotate_dir=False, speed=50)

    time.sleep(0.5)

    print("\n[5] 상태 출력")
    node.print_state("front_right")

    time.sleep(0.5)

    node.stop()
    print("\n완료.")