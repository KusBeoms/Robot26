"""
2026 kusbeoms

robot_server.py
라즈베리파이에서 실행. TCP 소켓으로 명령 수신 -> GaitController 제어.

ROS2 mpu6050_node 가 /imu/data 를 퍼블리시하고 있어야 함.
imu_reader.py 는 사용하지 않음.

명령 프로토콜 (JSON, 줄바꿈 구분):
    {"cmd": "move", "angle": 0.0, "magnitude": 0.8}
    {"cmd": "stop"}
    {"cmd": "status"}
    {"cmd": "quit"}

실행:
    # 터미널 1: ROS2 IMU 노드
    ros2 run mpu6050_driver mpu6050_driver

    # 터미널 2: 로봇 서버
    python robot_server.py

포트: 9000 (변경 가능)
"""

import json
import math
import socket
import logging
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

from robot_node import RobotNode
from gait_controller import GaitController, LEG_CONFIG, NEUTRAL_X, NEUTRAL_Y, NEUTRAL_Z

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("server")

HOST = "0.0.0.0"
PORT = 9000


# ──────────────────────────────────────────────
# ROS2 IMU 구독자
# ──────────────────────────────────────────────
class ImuSubscriber(Node):
    """
    /imu/data (sensor_msgs/Imu) 구독 -> 쿼터니언 + 오일러각 보관.
    별도 스레드(rclpy.spin)에서 실행.
    """

    def __init__(self):
        super().__init__("robot_server_imu")
        self._lock = threading.Lock()
        self._q    = (1.0, 0.0, 0.0, 0.0)   # (w, x, y, z)

        self.create_subscription(
            Imu,
            "/imu/data",
            self._cb,
            10,
        )
        self.get_logger().info("ImuSubscriber: /imu/data 구독 시작")

    def _cb(self, msg: Imu):
        o = msg.orientation
        with self._lock:
            self._q = (o.w, o.x, o.y, o.z)

    @property
    def quaternion(self):
        """(qw, qx, qy, qz) thread-safe."""
        with self._lock:
            return self._q

    @property
    def euler_deg(self):
        """(roll, pitch, yaw) 도 단위."""
        qw, qx, qy, qz = self.quaternion
        # roll
        sinr = 2.0 * (qw * qx + qy * qz)
        cosr = 1.0 - 2.0 * (qx * qx + qy * qy)
        roll = math.degrees(math.atan2(sinr, cosr))
        # pitch
        sinp = 2.0 * (qw * qy - qz * qx)
        sinp = max(-1.0, min(1.0, sinp))
        pitch = math.degrees(math.asin(sinp))
        # yaw
        siny = 2.0 * (qw * qz + qx * qy)
        cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = math.degrees(math.atan2(siny, cosy))
        return roll, pitch, yaw


# ──────────────────────────────────────────────
# TCP 클라이언트 핸들러
# ──────────────────────────────────────────────
def handle_client(conn, addr, gait, imu_node, node, halt_event):
    log.info("클라이언트 연결: %s", addr)
    buf = ""
    try:
        while True:
            data = conn.recv(1024)
            if not data:
                break

            buf += data.decode("utf-8", errors="ignore")
            while "\n" in buf:
                line, buf = buf.split("\n", 1)
                line = line.strip()
                if not line:
                    continue
                try:
                    msg = json.loads(line)
                except json.JSONDecodeError:
                    conn.sendall(b'{"ok":false,"error":"invalid json"}\n')
                    continue

                cmd = msg.get("cmd", "")

                if cmd == "move":
                    if halt_event.is_set():
                        conn.sendall(b'{"ok":false,"error":"halted: motor delay exceeded, send stop to resume"}\n')
                        continue
                    angle     = float(msg.get("angle", 0.0))
                    magnitude = float(msg.get("magnitude", 0.0))
                    gait.set_command(angle=angle, magnitude=magnitude)
                    conn.sendall(b'{"ok":true}\n')

                elif cmd == "stop":
                    halt_event.clear()
                    gait.set_command(angle=0.0, magnitude=0.0)
                    conn.sendall(b'{"ok":true}\n')

                elif cmd == "status":
                    roll, pitch, yaw = imu_node.euler_deg
                    resp = json.dumps({
                        "ok": True,
                        "halted": halt_event.is_set(),
                        "imu": {
                            "roll":  round(roll,  2),
                            "pitch": round(pitch, 2),
                            "yaw":   round(yaw,   2),
                        },
                        "gait": {
                            "magnitude": round(gait._filt_magnitude, 3),
                            "angle_deg": round(math.degrees(gait._filt_angle), 1),
                        },
                        "delay": node.get_delay_stats(),
                    }) + "\n"
                    conn.sendall(resp.encode())

                elif cmd == "quit":
                    conn.sendall(b'{"ok":true,"msg":"bye"}\n')
                    return

                else:
                    conn.sendall(b'{"ok":false,"error":"unknown cmd"}\n')

    except Exception as e:
        log.error("클라이언트 오류 %s: %s", addr, e)
    finally:
        conn.close()
        log.info("클라이언트 종료: %s", addr)


# ──────────────────────────────────────────────
# 메인
# ──────────────────────────────────────────────
def main():
    log.info("로봇 초기화 중...")

    # ROS2 초기화
    rclpy.init()
    imu_node = ImuSubscriber()

    # ROS2 spin 을 별도 스레드에서 실행
    ros_thread = threading.Thread(
        target=rclpy.spin, args=(imu_node,), daemon=True
    )
    ros_thread.start()

    # 로봇 노드 + 보행 제어기
    node = RobotNode(poll_interval_s=0.05)
    gait = GaitController(node)

    # 안전 정지 이벤트
    halt_event = threading.Event()

    def _on_delay_exceeded(leg_name, elapsed_ms):
        log.warning("안전 정지 트리거: %s %.1fms 지연", leg_name, elapsed_ms)
        halt_event.set()
        gait.set_command(angle=0.0, magnitude=0.0)
        for lg, cfg in LEG_CONFIG.items():
            xs = 1.0 if cfg["right"] else -1.0
            node.set_target(lg, xs * NEUTRAL_X, NEUTRAL_Y, NEUTRAL_Z)

    node.on_delay_exceeded = _on_delay_exceeded

    node.start()
    gait.start()

    # IMU -> GaitController 전달 스레드 (50Hz)
    imu_running = [True]
    def imu_loop():
        import time
        while imu_running[0]:
            gait.update_imu(*imu_node.quaternion)
            time.sleep(0.02)
    threading.Thread(target=imu_loop, daemon=True).start()

    # TCP 서버
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((HOST, PORT))
    srv.listen(5)
    log.info("서버 대기 중: %s:%d", HOST, PORT)

    try:
        while True:
            conn, addr = srv.accept()
            t = threading.Thread(
                target=handle_client,
                args=(conn, addr, gait, imu_node, node, halt_event),
                daemon=True,
            )
            t.start()

    except KeyboardInterrupt:
        log.info("종료 중...")
    finally:
        imu_running[0] = False
        gait.set_command(angle=0.0, magnitude=0.0)
        import time; time.sleep(0.3)
        gait.stop()
        node.stop()
        imu_node.destroy_node()
        rclpy.shutdown()
        srv.close()
        log.info("종료 완료")


if __name__ == "__main__":
    main()