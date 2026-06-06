#!/usr/bin/env python3
"""
I2C 버스 및 연결된 장치 확인 스크립트.

사용법:
    python3 check_bus.py [bus_num]
    python3 check_bus.py        # 기본값: bus 1
    python3 check_bus.py 1 20 21
"""

import struct
import sys

try:
    import smbus2
except ImportError:
    print("[ERROR] smbus2 설치 필요: pip install smbus2")
    sys.exit(1)

# 알려진 장치 주소 및 이름
KNOWN_DEVICES = {
    0x08: "leg_board_0 (front_left?)",
    0x0A: "leg_board_1 (front_right?)",
    0x0B: "leg_board_2 (rear_right?)",
    0x68: "MPU6050 (IMU)",
}

LEG_ADDRS  = {0x08, 0x0A, 0x0B}
READ_LEN   = 20   # 다리 보드 응답 패킷 길이
REG        = 0
CMD_READ   = [1, 0, 0, 0]


def scan_bus(bus_num: int) -> list[int]:
    """버스에서 응답하는 주소 목록 반환."""
    found = []
    try:
        bus = smbus2.SMBus(bus_num)
    except Exception as e:
        print(f"  버스 열기 실패: {e}")
        return found

    for addr in range(0x03, 0x78):
        try:
            bus.read_byte(addr)
            found.append(addr)
        except Exception:
            pass

    bus.close()
    return found


def read_leg_board(bus_num: int, addr: int) -> dict | None:
    """다리 보드에서 20바이트 읽어 파싱."""
    try:
        bus = smbus2.SMBus(bus_num)
        write_msg = smbus2.i2c_msg.write(addr, CMD_READ)
        read_msg  = smbus2.i2c_msg.read(addr, READ_LEN)
        bus.i2c_rdwr(write_msg)
        bus.i2c_rdwr(read_msg)
        raw = bytes(read_msg)
        bus.close()
    except Exception as e:
        return {"error": str(e)}

    return {
        "raw":      list(raw),
        "position": list(struct.unpack_from("<4h", raw, 0)),
        "current":  struct.unpack_from("<H", raw, 8)[0],
        "accel":    struct.unpack_from("<h", raw, 10)[0],
        "next_pos": list(struct.unpack_from("<4h", raw, 12)),
    }


def check_bus(bus_num: int):
    print(f"\n{'='*50}")
    print(f"  I2C bus {bus_num} 스캔 중...")
    print(f"{'='*50}")

    found = scan_bus(bus_num)

    if not found:
        print("  응답하는 장치 없음.")
        return

    for addr in found:
        label = KNOWN_DEVICES.get(addr, "알 수 없는 장치")
        print(f"\n  [0x{addr:02X}] {label}")

        if addr in LEG_ADDRS:
            data = read_leg_board(bus_num, addr)
            if "error" in data:
                print(f"    읽기 실패: {data['error']}")
            else:
                print(f"    position : {data['position']}")
                print(f"    current  : {data['current']}")
                print(f"    accel    : {data['accel']}")
                print(f"    next_pos : {data['next_pos']}")
                print(f"    raw      : {data['raw']}")
        else:
            print(f"    (다리 보드 아님, 읽기 생략)")


def main():
    bus_list = [int(x) for x in sys.argv[1:]] if len(sys.argv) > 1 else [1]

    print(f"검사할 버스: {bus_list}")
    for b in bus_list:
        check_bus(b)

    print(f"\n{'='*50}")
    print("  완료")
    print(f"{'='*50}")


if __name__ == "__main__":
    main()
