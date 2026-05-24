#!/usr/bin/env python3
"""I2C 연결 확인 스크립트 — 각 다리 Arduino가 응답하는지 확인."""

import sys

try:
    import smbus2
except ImportError:
    print("[ERROR] smbus2 가 설치되지 않았습니다. 'pip install smbus2' 를 실행하세요.")
    sys.exit(1)

I2C_BUS = 1

LEG_ADDR = {
    "front_left":  0x08,
    "rear_left":   0x09,
    "front_right": 0x0A,
    "rear_right":  0x0B,
}

def check_device(bus: smbus2.SMBus, leg: str, addr: int) -> bool:
    try:
        # 1바이트 읽기 시도 — 응답하면 연결됨
        bus.read_byte(addr)
        return True
    except OSError:
        return False

def main():
    try:
        bus = smbus2.SMBus(I2C_BUS)
    except Exception as e:
        print(f"[ERROR] I2C 버스 {I2C_BUS} 열기 실패: {e}")
        sys.exit(1)

    print(f"I2C 버스 {I2C_BUS} 연결 확인\n")
    print(f"{'다리':<14} {'주소':<8} {'상태'}")
    print("-" * 32)

    all_ok = True
    for leg, addr in LEG_ADDR.items():
        ok = check_device(bus, leg, addr)
        status = "OK" if ok else "FAIL (응답 없음)"
        print(f"{leg:<14} 0x{addr:02X}    {status}")
        if not ok:
            all_ok = False

    bus.close()
    print()
    if all_ok:
        print("모든 다리 연결 확인 완료.")
    else:
        print("일부 다리가 응답하지 않습니다. 배선 및 Arduino 전원을 확인하세요.")
        sys.exit(1)

if __name__ == "__main__":
    main()
