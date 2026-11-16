"""Host-side V5 smart-port device simulator for the vex-v5 QEMU machine.

Connects to a port's chardev socket (-chardev socket,id=portN,path=...,
server=on,wait=off) and speaks the 16-byte smart-port frame protocol from
notes/smartport-tx.md: bytes 0..13 payload, CRC16-CCITT-FALSE big-endian in
bytes 14..15. Devices speak first; the brain answers each valid frame.

Usage: python3 -I devsim.py <socket> [seconds] [device_type]
"""
import socket
import sys
import time

FRAME_LEN = 16
CMD_ANNOUNCE = 0x0F
CMD_READY = 0x13
DEVICE_MOTOR = 0x02
# The brain resets a port after ~1 s without a valid frame.
KEEPALIVE_S = 0.1


def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021 if crc & 0x8000 else crc << 1) & 0xFFFF
    return crc


def frame(cmd: int, device_type: int, payload: bytes = b"") -> bytes:
    body = bytes([cmd, 0, device_type]) + payload
    body = body.ljust(FRAME_LEN - 2, b"\0")[: FRAME_LEN - 2]
    return body + crc16_ccitt_false(body).to_bytes(2, "big")


def check(raw: bytes) -> bool:
    return crc16_ccitt_false(raw[:14]) == int.from_bytes(raw[14:16], "big")


class Device:
    def __init__(self, path: str, device_type: int):
        self.sock = socket.socket(socket.AF_UNIX)
        self.sock.connect(path)
        self.sock.settimeout(0.05)
        self.device_type = device_type
        self.rx = b""

    def send(self, raw: bytes) -> None:
        self.sock.sendall(raw)

    def poll(self) -> list[bytes]:
        try:
            chunk = self.sock.recv(4096)
        except socket.timeout:
            return []
        if not chunk:
            raise ConnectionError("brain closed the port")
        self.rx += chunk
        frames = []
        while len(self.rx) >= FRAME_LEN:
            frames.append(self.rx[:FRAME_LEN])
            self.rx = self.rx[FRAME_LEN:]
        return frames


def main() -> None:
    path = sys.argv[1]
    seconds = float(sys.argv[2]) if len(sys.argv) > 2 else 10
    device_type = int(sys.argv[3], 0) if len(sys.argv) > 3 else DEVICE_MOTOR
    dev = Device(path, device_type)
    script = [frame(CMD_ANNOUNCE, device_type), frame(CMD_READY, device_type)]
    end = time.monotonic() + seconds
    next_tx = 0.0
    while time.monotonic() < end:
        now = time.monotonic()
        if now >= next_tx:
            raw = script.pop(0) if script else frame(CMD_READY, device_type)
            dev.send(raw)
            print(f"dev> {raw.hex(' ')}", flush=True)
            next_tx = now + KEEPALIVE_S
        for raw in dev.poll():
            print(f"brain> {raw.hex(' ')} {'ok' if check(raw) else 'BADCRC'}", flush=True)


if __name__ == "__main__":
    main()
