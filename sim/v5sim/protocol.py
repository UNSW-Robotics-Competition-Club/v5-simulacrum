"""V5 smart-port framing (notes/smartport-tx.md 1.7, motor-protocol.md 1).

Frames are 16 bytes by default; some devices negotiate longer ones (the IMU
switches to 64, imu-protocol.md 3). The last two bytes are a CRC16-CCITT-
FALSE (poly 0x1021, init 0xFFFF, no reflection, no xorout), big-endian,
over everything before them.
"""
from __future__ import annotations

FRAME_LEN = 16
PAYLOAD_LEN = FRAME_LEN - 2


def crc16_ccitt_false(data: bytes, init: int = 0xFFFF) -> int:
    """CRC-16/CCITT (poly 0x1021, MSB first). Smart-port frames use init
    0xFFFF; the AA frames inside controller 0x54 frames use 0."""
    crc = init
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021 if crc & 0x8000 else crc << 1) & 0xFFFF
    return crc


def build(payload: bytes, length: int = FRAME_LEN) -> bytes:
    """Pad or trim a payload to length-2 bytes and append the CRC."""
    body = bytes(payload).ljust(length - 2, b"\0")[:length - 2]
    return body + crc16_ccitt_false(body).to_bytes(2, "big")


def is_valid(frame: bytes) -> bool:
    return (len(frame) >= 4 and
            crc16_ccitt_false(frame[:-2]) == int.from_bytes(frame[-2:], "big"))


class FrameReader:
    """Splits a byte stream into frames of the current length."""

    def __init__(self, length: int = FRAME_LEN) -> None:
        self._buf = b""
        self.length = length

    def feed(self, data: bytes) -> list[bytes]:
        self.push(data)
        return self.drain()

    def push(self, data: bytes) -> None:
        """Buffer data without splitting it (see next_frame)."""
        self._buf += data

    def drain(self) -> list[bytes]:
        frames = []
        while (frame := self.next_frame()) is not None:
            frames.append(frame)
        return frames

    def next_frame(self) -> bytes | None:
        """One frame at the current length, so the caller can change the
        length between frames (the IMU does, mid-stream).

        Resyncs like the QEMU side: a chunk that fails its CRC is retried as
        a 16 B frame (the brain hasn't switched length yet), else one byte is
        dropped (we joined mid-frame)."""
        while len(self._buf) >= self.length:
            for n in dict.fromkeys((self.length, FRAME_LEN)):
                if is_valid(self._buf[:n]):
                    frame, self._buf = self._buf[:n], self._buf[n:]
                    return frame
            self._buf = self._buf[1:]
        return None
