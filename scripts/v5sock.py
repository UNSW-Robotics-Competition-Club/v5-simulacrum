"""A socket with its own send buffer, for the select loops that pass the
brain's USB ports around (v5usb_share.py, v5gdb_bridge.py): a slow reader
never stalls the loop or anyone else's bytes, and bytes are never half
sent. A peer that falls too far behind is dropped whole."""
from __future__ import annotations

import socket

CHUNK = 4096
MAX_PENDING = 1 << 20            # a peer this far behind isn't reading: drop it


class Peer:
    def __init__(self, sock: socket.socket):
        sock.setblocking(False)
        self.sock = sock
        self.pending = bytearray()
        self.alive = True

    def fileno(self) -> int:
        return self.sock.fileno()

    def send(self, data: bytes) -> bool:
        """Queues data and sends what the socket takes now. False (and
        closed) when the peer is gone or not reading."""
        if not self.alive:
            return False
        self.pending += data
        if len(self.pending) > MAX_PENDING:
            self.close()
            return False
        return self.flush()

    def flush(self) -> bool:
        while self.alive and self.pending:
            try:
                n = self.sock.send(self.pending)
            except (BlockingIOError, InterruptedError):
                return True
            except OSError:
                self.close()
                return False
            del self.pending[:n]
        return self.alive

    @property
    def wants_write(self) -> bool:
        return self.alive and bool(self.pending)

    def recv(self) -> bytes | None:
        """Bytes read; None when there's nothing yet; b"" when the peer
        closed (or failed)."""
        try:
            return self.sock.recv(CHUNK)
        except (BlockingIOError, InterruptedError):
            return None
        except OSError:
            return b""

    def close(self) -> None:
        self.alive = False
        self.pending.clear()
        try:
            self.sock.close()
        except OSError:
            pass


def close(p: Peer | None) -> None:
    if p:
        p.close()
