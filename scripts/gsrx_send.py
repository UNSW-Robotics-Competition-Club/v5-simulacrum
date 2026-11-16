#!/usr/bin/env python3
"""Connect to a brain smart-port socket and send bytes to trigger generic RX."""
import socket, sys, time
s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM); s.connect(sys.argv[1])
time.sleep(1)
for i in range(20):
    s.sendall(bytes([0x11, 0x22, 0x33, 0x44] * 4)); time.sleep(0.1)
print("sent")
time.sleep(2)
