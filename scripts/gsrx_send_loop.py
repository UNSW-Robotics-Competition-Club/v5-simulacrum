#!/usr/bin/env python3
import socket, sys, time
try:
    s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM); s.connect(sys.argv[1])
except Exception as e:
    print("connect failed:", e); sys.exit(1)
t0 = time.time()
while time.time() - t0 < 40:
    try: s.sendall(bytes([0x11,0x22,0x33,0x44]*8))
    except Exception: break
    time.sleep(0.2)
print("sender done")
