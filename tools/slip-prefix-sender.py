#!/usr/bin/env python3
"""
Minimal SLIP prefix sender for border-router testing.
Connects to a serial socket, sends the fd00::1/64 prefix via SLIP,
reads and prints responses. No TUN, no sudo needed.
"""
import socket
import sys
import time

SLIP_END = 0xC0
PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 60001
WAIT = int(sys.argv[2]) if len(sys.argv) > 2 else 60

def slip_encode(data):
    """SLIP-encode a byte string"""
    out = bytes([SLIP_END])
    for b in data:
        if b == SLIP_END:
            out += bytes([0xDB, 0xDC])
        elif b == 0xDB:
            out += bytes([0xDB, 0xDD])
        else:
            out += bytes([b])
    out += bytes([SLIP_END])
    return out

# Connect to serial socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
for attempt in range(30):
    try:
        sock.connect(('127.0.0.1', PORT))
        break
    except ConnectionRefusedError:
        time.sleep(0.1)
else:
    print("Failed to connect to serial socket", file=sys.stderr)
    sys.exit(1)

sock.setblocking(False)

# Read initial output from border-router (startup messages)
time.sleep(0.5)
try:
    while True:
        data = sock.recv(4096)
        if not data:
            break
        # Look for prefix request '?P' in SLIP frames
        sys.stdout.buffer.write(data)
        sys.stdout.buffer.flush()
except BlockingIOError:
    pass

# Send SLIP-encoded prefix command: !Pfd00::1/64\n
prefix_cmd = b'!Pfd00::1/64\n'
sock.sendall(slip_encode(prefix_cmd))
print(f"Sent prefix: {prefix_cmd}", file=sys.stderr)

# Read responses for WAIT seconds
start = time.time()
while time.time() - start < WAIT:
    try:
        data = sock.recv(4096)
        if not data:
            break
        sys.stdout.buffer.write(data)
        sys.stdout.buffer.flush()
    except BlockingIOError:
        time.sleep(0.1)

sock.close()
print("Done", file=sys.stderr)
