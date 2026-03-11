#!/usr/bin/env python3
"""
WebSocket debug tool for csim UI server.

Connects to the WS endpoint, requests full state, and analyzes
both JSON (full state) and CBOR (delta) messages.

Usage:
    python3 tools/ws_debug.py [host:port]     # default: 127.0.0.1:8080
    python3 tools/ws_debug.py --deltas 10     # capture 10 delta messages
"""

import socket
import struct
import os
import json
import sys
import time


def ws_connect(host, port):
    """Establish WebSocket connection."""
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((host, port))
    s.send(
        f'GET /ws HTTP/1.1\r\n'
        f'Host: {host}:{port}\r\n'
        f'Upgrade: websocket\r\n'
        f'Connection: Upgrade\r\n'
        f'Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\n'
        f'Sec-WebSocket-Version: 13\r\n\r\n'.encode()
    )
    resp = b''
    while b'\r\n\r\n' not in resp:
        resp += s.recv(4096)
    if b'101' not in resp.split(b'\r\n')[0]:
        first_line = resp.split(b'\r\n')[0]
        raise Exception(f"WebSocket handshake failed: {first_line}")
    return s


def ws_send(s, text):
    """Send a text WS frame (masked, as required by client)."""
    payload = text.encode()
    frame = bytearray([0x81])
    mask_bit = 0x80
    l = len(payload)
    if l < 126:
        frame.append(mask_bit | l)
    elif l < 65536:
        frame.append(mask_bit | 126)
        frame.extend(struct.pack('>H', l))
    else:
        frame.append(mask_bit | 127)
        frame.extend(struct.pack('>Q', l))
    mask = os.urandom(4)
    frame.extend(mask)
    for i, b in enumerate(payload):
        frame.append(b ^ mask[i % 4])
    s.send(bytes(frame))


def ws_read_frame(s, buf):
    """Read one WS frame. Returns (opcode, payload_bytes, remaining_buf)."""
    while len(buf) < 2:
        buf += s.recv(65536)
    b0, b1 = buf[0], buf[1]
    plen = b1 & 0x7f
    hlen = 2
    if plen == 126:
        while len(buf) < 4:
            buf += s.recv(65536)
        plen = struct.unpack('>H', buf[2:4])[0]
        hlen = 4
    elif plen == 127:
        while len(buf) < 10:
            buf += s.recv(65536)
        plen = struct.unpack('>Q', buf[2:10])[0]
        hlen = 10
    total = hlen + plen
    while len(buf) < total:
        buf += s.recv(65536)
    return b0 & 0x0f, buf[hlen:total], buf[total:]


def decode_cbor_simple(data):
    """Minimal CBOR decoder for delta messages."""
    off = [0]

    def read_uint():
        b = data[off[0]]; off[0] += 1
        ai = b & 0x1f
        if ai < 24: return ai
        if ai == 24: v = data[off[0]]; off[0] += 1; return v
        if ai == 25: v = struct.unpack('>H', data[off[0]:off[0]+2])[0]; off[0] += 2; return v
        if ai == 26: v = struct.unpack('>I', data[off[0]:off[0]+4])[0]; off[0] += 4; return v
        if ai == 27:
            hi = struct.unpack('>I', data[off[0]:off[0]+4])[0]
            lo = struct.unpack('>I', data[off[0]+4:off[0]+8])[0]
            off[0] += 8
            return hi * 0x100000000 + lo
        return 0

    def decode():
        if off[0] >= len(data):
            return None
        b = data[off[0]]
        mt = b >> 5
        if mt == 0: return read_uint()
        if mt == 1: return -1 - read_uint()
        if mt == 2:
            n = read_uint()
            d = data[off[0]:off[0]+n]; off[0] += n
            return d
        if mt == 3:
            n = read_uint()
            s = data[off[0]:off[0]+n].decode('utf-8', errors='replace')
            off[0] += n
            return s
        if mt == 4:
            n = read_uint()
            return [decode() for _ in range(n)]
        if mt == 5:
            n = read_uint()
            m = {}
            for _ in range(n):
                k = decode(); m[k] = decode()
            return m
        if mt == 7:
            ai = b & 0x1f; off[0] += 1
            if ai == 20: return False
            if ai == 21: return True
            if ai == 22: return None
            return None
        off[0] += 1
        return None

    return decode()


TL_TYPE_NAMES = ['off', 'on', 'tx', 'rx', 'intf', 'led_on', 'led_off', 'frame', 'log']


def analyze_full_state(data):
    """Analyze a full state JSON message."""
    # Check for binary corruption
    for i, b in enumerate(data):
        if b > 127:
            print(f"  WARNING: Non-ASCII byte 0x{b:02x} at position {i}/{len(data)}")
            start = max(0, i - 40)
            end = min(len(data), i + 20)
            print(f"  Context: ...{data[start:i].decode('ascii', errors='replace')}<<<CORRUPT>>>")
            return

    msg = json.loads(data)
    print(f"  type: {msg.get('type')}")
    print(f"  sim_time_ms: {msg.get('sim_time_ms')} ({msg.get('sim_time_ms',0)/1000:.1f}s)")
    print(f"  nodes: {len(msg.get('nodes', []))}")
    if msg.get('nodes'):
        print(f"  node IDs: {[n['id'] for n in msg['nodes']]}")

    tl = msg.get('timeline', [])
    print(f"  timeline events: {len(tl)}")
    if tl:
        types = {}
        node_events = {}
        for ev in tl:
            types[ev['e']] = types.get(ev['e'], 0) + 1
            n = ev['n']
            node_events[n] = node_events.get(n, 0) + 1
        print(f"  event types: {types}")
        times = [ev['t'] for ev in tl]
        print(f"  time range: {min(times)/1000:.1f}s - {max(times)/1000:.1f}s")
        print(f"  per-node TX/RX:")
        for nid in sorted(set(ev['n'] for ev in tl)):
            tx = sum(1 for ev in tl if ev['n'] == nid and ev['e'] == 'tx')
            rx = sum(1 for ev in tl if ev['n'] == nid and ev['e'] == 'rx')
            total = node_events[nid]
            print(f"    Node {nid}: tx={tx} rx={rx} total={total}")


def analyze_cbor_delta(data):
    """Analyze a CBOR delta message."""
    d = decode_cbor_simple(data)
    if not isinstance(d, dict):
        print(f"  Unexpected CBOR type: {type(d)}")
        return

    sim_ms = d.get(0, '?')
    print(f"  sim_time_ms: {sim_ms}")

    if 1 in d:
        s = d[1]
        print(f"  stats: rf={s[0]} uart={s[1]} queued={s[2]} collided={s[3]} speed={s[4]/10:.1f}x paused={s[5]}")

    if 2 in d:
        radio_names = ['off', 'on', 'tx', 'rx', 'intf']
        print(f"  radio changes: { {k: radio_names[v] if v < len(radio_names) else v for k,v in d[2].items()} }")

    if 3 in d:
        print(f"  LED changes: {d[3]}")

    if 4 in d:
        print(f"  last_tx_ms: {d[4]}")

    if 5 in d:
        for nid, lines in d[5].items():
            print(f"  console[{nid}]: {len(lines)} lines")

    if 6 in d:
        tl = d[6]
        if isinstance(tl, list) and len(tl) > 0:
            types = {}
            for ev in tl:
                if isinstance(ev, list) and len(ev) >= 3:
                    tn = TL_TYPE_NAMES[ev[2]] if ev[2] < len(TL_TYPE_NAMES) else f'?{ev[2]}'
                    types[tn] = types.get(tn, 0) + 1
            print(f"  timeline: {len(tl)} events {types}")
        else:
            print(f"  timeline: raw CBOR ({type(tl)})")


def main():
    host, port = '127.0.0.1', 8080
    num_deltas = 3

    for arg in sys.argv[1:]:
        if arg.startswith('--deltas'):
            num_deltas = int(sys.argv[sys.argv.index(arg) + 1])
        elif ':' in arg:
            host, port = arg.split(':')
            port = int(port)

    print(f"Connecting to ws://{host}:{port}/ws ...")
    s = ws_connect(host, port)
    s.settimeout(10)

    print("Sending {cmd: full}...")
    ws_send(s, '{"cmd":"full"}')

    buf = b''
    frame_count = 0
    target = 1 + num_deltas  # 1 full + N deltas

    while frame_count < target:
        try:
            opcode, payload, buf = ws_read_frame(s, buf)
        except socket.timeout:
            print("Timeout waiting for frame")
            break

        frame_count += 1
        kind = 'TEXT' if opcode == 1 else 'BINARY'
        print(f"\n--- Frame {frame_count}: {kind}, {len(payload)} bytes ---")

        if opcode == 1:
            analyze_full_state(payload)
        elif opcode == 2:
            analyze_cbor_delta(payload)

    s.close()
    print("\nDone.")


if __name__ == '__main__':
    main()
