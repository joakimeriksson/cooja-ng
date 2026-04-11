#!/usr/bin/env python3
"""
Smoke test for the csim GDB stub.

Connects to a running csim with --gdb 3333, exchanges a handful of RSP
packets, prints what comes back, and asserts on the basic shape.

Usage:
    # In one shell:
    ./build/test_runner mixed-multinode firmware/cc2538dk/hello-world.cc2538dk \
        -t 10000 --gdb 3333 --gdb-wait

    # In another shell:
    python3 tools/test-gdb-stub.py 3333
"""
import socket
import sys
import time


def make_packet(payload: bytes) -> bytes:
    cs = sum(payload) & 0xFF
    return b"$" + payload + b"#" + f"{cs:02x}".encode()


def send_packet(sock, payload: bytes) -> None:
    sock.sendall(make_packet(payload))
    # Wait for + ack
    ack = sock.recv(1)
    if ack != b"+":
        print(f"  WARNING: expected '+', got {ack!r}")


def recv_packet(sock) -> bytes:
    """Read until we have a complete $...#XX packet, send '+', return body."""
    buf = b""
    while True:
        c = sock.recv(1)
        if not c:
            return b""
        if c == b"+" or c == b"-":
            continue
        if c == b"$":
            break
    body = b""
    while True:
        c = sock.recv(1)
        if c == b"#":
            break
        body += c
    cs = sock.recv(2)  # checksum bytes
    sock.sendall(b"+")
    return body


def cmd(sock, payload: bytes, label: str) -> bytes:
    print(f"  -> {label}: {payload!r}")
    send_packet(sock, payload)
    reply = recv_packet(sock)
    short = reply if len(reply) < 80 else reply[:77] + b"..."
    print(f"  <- {short!r}")
    return reply


def main() -> int:
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 3333
    print(f"Connecting to localhost:{port} ...")
    sock = socket.create_connection(("127.0.0.1", port), timeout=5)
    print("Connected.\n")

    # 1. qSupported — feature negotiation
    reply = cmd(sock, b"qSupported:multiprocess+;swbreak+;hwbreak+", "qSupported")
    assert b"PacketSize" in reply, "qSupported reply must include PacketSize"

    # 2. ? — current stop reason. Stub halts on connect, expect S05 or T05
    reply = cmd(sock, b"?", "stop reason")
    assert reply.startswith(b"S") or reply.startswith(b"T"), \
        f"expected S/T reply, got {reply!r}"

    # 3. g — read all registers (ARM: 17 * 4 = 68 bytes = 136 hex chars)
    reply = cmd(sock, b"g", "read registers")
    assert len(reply) == 136, f"expected 136 hex chars, got {len(reply)}"

    # Extract PC (15th register, little-endian)
    pc_hex = reply[15 * 8:16 * 8].decode()
    pc_le = bytes.fromhex(pc_hex)
    pc = int.from_bytes(pc_le, "little")
    print(f"     PC = 0x{pc:08x}")

    # 4. m<addr>,<len> — read 16 bytes from PC
    reply = cmd(sock, f"m{pc:x},10".encode(), f"read 16 bytes @ PC")
    assert len(reply) == 32, f"expected 32 hex chars, got {len(reply)}"

    # 5. Set a breakpoint at PC + 8 (nonsense address but we just check the protocol)
    bp_addr = (pc + 8) & 0xFFFFFFFF
    reply = cmd(sock, f"Z0,{bp_addr:x},2".encode(), f"set breakpoint @ +8")
    assert reply == b"OK", f"expected OK, got {reply!r}"

    # 6. Clear it
    reply = cmd(sock, f"z0,{bp_addr:x},2".encode(), f"clear breakpoint")
    assert reply == b"OK", f"expected OK, got {reply!r}"

    # 7. Continue (no reply expected immediately — fire and forget)
    print("  -> continue (c)")
    send_packet(sock, b"c")
    print("  (running ... let csim run for 1s)")
    time.sleep(1.0)

    # 8. Send Ctrl+C (0x03 byte) to interrupt
    print("  -> interrupt (Ctrl+C)")
    sock.sendall(b"\x03")

    # Should receive a stop reply
    reply = recv_packet(sock)
    print(f"  <- {reply!r}")
    assert reply.startswith(b"S") or reply.startswith(b"T"), \
        f"expected stop reply after Ctrl+C, got {reply!r}"

    # 9. Detach
    cmd(sock, b"D", "detach")

    sock.close()
    print("\nALL CHECKS PASSED")
    return 0


if __name__ == "__main__":
    sys.exit(main())
