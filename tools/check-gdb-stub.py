#!/usr/bin/env python3
"""GDB stub robustness check.

The stub reads on the simulation's thread, so a peer that stalls can freeze
the run.  Each case starts the runner halted on a GDB connection
(--gdb-wait) with a short simulation, then stalls or sends a malformed
packet, and asserts:

  - a client that stops mid-packet is dropped, and the run then finishes;
  - a client that never acknowledges a reply is dropped the same way;
  - a bare "Z0" after another packet is refused (E01) rather than parsed
    out of the previous packet's leftover bytes into a breakpoint.

The stub waits 5 s for the rest of a packet, so the stall cases take ~5 s.

Usage: tools/check-gdb-stub.py         (RUNNER=... to override the binary)
"""
import os
import socket
import subprocess
import sys
import time

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
RUNNER = os.environ.get('RUNNER', os.path.join(ROOT, 'build', 'test_runner'))
FIRMWARE = os.path.join(ROOT, 'firmware', 'cc2538dk', 'hello-world.cc2538dk')
FINISH_WITHIN = 20      # s of wall time; the stall timeout is 5


def free_port():
    with socket.socket() as s:
        s.bind(('127.0.0.1', 0))
        return s.getsockname()[1]


def packet(body):
    return b'$' + body + b'#' + b'%02x' % (sum(body) & 0xff)


def start(sim_ms):
    port = free_port()
    proc = subprocess.Popen(
        [RUNNER, 'mixed-multinode', FIRMWARE, '-t', str(sim_ms),
         '--gdb', str(port), '--gdb-wait', '-q'],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    deadline = time.time() + 10
    while time.time() < deadline:
        try:
            return proc, socket.create_connection(('127.0.0.1', port), timeout=10)
        except OSError:
            time.sleep(0.1)
    proc.kill()
    sys.exit('check-gdb-stub: FAIL: stub never listened')


def finishes(proc):
    """'finished' if the run completes on its own, else 'hung'."""
    try:
        proc.wait(timeout=FINISH_WITHIN)
        return 'finished'
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait()
        return 'hung'


def reply(sock):
    """Body of the next packet from the stub (acking it), or None at EOF."""
    buf = b''
    while not buf.endswith(b'$'):
        c = sock.recv(1)
        if not c:
            return None
        buf = c if c == b'$' else b''
    body = b''
    while True:
        c = sock.recv(1)
        if not c:
            return None
        if c == b'#':
            break
        body += c
    sock.recv(2)
    sock.sendall(b'+')
    return body


failed = 0


def expect(name, got, want):
    global failed
    ok = got == want
    failed += not ok
    print(f"  {'ok  ' if ok else 'FAIL'} {name}: {got}"
          + ('' if ok else f' (want {want})'))


def main():
    # Stalled mid-packet: '$' and part of a body, then silence.
    proc, sock = start(2000)
    sock.sendall(b'$qSupp')
    expect('client stalled mid-packet', finishes(proc), 'finished')
    sock.close()

    # Never acks: a whole request, then no '+' for the reply.
    proc, sock = start(2000)
    sock.sendall(packet(b'?'))
    expect('client never acks a reply', finishes(proc), 'finished')
    sock.close()

    # Stale bytes: "m0,4" leaves ",4" in the receive buffer past a bare "Z0".
    proc, sock = start(2000)
    try:
        for body in (b'm0,4', b'Z0'):
            sock.sendall(packet(body))
            if sock.recv(1) != b'+':
                raise OSError('no ack')
            last = reply(sock)
        expect('bare Z0 after m0,4', last.decode() if last else 'EOF', 'E01')
        sock.sendall(packet(b'D'))
        reply(sock)
    except OSError as e:
        expect('bare Z0 after m0,4', f'error: {e}', 'E01')
    sock.close()
    finishes(proc)

    if failed:
        sys.exit(f'check-gdb-stub: FAIL: {failed} case(s)')
    print('check-gdb-stub: OK')


if __name__ == '__main__':
    main()
