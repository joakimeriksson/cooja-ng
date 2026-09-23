#!/usr/bin/env python3
"""GDB stub robustness check.

The stub reads on the simulation's thread, so a peer that stalls can freeze
the run.  Each case starts the runner halted on a GDB connection
(--gdb-wait) with a short simulation, then stalls or sends a malformed
packet, and asserts:

  - a client that stops mid-packet is dropped, and the run then finishes;
  - so is one that drips a packet a byte at a time: the 5 s bound is on the
    whole packet, not on each byte;
  - a client that never acknowledges a reply is dropped the same way;
  - a bare "Z0" after another packet is refused (E01) rather than parsed
    out of the previous packet's leftover bytes into a breakpoint;
  - breakpoints set by a client that is dropped are gone for the next one.

The stub waits 5 s for the rest of a packet, so each stall case takes ~5 s.

Usage: tools/check-gdb-stub.py         (RUNNER=... to override the binary)
"""
import os
import socket
import struct
import subprocess
import sys
import threading
import time

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
RUNNER = os.environ.get('RUNNER', os.path.join(ROOT, 'build', 'test_runner'))
FIRMWARE = os.path.join(ROOT, 'firmware', 'cc2538dk', 'hello-world.cc2538dk')
FINISH_WITHIN = 20      # s of wall time; the stall timeout is 5


def symbol(path, name):
    """Address of an ELF32 little-endian symbol, Thumb bit cleared."""
    with open(path, 'rb') as f:
        elf = f.read()
    shoff, = struct.unpack_from('<I', elf, 0x20)
    shentsize, shnum = struct.unpack_from('<HH', elf, 0x2e)
    sections = [struct.unpack_from('<IIIIIIIIII', elf, shoff + i * shentsize)
                for i in range(shnum)]
    for sh in sections:
        if sh[1] != 2:                       # SHT_SYMTAB
            continue
        strtab = sections[sh[6]]
        for off in range(sh[4], sh[4] + sh[5], 16):
            st_name, st_value = struct.unpack_from('<II', elf, off)
            end = elf.index(b'\0', strtab[4] + st_name)
            if elf[strtab[4] + st_name:end] == name:
                return st_value & ~1
    sys.exit(f'check-gdb-stub: FAIL: no symbol {name.decode()}')


def free_port():
    with socket.socket() as s:
        s.bind(('127.0.0.1', 0))
        return s.getsockname()[1]


def packet(body):
    return b'$' + body + b'#' + b'%02x' % (sum(body) & 0xff)


def start(sim_ms):
    port = free_port()
    global PORT
    PORT = port
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

    # Drip: '$', then one body byte every 3 s -- each byte inside the
    # timeout, the packet far beyond it.  The drip alone would hold a
    # halted run for a minute.
    proc, sock = start(2000)
    stop = threading.Event()
    def drip():
        try:
            sock.sendall(b'$')
            for c in b'qSupported:multiprocess+':
                if stop.wait(3):
                    return
                sock.sendall(bytes([c]))
        except OSError:
            pass
    dripper = threading.Thread(target=drip)
    dripper.start()
    expect('client drips a packet', finishes(proc), 'finished')
    stop.set()
    dripper.join()
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

    # Breakpoints of a dropped client: the first client sets one where the
    # firmware's main loop passes and sees it hit, then stalls mid-packet.
    # The second, already queued on the listener, continues -- and must not
    # stop at a breakpoint it never set.
    bp = b'%x' % symbol(FIRMWARE, b'process_run')
    proc, sock = start(3000)
    try:
        sock.sendall(packet(b'Z0,' + bp + b',2'))
        sock.recv(1)
        if reply(sock) != b'OK':
            raise OSError('Z0 refused')
        sock.sendall(packet(b'c'))
        sock.recv(1)
        stop = reply(sock) or b'EOF'
        expect('own breakpoint hit', stop[:3].decode(), 'S05')
        second = socket.create_connection(('127.0.0.1', PORT), timeout=20)
        second.sendall(packet(b'c'))
        sock.sendall(b'$qSupp')            # stall: the stub drops us
        second.recv(1)
        stop = reply(second)
        expect('dropped client\'s breakpoint', stop.decode() if stop else 'EOF',
               'EOF')
        second.close()
    except OSError as e:
        expect('dropped client\'s breakpoint', f'error: {e}', 'EOF')
    sock.close()
    finishes(proc)

    if failed:
        sys.exit(f'check-gdb-stub: FAIL: {failed} case(s)')
    print('check-gdb-stub: OK')


if __name__ == '__main__':
    main()
