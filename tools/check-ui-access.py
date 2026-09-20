#!/usr/bin/env python3
"""Web UI access check.

The UI accepts commands (pause, speed, restart, move), and WebSocket is
exempt from the same-origin policy.  This starts the runner with --ui and
sends raw requests shaped like the ones a browser would send, asserting:

  - by default it listens on loopback only: not reachable at this host's
    network address;
  - our own page's WebSocket upgrade (Origin == Host) and script clients
    (no Origin) are accepted;
  - a cross-origin page, a sandboxed ("null") page and a different port on
    the same host are refused;
  - on the loopback-bound server a non-loopback Host is refused, which is
    what stops DNS rebinding (evil.example resolving to 127.0.0.1, where
    Origin and Host agree with each other);
  - with --ui-bind 0.0.0.0 the Host rule is off (it cannot hold for a
    server meant to be reached by name) but the Origin rule still applies;
  - a ping is answered with a pong that echoes it, and a control frame
    longer than RFC 6455's 125 bytes closes the connection instead of
    drawing a pong whose header does not match its body.

Usage: tools/check-ui-access.py        (RUNNER=... to override the binary)
"""
import os
import socket
import struct
import subprocess
import sys
import time

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
RUNNER = os.environ.get('RUNNER', os.path.join(ROOT, 'build', 'test_runner'))
FIRMWARE = os.path.join(ROOT, 'firmware', 'sky', 'hello-world.sky')


def free_port():
    with socket.socket() as s:
        s.bind(('127.0.0.1', 0))
        return s.getsockname()[1]


def network_address():
    """This host's outward-facing IPv4 address, or None (no route: skip)."""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(('192.0.2.1', 9))          # TEST-NET-1; nothing is sent
            addr = s.getsockname()[0]
        return None if addr.startswith('127.') else addr
    except OSError:
        return None


def status(addr, port, path='/ws', host=None, origin=None, upgrade=True):
    """Send one request, return the response's status code (or an error)."""
    req = f'GET {path} HTTP/1.1\r\n'
    if host is not None:
        req += f'Host: {host}\r\n'
    if origin is not None:
        req += f'Origin: {origin}\r\n'
    if upgrade:
        req += ('Upgrade: websocket\r\nConnection: Upgrade\r\n'
                'Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\n'
                'Sec-WebSocket-Version: 13\r\n')
    try:
        with socket.create_connection((addr, port), timeout=3) as s:
            s.sendall((req + '\r\n').encode())
            line = s.recv(4096).split(b'\r\n')[0].decode()
    except OSError as e:
        return f'error: {e.__class__.__name__}'
    parts = line.split()
    return parts[1] if len(parts) > 1 else f'error: bad reply {line!r}'


def ws_open(port):
    s = socket.create_connection(('127.0.0.1', port), timeout=3)
    s.sendall(b'GET /ws HTTP/1.1\r\nHost: localhost\r\nUpgrade: websocket\r\n'
              b'Connection: Upgrade\r\n'
              b'Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\n'
              b'Sec-WebSocket-Version: 13\r\n\r\n')
    reply = b''
    while b'\r\n\r\n' not in reply:
        chunk = s.recv(1)
        if not chunk:
            raise OSError('closed during handshake')
        reply += chunk
    return s


def ws_send(s, opcode, payload):
    """One masked client frame (clients must mask, RFC 6455 5.3)."""
    mask = os.urandom(4)
    n = len(payload)
    head = bytes([0x80 | opcode])
    head += bytes([0x80 | n]) if n < 126 else bytes([0x80 | 126]) + struct.pack('>H', n)
    s.sendall(head + mask + bytes(b ^ mask[i % 4] for i, b in enumerate(payload)))


def recv_exact(s, n):
    data = b''
    while len(data) < n:
        chunk = s.recv(n - len(data))
        if not chunk:
            return None
        data += chunk
    return data


def ws_next(s):
    """(opcode, payload) of the server's next frame, or None at EOF."""
    head = recv_exact(s, 2)
    if head is None:
        return None
    n = head[1] & 0x7f
    if n == 126:
        n = struct.unpack('>H', recv_exact(s, 2))[0]
    elif n == 127:
        n = struct.unpack('>Q', recv_exact(s, 8))[0]
    payload = recv_exact(s, n)
    return None if payload is None else (head[0] & 0x0f, payload)


def pong_for(port, size):
    """'echo' if a size-byte ping draws a matching pong, 'closed' if the
    server hangs up, else what arrived instead.  Broadcast frames that
    arrive first are skipped."""
    ping = bytes(range(256))[:size] if size <= 256 else os.urandom(size)
    try:
        with ws_open(port) as s:
            ws_send(s, 0x9, ping)
            while True:
                frame = ws_next(s)
                if frame is None:
                    return 'closed'
                if frame[0] == 0xA:
                    return 'echo' if frame[1] == ping else f'pong of {len(frame[1])} bytes'
    except socket.timeout:
        return 'no reply'
    except OSError as e:
        return f'error: {e}'


class Runner:
    def __init__(self, port, *extra):
        self.port = port
        self.proc = subprocess.Popen(
            [RUNNER, 'mixed-multinode', FIRMWARE, '-t', '600000',
             '--ui', str(port), *extra, '-q'],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        deadline = time.time() + 10
        while time.time() < deadline:
            if self.proc.poll() is not None:
                sys.exit(f'check-ui-access: FAIL: runner exited '
                         f'({self.proc.returncode}) before listening')
            try:
                socket.create_connection(('127.0.0.1', port), timeout=0.2).close()
                return
            except OSError:
                time.sleep(0.1)
        self.close()
        sys.exit('check-ui-access: FAIL: UI never started listening')

    def close(self):
        self.proc.kill()
        self.proc.wait()


failed = 0


def expect(name, got, want):
    global failed
    ok = got == want
    failed += not ok
    print(f"  {'ok  ' if ok else 'FAIL'} {name}: {got}"
          + ('' if ok else f' (want {want})'))


def main():
    port = free_port()
    here = f'localhost:{port}'
    rebind = f'evil.example:{port}'

    print(f'== default bind (port {port})')
    r = Runner(port)
    try:
        lan = network_address()
        if lan:
            expect(f'not reachable at {lan}',
                   status(lan, port, host=f'{lan}:{port}')[:6], 'error:')
        else:
            print('  skip reachability: no non-loopback address')
        a = '127.0.0.1'
        expect('same-origin page', status(a, port, host=here, origin=f'http://{here}'), '101')
        expect('127.0.0.1 page', status(a, port, host=f'127.0.0.1:{port}',
                                        origin=f'http://127.0.0.1:{port}'), '101')
        expect('script client (no Origin)', status(a, port, host=here), '101')
        expect('page GET /', status(a, port, path='/', upgrade=False, host=here), '200')
        expect('cross-origin page', status(a, port, host=here, origin='http://evil.example'), '403')
        expect('sandboxed page (Origin null)', status(a, port, host=here, origin='null'), '403')
        expect('other port, same host', status(a, port, host=here, origin='http://localhost:1'), '403')
        expect('DNS rebinding, upgrade', status(a, port, host=rebind,
                                                origin=f'http://{rebind}'), '403')
        expect('DNS rebinding, GET /', status(a, port, path='/', upgrade=False, host=rebind), '403')
        expect('oversized Origin', status(a, port, host=here, origin='http://' + 'a' * 400), '403')
        expect('125-byte ping', pong_for(port, 125), 'echo')
        expect('200-byte ping (over the control-frame cap)', pong_for(port, 200), 'closed')
    finally:
        r.close()

    port = free_port()
    here = f'localhost:{port}'
    rebind = f'evil.example:{port}'
    print(f'== --ui-bind 0.0.0.0 (port {port})')
    r = Runner(port, '--ui-bind', '0.0.0.0')
    try:
        a = '127.0.0.1'
        expect('same-origin page', status(a, port, host=here, origin=f'http://{here}'), '101')
        expect('named host, same origin', status(a, port, host=rebind,
                                                 origin=f'http://{rebind}'), '101')
        expect('cross-origin page', status(a, port, host=here, origin='http://evil.example'), '403')
    finally:
        r.close()

    if failed:
        sys.exit(f'check-ui-access: FAIL: {failed} case(s)')
    print('check-ui-access: OK')


if __name__ == '__main__':
    main()
