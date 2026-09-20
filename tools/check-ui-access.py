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
    server meant to be reached by name) but the Origin rule still applies.

Usage: tools/check-ui-access.py        (RUNNER=... to override the binary)
"""
import os
import socket
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
