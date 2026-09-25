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
  - an Origin must be http:// plus the Host exactly (https://, other
    schemes and a bare "://" are refused), and a Host the server cannot
    trust -- repeated, "Host :" with a space, missing under an Origin --
    is refused rather than read as no Host at all;
  - a ping is answered with a pong that echoes it, and a control frame
    longer than RFC 6455's 125 bytes, or fragmented, closes the connection
    instead of drawing a pong;
  - the request line and headers are parsed, not searched: "/wsx" is not
    "/ws", and "upgrade: WebSocket" in lower case still upgrades;
  - a refused value is logged escaped: a Host carrying terminal control
    sequences cannot reach the operator's terminal raw;
  - a connection that never finishes its request gives its slot back, so
    eight idle ones do not lock everyone out for the rest of the run;
  - a bad --ui-bind (not IPv4, value missing, or given with no UI to bind)
    is refused when the options are parsed, and a --ui that cannot start
    (port in use) ends the run -- neither is ignored for the whole run;
  - a ui/index.html that is a FIFO with no writer, or a directory, does not
    hang startup or serve an empty page: the built-in page is served.

Usage: tools/check-ui-access.py        (RUNNER=... to override the binary)
"""
import os
import socket
import struct
import subprocess
import sys
import tempfile
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


def status(addr, port, path='/ws', host=None, origin=None, upgrade=True,
           headers=''):
    """Send one request, return the response's status code (or an error).
    `headers` is sent verbatim, for spellings the keyword forms cannot make."""
    req = f'GET {path} HTTP/1.1\r\n'
    if host is not None:
        req += f'Host: {host}\r\n'
    if origin is not None:
        req += f'Origin: {origin}\r\n'
    req += headers
    if upgrade:
        req += ('Upgrade: websocket\r\nConnection: Upgrade\r\n'
                'Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\n'
                'Sec-WebSocket-Version: 13\r\n')
    try:
        with socket.create_connection((addr, port), timeout=3) as s:
            s.sendall((req + '\r\n').encode())
            reply = b''
            while b'\r\n' not in reply:
                chunk = s.recv(4096)
                if not chunk:
                    break
                reply += chunk
            line = reply.split(b'\r\n')[0].decode(errors='replace')
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


def ws_send(s, opcode, payload, fin=True):
    """One masked client frame (clients must mask, RFC 6455 5.3)."""
    mask = os.urandom(4)
    n = len(payload)
    head = bytes([(0x80 if fin else 0) | opcode])
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
    if n >= 126:
        ext = recv_exact(s, 2 if n == 126 else 8)
        if ext is None:
            return None
        n = struct.unpack('>H' if n == 126 else '>Q', ext)[0]
    payload = recv_exact(s, n)
    return None if payload is None else (head[0] & 0x0f, payload)


def pong_for(port, size, fin=True):
    """'echo' if a size-byte ping draws a matching pong, 'closed' if the
    server hangs up, else what arrived instead.  Broadcast frames that
    arrive first are skipped."""
    ping = bytes(range(256))[:size] if size <= 256 else os.urandom(size)
    try:
        with ws_open(port) as s:
            ws_send(s, 0x9, ping, fin)
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


def exit_code(*args):
    """The runner's exit code for these options (they should be refused at
    once, so a run that is still going after a few seconds is a failure)."""
    try:
        return subprocess.run([RUNNER, 'mixed-multinode', FIRMWARE, '-t', '1000',
                               '-q', *args], stdout=subprocess.DEVNULL,
                              stderr=subprocess.DEVNULL, timeout=10).returncode
    except subprocess.TimeoutExpired:
        return 'still running'


class Runner:
    def __init__(self, port, *extra, cwd=None):
        self.port = port
        self.stderr = tempfile.TemporaryFile()
        self.proc = subprocess.Popen(
            [os.path.abspath(RUNNER), 'mixed-multinode', os.path.abspath(FIRMWARE),
             '-t', '600000', '--ui', str(port), *extra, '-q'],
            stdout=subprocess.DEVNULL, stderr=self.stderr, cwd=cwd)
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

    def errors(self):
        """What the runner has written to stderr so far."""
        self.stderr.seek(0)
        return self.stderr.read()


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
        expect('https:// Origin', status(a, port, host=here, origin=f'https://{here}'), '403')
        expect('other scheme', status(a, port, host=here, origin=f'x://{here}'), '403')
        expect('bare :// Origin', status(a, port, host=here, origin=f'://{here}'), '403')
        expect('Origin, no Host', status(a, port, origin=f'http://{here}'), '403')
        expect('"Host :" (space before colon)',
               status(a, port, headers=f'Host : evil.example:{port}\r\n'), '403')
        expect('Host twice', status(a, port, host=here,
                                    headers=f'Host: evil.example:{port}\r\n'), '403')
        expect('GET /wsx', status(a, port, path='/wsx', host=here), '404')
        expect('lower-case upgrade header', status(a, port, host=here, upgrade=False,
               headers='upgrade: WebSocket\r\nConnection: Upgrade\r\n'
                       'sec-websocket-key: dGhlIHNhbXBsZSBub25jZQ==\r\n'), '101')
        expect('Origin twice', status(a, port, host=here, origin=f'http://{here}',
                                      headers='Origin: http://evil.example\r\n'), '403')
        expect('125-byte ping', pong_for(port, 125), 'echo')
        expect('200-byte ping (over the control-frame cap)', pong_for(port, 200), 'closed')
        expect('fragmented ping (FIN=0)', pong_for(port, 4, fin=False), 'closed')

        time.sleep(1.1)             # past the one-line-a-second refusal log
        status(a, port, path='/', upgrade=False, host='\x1b]0;x\x07\x1b[2J:1')
        time.sleep(0.3)
        err = r.errors()
        expect('refused Host logged escaped',
               (b'\x1b' not in err, b'\\x1b]0;x\\x07' in err), (True, True))

        idle = [socket.create_connection((a, port)) for _ in range(8)]
        try:
            time.sleep(0.3)
            expect('ninth client while 8 sit idle', status(a, port, host=here), '503')
            time.sleep(5.5)         # HTTP_REQUEST_MS: the idle ones are let go
            expect('after the idle ones time out', status(a, port, host=here), '101')
        finally:
            for s in idle:
                s.close()
    finally:
        r.close()

    print('== --ui / --ui-bind that cannot be honoured')
    expect('--ui-bind ::1', exit_code('--ui', str(free_port()), '--ui-bind', '::1'), 2)
    expect('--ui-bind with no value', exit_code('--ui', str(free_port()), '--ui-bind'), 2)
    expect('--ui-bind without --ui', exit_code('--ui-bind', '0.0.0.0'), 2)
    with socket.socket() as taken:
        taken.bind(('127.0.0.1', 0))
        taken.listen()
        expect('--ui on a port already in use',
               exit_code('--ui', str(taken.getsockname()[1])), 2)

    for kind in ('FIFO', 'directory'):
        port = free_port()
        print(f'== ui/index.html is a {kind} (port {port})')
        with tempfile.TemporaryDirectory() as tmp:
            os.mkdir(os.path.join(tmp, 'ui'))
            page = os.path.join(tmp, 'ui', 'index.html')
            if kind == 'FIFO':
                os.mkfifo(page)
            else:
                os.mkdir(page)
            r = Runner(port, cwd=tmp)     # exits the check if it never listens
            try:
                expect('built-in page served', status('127.0.0.1', port, path='/',
                       upgrade=False, host=f'localhost:{port}'), '200')
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
