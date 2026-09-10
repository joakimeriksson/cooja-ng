#!/usr/bin/env python3
"""Mock Renode master for csim's co-simulation slave mode.

Plays the part of Renode's CoSimulationPlugin: listens on two TCP ports,
performs the handshake, drives the tick clock, pokes csim's register window
over the bus, and reads back the interrupts and log messages csim sends on
the async socket.

This is what proves the protocol end to end without installing Renode.  The
messages, the socket roles and the ordering are Renode's; only the policy
(how many ticks, what to poke) is ours.

  tools/renode-mock-master.py --spawn './build/test_runner test \\
      configs/test-renode-sky.yaml --renode {2}:{0}:{1} --renode-freq 1000000'

{0} is the main port, {1} the async port and {2} the address, the same
substitution Renode's SimulationContext uses.

Exit status is 0 only when the run did what it was asked to: the ticks were
acknowledged, at least one frame was received from csim's network and at
least one console line arrived, unless those checks are turned off.
"""

import argparse
import shlex
import socket
import struct
import subprocess
import sys
import time

# --- protocol (must match include/common/renode_proto.h) ---------------------

MSG = struct.Struct("<iQQi")            # action, addr, value, peripheralIndex
assert MSG.size == 24

INVALID, TICK_CLOCK, WRITE_REQUEST, READ_REQUEST, RESET_PERIPHERAL = 0, 1, 2, 3, 4
LOG_MESSAGE, INTERRUPT, DISCONNECT, ERROR, OK, HANDSHAKE = 5, 6, 7, 8, 9, 10
READ_BYTE, READ_WORD, READ_DWORD, READ_QWORD = 21, 22, 23, 24
WRITE_BYTE, WRITE_WORD, WRITE_DWORD, WRITE_QWORD = 25, 26, 27, 28

NO_PERIPHERAL = -1

# --- register map (must match include/native/renode_dev.h) -------------------

REG_ID, REG_VERSION, REG_CTRL, REG_STATUS = 0x00, 0x04, 0x08, 0x0C
REG_CHANNEL, REG_TXPOWER = 0x10, 0x14
REG_TX_LEN, REG_TX_DATA, REG_TX_CTRL, REG_TX_COUNT = 0x18, 0x1C, 0x20, 0x24
REG_RX_COUNT, REG_RX_LEN, REG_RX_DATA, REG_RX_POP = 0x28, 0x2C, 0x30, 0x34
REG_RX_RSSI, REG_RX_FROM, REG_RX_CHANNEL = 0x38, 0x3C, 0x40
REG_RX_TIME_LO, REG_RX_TIME_HI, REG_RX_DROPPED = 0x44, 0x48, 0x4C
REG_SIM_TIME_LO, REG_SIM_TIME_HI = 0x50, 0x54
REG_NODE_COUNT, REG_SELF_ID = 0x58, 0x5C
REG_UART_NODE, REG_UART_DATA, REG_UART_COUNT, REG_IRQ_STATUS = 0x60, 0x64, 0x68, 0x6C

CTRL_IRQ_RX_EN, CTRL_IRQ_UART_EN = 1 << 0, 1 << 1

EXPECTED_ID = 0x4353494D               # "CSIM"


class Master:
    def __init__(self, main_sock, async_sock, verbose=False):
        self.main = main_sock
        self.asyn = async_sock
        self.verbose = verbose
        self.interrupts = 0
        self.log_lines = 0
        self.irq_level = 0

    # -- wire ---------------------------------------------------------------

    def _send(self, sock, action, addr=0, value=0):
        sock.sendall(MSG.pack(action, addr, value, NO_PERIPHERAL))

    def _recv(self, sock):
        buf = b""
        while len(buf) < MSG.size:
            chunk = sock.recv(MSG.size - len(buf))
            if not chunk:
                raise ConnectionError("peer closed the connection")
            buf += chunk
        return MSG.unpack(buf)

    def _recv_text(self, sock, n):
        buf = b""
        while len(buf) < n:
            chunk = sock.recv(n - len(buf))
            if not chunk:
                raise ConnectionError("peer closed mid-log")
            buf += chunk
        return buf

    # -- protocol -----------------------------------------------------------

    def handshake(self):
        self._send(self.main, HANDSHAKE)
        action, _, _, _ = self._recv(self.main)
        if action != HANDSHAKE:
            raise RuntimeError(f"expected handshake, got action {action}")
        if self.verbose:
            print("[master] handshake complete")

    def read(self, offset, width=4):
        action = {1: READ_BYTE, 2: READ_WORD, 4: READ_DWORD, 8: READ_QWORD}[width]
        self._send(self.main, action, offset)
        act, _, value, _ = self._recv(self.main)
        if act != READ_REQUEST:
            raise RuntimeError(f"expected a read reply, got action {act}")
        return value

    def write(self, offset, value, width=4):
        action = {1: WRITE_BYTE, 2: WRITE_WORD, 4: WRITE_DWORD, 8: WRITE_QWORD}[width]
        self._send(self.main, action, offset, value)
        act, _, _, _ = self._recv(self.main)
        if act != OK:
            raise RuntimeError(f"expected ok for a write, got action {act}")

    def tick(self, ticks):
        """Advance the peer by `ticks` and wait for it to say it is done.

        Renode blocks its whole emulation here, which is what makes the two
        clocks stay together.

        The request goes out on the main socket and the confirmation comes
        back on the ASYNC one -- that asymmetry is Renode's, not ours (see
        renode_bus.cpp: `case tickClock` replies with sendSender), and it is
        the single easiest thing to get wrong.  Anything else arriving on
        async first (log messages, interrupts) is drained here so the caller
        still sees it."""
        self._send(self.main, TICK_CLOCK, 0, ticks)
        while True:
            action, addr, value, _ = self._recv(self.asyn)
            if action == TICK_CLOCK:
                return
            if action == INTERRUPT:
                self.interrupts += 1
                self.irq_level = value
            elif action == LOG_MESSAGE:
                text = self._recv_text(self.asyn, addr).decode("utf-8", "replace")
                for line in text.splitlines():
                    self.log_lines += 1
                    if self.verbose:
                        print(f"[csim log] {line}")
            else:
                raise RuntimeError(f"unexpected async action {action} while ticking")

    def disconnect(self):
        self._send(self.main, DISCONNECT)

    # -- async plane --------------------------------------------------------

    def drain_async(self):
        """Read whatever the peer pushed: interrupts and log messages."""
        self.asyn.setblocking(False)
        try:
            while True:
                try:
                    head = self.asyn.recv(MSG.size, socket.MSG_PEEK)
                except (BlockingIOError, InterruptedError):
                    return
                if len(head) < MSG.size:
                    return
                self.asyn.setblocking(True)
                action, addr, value, _ = self._recv(self.asyn)
                if action == INTERRUPT:
                    self.interrupts += 1
                    self.irq_level = value
                    if self.verbose:
                        print(f"[master] interrupt {addr} -> {value}")
                elif action == LOG_MESSAGE:
                    text = self._recv_text(self.asyn, addr).decode("utf-8", "replace")
                    for line in text.splitlines():
                        self.log_lines += 1
                        if self.verbose:
                            print(f"[csim log] {line}")
                elif action == OK:
                    if self.verbose:
                        print("[master] disconnect acknowledged")
                    return
                else:
                    if self.verbose:
                        print(f"[master] unexpected async action {action}")
                self.asyn.setblocking(False)
        finally:
            try:
                self.asyn.setblocking(True)
            except OSError:
                pass

    # -- device helpers -----------------------------------------------------

    def pull_frames(self):
        """Read every queued frame out of the RX FIFO."""
        frames = []
        while self.read(REG_RX_COUNT):
            length = self.read(REG_RX_LEN)
            meta = {
                "from": ctypes_i32(self.read(REG_RX_FROM)),
                "channel": ctypes_i32(self.read(REG_RX_CHANNEL)),
                "rssi": ctypes_i32(self.read(REG_RX_RSSI)),
                "start_ns": self.read(REG_RX_TIME_LO) |
                            (self.read(REG_RX_TIME_HI) << 32),
            }
            data = bytes(self.read(REG_RX_DATA, 1) for _ in range(length))
            self.write(REG_RX_POP, 1)
            meta["data"] = data
            frames.append(meta)
        return frames

    def send_frame(self, payload, channel=None):
        if channel is not None:
            self.write(REG_CHANNEL, channel)
        self.write(REG_TX_LEN, len(payload))
        for b in payload:
            self.write(REG_TX_DATA, b, width=1)
        self.write(REG_TX_CTRL, 1)

    def sim_time_ns(self):
        return self.read(REG_SIM_TIME_LO) | (self.read(REG_SIM_TIME_HI) << 32)


def ctypes_i32(v):
    v &= 0xFFFFFFFF
    return v - (1 << 32) if v & 0x80000000 else v


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--spawn", metavar="CMD",
                    help="command to start csim; {0}=main port {1}=async port {2}=address")
    ap.add_argument("--address", default="127.0.0.1")
    ap.add_argument("--ticks", type=int, default=2000,
                    help="number of quanta to run (default 2000)")
    ap.add_argument("--limit", type=int, default=1000,
                    help="ticks per quantum, Renode's limitBuffer (default 1000)")
    ap.add_argument("--freq", type=int, default=1000000,
                    help="tick frequency in Hz, for reporting (default 1e6)")
    ap.add_argument("--inject-at", type=int, default=-1, metavar="Q",
                    help="transmit a frame into csim's network at quantum Q")
    ap.add_argument("--inject-burst", type=int, default=1, metavar="N",
                    help="inject N frames back to back starting at --inject-at, one "
                         "per quantum. A frame-level sender (Renode's radio has no "
                         "air time) can emit a burst far faster than csim's medium "
                         "carries it; this is how to see what that costs.")
    ap.add_argument("--channel", type=int, default=26)
    ap.add_argument("--frame", metavar="HEX",
                    help="MAC frame to inject, hex, without FCS (csim appends it). "
                         "Default is a broadcast data frame; pass a unicast one with "
                         "the ACK-request bit set to exercise the auto-ACK path.")
    ap.add_argument("--irq-uart", action="store_true",
                    help="also interrupt on console output. Off by default: this "
                         "master never reads UART_DATA, and an undrained source "
                         "holds the level line high forever (as real hardware would)")
    ap.add_argument("--require-frames", type=int, default=1,
                    help="fail unless at least this many frames were received")
    ap.add_argument("--require-logs", type=int, default=1,
                    help="fail unless at least this many log lines arrived")
    ap.add_argument("--connect-timeout", type=float, default=30.0)
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args()

    main_srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    main_srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    main_srv.bind((args.address, 0))
    main_srv.listen(1)
    async_srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    async_srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    async_srv.bind((args.address, 0))
    async_srv.listen(1)

    main_port = main_srv.getsockname()[1]
    async_port = async_srv.getsockname()[1]
    if args.verbose:
        print(f"[master] listening on {main_port} (main) and {async_port} (async)")

    child = None
    if args.spawn:
        cmd = args.spawn.format(main_port, async_port, args.address)
        if args.verbose:
            print(f"[master] spawning: {cmd}")
        child = subprocess.Popen(shlex.split(cmd))

    main_srv.settimeout(args.connect_timeout)
    async_srv.settimeout(args.connect_timeout)
    try:
        main_sock, _ = main_srv.accept()
        async_sock, _ = async_srv.accept()
    except socket.timeout:
        print("[master] csim did not connect", file=sys.stderr)
        if child:
            child.kill()
        return 2

    main_sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    m = Master(main_sock, async_sock, verbose=args.verbose)

    frames = []
    rc = 0
    try:
        m.handshake()

        dev_id = m.read(REG_ID)
        if dev_id != EXPECTED_ID:
            raise RuntimeError(f"device id 0x{dev_id:08x}, expected 0x{EXPECTED_ID:08x}")
        if args.verbose:
            print(f"[master] device: id=CSIM version={m.read(REG_VERSION)} "
                  f"self_id={ctypes_i32(m.read(REG_SELF_ID))} "
                  f"nodes={m.read(REG_NODE_COUNT)}")

        # Listen on the network and ask to be interrupted for frames and for
        # the bridged node's console.
        m.write(REG_CHANNEL, args.channel)
        ctrl = CTRL_IRQ_RX_EN | (CTRL_IRQ_UART_EN if args.irq_uart else 0)
        m.write(REG_CTRL, ctrl)

        # csim's clock has its own origin (nodes boot at a staggered start
        # offset), so what must track the tick clock is ELAPSED time, not the
        # absolute reading.  Take the origin before ticking.
        t0_ns = m.sim_time_ns()
        if args.verbose:
            print(f"[master] csim clock origin {t0_ns} ns")

        for q in range(args.ticks):
            m.tick(args.limit)
            m.drain_async()
            if m.irq_level:
                frames.extend(m.pull_frames())
            if args.inject_at >= 0 and args.inject_at <= q < args.inject_at + args.inject_burst:
                # A minimal 802.15.4 data frame; csim's PHY wrap adds the
                # preamble and a valid CRC.
                frame = (bytes.fromhex(args.frame) if args.frame
                         else bytes([0x41, 0x88, 0x2A, 0xCD, 0xAB,
                                     0xFF, 0xFF, 0x01, 0x00]))
                m.send_frame(frame, args.channel)
                if args.verbose:
                    print(f"[master] injected a frame at quantum {q}")

        elapsed_ns = m.sim_time_ns() - t0_ns
        expected_ns = args.ticks * args.limit * 10**9 // args.freq
        print(f"[master] {args.ticks} quanta x {args.limit} ticks at {args.freq} Hz")
        print(f"[master] csim advanced {elapsed_ns} ns, expected {expected_ns} ns")
        print(f"[master] frames received {len(frames)}, "
              f"interrupts {m.interrupts}, log lines {m.log_lines}")
        for f in frames[:5]:
            print(f"  frame from node {f['from']} ch {f['channel']} "
                  f"rssi {f['rssi']} dBm at {f['start_ns']} ns: {f['data'].hex()}")

        if elapsed_ns != expected_ns:
            print("[master] FAIL: csim's clock did not follow the tick clock",
                  file=sys.stderr)
            rc = 1
        if len(frames) < args.require_frames:
            print(f"[master] FAIL: {len(frames)} frames, "
                  f"wanted {args.require_frames}", file=sys.stderr)
            rc = 1
        if m.log_lines < args.require_logs:
            print(f"[master] FAIL: {m.log_lines} log lines, "
                  f"wanted {args.require_logs}", file=sys.stderr)
            rc = 1

        m.disconnect()
        m.drain_async()
    except (ConnectionError, RuntimeError, OSError) as e:
        print(f"[master] {e}", file=sys.stderr)
        rc = 1
    finally:
        main_sock.close()
        async_sock.close()
        main_srv.close()
        async_srv.close()
        if child:
            try:
                child.wait(timeout=15)
            except subprocess.TimeoutExpired:
                child.kill()
                child.wait()
            if child.returncode not in (0, None):
                print(f"[master] csim exited with {child.returncode}", file=sys.stderr)
                rc = rc or 1
    return rc


if __name__ == "__main__":
    sys.exit(main())
