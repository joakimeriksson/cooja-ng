#!/usr/bin/env bash
# check-renode-cosim.sh — end-to-end check of csim as a clock slave.
#
# Runs the Python mock Renode master (tools/renode-mock-master.py) against
# csim and asserts the three things the feature exists for:
#
#   1. Renode's tick clock drives csim's clock, exactly: after N quanta
#      csim's simulation time has advanced by N * limitBuffer / frequency.
#   2. The data plane works both ways — csim's network delivers frames into
#      the register window (with sender, channel, RSSI and on-air time), and
#      a frame written into the window reaches the emulated firmware.
#   3. A master that dies mid-run ends the csim run cleanly, rather than
#      leaving it free-running without the clock that was meant to drive it.
#
# It also asserts the exchange is deterministic: the same master script twice
# must produce the same csim output, which is a gated guarantee for every
# other path in this project and must not be lost when another simulator
# supplies the horizon.
#
# Usage: tools/check-renode-cosim.sh
# Exit 0 = pass, 1 = a check failed, 2 = usage/build error.
#
# Note: the master binds two localhost ports, so this needs an environment
# that permits listening sockets.

set -u
cd "$(dirname "$0")/.."

RUNNER=${RUNNER:-./build/test_runner}
MASTER=tools/renode-mock-master.py
CFG=${CFG:-configs/test-renode-sky.yaml}
# Quantum size is a CI cost knob, not a coverage knob: the protocol exchange
# is identical whether a quantum is 1 ms or 100 ms — same handshake, same
# tickClock round trip, same bus accesses, same async events — but 100 ms
# quanta need a hundredth of the round trips to cover the same span of
# simulated time.  CI therefore runs coarse.  Drop LIMIT to 1000 for the
# fine-grained timing a real Renode run uses (examples/renode/).
FREQ=${FREQ:-1000000}
LIMIT=${LIMIT:-100000}          # 100 ms per quantum
TICKS=${TICKS:-200}             # 20 s of simulated time
INJECT=${INJECT:-90}            # inject at ~9 s, once the network is up
TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT

[ -x "$RUNNER" ] || { echo "FAIL: $RUNNER not built (run make)"; exit 2; }
[ -f "$MASTER" ] || { echo "FAIL: $MASTER missing"; exit 2; }

spawn_cmd() {
    echo "$RUNNER test $CFG --renode {2}:{0}:{1} --renode-freq $FREQ -q"
}

run_master() {   # $1 = output file, rest = extra master args
    local out=$1; shift
    python3 "$MASTER" --ticks "$TICKS" --limit "$LIMIT" --freq "$FREQ" \
        --spawn "$(spawn_cmd)" "$@" >"$out" 2>&1
    return $?
}

# --- 1 + 2: the clock follows, and frames cross in both directions --------

echo "== run 1: tick clock + data plane =="
if ! run_master "$TMP/run1.txt" --inject-at "$INJECT"; then
    echo "FAIL: the mock master reported an error"
    cat "$TMP/run1.txt"
    exit 1
fi
grep -E '^\[master\] csim advanced' "$TMP/run1.txt" || true

expected_ns=$(( TICKS * LIMIT * 1000000000 / FREQ ))
if ! grep -q "csim advanced ${expected_ns} ns, expected ${expected_ns} ns" "$TMP/run1.txt"; then
    echo "FAIL: csim's clock did not follow the tick clock exactly"
    grep 'csim advanced' "$TMP/run1.txt"
    exit 1
fi

# Frames out of csim into the window.
if ! grep -qE '^\[master\] frames received [1-9]' "$TMP/run1.txt"; then
    echo "FAIL: no frames reached the register window"
    exit 1
fi
# Console lines forwarded as Renode log messages.
if ! grep -qE 'log lines [1-9]' "$TMP/run1.txt"; then
    echo "FAIL: no console lines were forwarded"
    exit 1
fi
# A frame written into the window must reach the emulated firmware: with one
# injection each Sky node sees one extra CRC-valid reception.
rx_with=$(grep -oE 'crc_ok=[0-9]+' "$TMP/run1.txt" | head -1 | cut -d= -f2)

echo "== run 2: same script, no injection (baseline for the TX check) =="
if ! run_master "$TMP/run2.txt"; then
    echo "FAIL: the mock master reported an error on the baseline run"
    cat "$TMP/run2.txt"
    exit 1
fi
rx_without=$(grep -oE 'crc_ok=[0-9]+' "$TMP/run2.txt" | head -1 | cut -d= -f2)

if [ -z "${rx_with:-}" ] || [ -z "${rx_without:-}" ]; then
    echo "FAIL: could not read the CC2420 reception counters"
    exit 1
fi
if [ "$rx_with" -le "$rx_without" ]; then
    echo "FAIL: the injected frame did not reach the emulated firmware"
    echo "  crc_ok with injection: $rx_with, without: $rx_without"
    exit 1
fi
echo "  injected frame received by the firmware (crc_ok $rx_without -> $rx_with)"

# --- determinism: the same master script twice, same csim output ---------

echo "== run 3: determinism =="
if ! run_master "$TMP/run3.txt"; then
    echo "FAIL: the mock master reported an error on the repeat run"
    cat "$TMP/run3.txt"
    exit 1
fi
# Filter the lines that legitimately vary run to run (host wall-clock).
filter() { grep -vE 'Wall-clock time|Speed ratio|MIPS|real-time' "$1"; }
if ! diff <(filter "$TMP/run2.txt") <(filter "$TMP/run3.txt") >"$TMP/diff.txt"; then
    echo "FAIL: two identical master scripts produced different output"
    head -40 "$TMP/diff.txt"
    exit 1
fi
echo "  two identical runs produced identical output"

# --- 3: the master dies mid-run ------------------------------------------

echo "== run 4: master disappears mid-run =="
python3 - "$RUNNER" "$CFG" "$FREQ" >"$TMP/run4.txt" 2>&1 <<'PY'
import socket, subprocess, struct, sys, time
runner, cfg, freq = sys.argv[1], sys.argv[2], sys.argv[3]
MSG = struct.Struct("<iQQi")
HANDSHAKE, TICK_CLOCK, LOG_MESSAGE, INTERRUPT = 10, 1, 5, 6

def recv_msg(sock):
    buf = b""
    while len(buf) < MSG.size:
        chunk = sock.recv(MSG.size - len(buf))
        if not chunk:
            raise ConnectionError("closed")
        buf += chunk
    return MSG.unpack(buf)

def await_tick(async_sock):
    """The tick confirmation comes back on the ASYNC socket (Renode's own
    integration library replies with sendSender), with log messages and
    interrupts possibly ahead of it."""
    while True:
        action, addr, _, _ = recv_msg(async_sock)
        if action == TICK_CLOCK:
            return
        if action == LOG_MESSAGE:
            got = 0
            while got < addr:
                got += len(async_sock.recv(addr - got))

srv = [socket.socket(socket.AF_INET, socket.SOCK_STREAM) for _ in range(2)]
for s in srv:
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("127.0.0.1", 0)); s.listen(1)
ports = [s.getsockname()[1] for s in srv]

child = subprocess.Popen([runner, "test", cfg,
                          "--renode", f"127.0.0.1:{ports[0]}:{ports[1]}",
                          "--renode-freq", freq, "-q"],
                         stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
main_sock, _ = srv[0].accept()
async_sock, _ = srv[1].accept()

main_sock.sendall(MSG.pack(HANDSHAKE, 0, 0, -1))
recv_msg(main_sock)                       # handshake echo, on main
for _ in range(50):                       # a few quanta, then vanish
    main_sock.sendall(MSG.pack(TICK_CLOCK, 0, 1000, -1))
    await_tick(async_sock)
main_sock.close(); async_sock.close()

try:
    out, _ = child.communicate(timeout=60)
except subprocess.TimeoutExpired:
    child.kill(); child.communicate()
    print("HUNG"); sys.exit(1)
text = out.decode("utf-8", "replace")
print(f"exit={child.returncode}")
for line in text.splitlines():
    if "renode:" in line:
        print(line.strip())
PY
rc4=$?
if [ $rc4 -ne 0 ] || grep -q HUNG "$TMP/run4.txt"; then
    echo "FAIL: csim did not exit after the master disappeared"
    cat "$TMP/run4.txt"
    exit 1
fi
exit_code=$(grep -oE '^exit=-?[0-9]+' "$TMP/run4.txt" | cut -d= -f2)
if [ -z "${exit_code:-}" ] || [ "$exit_code" -lt 0 ] || [ "$exit_code" -ge 128 ]; then
    echo "FAIL: csim did not exit cleanly (exit=$exit_code)"
    cat "$TMP/run4.txt"
    exit 1
fi
if ! grep -q 'peer closed the connection' "$TMP/run4.txt"; then
    echo "FAIL: csim did not report the lost master"
    cat "$TMP/run4.txt"
    exit 1
fi
echo "  csim exited cleanly and said why"

echo
echo "PASS: Renode co-simulation checks"
