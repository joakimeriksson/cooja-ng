#!/bin/bash
# Shell / script-engine smoke check (docs/shell.md).  Unit tests, the
# scripted nRF54L15 Contiki-NG shell test (pass + fail), a pipe-driven
# interactive session, --paused + run/step, and a determinism diff.
set -e
cd "$(dirname "$0")/.."
BIN=./build/test_runner
CFG=configs/shell-nrf54l15-dk.yaml
TMP=${TMPDIR:-/tmp}/csim-shell-check.$$
mkdir -p "$TMP"
trap 'rm -rf "$TMP"' EXIT
fail() { echo "check-shell: FAIL: $*"; exit 1; }
strip() { grep -v 'Wall-clock\|Speed ratio\|Throughput' "$1"; }
# expect_rc N cmd...: the exact exit code matters (docs/shell.md "Exit codes").
expect_rc() {
    local want=$1 rc=0; shift
    "$@" > "$TMP/rc.out" 2>&1 || rc=$?
    [ "$rc" = "$want" ] || { tail -5 "$TMP/rc.out"; fail "$* -> exit $rc, expected $want"; }
}

echo "== unit tests"
$BIN shell > "$TMP/unit.out" || { cat "$TMP/unit.out"; fail "unit tests"; }
tail -1 "$TMP/unit.out"

echo "== scripted test (pass)"
$BIN test $CFG --script test/scripts/shell-nrf54l15.cnsh > "$TMP/pass.out" 2>&1 || fail "pass script exited non-zero"
grep -q "SCRIPT PASSED" "$TMP/pass.out" || fail "no SCRIPT PASSED"

echo "== scripted test (fail): assertion, exit 1"
expect_rc 1 $BIN test $CFG --script test/scripts/shell-nrf54l15-fail.cnsh
grep -q "SCRIPT FAILED: expect" "$TMP/rc.out" || fail "no SCRIPT FAILED"

echo "== a run that never starts is a configuration error, exit 2"
expect_rc 2 $BIN test $CFG -q --script /nonexistent/missing.cnsh
grep -q "cannot open" "$TMP/rc.out" || fail "missing --script not reported"
expect_rc 2 $BIN test $CFG -q --wall-timeout abc
grep -q "bad value" "$TMP/rc.out" || fail "bad --wall-timeout not reported"
expect_rc 2 $BIN test $CFG -q --wall-timeout
grep -q "missing value" "$TMP/rc.out" || fail "flag without a value not reported"
expect_rc 2 $BIN test $CFG -q --wall-timout 5
grep -q "unknown option" "$TMP/rc.out" || fail "unknown option not reported"
expect_rc 2 $BIN test configs/does-not-exist.yaml -q

echo "== determinism"
$BIN test $CFG --script test/scripts/shell-nrf54l15.cnsh > "$TMP/pass2.out" 2>&1 || fail "second run"
diff <(strip "$TMP/pass.out") <(strip "$TMP/pass2.out") > /dev/null || fail "script run is not deterministic"

echo "== pipe-driven interactive session"
printf 'nodes\nstatus\nsendln 1 help\nexpect 1 "Shows this help" 5s\nlog off all\nexit\n' \
    | $BIN test $CFG --shell > "$TMP/pipe.out" 2>&1 || fail "pipe session exited non-zero"
grep -q "shell.nrf54l15-dk" "$TMP/pipe.out" || fail "nodes table missing"
grep -q "SCRIPT PASSED" "$TMP/pipe.out" || fail "interactive expect not reported"

echo "== --paused, run 500ms, step"
printf 'status\nrun 500ms\nstatus\nstep 5\nstatus\nexit\n' \
    | $BIN test $CFG --shell --paused -q > "$TMP/paused.out" 2>&1 || fail "paused session"
grep -q "time: 0.000 s  state: paused" "$TMP/paused.out" || fail "not paused at start"
grep -q "time: 0.500 s  state: paused" "$TMP/paused.out" || fail "run 500ms did not pause at 0.500 s"

echo "== log-file"
printf "log-file $TMP/n1.log 1\nsendln 1 help\nexpect 1 \"Shows this help\" 5s\nexit\n" | $BIN test $CFG --shell -q > /dev/null 2>&1 || fail "log-file session"
grep -q "\[Node 1/ARM\]" "$TMP/n1.log" || fail "log file empty"

echo "== speed change while running does not stall"
SECONDS=0
printf 'sleep 30s\nspeed 1\nsleep 1s\nexit\n' \
    | timeout 30 $BIN test $CFG --shell -q > "$TMP/speed.out" 2>&1 || fail "speed session"
[ "$SECONDS" -lt 5 ] || fail "1 s at speed 1 took ${SECONDS}s wall (pacing not rebased)"

echo "== --wall-timeout: a bare number is seconds, exit 6"
SECONDS=0
printf 'speed 1\nsleep 60s\nexit\n' > "$TMP/long.cnsh"
expect_rc 6 $BIN test $CFG -q --script "$TMP/long.cnsh" --wall-timeout 1
[ "$SECONDS" -lt 5 ] || fail "--wall-timeout 1 took ${SECONDS}s wall (read as 1 ms or not at all?)"
grep -q -- "--wall-timeout: 1.000 s" "$TMP/rc.out" || fail "wall-timeout not reported as 1 s"
expect_rc 6 $BIN test $CFG -q --script "$TMP/long.cnsh" --wall-timeout=500ms
grep -q -- "--wall-timeout: 0.500 s" "$TMP/rc.out" || fail "500ms not reported as 0.5 s"

echo "== paused + blocked pipe fails instead of hanging (a deadlock is exit 2)"
expect_rc 2 bash -c "printf 'pause\nsleep 1s\nexit\n' | timeout 20 $BIN test $CFG --shell -q"
grep -q "deadlock" "$TMP/rc.out" || fail "deadlock not reported (hang or wrong failure)"

echo "== piped session is deterministic"
PIPED='sendln 1 help\nexpect 1 "Shows this help" 5s\nstatus\nsleep 250ms\nsendln 1 ip-addr\nexpect 1 "Node IPv6" 5s\nnodes\nexit\n'
printf "$PIPED" | $BIN test $CFG --shell > "$TMP/piped1.out" 2>&1 || fail "piped run 1"
printf "$PIPED" | $BIN test $CFG --shell > "$TMP/piped2.out" 2>&1 || fail "piped run 2"
diff <(strip "$TMP/piped1.out") <(strip "$TMP/piped2.out") > /dev/null || fail "piped session is not deterministic"

echo "== --script EOF waits for pending at"
printf 'at 2s echo fired-at-2s\n' > "$TMP/ateof.cnsh"
$BIN test $CFG -q --script "$TMP/ateof.cnsh" > "$TMP/ateof.out" 2>&1 || fail "at-EOF script"
grep -q "fired-at-2s" "$TMP/ateof.out" || fail "pending at did not fire before the run ended"

if command -v python3 > /dev/null; then
    echo "== terminal paths (tools/check-shell-tty.py)"
    python3 tools/check-shell-tty.py > "$TMP/tty.out" 2>&1 || { cat "$TMP/tty.out"; fail "tty checks"; }
fi

echo "check-shell: OK"
