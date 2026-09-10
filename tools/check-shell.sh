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

echo "== unit tests"
$BIN shell > "$TMP/unit.out" || { cat "$TMP/unit.out"; fail "unit tests"; }
tail -1 "$TMP/unit.out"

echo "== scripted test (pass)"
$BIN test $CFG --script test/scripts/shell-nrf54l15.cnsh > "$TMP/pass.out" 2>&1 || fail "pass script exited non-zero"
grep -q "SCRIPT PASSED" "$TMP/pass.out" || fail "no SCRIPT PASSED"

echo "== scripted test (fail)"
if $BIN test $CFG --script test/scripts/shell-nrf54l15-fail.cnsh > "$TMP/fail.out" 2>&1; then
    fail "fail script exited 0"
fi
grep -q "SCRIPT FAILED: expect" "$TMP/fail.out" || fail "no SCRIPT FAILED"

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

echo "check-shell: OK"
