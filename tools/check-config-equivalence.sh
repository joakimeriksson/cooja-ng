#!/bin/bash
#
# Config v1 <-> v2 equivalence check (Phase 7).
#
# Runs a v1 config and its v2 twin through the runner and asserts the
# simulation output is byte-identical (minus host-timing lines), proving the
# v2 parser produces the same normalized config as the equivalent v1 config.
#
# Usage: tools/check-config-equivalence.sh <v1.json> <v2.json> [-t ms] [runner args...]
#
set -u
RUNNER=${RUNNER:-./build/test_runner}
V1=$1; V2=$2; shift 2

# Host-timing lines, plus the loaded-config summary sim_config_print emits:
# two equivalent configs may legitimately SAY different things (a saved
# config carries explicit positions and a medium block the original left
# implicit) while simulating identically — the simulation is what is compared.
NOISE='Wall-clock time|Speed ratio|Throughput|real-time|Phase Timing|^  (distribute|deliver|step \(CPU\)|flush/output|channel sync|other/overhead):|^Config: |^  (timeout_ms|seed|startup_delay_ms|speed|radiomedium|nodes): |^    (tx_range|interference_range|success_ratio_tx|success_ratio_rx): |^    \[[0-9]+\] id='
tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT

# stdout and stderr are captured and compared SEPARATELY.  Merging them with
# 2>&1 through a pipe made the comparison depend on the interleaving of two
# independently buffered streams — on Linux this produced a "difference" that
# was the same lines in a different order, with a console line spliced into
# the middle of a [PKT] line.  Each stream on its own is deterministic.
"$RUNNER" test "$V1" "$@" 2>"$tmp/v1.err" | grep -vE "$NOISE" > "$tmp/v1.txt"
"$RUNNER" test "$V2" "$@" 2>"$tmp/v2.err" | grep -vE "$NOISE" > "$tmp/v2.txt"
grep -vE "$NOISE" "$tmp/v1.err" > "$tmp/v1.err.f"; grep -vE "$NOISE" "$tmp/v2.err" > "$tmp/v2.err.f"

if diff -q "$tmp/v1.txt" "$tmp/v2.txt" >/dev/null && diff -q "$tmp/v1.err.f" "$tmp/v2.err.f" >/dev/null; then
    n=$(wc -l < "$tmp/v1.txt"); e=$(wc -l < "$tmp/v1.err.f")
    echo "EQUIVALENT: $V1 <-> $V2 ($n stdout + $e stderr lines compared)"
    exit 0
else
    echo "DIFF: $V1 <-> $V2"
    diff "$tmp/v1.txt" "$tmp/v2.txt" | head -40
    diff "$tmp/v1.err.f" "$tmp/v2.err.f" | head -20
    exit 1
fi
