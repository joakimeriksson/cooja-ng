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

NOISE='Wall-clock time|Speed ratio|Throughput|real-time|Phase Timing|^  (distribute|deliver|step \(CPU\)|flush/output|channel sync|other/overhead):'
tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT

"$RUNNER" test "$V1" "$@" 2>&1 | grep -vE "$NOISE" > "$tmp/v1.txt"
"$RUNNER" test "$V2" "$@" 2>&1 | grep -vE "$NOISE" > "$tmp/v2.txt"

if diff -q "$tmp/v1.txt" "$tmp/v2.txt" >/dev/null; then
    n=$(wc -l < "$tmp/v1.txt")
    echo "EQUIVALENT: $V1 <-> $V2 ($n lines compared)"
    exit 0
else
    echo "DIFF: $V1 <-> $V2"
    diff "$tmp/v1.txt" "$tmp/v2.txt" | head -40
    exit 1
fi
