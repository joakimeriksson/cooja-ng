#!/bin/bash
#
# External-node peer check.  Runs a Sky node beside
# test/ext/fault-injecting-peer.py in each of its modes:
#
#   stuck, rewind  a peer that keeps asking to be woken at (or before) where
#                  it stands must be failed, and the run must still finish;
#   inf            a non-finite time from the peer must fail the node, not
#                  be converted to an integer (undefined behavior);
#   ok             the fault-free control must not be failed.
#
# A livelock never returns to the runner's loop, where --wall-timeout is
# checked, so each run is also killed after LIMIT seconds of wall time
# (a portable stand-in for coreutils timeout, which macOS lacks).
#
# Usage: tools/check-ext-peer.sh          (RUNNER=... to override)
#
set -u
cd "$(dirname "$0")/.."
RUNNER=${RUNNER:-./build/test_runner}
TMP=${TMPDIR:-/tmp}/csim-ext-peer.$$
mkdir -p "$TMP"
trap 'rm -rf "$TMP"' EXIT

cat > "$TMP/peer.json" <<JSON
{
  "title": "faulty external peer",
  "timeout_ms": 2000,
  "nodes": [
    { "firmware": "firmware/sky/hello-world.sky", "id": 1, "x": 0.0, "y": 0.0 },
    { "firmware": "test/ext/fault-injecting-peer.py", "id": 2, "x": 1.0, "y": 0.0 }
  ],
  "test": { "timeout_is_success": true }
}
JSON

LIMIT=${LIMIT:-60}

# Run "$@" and return its exit status, or 124 if it outlives LIMIT.
bounded() {
    "$@" &
    local pid=$!
    ( sleep "$LIMIT"; kill -9 "$pid" 2>/dev/null ) &
    local watcher=$!
    wait "$pid"
    local rc=$?
    kill "$watcher" 2>/dev/null
    wait "$watcher" 2>/dev/null
    [ "$rc" -eq 137 ] && rc=124
    return "$rc"
}

failed=0
check() {   # mode, expected stderr pattern ("" = the node must NOT fail)
    local mode=$1 want=$2 log="$TMP/$1.log"
    PEER_MODE=$mode bounded "$RUNNER" test "$TMP/peer.json" \
        --wall-timeout 30s -q > "$log" 2>&1
    local rc=$? why=""
    if [ "$rc" -eq 6 ] || [ "$rc" -eq 124 ]; then
        why="run did not finish (livelock)"
    elif [ "$rc" -ge 128 ]; then
        why="killed by signal $((rc - 128))"
    elif [ -n "$want" ] && ! grep -q "ext_node\[2\].*$want" "$log"; then
        why="node not failed with '$want'"
    elif [ -z "$want" ] && grep -q "ext_node\[2\]" "$log"; then
        why="fault-free peer failed: $(grep -m1 'ext_node\[2\]' "$log")"
    fi
    if [ -n "$why" ]; then
        echo "  FAIL $mode: $why (exit $rc)"
        failed=$((failed + 1))
    else
        echo "  ok   $mode (exit $rc)"
    fi
}

check stuck  "steps in a row without its clock moving"
check rewind "steps in a row without its clock moving"
check inf    "is not a time in ns"
check ok     ""

if [ "$failed" -ne 0 ]; then
    echo "check-ext-peer: FAIL: $failed case(s)"
    exit 1
fi
echo "check-ext-peer: OK"
