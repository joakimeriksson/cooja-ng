#!/usr/bin/env bash
# check-determinism.sh — run a simulation twice and assert the output is
# byte-identical once wall-clock/throughput noise is filtered out.
#
# The event-driven kernel is deterministic for a fixed config + seed
# (docs/design/refactor-plan.md §10, §3.14 invariant), so two runs of the
# same scenario must produce identical console output, packet traces, and
# end-of-run counters.  Only the host's wall-clock timing varies run to
# run.  This is the Phase 5 per-milestone guardrail: a behaviour-
# preserving radio-bus extraction must keep this diff empty.
#
# Usage (everything after the script name is passed verbatim to the
# runner, so any mode works):
#   ./tools/check-determinism.sh multinode \
#       firmware/sky/udp-server.sky firmware/sky/udp-client.sky -t 60000
#   ./tools/check-determinism.sh test configs/chain-4node-sky.json
#
# Exit 0 = deterministic (empty diff); exit 1 = divergence (diff shown);
# exit 2 = usage/build error.

set -u

cd "$(dirname "$0")/.."

RUNNER=build/test_runner
if [ ! -x "$RUNNER" ]; then
    echo "error: $RUNNER not built. Run 'make' first." >&2
    exit 2
fi
if [ "$#" -eq 0 ]; then
    echo "usage: $0 <runner args...>" >&2
    exit 2
fi

# Lines that legitimately vary between runs (host timing, not sim state):
# the Performance summary and the per-phase wall-clock breakdown.
NOISE='Wall-clock time|Speed ratio|Throughput|real-time|^  (distribute|deliver|step \(CPU\)|flush/output|channel sync|other/overhead):'

OUT_DIR="${TMPDIR:-/tmp}/csim-determinism.$$"
mkdir -p "$OUT_DIR"
trap 'rm -rf "$OUT_DIR"' EXIT

run_once() {
    # Merge stdout+stderr, strip the noise lines, normalise nothing else.
    "$RUNNER" "$@" 2>&1 | grep -avE "$NOISE"
}

run_once "$@" > "$OUT_DIR/a.txt"
run_once "$@" > "$OUT_DIR/b.txt"

if diff -u "$OUT_DIR/a.txt" "$OUT_DIR/b.txt" > "$OUT_DIR/diff.txt"; then
    lines=$(wc -l < "$OUT_DIR/a.txt")
    echo "DETERMINISTIC: 2 runs identical ($lines lines compared) — $*"
    exit 0
else
    echo "DIVERGENCE between two runs of: $*" >&2
    echo "--- run A   +++ run B ---" >&2
    head -60 "$OUT_DIR/diff.txt" >&2
    exit 1
fi
