#!/bin/bash
#
# check-baseline.sh — does this tree still produce the same simulations as a
# reference revision?
#
# A given config plus seed must produce the same run, byte for byte, and a
# refactor must not move it.  "The tests still pass" is too weak a signal for
# anything touching the kernel clock, the event pump or a mote tick: TSCH can
# still associate and RPL can still form a DAG after a timing shift that has,
# in fact, changed the simulation.
#
# So: build a reference revision, run a set of workloads with both binaries
# *from this working tree* (same configs, same firmware — only the engine
# differs), and diff the output line by line.  Every console line carries a
# simulated timestamp, so identical output means every mote executed at the
# same nanosecond and printed the same bytes.
#
# Its sibling tools/check-determinism.sh answers a different question — is one
# run reproducible? — by running the same simulation twice with one binary.
#
#   tools/check-baseline.sh                 # reference: merge-base with main
#   tools/check-baseline.sh main            # or any ref
#   tools/check-baseline.sh HEAD~3
#   KEEP=1 tools/check-baseline.sh          # keep the logs for inspection
#   TIMING=1 tools/check-baseline.sh        # run the workloads one at a time
#
# Each line also carries both binaries' wall time and the change.  The
# workloads run concurrently by default, so those numbers are contended and
# only indicative (tens of percent): a regression signal to follow up with
# a real measurement, not a benchmark.  TIMING=1 serialises the runs, ref and
# head back to back per workload, for numbers worth quoting, at about one
# workload's worth of time per workload.
# The simulation output is compared the same way either way.
#
# Exit code 0 = identical, 1 = a workload differs, or did not complete under
# this tree's binary (non-zero exit, timeout, no Wall-clock line).
#
set -u

ROOT=$(cd "$(dirname "$0")/.." && pwd)
cd "$ROOT" || exit 1

# Default: the merge-base with main, preferring a remote main (a local one can
# be many merges stale — and a reference too old to build is no reference at
# all).  Remote refs are as of your last `git fetch`.
REF=${1:-}
REF_NAME=$REF
if [ -z "$REF" ]; then
    for base in upstream/main origin/main main; do
        git rev-parse --verify --quiet "$base" >/dev/null || continue
        REF=$(git merge-base HEAD "$base" 2>/dev/null) && [ -n "$REF" ] || continue
        REF_NAME="merge-base with $base"
        break
    done
    [ -n "$REF" ] || { echo "no main to branch from; pass a ref explicitly" >&2; exit 1; }
fi
REF_SHA=$(git rev-parse --short "$REF") || exit 1

WORK=$(mktemp -d "${TMPDIR:-/tmp}/csim-baseline.XXXXXX")
cleanup() {
    git worktree remove --force "$WORK/ref" >/dev/null 2>&1
    if [ -n "${KEEP:-}" ]; then echo "logs kept in $WORK"; else rm -rf "$WORK"; fi
}
trap cleanup EXIT

# The workloads: MSP430, ARM and mixed-ISA, the timing-sensitive TSCH run, both
# TrustZone configs, and the shell service in a headless run.  Add a line here
# when a platform grows a config worth protecting.
WORKLOADS=(
    "chain4sky|test configs/chain-4node-sky.yaml"
    "mixed|test configs/test-mixed-platform-rpl.yaml"
    "tsch52|test configs/test-tsch-nrf52840-dk.json"
    "tzrpl|test configs/test-tz-rpl-udp-nrf54l15-xiao.yaml"
    "tzwdt|test configs/test-tz-watchdog-nrf54l15-xiao.yaml"
    "shell54|test configs/test-shell-nrf54l15-dk.yaml"
    "nrf2|test configs/test-2node-nrf54l15-dk.json"
    "mn|multinode -t 20000"
    "armmn|arm-multinode firmware/cc2538dk/udp-server.cc2538dk firmware/cc2538dk/udp-client.cc2538dk -t 60000"
)

# Wall-clock lines legitimately differ between two builds of the same code,
# and the MSP430 PC-trace diagnostics ("PC trace:" / "FW cc2420_transmit=")
# are opt-in (CSIM_PC_TRACE=1) since the hook stopped being installed by
# default, so a reference from before that prints two lines this tree does
# not.  Everything else — simulated timestamps, packet counts, cycle totals,
# exit codes — must match exactly.
FILTER='^ *(Wall-clock time|Speed ratio|Throughput|PC trace):|^ *FW cc2420_transmit='

build() {  # build <srcdir> <label>
    echo "  building $2 ..."
    if ! make -C "$1" -j"$(nproc 2>/dev/null || echo 4)" >"$WORK/build-$2.log" 2>&1; then
        echo "FAIL: $2 does not build; see $WORK/build-$2.log" >&2
        [ "$2" = ref ] && echo "      (a reference older than a build fix cannot be" \
             "compared against — try a newer one, e.g. upstream/main after a fetch)" >&2
        KEEP=1
        return 1
    fi
}

# stdout and stderr are captured separately.  Merged into one pipe they
# interleave at stdout's buffer-flush boundaries, so a run that prints one
# line more or less early on shows every later stderr line ([PKT], [RF],
# warnings) at a different place in the merged log -- a diff with nothing
# behind it.  Compared apart, each stream is exactly what the run wrote.
run_one() {  # run_one <binary> <outdir> <workload>
    ( timeout 900 "$1" ${3#*|} >"$2/${3%%|*}.log" 2>"$2/${3%%|*}.err"
      echo "rc=$?" >>"$2/${3%%|*}.log" ) &
    if [ -n "${TIMING:-}" ]; then wait; fi
}

# The runner's own "Wall-clock time: N ms" line.  It prints exactly one, and
# after a restart it covers only the segment since the restart (the start
# time is reset), so a whole-run total is not recoverable from the log.
# Empty when the run did not get that far.
wall_ms() { sed -n 's/^ *Wall-clock time: *\([0-9.]*\) ms.*/\1/p' "$1" | tail -1; }

# The exit code the run appended to its log.
run_rc() { sed -n 's/^rc=//p' "$1" | tail -1; }

# "ref 437.7 ms  head 548.7 ms  +25.4 %" for the two logs, or a note.
timing() {
    local a b
    a=$(wall_ms "$1"); b=$(wall_ms "$2")
    if [ -z "$a" ] || [ -z "$b" ]; then echo "(no wall time)"; return; fi
    awk -v a="$a" -v b="$b" 'BEGIN {
        if (a + 0 <= 0) print "(no wall time)"
        else printf "ref %8.1f ms  head %8.1f ms  %+6.1f %%", a, b, (b - a) * 100 / a }'
}

echo "=== baseline: this tree vs $REF_SHA${REF_NAME:+ ($REF_NAME)}"
git worktree add --detach "$WORK/ref" "$REF" >/dev/null 2>&1 || {
    echo "cannot create a worktree at $REF" >&2; exit 1; }
build "$WORK/ref" ref || exit 1
build "$ROOT" head || exit 1

# Both binaries run from THIS tree, so configs and firmware are identical and
# only the engine differs.  (A config that changed since the reference may not
# load in the older binary — that shows up as a diff in its log.)
echo "  running ${#WORKLOADS[@]} workloads with each binary${TIMING:+, one at a time (TIMING)} ..."
# Interleaved per workload, so with TIMING=1 each pair runs back to back under
# the same machine state (thermal, turbo) rather than all of head minutes later.
mkdir -p "$WORK/out-ref" "$WORK/out-head"
for w in "${WORKLOADS[@]}"; do
    run_one "$WORK/ref/build/test_runner" "$WORK/out-ref"  "$w"
    run_one "$ROOT/build/test_runner"     "$WORK/out-head" "$w"
done
wait

rc=0
for w in "${WORKLOADS[@]}"; do
    name=${w%%|*}
    a="$WORK/out-ref/$name.log"
    b="$WORK/out-head/$name.log"
    # A workload that fails the same way under both binaries diffs clean, so
    # it must not complete: a non-zero exit or no Wall-clock line fails it.
    brc=$(run_rc "$b")
    bwall=$(wall_ms "$b")
    if [ "$brc" != 0 ] || [ -z "$bwall" ]; then
        note="rc=${brc:-?}"
        [ -n "$bwall" ] || note="$note, no wall time"
        printf "  FAIL  %-9s (%s)\n" "$name" "$note"
        tail -10 "$b"
        rc=1
        KEEP=1
        continue
    fi
    ae="$WORK/out-ref/$name.err"
    be="$WORK/out-head/$name.err"
    n=$(diff <(grep -vE "$FILTER" "$a") <(grep -vE "$FILTER" "$b") | wc -l)
    ne=$(diff "$ae" "$be" | wc -l)
    if [ "$n" -eq 0 ] && [ "$ne" -eq 0 ]; then
        printf "  ok    %-9s %s\n" "$name" "$(timing "$a" "$b")"
    else
        printf "  DIFF  %-9s (stdout %d lines, stderr %d lines)  %s\n" \
               "$name" "$n" "$ne" "$(timing "$a" "$b")"
        diff <(grep -vE "$FILTER" "$a") <(grep -vE "$FILTER" "$b") | head -20
        [ "$ne" -eq 0 ] || diff "$ae" "$be" | head -20
        rc=1
        KEEP=1
    fi
done

[ -n "${TIMING:-}" ] || echo "  (wall times from concurrent runs: indicative only; TIMING=1 serialises them)"
if [ $rc -eq 0 ]; then
    echo "check-baseline: OK (identical to $REF_SHA)"
else
    echo "check-baseline: FAILED — the simulation moved or a workload did not complete" >&2
fi
exit $rc
