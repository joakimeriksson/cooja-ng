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
#
# Exit code 0 = identical, 1 = a workload differs (or would not run).
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

# Wall-clock lines legitimately differ between two builds of the same code.
# Everything else — simulated timestamps, packet counts, cycle totals, exit
# codes — must match exactly.
FILTER='^ *(Wall-clock time|Speed ratio|Throughput):'

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

run_all() {  # run_all <binary> <outdir>
    mkdir -p "$2"
    for w in "${WORKLOADS[@]}"; do
        ( timeout 900 "$1" ${w#*|} >"$2/${w%%|*}.log" 2>&1; echo "rc=$?" >>"$2/${w%%|*}.log" ) &
    done
    wait
}

echo "=== baseline: this tree vs $REF_SHA${REF_NAME:+ ($REF_NAME)}"
git worktree add --detach "$WORK/ref" "$REF" >/dev/null 2>&1 || {
    echo "cannot create a worktree at $REF" >&2; exit 1; }
build "$WORK/ref" ref || exit 1
build "$ROOT" head || exit 1

# Both binaries run from THIS tree, so configs and firmware are identical and
# only the engine differs.  (A config that changed since the reference may not
# load in the older binary — that shows up as a diff in its log.)
echo "  running ${#WORKLOADS[@]} workloads with each binary ..."
run_all "$WORK/ref/build/test_runner" "$WORK/out-ref"
run_all "$ROOT/build/test_runner"     "$WORK/out-head"

rc=0
for w in "${WORKLOADS[@]}"; do
    name=${w%%|*}
    a="$WORK/out-ref/$name.log"
    b="$WORK/out-head/$name.log"
    n=$(diff <(grep -vE "$FILTER" "$a") <(grep -vE "$FILTER" "$b") | wc -l)
    if [ "$n" -eq 0 ]; then
        echo "  ok    $name"
    else
        echo "  DIFF  $name ($n lines)"
        diff <(grep -vE "$FILTER" "$a") <(grep -vE "$FILTER" "$b") | head -20
        rc=1
        KEEP=1
    fi
done

if [ $rc -eq 0 ]; then
    echo "check-baseline: OK (identical to $REF_SHA)"
else
    echo "check-baseline: FAILED — the simulation moved" >&2
fi
exit $rc
