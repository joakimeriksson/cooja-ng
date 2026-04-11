#!/bin/bash
#
# Compare wall-clock timing for the same Cooja .csc test in:
#   1. csim (via build/test_runner + generated JSON)
#   2. upstream Cooja (via ./gradlew run --args='--no-gui ...')
#
# Keeps all artifacts under /tmp for inspection.
#
# Examples:
#   tools/compare-cooja-timing.sh 07-simulation-base/23-rpl-tsch-z1
#   tools/compare-cooja-timing.sh 17-tun-rpl-br/02-border-router-cooja-tsch
#   tools/compare-cooja-timing.sh \
#     17-tun-rpl-br/02-border-router-cooja-tsch \
#     17-tun-rpl-br/04-border-router-traceroute
#
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
CSIM_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
CONTIKI_DIR="${CONTIKI_DIR:-$CSIM_DIR/../contiki-ng}"
COOJA_DIR="$CONTIKI_DIR/tools/cooja"
CSC2JSON="$SCRIPT_DIR/csc2json.py"
TEST_RUNNER="$CSIM_DIR/build/test_runner"
FIRMWARE_DIR="$CSIM_DIR/firmware/cooja"

if [ ! -x "$TEST_RUNNER" ]; then
    echo "Error: missing $TEST_RUNNER" >&2
    echo "Build it first with: make build/test_runner" >&2
    exit 1
fi

if [ ! -d "$COOJA_DIR" ]; then
    echo "Error: missing Cooja dir $COOJA_DIR" >&2
    exit 1
fi

if [ "$#" -eq 0 ]; then
    echo "Usage: $0 test/path [test/path ...]" >&2
    exit 1
fi

echo "=== csim vs Cooja Timing ==="
echo "  Contiki-NG: $CONTIKI_DIR"
echo "  csim:       $CSIM_DIR"
echo "  Cooja:      $COOJA_DIR"
echo "  Artifacts:  /tmp"
echo ""

run_with_time() {
    local time_file="$1"
    shift
    /usr/bin/time -p -o "$time_file" "$@"
}

extract_real() {
    local time_file="$1"
    awk '/^real / { print $2; exit }' "$time_file"
}

needs_sudo_for_test() {
    local test_name="$1"
    case "$test_name" in
        17-tun-rpl-br/*) return 0 ;;
        *) return 1 ;;
    esac
}

failures=0

for test_name in "$@"; do
    csc_file="$CONTIKI_DIR/tests/$test_name.csc"
    if [ ! -f "$csc_file" ]; then
        echo "ERROR  $test_name (missing $csc_file)"
        failures=$((failures + 1))
        continue
    fi

    base="$(basename "$test_name")"
    json_file="/tmp/${base}.compare.json"
    csim_log="/tmp/${base}.csim.log"
    csim_time="/tmp/${base}.csim.time"
    cooja_log="/tmp/${base}.cooja.log"
    cooja_time="/tmp/${base}.cooja.time"

    echo "=== $test_name ==="
    python3 "$CSC2JSON" "$csc_file" \
        --contiki "$CONTIKI_DIR" \
        --firmware-dir "$FIRMWARE_DIR" \
        --js-native \
        -o "$json_file" >/dev/null

    if needs_sudo_for_test "$test_name"; then
        echo "Running csim with sudo..."
        if run_with_time "$csim_time" sudo -E "$TEST_RUNNER" mixed-multinode "$json_file" \
            >"$csim_log" 2>&1; then
            csim_status="PASS"
        else
            csim_status="FAIL"
            failures=$((failures + 1))
        fi
    else
        echo "Running csim..."
        if run_with_time "$csim_time" "$TEST_RUNNER" mixed-multinode "$json_file" \
            >"$csim_log" 2>&1; then
            csim_status="PASS"
        else
            csim_status="FAIL"
            failures=$((failures + 1))
        fi
    fi

    if needs_sudo_for_test "$test_name"; then
        echo "Running upstream Cooja with sudo..."
        if (
            cd "$COOJA_DIR"
            run_with_time "$cooja_time" sudo -E ./gradlew run --args="--no-gui ../../tests/$test_name.csc" \
                >"$cooja_log" 2>&1
        ); then
            cooja_status="PASS"
        else
            cooja_status="FAIL"
            failures=$((failures + 1))
        fi
    else
        echo "Running upstream Cooja..."
        if (
            cd "$COOJA_DIR"
            run_with_time "$cooja_time" ./gradlew run --args="--no-gui ../../tests/$test_name.csc" \
                >"$cooja_log" 2>&1
        ); then
            cooja_status="PASS"
        else
            cooja_status="FAIL"
            failures=$((failures + 1))
        fi
    fi

    csim_real="$(extract_real "$csim_time" 2>/dev/null || echo n/a)"
    cooja_real="$(extract_real "$cooja_time" 2>/dev/null || echo n/a)"

    echo "Results:"
    echo "  csim:  $csim_status  real=${csim_real}s  log=$csim_log"
    echo "  Cooja: $cooja_status  real=${cooja_real}s  log=$cooja_log"
    echo "  JSON:  $json_file"
    echo ""
done

if [ "$failures" -ne 0 ]; then
    echo "Completed with failures: $failures"
    exit 1
fi

echo "All requested comparisons completed successfully."
