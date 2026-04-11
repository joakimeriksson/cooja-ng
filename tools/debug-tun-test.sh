#!/bin/bash
#
# Run one or more TUN/serial-socket Cooja tests directly and keep artifacts.
# This avoids run-cooja-tests.sh deleting the per-test logs on exit.
#
# Examples:
#   tools/debug-tun-test.sh 17-tun-rpl-br/02-border-router-cooja-tsch
#   tools/debug-tun-test.sh 17-tun-rpl-br/04-border-router-traceroute
#   tools/debug-tun-test.sh
#
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
CSIM_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
CONTIKI_DIR="${CONTIKI_DIR:-$CSIM_DIR/../contiki-ng}"
TEST_RUNNER="$CSIM_DIR/build/test_runner"
CSC2JSON="$SCRIPT_DIR/csc2json.py"
FIRMWARE_DIR="$CSIM_DIR/firmware/cooja"

if [ ! -x "$TEST_RUNNER" ]; then
    echo "Error: missing $TEST_RUNNER" >&2
    echo "Build it first with: make build/test_runner" >&2
    exit 1
fi

DEFAULT_TESTS=(
    "17-tun-rpl-br/02-border-router-cooja-tsch"
    "17-tun-rpl-br/04-border-router-traceroute"
)

if [ "$#" -eq 0 ]; then
    TESTS=("${DEFAULT_TESTS[@]}")
else
    TESTS=("$@")
fi

echo "=== TUN Test Debug Runner ==="
echo "  Contiki-NG: $CONTIKI_DIR"
echo "  Firmware dir: $FIRMWARE_DIR"
echo "  Output dir: /tmp"
echo ""

failures=0

for test_name in "${TESTS[@]}"; do
    csc_file="$CONTIKI_DIR/tests/$test_name.csc"
    if [ ! -f "$csc_file" ]; then
        echo "ERROR  $test_name (missing $csc_file)"
        failures=$((failures + 1))
        continue
    fi

    base="$(basename "$test_name")"
    json_file="/tmp/${base}.json"
    log_file="/tmp/${base}.log"

    echo "=== $test_name ==="
    echo "Generating: $json_file"
    python3 "$CSC2JSON" "$csc_file" \
        --contiki "$CONTIKI_DIR" \
        --firmware-dir "$FIRMWARE_DIR" \
        --js-native \
        -o "$json_file"

    echo "Running: $log_file"
    if sudo -E "$TEST_RUNNER" mixed-multinode "$json_file" -v > "$log_file" 2>&1; then
        echo "PASS  $test_name"
    else
        echo "FAIL  $test_name"
        failures=$((failures + 1))
    fi

    echo "Artifacts:"
    echo "  JSON: $json_file"
    echo "  LOG:  $log_file"
    echo "Tail:"
    tail -n 20 "$log_file" || true
    echo ""
done

if [ "$failures" -ne 0 ]; then
    echo "Completed with failures: $failures"
    exit 1
fi

echo "All requested tests passed."
