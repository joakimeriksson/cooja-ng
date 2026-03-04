#!/bin/bash
#
# Run Contiki-NG Cooja test suite using csim
#
# Usage: ./tools/run-cooja-tests.sh <contiki-ng-root> [test-dir-pattern] [options]
#
# Converts each .csc → JSON, runs csim mixed-multinode, reports PASS/FAIL/SKIP.
# Exit code 0 if all non-skipped tests pass.
#
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
CSIM_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
CSC2JSON="$SCRIPT_DIR/csc2json.py"
TEST_RUNNER="$CSIM_DIR/build/test_runner"
FIRMWARE_DIR="$CSIM_DIR/firmware/cc2538dk"

if [ -z "$1" ]; then
    echo "Usage: $0 <contiki-ng-root> [test-dir-pattern] [--verbose]"
    echo "  test-dir-pattern: glob to filter test dirs (e.g. '07-*' or '14-rpl-lite')"
    echo "  --verbose: show test output"
    exit 1
fi

CONTIKI_DIR="$(cd "$1" && pwd)"
shift
TEST_PATTERN="*"
VERBOSE=""

while [ $# -gt 0 ]; do
    case "$1" in
        --verbose|-v)
            VERBOSE="-v"
            ;;
        *)
            TEST_PATTERN="$1"
            ;;
    esac
    shift
done

if [ ! -f "$TEST_RUNNER" ]; then
    echo "Error: test_runner not found at $TEST_RUNNER (run 'make' first)"
    exit 1
fi

echo "=== Cooja Test Suite (csim) ==="
echo "  Contiki-NG: $CONTIKI_DIR"
echo "  Test pattern: $TEST_PATTERN"
echo "  Firmware: $FIRMWARE_DIR"
echo ""

# Skip patterns: TSCH tests, border-router, node-reboot tests
SKIP_PATTERNS="tsch|border-router|br-|reboot|TSCH"

TMP_DIR=$(mktemp -d)
trap "rm -rf $TMP_DIR" EXIT

passed=0
failed=0
skipped=0
errors=0
total=0

declare -a FAILED_TESTS
declare -a SKIPPED_TESTS

for csc_file in "$CONTIKI_DIR"/tests/$TEST_PATTERN/*.csc; do
    [ -f "$csc_file" ] || continue
    total=$((total + 1))

    test_name="$(basename "$(dirname "$csc_file")")/$(basename "$csc_file" .csc)"

    # Skip TSCH and known-unsupported tests
    if echo "$test_name" | grep -qiE "$SKIP_PATTERNS"; then
        echo "  SKIP  $test_name (filtered)"
        SKIPPED_TESTS+=("$test_name (filtered)")
        skipped=$((skipped + 1))
        continue
    fi

    # Convert .csc to JSON (use relative firmware paths for portability)
    json_file="$TMP_DIR/$(echo "$test_name" | tr '/' '_').json"
    if ! python3 "$CSC2JSON" "$csc_file" --firmware-dir "firmware/cc2538dk" -o "$json_file" 2>/dev/null; then
        echo "  ERROR $test_name (conversion failed)"
        errors=$((errors + 1))
        continue
    fi

    # Note unsupported features (advisory, still try to run)
    unsupported=$(python3 -c "
import json, sys
with open('$json_file') as f:
    d = json.load(f)
t = d.get('test', {})
u = t.get('unsupported_features', [])
if u:
    print(','.join(u))
" 2>/dev/null || true)

    # Check if test has any meaningful test criteria
    has_test=$(python3 -c "
import json, sys
with open('$json_file') as f:
    d = json.load(f)
t = d.get('test', {})
has_steps = len(t.get('steps', [])) > 0
has_fail_on = len(t.get('fail_on', [])) > 0
has_tis = t.get('timeout_is_success', False)
if has_steps or has_fail_on or has_tis:
    print('yes')
" 2>/dev/null || true)

    if [ "$has_test" != "yes" ]; then
        echo "  SKIP  $test_name (no test criteria)"
        SKIPPED_TESTS+=("$test_name (no test criteria)")
        skipped=$((skipped + 1))
        continue
    fi

    # Check firmware exists
    missing_fw=""
    for fw_path in $(python3 -c "
import json
with open('$json_file') as f:
    d = json.load(f)
for n in d.get('nodes', []):
    print(n['firmware'])
" 2>/dev/null | sort -u); do
        # Handle both absolute and relative firmware paths
        if [ "${fw_path:0:1}" = "/" ]; then
            check_path="$fw_path"
        else
            check_path="$CSIM_DIR/$fw_path"
        fi
        if [ ! -f "$check_path" ]; then
            missing_fw="$fw_path"
            break
        fi
    done

    if [ -n "$missing_fw" ]; then
        echo "  SKIP  $test_name (missing: $(basename "$missing_fw"))"
        SKIPPED_TESTS+=("$test_name (missing firmware)")
        skipped=$((skipped + 1))
        continue
    fi

    # Run the test
    log_file="$TMP_DIR/$(echo "$test_name" | tr '/' '_').log"
    printf "  RUN   %-60s" "$test_name"

    start_time=$(date +%s)
    if "$TEST_RUNNER" mixed-multinode "$json_file" $VERBOSE > "$log_file" 2>&1; then
        exit_code=0
    else
        exit_code=$?
    fi
    end_time=$(date +%s)
    elapsed=$((end_time - start_time))

    if [ $exit_code -eq 0 ]; then
        echo "PASS (${elapsed}s)"
        passed=$((passed + 1))
    else
        echo "FAIL (${elapsed}s)"
        FAILED_TESTS+=("$test_name")
        failed=$((failed + 1))
        # Show last few lines of output on failure
        if [ -z "$VERBOSE" ]; then
            echo "    --- Last 5 lines ---"
            tail -5 "$log_file" | sed 's/^/    /'
        fi
    fi
done

echo ""
echo "=== Results ==="
echo "  Total: $total"
echo "  Passed: $passed"
echo "  Failed: $failed"
echo "  Skipped: $skipped"
echo "  Errors: $errors"

if [ ${#FAILED_TESTS[@]} -gt 0 ]; then
    echo ""
    echo "Failed tests:"
    for t in "${FAILED_TESTS[@]}"; do
        echo "  - $t"
    done
fi

if [ ${#SKIPPED_TESTS[@]} -gt 0 ] && [ -n "$VERBOSE" ]; then
    echo ""
    echo "Skipped tests:"
    for t in "${SKIPPED_TESTS[@]}"; do
        echo "  - $t"
    done
fi

# Exit with failure if any test failed
if [ $failed -gt 0 ]; then
    exit 1
fi
exit 0
