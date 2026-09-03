#!/bin/bash
#
# Run Contiki-NG Cooja test suite using csim
#
# Usage: ./tools/run-cooja-tests.sh [test-dir-pattern] [-v] [--no-build]
#                                   [--seed N] [--logdir DIR] [--with-tun] [--clean]
#
# Converts each .csc → JSON, runs csim mixed-multinode, reports PASS/FAIL/SKIP.
# Auto-builds missing firmware unless --no-build is passed.
#
# --seed N     run every test with random seed N instead of the .csc's own.
#              This is Cooja's --random-seed: Contiki-NG's
#              tests/Makefile.simulation-test loops BASESEED..BASESEED+RUNCOUNT
#              and passes one seed per run.  Same seed => byte-identical run.
# --logdir DIR write each test's full log to DIR/<category>/<csc-name>.testlog
#              — Contiki-NG's tests/ layout, so `--logdir contiki-ng/tests`
#              puts every log beside its .csc (their .gitignore covers
#              *.testlog).  Errored tests (conversion / firmware build) get a
#              log too, so a red test never leaves nothing behind.
#
# Exit code 0 only if every non-skipped test passed AND nothing errored.
#
# CONTIKI_DIR resolution: env variable → csim.conf → ../contiki-ng
#
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
CSIM_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
CSC2JSON="$SCRIPT_DIR/csc2json.py"
BUILD_FIRMWARE="$SCRIPT_DIR/build-test-firmware.sh"
TEST_RUNNER="$CSIM_DIR/build/test_runner"

# Parse arguments
TEST_PATTERN="*"
VERBOSE=""
AUTO_BUILD=1
WITH_TUN=0
CLEAN=0
SEED=""
LOGDIR=""

while [ $# -gt 0 ]; do
    case "$1" in
        --verbose|-v)
            VERBOSE="-v"
            ;;
        --no-build)
            AUTO_BUILD=0
            ;;
        --with-tun)
            WITH_TUN=1
            ;;
        --clean)
            CLEAN=1
            ;;
        --seed)
            SEED="$2"; shift
            ;;
        --seed=*)
            SEED="${1#--seed=}"
            ;;
        --logdir)
            LOGDIR="$2"; shift
            ;;
        --logdir=*)
            LOGDIR="${1#--logdir=}"
            ;;
        -h|--help)
            echo "Usage: $0 [test-dir-pattern] [-v] [--no-build] [--with-tun] [--clean]"
            echo "  test-dir-pattern: glob to filter test dirs (e.g. '07-*' or '14-rpl-lite')"
            echo "  -v, --verbose: show test output"
            echo "  --no-build: skip auto-building missing firmware"
            echo "  --with-tun: include border-router tests (requires sudo for TUN)"
            echo "  --clean: wipe firmware/<target>/* before running (forces full rebuild)"
            echo "  --seed N: run with random seed N instead of each .csc's own (Cooja --random-seed)"
            echo "  --logdir DIR: write DIR/<category>/<csc-name>.testlog per test (Contiki-NG tests/ layout)"
            echo ""
            echo "CONTIKI_DIR resolution: env variable -> csim.conf -> ../contiki-ng"
            exit 0
            ;;
        *)
            TEST_PATTERN="$1"
            ;;
    esac
    shift
done

# Fail loudly on bad option values rather than running something else.
if [ -n "$SEED" ]; then
    case "$SEED" in
        ''|*[!0-9]*) echo "Error: --seed expects a positive integer, got '$SEED'"; exit 2 ;;
    esac
    if [ "$SEED" -eq 0 ]; then echo "Error: --seed 0 is not a valid seed"; exit 2; fi
fi
if [ -n "$LOGDIR" ]; then
    mkdir -p "$LOGDIR" || { echo "Error: cannot create --logdir '$LOGDIR'"; exit 2; }
    LOGDIR="$(cd "$LOGDIR" && pwd)"
fi
SEED_ARGS=""
[ -n "$SEED" ] && SEED_ARGS="--seed $SEED"

# Run from the Cooja-NG tree regardless of the caller's cwd.  The generated
# JSON refers to firmware as firmware/<target>/<name> relative to this tree,
# and test_runner opens it relative to ITS cwd — so invoked from elsewhere
# (make -C contiki-ng/tests/<category> SIMULATOR=cooja-ng, a CI step, the
# unpacked release tarball) every test failed with "Cannot open ELF file".
# Every user-supplied path is already absolute by this point (CONTIKI_DIR,
# LOGDIR); TEST_PATTERN is relative to $CONTIKI_DIR/tests, not to the cwd.
cd "$CSIM_DIR" || exit 2

# Resolve CONTIKI_DIR: env -> csim.conf -> default
if [ -z "$CONTIKI_DIR" ]; then
    if [ -f "$CSIM_DIR/csim.conf" ]; then
        CONTIKI_DIR=$(grep -s '^CONTIKI_DIR=' "$CSIM_DIR/csim.conf" | cut -d= -f2-)
    fi
fi
if [ -z "$CONTIKI_DIR" ]; then
    CONTIKI_DIR="$CSIM_DIR/../contiki-ng"
fi

if [ ! -d "$CONTIKI_DIR" ]; then
    echo "Error: Contiki-NG directory not found: $CONTIKI_DIR"
    echo "Set CONTIKI_DIR via environment, csim.conf, or make configure"
    echo "  Example: make configure CONTIKI_DIR=/path/to/contiki-ng"
    exit 1
fi

CONTIKI_DIR="$(cd "$CONTIKI_DIR" && pwd)"

# Firmware target for the Cooja suite: cooja motes, always, unless the caller
# asks for cc2538dk explicitly (FIRMWARE_TARGET=cc2538dk).  The old auto-detect
# ("cooja if any .cooja exists, else cc2538dk if any .cc2538dk exists") picked
# cc2538dk on every fresh checkout — firmware/cooja is untracked and empty while
# firmware/cc2538dk ships tracked ARM-suite binaries — and then tried to
# cross-build every scenario for cc2538dk, turning 12 tests into ERRORs.
FIRMWARE_TARGET="${FIRMWARE_TARGET:-cooja}"
case "$FIRMWARE_TARGET" in
    cooja|cc2538dk) ;;
    *) echo "Error: FIRMWARE_TARGET must be cooja or cc2538dk (got '$FIRMWARE_TARGET')"; exit 1 ;;
esac
FIRMWARE_DIR="$CSIM_DIR/firmware/$FIRMWARE_TARGET"

if [ "$CLEAN" -eq 1 ]; then
    # Wipe every Cooja-suite firmware target so nothing is reused across the
    # rebuild.  cc2538dk is left alone — it belongs to the standalone ARM
    # test_runner suite, not to the Cooja suite.
    for sub in cooja sky z1; do
        d="$CSIM_DIR/firmware/$sub"
        [ -d "$d" ] || continue
        wiped=$(find "$d" -maxdepth 1 -type f -name "*.$sub" | wc -l | tr -d ' ')
        find "$d" -maxdepth 1 -type f -name "*.$sub" -delete
        echo "  CLEAN $d (removed $wiped firmware artifacts)"
    done
fi

if [ ! -f "$TEST_RUNNER" ]; then
    echo "Error: test_runner not found at $TEST_RUNNER (run 'make' first)"
    exit 1
fi

echo "=== Cooja Test Suite (csim) ==="
echo "  Contiki-NG: $CONTIKI_DIR"
echo "  Test pattern: $TEST_PATTERN"
[ -n "$SEED" ]   && echo "  Seed: $SEED (overrides each .csc's randomseed)"
[ -n "$LOGDIR" ] && echo "  Log dir: $LOGDIR (<category>/<csc-name>.testlog per test)"
echo "  Firmware target: $FIRMWARE_TARGET"
echo "  Firmware dir: $FIRMWARE_DIR"
echo ""

# Skip patterns: border-router tests require TUN (sudo)
if [ "$WITH_TUN" -eq 0 ]; then
    SKIP_PATTERNS="border-router|tun-rpl-br"
else
    SKIP_PATTERNS=""
fi

TMP_DIR=$(mktemp -d "${TMPDIR:-/tmp}/cooja-tests.XXXXXX")
trap "rm -rf $TMP_DIR" EXIT

passed=0
failed=0
skipped=0
errors=0
total=0

FAILED_TESTS=""
SKIPPED_TESTS=""
ERRORED_TESTS=""
failed_count=0
skipped_count=0

# Track tests needing firmware rebuild
NEED_REBUILD_FILE="$TMP_DIR/need_rebuild.txt"
touch "$NEED_REBUILD_FILE"

# Persist a test's log under --logdir as <category>/<csc-name>.testlog — the
# same layout as Contiki-NG's tests/ tree, so `--logdir contiki-ng/tests` puts
# every log beside its .csc.  Mirroring the category is not cosmetic: the same
# .csc basename exists in several categories (07-rpl-random-rearrangement in
# both 14-rpl-lite and 15-rpl-classic), and a flat directory silently lost 13
# of 85 logs on a full run.  No-op without --logdir.
save_testlog() {
    local test_name="$1" src="$2"
    [ -n "$LOGDIR" ] || return 0
    local cat="$(dirname "$test_name")"
    mkdir -p "$LOGDIR/$cat"
    cp "$src" "$LOGDIR/$cat/$(basename "$test_name").testlog"
}

run_test() {
    local test_name="$1"
    local json_file="$2"

    log_file="$TMP_DIR/$(echo "$test_name" | tr '/' '_').log"
    printf "  RUN   %-60s" "$test_name"

    # Per-test wall-clock timeout (5 minutes default, longer for some tests)
    local wall_timeout=300
    if grep -q serial_socket "$json_file" 2>/dev/null; then
        wall_timeout=600
    fi
    # TSCH drift test simulates 600s at ~2x speed, needs longer wall time
    if echo "$test_name" | grep -q "tsch-drift"; then
        wall_timeout=600
    fi

    start_time=$(date +%s)
    if timeout "$wall_timeout" "$TEST_RUNNER" mixed-multinode "$json_file" $VERBOSE $SEED_ARGS > "$log_file" 2>&1; then
        exit_code=0
    else
        exit_code=$?
    fi
    end_time=$(date +%s)
    elapsed=$((end_time - start_time))
    save_testlog "$test_name" "$log_file"

    if [ $exit_code -eq 0 ]; then
        echo "PASS (${elapsed}s)"
        passed=$((passed + 1))
    else
        echo "FAIL (${elapsed}s)"
        FAILED_TESTS="$FAILED_TESTS
  - $test_name"
        failed=$((failed + 1))
        failed_count=$((failed_count + 1))
        if [ -z "$VERBOSE" ]; then
            echo "    --- Last 5 lines ---"
            tail -5 "$log_file" | sed 's/^/    /'
        fi
    fi
}

# Support both directory patterns (17-tun-rpl-br) and specific tests
# (17-tun-rpl-br/03-border-router-sky)
if [ -f "$CONTIKI_DIR/tests/$TEST_PATTERN.csc" ]; then
    csc_files="$CONTIKI_DIR/tests/$TEST_PATTERN.csc"
else
    csc_files="$CONTIKI_DIR/tests/$TEST_PATTERN/*.csc"
fi
for csc_file in $csc_files; do
    [ -f "$csc_file" ] || continue
    total=$((total + 1))

    test_name="$(basename "$(dirname "$csc_file")")/$(basename "$csc_file" .csc)"

    # Skip known-unsupported tests
    if [ -n "$SKIP_PATTERNS" ] && echo "$test_name" | grep -qiE "$SKIP_PATTERNS"; then
        echo "  SKIP  $test_name (filtered)"
        SKIPPED_TESTS="$SKIPPED_TESTS
  - $test_name (filtered)"
        skipped=$((skipped + 1))
        skipped_count=$((skipped_count + 1))
        continue
    fi

    # Convert .csc to JSON with native JS execution
    json_file="$TMP_DIR/$(echo "$test_name" | tr '/' '_').json"
    conv_log="$TMP_DIR/$(echo "$test_name" | tr '/' '_').convert.log"
    if ! python3 "$CSC2JSON" "$csc_file" --contiki "$CONTIKI_DIR" --firmware-dir "firmware/$FIRMWARE_TARGET" --js-native -o "$json_file" 2>"$conv_log"; then
        echo "  ERROR $test_name (conversion failed)"
        sed 's/^/        /' "$conv_log"
        save_testlog "$test_name" "$conv_log"
        ERRORED_TESTS="$ERRORED_TESTS
  - $test_name (conversion failed)"
        errors=$((errors + 1))
        continue
    fi

    # Check if test has any meaningful test criteria
    has_test=$(python3 -c "
import json, sys
with open('$json_file') as f:
    d = json.load(f)
t = d.get('test', {})
has_steps = len(t.get('steps', [])) > 0
has_fail_on = len(t.get('fail_on', [])) > 0
has_tis = t.get('timeout_is_success', False)
has_js = 'js_script_inline' in t
has_ss = 'serial_socket' in d
if has_steps or has_fail_on or has_tis or has_js or has_ss:
    print('yes')
" 2>/dev/null || echo inspect-failed)

    if [ "$has_test" != "yes" ]; then
        # A test with no assertions must not pass by simply existing, and a
        # crash in the inspection itself must not demote to SKIP.
        echo "  ERROR $test_name (no test criteria)"
        ERRORED_TESTS="$ERRORED_TESTS
  - $test_name (no test criteria)"
        errors=$((errors + 1))
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
        if [ $AUTO_BUILD -eq 1 ]; then
            echo "  NEED  $test_name (missing: $(basename "$missing_fw"))"
            echo "$test_name" >> "$NEED_REBUILD_FILE"
            # Save JSON file path for --from-json build
            echo "$json_file" >> "$NEED_REBUILD_FILE.jsons"
        else
            echo "  SKIP  $test_name (missing: $(basename "$missing_fw"))"
            SKIPPED_TESTS="$SKIPPED_TESTS
  - $test_name (missing firmware)"
            skipped=$((skipped + 1))
            skipped_count=$((skipped_count + 1))
        fi
        continue
    fi

    run_test "$test_name" "$json_file"
done

# Auto-build missing firmware and re-run those tests
need_rebuild_count=$(wc -l < "$NEED_REBUILD_FILE" | tr -d ' ')
if [ "$need_rebuild_count" -gt 0 ]; then
    echo ""
    echo "=== Auto-building missing firmware ==="
    mkdir -p "$FIRMWARE_DIR"

    # Use --from-json to get per-firmware build info (target, board, make_args)
    json_list=$(cat "$NEED_REBUILD_FILE.jsons" 2>/dev/null | sort -u)
    CONTIKI_DIR="$CONTIKI_DIR" "$BUILD_FIRMWARE" --from-json $json_list
    rm -f "$NEED_REBUILD_FILE.jsons"

    echo ""
    echo "=== Re-running tests that needed firmware ==="

    while IFS= read -r test_name; do
        [ -n "$test_name" ] || continue

        csc_base="$(echo "$test_name" | cut -d/ -f2)"
        test_suite="$(echo "$test_name" | cut -d/ -f1)"
        csc_file="$CONTIKI_DIR/tests/$test_suite/$csc_base.csc"

        [ -f "$csc_file" ] || continue

        json_file="$TMP_DIR/$(echo "$test_name" | tr '/' '_').json"
        conv_log="$TMP_DIR/$(echo "$test_name" | tr '/' '_').convert.log"
        if ! python3 "$CSC2JSON" "$csc_file" --contiki "$CONTIKI_DIR" --firmware-dir "firmware/$FIRMWARE_TARGET" --js-native -o "$json_file" 2>"$conv_log"; then
            echo "  ERROR $test_name (conversion failed)"
            sed 's/^/        /' "$conv_log"
        save_testlog "$test_name" "$conv_log"
            ERRORED_TESTS="$ERRORED_TESTS
  - $test_name (conversion failed)"
            errors=$((errors + 1))
            continue
        fi

        # Check firmware exists now
        missing_fw=""
        for fw_path in $(python3 -c "
import json
with open('$json_file') as f:
    d = json.load(f)
for n in d.get('nodes', []):
    print(n['firmware'])
" 2>/dev/null | sort -u); do
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
            # The rebuild ran and the firmware is STILL missing: the build
            # failed. That is an error, not a skip — a broken firmware build
            # must never turn into a green suite.
            echo "  ERROR $test_name (firmware build failed: $(basename "$missing_fw"))"
            ERRORED_TESTS="$ERRORED_TESTS
  - $test_name (firmware build failed)"
            errors=$((errors + 1))
            continue
        fi

        run_test "$test_name" "$json_file"
    done < "$NEED_REBUILD_FILE"
fi

# FAIL-LOUDLY: a pattern that matched no .csc at all is a mistake (typo,
# wrong CONTIKI_DIR, renamed test) — not a passing run of zero tests.
if [ "$total" -eq 0 ]; then
    echo ""
    echo "Error: no .csc files matched '$TEST_PATTERN' under $CONTIKI_DIR/tests"
    exit 2
fi

echo ""
echo "=== Results ==="
echo "  Total: $total"
echo "  Passed: $passed"
echo "  Failed: $failed"
echo "  Skipped: $skipped"
echo "  Errors: $errors"

if [ $failed_count -gt 0 ]; then
    echo ""
    echo "Failed tests:$FAILED_TESTS"
fi

if [ $skipped_count -gt 0 ] && [ -n "$VERBOSE" ]; then
    echo ""
    echo "Skipped tests:$SKIPPED_TESTS"
fi

if [ $errors -gt 0 ]; then
    echo ""
    echo "Errored tests:$ERRORED_TESTS"
fi

# FAIL-LOUDLY: failures AND errors fail the suite.  An error means a test
# could not even be attempted (conversion, firmware build, or a test with
# nothing to assert) — none of which may produce a green result.  The only
# accepted skips are the explicit opt-outs: TUN tests without --with-tun,
# and missing firmware under --no-build.
if [ $failed -gt 0 ] || [ $errors -gt 0 ]; then
    exit 1
fi
exit 0
