#!/bin/bash
#
# Malformed-firmware check.  Builds malformed MSP430 ELF images with
# tools/mk-malformed-elf.py and boots each one in its own process, so a
# regression is reported as a failing case instead of taking the suite down
# with it.
#
# Every image must run to completion: exit below 128 (no signal) and no
# sanitizer report.  An image whose expectation is "patch-skipped" must also
# say that a symbol lay outside memory -- proof that the bound, not luck,
# kept the write out.
#
# The stock build only catches wild writes that land on unmapped memory; a
# straddling write needs a sanitizer build to be seen.  Point RUNNER at one
# for the full check.
#
# Usage: tools/check-elf-malformed.sh        (RUNNER=... to override)
#
set -u
cd "$(dirname "$0")/.."
RUNNER=${RUNNER:-./build/test_runner}
TMP=${TMPDIR:-/tmp}/csim-elf-malformed.$$
mkdir -p "$TMP"
trap 'rm -rf "$TMP"' EXIT
export ASAN_OPTIONS=${ASAN_OPTIONS:-detect_leaks=0}

python3 tools/mk-malformed-elf.py "$TMP" > "$TMP/manifest" ||
    { echo "check-elf-malformed: FAIL: generator"; exit 1; }

failed=0
total=0
while read -r name expect; do
    total=$((total + 1))
    log="$TMP/$name.log"
    "$RUNNER" mixed-multinode "$TMP/$name" -t 10 > "$log" 2>&1
    rc=$?
    why=""
    if [ "$rc" -ge 128 ]; then
        why="killed by signal $((rc - 128))"
    elif grep -qE 'ERROR: AddressSanitizer|runtime error:' "$log"; then
        why="sanitizer report: $(grep -m1 -E 'ERROR: AddressSanitizer|runtime error:' "$log")"
    elif [ "$expect" = patch-skipped ] &&
         ! grep -q 'lies outside memory, not patched' "$log"; then
        why="no 'not patched' warning"
    fi
    if [ -n "$why" ]; then
        echo "  FAIL $name: $why (exit $rc)"
        failed=$((failed + 1))
    else
        echo "  ok   $name (exit $rc)"
    fi
done < "$TMP/manifest"

if [ "$failed" -ne 0 ]; then
    echo "check-elf-malformed: FAIL: $failed of $total"
    exit 1
fi
echo "check-elf-malformed: OK ($total images)"
