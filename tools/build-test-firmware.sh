#!/bin/bash
#
# Build Contiki-NG firmware needed for Cooja test suite
#
# Usage: ./tools/build-test-firmware.sh <contiki-ng-root> [test-dir-pattern]
#
# Scans .csc files for source references, builds each as TARGET=cc2538dk.
# Firmware is placed in firmware/cc2538dk/.
#
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
CSIM_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
CSC2JSON="$SCRIPT_DIR/csc2json.py"

if [ -z "$1" ]; then
    echo "Usage: $0 <contiki-ng-root> [test-dir-pattern]"
    echo "  test-dir-pattern: glob to filter test dirs (e.g. '07-*' or '14-rpl-lite')"
    exit 1
fi

CONTIKI_DIR="$(cd "$1" && pwd)"
TEST_PATTERN="${2:-*}"
FIRMWARE_DIR="$CSIM_DIR/firmware/cc2538dk"

mkdir -p "$FIRMWARE_DIR"

echo "=== Build Test Firmware ==="
echo "  Contiki-NG: $CONTIKI_DIR"
echo "  Test pattern: $TEST_PATTERN"
echo "  Output: $FIRMWARE_DIR"
echo ""

# Collect all needed firmware from all .csc files
declare -A FIRMWARE_SOURCES  # firmware_name -> source_dir

for csc_file in "$CONTIKI_DIR"/tests/$TEST_PATTERN/*.csc; do
    [ -f "$csc_file" ] || continue
    test_dir="$(dirname "$csc_file")"

    # Get firmware names needed
    firmware_list=$(python3 "$CSC2JSON" --list-firmware "$csc_file" 2>/dev/null || true)
    for fw in $firmware_list; do
        if [ -z "${FIRMWARE_SOURCES[$fw]+x}" ]; then
            # Find the source file
            code_dir="$test_dir/code"
            if [ -f "$code_dir/$fw.c" ]; then
                FIRMWARE_SOURCES[$fw]="$code_dir"
            else
                # Try looking in parent test dir
                for d in "$test_dir" "$test_dir/.."; do
                    if [ -f "$d/code/$fw.c" ]; then
                        FIRMWARE_SOURCES[$fw]="$d/code"
                        break
                    fi
                done
            fi
        fi
    done
done

echo "Firmware to build: ${#FIRMWARE_SOURCES[@]}"
echo ""

built=0
skipped=0
failed=0

for fw in $(echo "${!FIRMWARE_SOURCES[@]}" | tr ' ' '\n' | sort); do
    src_dir="${FIRMWARE_SOURCES[$fw]}"
    target="$FIRMWARE_DIR/$fw.cc2538dk"

    if [ -f "$target" ]; then
        echo "  SKIP $fw (already exists)"
        skipped=$((skipped + 1))
        continue
    fi

    if [ -z "$src_dir" ] || [ ! -f "$src_dir/$fw.c" ]; then
        echo "  MISS $fw (source not found)"
        failed=$((failed + 1))
        continue
    fi

    echo "  BUILD $fw from $src_dir"
    (
        cd "$src_dir"
        make TARGET=cc2538dk clean >/dev/null 2>&1 || true
        if make -j"$(nproc 2>/dev/null || sysctl -n hw.ncpu)" TARGET=cc2538dk "$fw.cc2538dk" \
            WERROR=0 CONTIKI="$CONTIKI_DIR" >/dev/null 2>&1; then
            cp "$fw.cc2538dk" "$target"
            echo "    -> $target"
            make TARGET=cc2538dk clean >/dev/null 2>&1 || true
        else
            echo "    FAILED to build $fw"
        fi
    )

    if [ -f "$target" ]; then
        built=$((built + 1))
    else
        failed=$((failed + 1))
    fi
done

echo ""
echo "=== Summary: $built built, $skipped skipped, $failed failed ==="
