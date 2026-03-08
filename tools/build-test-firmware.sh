#!/bin/bash
#
# Build Contiki-NG firmware needed for Cooja test suite
#
# Usage:
#   ./tools/build-test-firmware.sh [--target <cooja|cc2538dk>] [test-dir-pattern]
#   ./tools/build-test-firmware.sh --from-json <config.json> [config2.json ...]
#
# Mode 1 (default): Scans .csc files for source references, builds each firmware.
# Mode 2 (--from-json): Reads build info from JSON configs (target, board, make_args).
#
# CONTIKI_DIR resolution: env variable -> csim.conf -> ../contiki-ng
#
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
CSIM_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
CSC2JSON="$SCRIPT_DIR/csc2json.py"

# Resolve CONTIKI_DIR: env -> csim.conf -> default
resolve_contiki_dir() {
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
        exit 1
    fi
    CONTIKI_DIR="$(cd "$CONTIKI_DIR" && pwd)"
}

# Build a single firmware
# Args: fw_name src_dir target_ext output_file extra_make_args...
build_one() {
    local fw="$1"
    local src_dir="$2"
    local target="$3"
    local target_file="$4"
    shift 4
    local extra_args="$@"

    if [ -f "$target_file" ]; then
        echo "  SKIP $fw (already exists)"
        echo "skip" >> "$RESULTS_FILE"
        return
    fi

    if [ -z "$src_dir" ] || [ ! -f "$src_dir/$fw.c" ]; then
        echo "  MISS $fw (source not found: $src_dir)"
        echo "fail" >> "$RESULTS_FILE"
        return
    fi

    local extra_info=""
    if [ -n "$extra_args" ]; then
        extra_info=" $extra_args"
    fi
    echo "  BUILD $fw (TARGET=$target$extra_info)"

    (
        cd "$src_dir"
        make TARGET="$target" clean >/dev/null 2>&1 || true
        if make -j"$(nproc 2>/dev/null || sysctl -n hw.ncpu)" TARGET="$target" "$fw.$target" \
            WERROR=0 CONTIKI="$CONTIKI_DIR" $extra_args >/dev/null 2>&1; then
            # Contiki-NG may place output in build/<target>/ or current dir
            if [ -f "$fw.$target" ]; then
                cp "$fw.$target" "$target_file"
            elif [ -f "build/$target/$fw.$target" ]; then
                cp "build/$target/$fw.$target" "$target_file"
            fi
            if [ -f "$target_file" ]; then
                echo "    -> $target_file"
            else
                echo "    FAILED: output not found"
            fi
            make TARGET="$target" clean >/dev/null 2>&1 || true
        else
            echo "    FAILED to build $fw"
        fi
    )

    if [ -f "$target_file" ]; then
        echo "built" >> "$RESULTS_FILE"
    else
        echo "fail" >> "$RESULTS_FILE"
    fi
}

print_summary() {
    local built=0 skipped=0 failed=0
    if [ -f "$RESULTS_FILE" ]; then
        built=$(grep -c '^built$' "$RESULTS_FILE" || true)
        skipped=$(grep -c '^skip$' "$RESULTS_FILE" || true)
        failed=$(grep -c '^fail$' "$RESULTS_FILE" || true)
        rm -f "$RESULTS_FILE"
    fi
    echo ""
    echo "=== Summary: $built built, $skipped skipped, $failed failed ==="
}

# ---- Mode 2: Build from JSON configs ----
build_from_json() {
    local json_files="$@"

    resolve_contiki_dir

    echo "=== Build Firmware from JSON ==="
    echo "  Contiki-NG: $CONTIKI_DIR"
    echo ""

    RESULTS_FILE=$(mktemp)
    FW_LIST_FILE=$(mktemp)
    trap "rm -f $RESULTS_FILE $FW_LIST_FILE" EXIT

    # Extract unique firmware entries from all JSON files
    for json_file in $json_files; do
        [ -f "$json_file" ] || continue
        python3 -c "
import json, os, sys
with open('$json_file') as f:
    d = json.load(f)
seen = set()
for n in d.get('nodes', []):
    fw = n.get('firmware', '')
    if fw in seen:
        continue
    seen.add(fw)
    build = n.get('build', {})
    target = build.get('target', '-')
    board = build.get('board', '-')
    source_dir = build.get('source_dir', '-')
    make_args = ' '.join(build.get('make_args', []))
    if not make_args: make_args = '-'
    # fw_basename (without dir and extension)
    base = os.path.basename(fw)
    name = base.rsplit('.', 1)[0] if '.' in base else base
    # tab-separated: name, source_dir, target, board, make_args
    print(f'{name}\t{source_dir}\t{target}\t{board}\t{make_args}')
" 2>/dev/null >> "$FW_LIST_FILE" || true
    done

    # Deduplicate
    sort -u "$FW_LIST_FILE" -o "$FW_LIST_FILE"

    fw_count=$(wc -l < "$FW_LIST_FILE" | tr -d ' ')
    echo "Firmware to build: $fw_count"
    echo ""

    while IFS='	' read -r fw src_dir target board make_args; do
        [ -n "$fw" ] || continue

        # Treat "-" as empty
        [ "$src_dir" = "-" ] && src_dir=""
        [ "$target" = "-" ] && target=""
        [ "$board" = "-" ] && board=""
        [ "$make_args" = "-" ] && make_args=""

        # Use provided target or fall back to default
        if [ -z "$target" ]; then
            target="$DEFAULT_TARGET"
        fi

        # Resolve source_dir: may contain [CONTIKI_DIR]
        src_dir=$(echo "$src_dir" | sed "s|\[CONTIKI_DIR\]|$CONTIKI_DIR|g")

        # Determine output directory and file
        firmware_dir="$CSIM_DIR/firmware/$target"
        mkdir -p "$firmware_dir"
        target_file="$firmware_dir/$fw.$target"

        # Build extra args: BOARD + any make_args
        extra=""
        if [ -n "$board" ]; then
            extra="BOARD=$board"
        fi
        if [ -n "$make_args" ]; then
            extra="$extra $make_args"
        fi

        build_one "$fw" "$src_dir" "$target" "$target_file" $extra
    done < "$FW_LIST_FILE"

    print_summary
}

# ---- Mode 1: Scan .csc files ----
build_from_csc() {
    local test_pattern="$1"

    resolve_contiki_dir

    FIRMWARE_DIR="$CSIM_DIR/firmware/$DEFAULT_TARGET"
    mkdir -p "$FIRMWARE_DIR"

    echo "=== Build Test Firmware ==="
    echo "  Contiki-NG: $CONTIKI_DIR"
    echo "  Target: $DEFAULT_TARGET"
    echo "  Test pattern: $test_pattern"
    echo "  Output: $FIRMWARE_DIR"
    echo ""

    RESULTS_FILE=$(mktemp)
    FW_LIST_FILE=$(mktemp)
    trap "rm -f $RESULTS_FILE $FW_LIST_FILE" EXIT

    for csc_file in "$CONTIKI_DIR"/tests/$test_pattern/*.csc; do
        [ -f "$csc_file" ] || continue
        test_dir="$(dirname "$csc_file")"

        firmware_list=$(python3 "$CSC2JSON" --list-firmware "$csc_file" 2>/dev/null || true)
        for fw in $firmware_list; do
            if grep -q "^${fw}	" "$FW_LIST_FILE" 2>/dev/null; then
                continue
            fi

            src_dir=""
            code_dir="$test_dir/code"
            if [ -f "$code_dir/$fw.c" ]; then
                src_dir="$code_dir"
            else
                for d in "$test_dir" "$test_dir/.."; do
                    if [ -f "$d/code/$fw.c" ]; then
                        src_dir="$d/code"
                        break
                    fi
                done
            fi
            echo "${fw}	${src_dir}" >> "$FW_LIST_FILE"
        done
    done

    fw_count=$(wc -l < "$FW_LIST_FILE" | tr -d ' ')
    echo "Firmware to build: $fw_count"
    echo ""

    sort "$FW_LIST_FILE" | while IFS='	' read -r fw src_dir; do
        target_file="$FIRMWARE_DIR/$fw.$DEFAULT_TARGET"
        build_one "$fw" "$src_dir" "$DEFAULT_TARGET" "$target_file"
    done

    print_summary
}

# ---- Main ----
DEFAULT_TARGET="cooja"
TEST_PATTERN="*"
JSON_MODE=0
JSON_FILES=""

while [ $# -gt 0 ]; do
    case "$1" in
        --target)
            DEFAULT_TARGET="$2"
            shift 2
            ;;
        --from-json)
            JSON_MODE=1
            shift
            # Collect all remaining args as JSON files
            JSON_FILES="$@"
            break
            ;;
        -h|--help)
            echo "Usage: $0 [--target <cooja|cc2538dk>] [test-dir-pattern]"
            echo "       $0 --from-json <config.json> [config2.json ...]"
            echo ""
            echo "  --target: default build target (default: cooja)"
            echo "  --from-json: build using per-node build info from JSON configs"
            echo "  test-dir-pattern: glob to filter test dirs (e.g. '14-rpl-lite')"
            echo ""
            echo "JSON build info per node:"
            echo "  {\"build\": {\"target\": \"cooja\", \"board\": \"srf06\", \"make_args\": [\"DEFINES=...\"],"
            echo "              \"source_dir\": \"/path/to/code\"}}"
            echo ""
            echo "CONTIKI_DIR resolution: env variable -> csim.conf -> ../contiki-ng"
            exit 0
            ;;
        *)
            TEST_PATTERN="$1"
            shift
            ;;
    esac
done

if [ $JSON_MODE -eq 1 ]; then
    build_from_json $JSON_FILES
else
    build_from_csc "$TEST_PATTERN"
fi
