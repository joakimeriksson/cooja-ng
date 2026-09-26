#!/bin/bash
#
# Build Contiki-NG firmware needed for Cooja test suite
#
# Usage:
#   ./tools/build-test-firmware.sh [--force] [--dry-run] [test-dir-pattern]
#   ./tools/build-test-firmware.sh [--force] [--dry-run] [--target <cooja|cc2538dk>] \
#       --from-json <config.json> [config2.json ...]
#
# Mode 1 (default): Converts each matching .csc the way run-cooja-tests.sh does
#                   and builds the firmware it will look up, under the same name.
#                   Each .csc names its own TARGET, so there is no --target here.
# Mode 2 (--from-json): Reads build info from JSON configs (target, board, make_args);
#                   --target is the target for a node whose JSON names none.
#
# --force rebuilds firmware that already exists; --dry-run only reports what
# would be built.
#
# CONTIKI_DIR resolution: env variable -> csim.conf -> ../contiki-ng
#
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
CSIM_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
CSC2JSON="$SCRIPT_DIR/csc2json.py"

# Every temporary this script makes, removed on exit whichever mode ran.
RESULTS_FILE=""
FW_LIST_FILE=""
JSON_DIR=""
cleanup() {
    local f
    for f in "$RESULTS_FILE" "$FW_LIST_FILE" "$JSON_DIR"; do
        [ -n "$f" ] && rm -rf "$f"
    done
    return 0
}
trap cleanup EXIT

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

# Strip the cache-name suffix from a firmware name to find the source .c
# file: node-378324 -> node.  Every name csc2json asks this script to build
# is <source>-<6 hex digits> (firmware_variant); the shipped images under the
# previous scheme's names are never built here.
strip_variant_suffix() {
    echo "$1" | sed -E 's/-[0-9a-f]{6}$//'
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

    if [ -f "$target_file" ] && [ "$FORCE" -eq 0 ]; then
        echo "  SKIP $fw (already exists)"
        echo "skip" >> "$RESULTS_FILE"
        return
    fi

    # The firmware name may have a hash suffix (e.g., node-378324) derived
    # from make_args.  The actual source file uses the base name (node.c).
    local fw_base
    fw_base="$(strip_variant_suffix "$fw")"

    if [ -z "$src_dir" ] || { [ ! -f "$src_dir/$fw.c" ] && [ ! -f "$src_dir/$fw_base.c" ]; }; then
        echo "  MISS $fw (source not found: $src_dir)"
        echo "fail" >> "$RESULTS_FILE"
        return
    fi

    # Determine which source name to build
    local build_name="$fw"
    if [ ! -f "$src_dir/$fw.c" ] && [ -f "$src_dir/$fw_base.c" ]; then
        build_name="$fw_base"
    fi

    local extra_info=""
    if [ -n "$extra_args" ]; then
        extra_info=" $extra_args"
    fi
    if [ "$DRY_RUN" -eq 1 ]; then
        echo "  WOULD $fw ($build_name.c in $src_dir, TARGET=$target$extra_info)"
        echo "built" >> "$RESULTS_FILE"
        return
    fi
    echo "  BUILD $fw (TARGET=$target$extra_info)"

    (
        cd "$src_dir"
        make TARGET="$target" clean >/dev/null 2>&1 || true
        if make -j"$(nproc 2>/dev/null || sysctl -n hw.ncpu)" TARGET="$target" "$build_name.$target" \
            WERROR=0 CONTIKI="$CONTIKI_DIR" COOJA_CI=1 $extra_args >/dev/null 2>&1; then
            # Contiki-NG may place output in build/<target>/ or current dir
            local built_file=""
            if [ -f "$build_name.$target" ]; then
                built_file="$build_name.$target"
            elif [ -f "build/$target/$build_name.$target" ]; then
                built_file="build/$target/$build_name.$target"
            fi
            if [ -n "$built_file" ]; then
                cp "$built_file" "$target_file"
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
    if [ "$DRY_RUN" -eq 1 ]; then
        echo "=== Summary: $built would be built, $skipped skipped, $failed failed ==="
    else
        echo "=== Summary: $built built, $skipped skipped, $failed failed ==="
    fi
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
# Each .csc is converted exactly as run-cooja-tests.sh converts it, and the
# JSON is built as by Mode 2.  So the firmware lands under the name the suite
# looks up — per source directory and make arguments, with the .csc's own
# target, board and make arguments — instead of a plain <name>.<target> the
# suite no longer trusts, since it may have been built from another directory.
build_from_csc() {
    local test_pattern="$1"

    resolve_contiki_dir

    echo "=== Build Test Firmware ==="
    echo "  Test pattern: $test_pattern"

    local n=0 name log
    JSON_DIR=$(mktemp -d)

    # csc2json looks for existing firmware relative to its cwd, and the suite
    # runs it from this tree.  Its stderr is kept: the reason a .csc could not
    # be converted, and the warning that a test falls back to shipped firmware
    # under the previous scheme's name, are the two things a user building
    # firmware needs to see.
    for csc_file in "$CONTIKI_DIR"/tests/$test_pattern/*.csc; do
        [ -f "$csc_file" ] || continue
        n=$((n + 1))
        name="$(basename "$(dirname "$csc_file")")/$(basename "$csc_file" .csc)"
        log="$JSON_DIR/$n.log"
        if ! (cd "$CSIM_DIR" && python3 "$CSC2JSON" "$csc_file" \
                --contiki "$CONTIKI_DIR" --firmware-dir "firmware/cooja" \
                --js-native -o "$JSON_DIR/$n.json" 2>"$log"); then
            echo "  SKIP $name (conversion failed)"
            grep -E '^ *ERROR: ' "$log" | sed 's/^ */        /' || true
            rm -f "$JSON_DIR/$n.json"
        else
            grep '^WARNING: ' "$log" | sed "s|^WARNING: |  WARN $name: |" || true
        fi
    done
    echo ""

    build_from_json "$JSON_DIR"/*.json
}

# ---- Main ----
DEFAULT_TARGET="cooja"
TARGET_GIVEN=0
TEST_PATTERN="*"
JSON_MODE=0
JSON_FILES=""
FORCE=0
DRY_RUN=0

while [ $# -gt 0 ]; do
    case "$1" in
        --target)
            DEFAULT_TARGET="$2"
            TARGET_GIVEN=1
            shift 2
            ;;
        --force)
            FORCE=1
            shift
            ;;
        --dry-run)
            DRY_RUN=1
            shift
            ;;
        --from-json)
            JSON_MODE=1
            shift
            # Collect all remaining args as JSON files
            JSON_FILES="$@"
            break
            ;;
        -h|--help)
            echo "Usage: $0 [--force] [--dry-run] [test-dir-pattern]"
            echo "       $0 [--force] [--dry-run] [--target <cooja|cc2538dk>] --from-json <config.json> [config2.json ...]"
            echo ""
            echo "  test-dir-pattern: glob to filter test dirs (e.g. '14-rpl-lite'); each .csc"
            echo "                    is built for the TARGET it names"
            echo "  --from-json: build using per-node build info from JSON configs"
            echo "  --target: with --from-json, the target for a node whose JSON names none"
            echo "            (default: cooja)"
            echo "  --force: rebuild firmware that already exists"
            echo "  --dry-run: report what would be built without building"
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
    if [ $TARGET_GIVEN -eq 1 ]; then
        echo "Error: --target applies to --from-json only; a .csc names its own TARGET"
        exit 1
    fi
    build_from_csc "$TEST_PATTERN"
fi
