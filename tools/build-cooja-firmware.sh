#!/bin/bash
#
# Build all native Cooja firmware needed by Contiki-NG test suite.
#
# Scans .csc files, extracts build info via csc2json.py, and builds
# any missing firmware. Behaves like COOJA's auto-build: if the binary
# exists, skip it; otherwise build from source with the correct target
# and make arguments.
#
# Usage:
#   ./tools/build-cooja-firmware.sh [test-dir-pattern] [--force] [--dry-run]
#
# Examples:
#   ./tools/build-cooja-firmware.sh                  # build all
#   ./tools/build-cooja-firmware.sh 07-simulation     # only 07-*
#   ./tools/build-cooja-firmware.sh --force           # rebuild everything
#   ./tools/build-cooja-firmware.sh --dry-run         # just show what would build
#
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
CSIM_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
CSC2JSON="$SCRIPT_DIR/csc2json.py"

# Resolve CONTIKI_DIR
if [ -z "$CONTIKI_DIR" ]; then
    if [ -f "$CSIM_DIR/csim.conf" ]; then
        CONTIKI_DIR=$(grep -s '^CONTIKI_DIR=' "$CSIM_DIR/csim.conf" | cut -d= -f2-)
    fi
fi
CONTIKI_DIR="${CONTIKI_DIR:-$CSIM_DIR/../contiki-ng}"
if [ ! -d "$CONTIKI_DIR" ]; then
    echo "Error: Contiki-NG not found at $CONTIKI_DIR"
    echo "Set CONTIKI_DIR or run: make configure CONTIKI_DIR=/path/to/contiki-ng"
    exit 1
fi
CONTIKI_DIR="$(cd "$CONTIKI_DIR" && pwd)"

# Parse args
PATTERN="*"
FORCE=0
DRY_RUN=0
FW_DIR="$CSIM_DIR/firmware/cooja"

while [ $# -gt 0 ]; do
    case "$1" in
        --force) FORCE=1 ;;
        --dry-run) DRY_RUN=1 ;;
        -h|--help)
            echo "Usage: $0 [test-dir-pattern] [--force] [--dry-run]"
            exit 0 ;;
        *) PATTERN="$1" ;;
    esac
    shift
done

mkdir -p "$FW_DIR"

echo "=== Build Cooja Firmware ==="
echo "  Contiki-NG: $CONTIKI_DIR"
echo "  Output:     $FW_DIR"
echo "  Pattern:    $PATTERN"
echo ""

# Collect unique firmware build specs from all .csc files
TMP=$(mktemp)
trap "rm -f $TMP" EXIT

for csc in "$CONTIKI_DIR"/tests/$PATTERN/*.csc; do
    [ -f "$csc" ] || continue
    # Skip z1-only and drift tests (no z1 platform support)
    echo "$csc" | grep -qiE "drift|z1" && continue

    python3 "$CSC2JSON" "$csc" --firmware-dir "$FW_DIR" --js-native 2>/dev/null | \
    python3 -c "
import json, sys, os, re
d = json.load(sys.stdin)
seen = set()
for n in d.get('nodes', []):
    fw = n.get('firmware', '')
    build = n.get('build', {})
    base = os.path.basename(fw)
    name = base.rsplit('.', 1)[0] if '.' in base else base
    src_dir = build.get('source_dir', '')
    target = build.get('target', 'cooja')
    make_args = build.get('make_args', [])
    make_args_str = ' '.join(make_args)

    # Only build for TARGET=cooja
    if target != 'cooja':
        continue

    # out_name is the hashed firmware name from csc2json
    # src_name is the original source file name (for make)
    out_name = name
    # Derive source name by stripping hash suffix
    src_name = name.rsplit('-', 1)[0] if '-' in name and len(name.rsplit('-', 1)[1]) == 6 else name
    # Also strip '-classic' suffix
    if src_name.endswith('-classic'):
        src_name = src_name[:-8]

    key = f'{out_name}|{src_dir}|{make_args_str}'
    if key in seen:
        continue
    seen.add(key)

    # Tab-separated: output_name, source_name, source_dir, target, make_args
    print(f'{out_name}\t{src_name}\t{src_dir}\t{target}\t{make_args_str}')
" 2>/dev/null >> "$TMP"
done

# Deduplicate and sort
sort -u "$TMP" -o "$TMP"

built=0
skipped=0
failed=0

while IFS='	' read -r out_name src_name src_dir target make_args; do
    [ -n "$out_name" ] || continue

    output="$FW_DIR/$out_name.$target"

    # Skip if already exists (unless --force)
    if [ -f "$output" ] && [ $FORCE -eq 0 ]; then
        skipped=$((skipped + 1))
        continue
    fi

    # Resolve [CONTIKI_DIR]
    src_dir=$(echo "$src_dir" | sed "s|\[CONTIKI_DIR\]|$CONTIKI_DIR|g")

    # Check source exists
    if [ -z "$src_dir" ] || [ ! -d "$src_dir" ]; then
        echo "  MISS  $out_name (source not found: $src_dir)"
        failed=$((failed + 1))
        continue
    fi

    # Check for source file
    if [ ! -f "$src_dir/$src_name.c" ]; then
        echo "  MISS  $out_name (no $src_name.c in $src_dir)"
        failed=$((failed + 1))
        continue
    fi

    if [ $DRY_RUN -eq 1 ]; then
        echo "  WOULD $out_name ($src_name.c TARGET=$target $make_args)"
        built=$((built + 1))
        continue
    fi

    echo -n "  BUILD $out_name ($src_name.c TARGET=$target"
    [ -n "$make_args" ] && echo -n " $make_args"
    echo ")"

    (
        cd "$src_dir"
        make TARGET="$target" clean >/dev/null 2>&1 || true
        if make -j"$(nproc 2>/dev/null || sysctl -n hw.ncpu)" \
            TARGET="$target" "$src_name.$target" \
            WERROR=0 CONTIKI="$CONTIKI_DIR" $make_args >/dev/null 2>&1; then

            # Find the output (may be in build/ subdir or current dir)
            built_file=""
            if [ -f "$src_name.$target" ]; then
                built_file="$src_name.$target"
            elif [ -f "build/$target/$src_name.$target" ]; then
                built_file="build/$target/$src_name.$target"
            fi

            if [ -n "$built_file" ]; then
                cp "$built_file" "$output"
                echo "    -> $(basename "$output")"
            else
                echo "    FAILED: output not found"
            fi
            make TARGET="$target" clean >/dev/null 2>&1 || true
        else
            echo "    FAILED to build"
        fi
    )

    if [ -f "$output" ]; then
        built=$((built + 1))
    else
        failed=$((failed + 1))
    fi
done < "$TMP"

echo ""
echo "=== Summary ==="
echo "  Built:   $built"
echo "  Skipped: $skipped (already exist)"
echo "  Failed:  $failed"
echo "  Total:   $(ls "$FW_DIR"/*.cooja 2>/dev/null | wc -l | tr -d ' ') firmware files in $FW_DIR"
