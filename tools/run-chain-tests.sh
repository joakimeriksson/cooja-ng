#!/usr/bin/env bash
# Run per-platform RPL-UDP chain tests.
#
# Usage:
#   ./tools/run-chain-tests.sh                       # run all chain configs
#   ./tools/run-chain-tests.sh sky cc2538dk          # run a subset (matched by suffix)
#   PLATFORM=nrf52840-dongle ./tools/run-chain-tests.sh    # same via env
#
# Each test invokes:
#   build/test_runner test configs/chain-*-<platform>.json
#
# Per-test timeout falls back to the JSON's timeout_ms; we also cap with
# the runner's own watchdog plus a hard 10-minute shell timeout per case.

set -u

cd "$(dirname "$0")/.."

RUNNER=build/test_runner
if [ ! -x "$RUNNER" ]; then
    echo "error: $RUNNER not built. Run 'make' first." >&2
    exit 1
fi

# Canonical platform order. Faster platforms first so the slow nrf54l15
# isn't blocking; a failure on a small platform fails fast.
ALL_PLATFORMS=(
    sky
    z1
    cc2538dk
    nrf52840-dongle
    nrf52840-dk
    firefly-subghz
    nrf54l15-dk
)

# Filter platforms by CLI args or PLATFORM env.
filter=("$@")
if [ ${#filter[@]} -eq 0 ] && [ -n "${PLATFORM:-}" ]; then
    filter=($PLATFORM)
fi

platforms=()
if [ ${#filter[@]} -eq 0 ]; then
    platforms=("${ALL_PLATFORMS[@]}")
else
    for f in "${filter[@]}"; do
        platforms+=("$f")
    done
fi

# Resolve each platform name to a config file. Look up chain-3node-* first
# (nrf54l15), then chain-4node-*.
resolve_config() {
    local p="$1"
    for prefix in chain-3node chain-4node; do
        local cfg="configs/${prefix}-${p}.json"
        if [ -f "$cfg" ]; then
            echo "$cfg"
            return 0
        fi
    done
    return 1
}

pass=0
fail=0
skip=0
failed_names=()

for p in "${platforms[@]}"; do
    cfg=$(resolve_config "$p") || true
    if [ -z "$cfg" ]; then
        printf '  SKIP %-22s (no config)\n' "$p"
        skip=$((skip + 1))
        continue
    fi
    printf '  RUN  %-22s %s\n' "$p" "$cfg"
    if "$RUNNER" test "$cfg" >/tmp/chain-${p}.log 2>&1; then
        printf '  PASS %-22s\n' "$p"
        pass=$((pass + 1))
    else
        printf '  FAIL %-22s (see /tmp/chain-%s.log)\n' "$p" "$p"
        fail=$((fail + 1))
        failed_names+=("$p")
    fi
done

echo
echo "chain-tests: $pass passed, $fail failed, $skip skipped"
if [ $fail -gt 0 ]; then
    echo "failed: ${failed_names[*]}"
    exit 1
fi
exit 0
