#!/bin/bash
#
# Run csim Cooja tests inside Contiki-NG Docker container
# Mirrors the Contiki-NG CI setup: --privileged, IPv6 enabled, root access.
#
# Usage: ./tools/run-tests-docker.sh [test-pattern] [-v]
#   Example: ./tools/run-tests-docker.sh              # all tests
#            ./tools/run-tests-docker.sh 17-tun-rpl-br # border-router only
#            ./tools/run-tests-docker.sh -v            # verbose
#
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
CSIM_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
CONTIKI_DIR="${CONTIKI_DIR:-$CSIM_DIR/../contiki-ng}"
DOCKER_IMAGE="contiker/contiki-ng"

if [ ! -d "$CONTIKI_DIR" ]; then
    echo "Error: Contiki-NG not found at $CONTIKI_DIR"
    exit 1
fi

CONTIKI_DIR="$(cd "$CONTIKI_DIR" && pwd)"
ARGS="$*"

echo "=== csim Docker Test Runner ==="
echo "  csim:       $CSIM_DIR"
echo "  Contiki-NG: $CONTIKI_DIR"
echo "  Docker:     $DOCKER_IMAGE"
echo "  Args:       $ARGS"
echo ""

DOCKER_TTY=""
if [ -t 0 ]; then DOCKER_TTY="-t"; fi

# Match Contiki-NG CI: --privileged + IPv6 enabled
# Override entrypoint (remap-user.sh drops root privileges)
docker run --rm -i $DOCKER_TTY \
    --privileged \
    --sysctl net.ipv6.conf.all.disable_ipv6=0 \
    --entrypoint bash \
    -v "$CSIM_DIR":/home/user/csim:ro \
    -v "$CONTIKI_DIR":/home/user/contiki-ng \
    -e CONTIKI_DIR=/home/user/contiki-ng \
    "$DOCKER_IMAGE" -c '
        set -e

        # --- Install GNU Lightning for JIT ---
        echo "--- Installing GNU Lightning ---"
        apt-get update -qq
        apt-get install -y -qq pkg-config wget >/dev/null 2>&1
        cd /tmp
        wget -q https://ftp.gnu.org/gnu/lightning/lightning-2.2.3.tar.gz
        tar xzf lightning-2.2.3.tar.gz
        cd lightning-2.2.3
        # Build without disassembler (avoids libopcodes dependency)
        ./configure --prefix=/usr/local --enable-disassembler=no --quiet
        make -j$(nproc) --quiet
        make install --quiet
        ldconfig
        echo "  GNU Lightning installed: $(pkg-config --modversion lightning)"

        # --- Copy and build csim ---
        echo "--- Building csim ---"
        cp -r /home/user/csim /tmp/csim-build
        cd /tmp/csim-build
        # Remove -march=native (host CPU features may differ from Docker)
        sed -i "s/-march=native//" Makefile
        make clean >/dev/null 2>&1 || true
        make -j$(nproc) 2>&1 | tail -3

        # Verify JIT is linked
        if ldd build/test_runner | grep -q lightning; then
            echo "  JIT: enabled"
        else
            echo "  WARNING: JIT not linked"
        fi

        # --- Build external tools ---
        echo "--- Building tunslip6 ---"
        make -C /home/user/contiki-ng/tools/serial-io tunslip6 2>&1 | tail -1

        echo "--- Building border-router.native ---"
        make -C /home/user/contiki-ng/examples/rpl-border-router \
            TARGET=native border-router.native \
            WERROR=0 CONTIKI=/home/user/contiki-ng 2>&1 | tail -1

        # --- Run tests ---
        echo ""
        tools/run-cooja-tests.sh --with-tun '"$ARGS"'
    '
