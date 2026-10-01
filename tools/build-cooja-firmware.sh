#!/bin/bash
#
# Build the native Cooja firmware the Contiki-NG test suite needs.
#
# A thin front for build-test-firmware.sh, kept for its name and its flags.
# The conversion, the naming and the "does it exist already" check all live
# there, so this script cannot build firmware under a name run-cooja-tests.sh
# would not look up.
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

for arg in "$@"; do
    case "$arg" in
        -h|--help)
            echo "Usage: $0 [test-dir-pattern] [--force] [--dry-run]"
            exit 0 ;;
    esac
done

exec "$SCRIPT_DIR/build-test-firmware.sh" "$@"
