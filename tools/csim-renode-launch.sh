#!/usr/bin/env bash
# csim-renode-launch.sh — the wrapper Renode spawns to start csim.
#
# Renode's CoSimulationPlugin starts an external simulator with the two port
# numbers and the address on the command line, positionally:
#
#     <program> <mainPort> <asyncPort> <address>
#
# csim takes them as one "ADDR:MAIN:ASYNC" argument, so this maps between
# the two.  Point the peripheral at it in your .repl:
#
#     cosim: CoSimulated.CoSimulatedPeripheral @ sysbus <0x40100000, +0x100>
#         frequency: 1000000
#         limitBuffer: 1000
#         timeout: 10000
#         simulationFilePath: "/path/to/csim/tools/csim-renode-launch.sh"
#
# Which csim scenario to run comes from the environment, since Renode has no
# way to pass it:
#
#     CSIM_CONFIG            config to run (default configs/test-renode-sky.yaml)
#     CSIM_RENODE_FREQ_HZ    must match the peripheral's `frequency`
#     CSIM_RUNNER            path to the test_runner binary
#     CSIM_DIR               working directory for relative paths in the config
#                            (default: the csim checkout this script lives in)
#     CSIM_LOG               file to write csim's own output to (default: inherit
#                            Renode's stdout, which interleaves the two)
#
# Alternatively, drop simulationFilePath, keep `address`, start csim yourself
# with --renode and let Renode connect to it.
set -u

if [ $# -lt 3 ]; then
    echo "usage: $0 <mainPort> <asyncPort> <address>" >&2
    exit 2
fi

HERE=$(cd "$(dirname "$0")/.." && pwd)

# Renode spawns us with ITS working directory, not the csim checkout, so a
# config's relative firmware paths would not resolve.  Move to the checkout
# root first; CSIM_DIR overrides it for an out-of-tree scenario.
cd "${CSIM_DIR:-$HERE}" || exit 2

RUNNER=${CSIM_RUNNER:-$HERE/build/test_runner}
CONFIG=${CSIM_CONFIG:-$HERE/configs/test-renode-sky.yaml}
FREQ=${CSIM_RENODE_FREQ_HZ:-1000000}

if [ -n "${CSIM_LOG:-}" ]; then
    exec >"$CSIM_LOG" 2>&1
fi

# Record what Renode actually handed us: the argument order is Renode's, and
# a mismatch here is otherwise invisible (csim just never connects).
echo "csim-renode-launch: main=$1 async=$2 address=$3 config=$CONFIG freq=$FREQ"

exec "$RUNNER" test "$CONFIG" \
    --renode "$3:$1:$2" \
    --renode-freq "$FREQ" \
    ${CSIM_RUNNER_ARGS:-}
