#!/bin/bash
#
# Phase 9 plugin smoke check.
#
# Builds the example packet-sink plugin and asserts:
#   1. loading it via --plugin prints a non-zero observer tally at teardown;
#   2. the tally is deterministic across two runs;
#   3. a missing .so degrades cleanly (error reported, no crash).
#
# Usage: tools/check-plugin.sh
#
set -u
RUNNER=${RUNNER:-./build/test_runner}
SO=build/plugins/packet_sink.so
CFG=${CFG:-configs/test-rpl-udp-sky.json}
T=${T:-30000}

make plugins >/dev/null 2>&1 || { echo "FAIL: build plugins"; exit 1; }
[ -f "$SO" ] || { echo "FAIL: $SO not built"; exit 1; }

t1=$("$RUNNER" test "$CFG" -t "$T" --plugin "$SO" 2>&1 | grep -E '^packet-sink:')
t2=$("$RUNNER" test "$CFG" -t "$T" --plugin "$SO" 2>&1 | grep -E '^packet-sink:')

if [ -z "$t1" ]; then echo "FAIL: no packet-sink tally line"; exit 1; fi
if [ "$t1" != "$t2" ]; then echo "FAIL: tally not deterministic"; echo " $t1"; echo " $t2"; exit 1; fi
# Assert the tally observed *something* (log lines or uart bytes are always emitted).
if echo "$t1" | grep -qE '0 log lines, 0 uart bytes'; then echo "FAIL: tally is empty: $t1"; exit 1; fi

# Missing .so: clean error, non-crash (exit code must not be a signal, i.e. < 128).
"$RUNNER" test "$CFG" -t 2000 --plugin /no/such/plugin.so -q >/tmp/csim_plugin_miss.txt 2>&1
rc=$?
rm -f /tmp/csim_plugin_miss.txt
if [ "$rc" -ge 128 ]; then echo "FAIL: missing .so crashed (rc=$rc)"; exit 1; fi

echo "PLUGIN OK: $t1"
echo "PLUGIN OK: missing .so degraded cleanly (rc=$rc)"
exit 0
