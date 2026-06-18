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

# --- Radio-medium plugin (Phase 11): the lossy medium selected by config v2 ---
MED_SO=build/plugins/lossy_medium.so
MED_CFG=configs/medium-plugin-sky-v2.json
[ -f "$MED_SO" ] || { echo "FAIL: $MED_SO not built"; exit 1; }
m1=$("$RUNNER" test "$MED_CFG" -t 30000 2>&1 | grep -E '^Radio medium:|^  Total RF bytes:')
m2=$("$RUNNER" test "$MED_CFG" -t 30000 2>&1 | grep -E '^Radio medium:|^  Total RF bytes:')
if ! echo "$m1" | grep -q 'Radio medium: lossy (plugin)'; then
    echo "FAIL: lossy medium not selected"; echo " $m1"; exit 1; fi
if [ "$m1" != "$m2" ]; then echo "FAIL: medium plugin not deterministic"; exit 1; fi
echo "PLUGIN OK: medium plugin '$(echo "$m1" | head -1)' (deterministic)"

# --- Energy plugin (energest): a COMPILED-IN plugin selected by config name ---
# (no .so — config plugins:["energest"] resolves to the built-in service).
EN_CFG=configs/plugin-energest-builtin-v2.json
e1=$("$RUNNER" test "$EN_CFG" -t "$T" 2>&1 | grep -E '^energest: network total')
e2=$("$RUNNER" test "$EN_CFG" -t "$T" 2>&1 | grep -E '^energest: network total')
if [ -z "$e1" ]; then echo "FAIL: no energest total line (built-in not attached?)"; exit 1; fi
if [ "$e1" != "$e2" ]; then echo "FAIL: energest not deterministic"; echo " $e1"; echo " $e2"; exit 1; fi
# A radio that was tracked at all charges some energy.
if echo "$e1" | grep -qE '~0\.000 mJ'; then echo "FAIL: energest charged nothing: $e1"; exit 1; fi
# The per-mote line carries the CPU/LPM axis (SIM_OBS_CPU_STATE snapshot).
em=$("$RUNNER" test "$EN_CFG" -t "$T" 2>&1 | grep -E '^energest:  mote ' | head -1)
if ! echo "$em" | grep -qE 'cpu [0-9].* lpm [0-9]'; then
    echo "FAIL: energest missing cpu/lpm axis: $em"; exit 1; fi
echo "PLUGIN OK: energest built-in by name '$e1' (radio+cpu/lpm, deterministic)"
exit 0
