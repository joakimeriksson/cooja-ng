# Status: Non-TUN Cooja Suite Green; TSCH Drift and Native TSCH Regressions Fixed

## Current State (2026-04-10)

The current worktree now passes the full non-TUN Cooja wrapper suite:

- `tools/run-cooja-tests.sh`
- Result: `Total 88 / Passed 81 / Failed 0 / Skipped 7 / Errors 0`
- The 7 skipped tests are the `17-tun-rpl-br/...` border-router cases filtered
  by the wrapper when `--with-tun` is not used.

That includes both of the previously important failure groups:

- `07-simulation-base/26-tsch-drift-z1`
- the native/cooja TSCH/RPL block:
  - `19-cooja-rpl-tsch`
  - `20-cooja-rpl-tsch-orchestra`
  - `21-cooja-rpl-tsch-security`
  - `23-rpl-tsch-z1`
  - `24-cooja-rpl-tsch-orchestra-storing`
  - `25-cooja-rpl-tsch-orchestra-link-based`
  - `26-cooja-rpl-tsch-orchestra-perfect-link`
  - `27-cooja-rpl-tsch-orchestra-root-rule-storing`
  - `28-cooja-rpl-tsch-orchestra-root-rule-ns`

## What Fixed `26-tsch-drift-z1`

The decisive MSP430/MSPSim-aligned fixes were:

1. `src/msp430/msp430_cpu.c`
   - `msp430_step_micros()` no longer forces an extra CPU cycle for
     zero-duration `execute(t, 0)` slices.

2. `test/test_mixed_multinode.c`
   - MSP430 RX byte delivery now uses the sender's actual event time.
   - `current_sim_ns` is pinned to the exact event-loop time before dispatch.
   - MSP430 byte delivery remains per-byte in the event loop with same-time
     wakeup requests.

3. `src/common/sim_event_queue.c`
   - rescheduling a queued node event now removes and reinserts it, refreshing
     same-time FIFO ordering.

4. `src/msp430/cc2420.c`
   - bytes outside active RX states are ignored instead of replayed later.
   - ACK RX completion is counted explicitly.

Verified:
- Direct runner:
  `build/test_runner mixed-multinode /tmp/tsch-drift-VICa2P.json -q`
  -> `TEST PASSED (514482 ms simulated)`
- Wrapper:
  `tools/run-cooja-tests.sh 07-simulation-base/26-tsch-drift-z1 -v`
  -> `PASS`

## What Fixed the Native TSCH/RPL Regressions

The native/cooja regressions were not another RF-geometry or JSON-conversion
 problem. They were a wakeup-scheduling mismatch.

Observed failure signature before the fix:
- Node 1 became coordinator and enqueued an EB.
- The other nodes scanned forever.
- `Routing links` stayed `0`.
- `Total RF bytes` stayed `0`.

Root cause:
- In the native/cooja path, stale `simRtimerNextExpirationTime` values could be
  left behind after startup and then scheduled literally in the past.
- That let the root miss its first real TSCH slot scheduling window, so the
  queued EB never reached RF transmit.

Fixes:

1. `test/test_mixed_multinode.c`
   - native wakeup scheduling now clamps stale native rtimer deadlines to
     `now` instead of scheduling a wakeup into the past.

2. `include/native/native_node.h`
   - explicit native radio transmission state was added:
     `radio_is_transmitting`, `radio_tx_finished`, `radio_tx_end_ns`

3. `src/native/native_node.c`
   - native radio transmission now keeps a transmission-active interval instead
     of clearing `simOutSize` immediately.

4. `test/test_mixed_multinode.c`
   - native event scheduling now honors exact transmission-end wakeups in the
     same spirit as Cooja's `ContikiRadio.doActionsAfterTick()`.

Representative verification:
- `build/test_runner mixed-multinode /tmp/23-rpl-tsch-z1.json -v`
  -> `PASS`
- `build/test_runner mixed-multinode /tmp/19-cooja-rpl-tsch.json -v`
  -> `PASS`
- `tools/run-cooja-tests.sh 07-simulation-base`
  -> `PASS`

## Remaining Work

The remaining unverified scope is the TUN/border-router block that the default
wrapper skips:

- `tools/run-cooja-tests.sh --with-tun`

That still needs a dedicated rerun before claiming a fully verified
`88/88 including TUN` result for the current tree.
