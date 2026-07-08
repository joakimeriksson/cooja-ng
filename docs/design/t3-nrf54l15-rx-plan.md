# T3 — nRF54L15 multi-node 802.15.4 receive: fix plan

Status: **RESOLVED for single-hop (2026-07-08)** — the receive-path root cause is
fixed; multi-hop forwarding is a separate remaining issue (see Resolution).
Companion to [`release-0.1.1-hardening.md`](release-0.1.1-hardening.md) §T3 and
[`riscv-vpr-plan.md`](riscv-vpr-plan.md).

## Resolution (2026-07-08)

The two "layered bugs" below turned out to be **one root cause**: the nRF54L15
TX-completion event (`tx_end_event`, which fires PAYLOAD/END/PHYEND and returns
the radio to TXIDLE ~100 µs after `TASKS_START`) was scheduled via
`arm_schedule_event_ns` off `sim_time_ns + air_dur_ns`. `sim_time_ns` lags the
live cycle counter, so the fire-cycle landed in the past and **PHYEND fired ~1
cycle after START instead of ~100 µs later**. That collapsed the whole TX into a
single cycle, so the driver's DPPI TXEN/START fan-out (same task, a few µs apart)
started a **second full emit of the same frame**. Two frames back-to-back put two
SFDs (0x7A) on air; the per-byte receiver latched the *second* SFD as the PHR
(length 0x7A = 122), every frame misaligned by one byte, the FCS landed in zeroed
buffer, and CRC failed → the driver rejected the frame (the "mid-frame
`TASKS_DISABLE`") and the peer never joined the DAG.

Fix (commit on `fix/t3-nrf54l15-rx`): schedule `tx_end_event` in **cycles**
(`cpu->cycles + ns_to_cycles(air_dur)`), the same cycle-vs-ns fix already used for
the rx-disable timeout. One emit per frame; the redundant DPPI START is absorbed
(state != TXIDLE). Both diagnosed symptoms (mid-frame DISABLE, CRC failure) were
this one cause. `configs/test-2node-nrf54l15-dk.json` now passes end-to-end; no
parity regressions (nrf52840, cc2538, MSP430, Zephyr, arm-correctness all green).

## Multi-hop — RESOLVED (2026-07-08)

The multi-hop failure is fixed for both nRF54L15 and nRF52840; full root cause
and fix in [`nrf-multihop-forwarding-plan.md`](nrf-multihop-forwarding-plan.md)
§Resolution. In short: a mid-frame **aborted reception** left the driver's
`psdu_being_received` flag set (it needs a CRCOK/CRCERROR *with* its RX IRQ to
clear, not the bare PHYEND the abort paths fired), so the router's every
subsequent TX returned `BUSY_CHANNEL` and it wedged. Firing `END+PHYEND+CRCERROR`
on abort clears it. nRF52840 additionally needed a destination-address check on
the fabricated auto-ACK (two neighbours were both ACKing every unicast, colliding
at the sender). `chain-3node-nrf54l15-dk`, `chain-3node/4node-nrf52840-dk` all
pass. The original investigation notes are kept below for context.

## Still open — multi-hop (2026-07-08 investigation) [superseded — see above]

3+ node chains route the first hop but not the second: the far node (node 3)
joins the DAG and sends UDP requests, but they don't reach the root. Findings so
far — this is a **distinct, nRF-radio-specific** issue, NOT the double-emit fix:

- **Platform-localized to the nRF radios.** 3-node chains with the *same*
  Contiki-NG rpl-udp firmware **pass** on Tmote Sky (MSP430/CC2420, node 3 gets
  8 responses) and on CC2538 (ARM RF Core, 9 responses). They **fail** only on
  nRF54L15 and nRF52840. So the kernel, radio medium, and RPL/6LoWPAN forwarding
  are all fine; the bug is in the nRF radio model (`nrf52840_soc.c` /
  `nrf54l15_soc.c` — they share the deferred-PHYEND / driver-ACK machinery that
  CC2538/CC2420 don't have).
- **Node 2 (the nRF router) barely receives node 3's UDP requests** — ~1 of ~6
  over 90 s (CRCOK trace, `phr=56` frames). It receives node 3's DIOs/DAOs and
  sends its own UDP to the root reliably, so its routing/forwarding logic works;
  the loss is on the radio RX side under concurrent load.
- **Not the DISABLED-gap drops.** The nRF `receive_byte` drops bytes when the
  radio isn't in `RX` (389 such drops in 3-node vs **0** in the working 2-node
  case, almost all `state=DISABLED`), because the driver's DISABLE→RXEN re-arm
  leaves brief (~1 µs) gaps that async third-node traffic lands in. But an
  experiment that processed bytes during the DISABLED/RXIDLE gap did **not** fix
  multi-hop — so the drops are a symptom, not the whole cause. (Contrast CC2538,
  which "receives frames regardless of radio state" — perfect-reception leniency
  — and works.)
- **The one request node 2 did receive was not forwarded to the root** — so
  there is also a receive→forward-TX turnaround failure (the nRF RX-then-
  immediately-TX path), on top of the reception loss.

Next steps: instrument node 2's receive→ACK→forward-TX sequence for a single
node-3 UDP request end to end (does the forward TX even get emitted? does the
root drop it?); compare the nRF RX-re-arm duty cycle against CC2538's; and decide
between (a) a bounded perfect-reception / gap-buffer leniency like CC2538's and
(b) tightening the RX→TX turnaround. Same class blocks the nRF52840 4-node
chains, so a fix should be checked against both.

---


## 1. Problem

Two nRF54L15 nodes never complete RPL/UDP: the sender transmits and the medium
delivers the bytes, but the receiver's radio model drops every frame, so the
peer never joins the DAG ("Not reachable yet"). **This is a cooja-ng emulator
bug, not Contiki-NG** — the *identical* firmware routes on the nRF52840 model
(2-node RPL-UDP = 8 receptions), and one of the two sub-bugs (byte
double-delivery) is something firmware cannot cause. Confirmed failing at
`v0.1.0` and pre-CRC, so it has **never worked** and is not a regression. The
FLPR single-node dual-core demo is unaffected (it uses no 802.15.4 RX).

The same class of receive-path failure blocks the nRF52840 4-node multi-hop
chains (`chain-4node-nrf52840-dk/-dongle.json`); a fix here should be checked
against those too.

Reproducer: `configs/test-2node-nrf54l15-dk.json` (already in the tree).
Traces: `NRF54L_RADIO_TRACE=1` (chip RX events), `CSIM_TRACE_RADIO=1` (bus
byte delivery + filter), `NRF54L_RX_DROP_TRACE=1` (bytes dropped out of RX).

## 2. Established root cause (two layered bugs)

Traced end-to-end on the receiver (`NRF54L_RADIO_TRACE`):

**Bug A — deferred-disable timeout fires mid-frame.** On the driver's
BCMATCH → `TASKS_DISABLE`, the RADIO model defers the state change via a fixed
**5 µs** safety timeout (`src/arm/nrf54l15_soc.c`, `R_TASKS_DISABLE` handler,
~line 1313 pre-fix). A frame's air-time is 32 µs *per byte* × 20+ bytes, so the
5 µs timeout always fires **mid-frame**: the radio snaps to DISABLED, re-arms at
`rx_phase = WAIT_PREAMBLE`, and the frame's remaining bytes (still arriving one
per 32 µs) are parsed as phase-0 garbage. The frame never reaches
`rx_remaining == 0`, so no CRCOK/END fires and the stack sees nothing. The
in-code comment even claims "5 µs … well above the … byte period (32 µs)",
which is backwards. Confirmed: extending the timeout to 5 ms lets the frame
complete.

**Bug B — completed frames fail CRC (reception corruption).** With Bug A
neutralised (long timeout), the frame completes but fires **CRCERROR**, so it is
still dropped. The `NRF54L_RADIO_TRACE` byte log shows the *same byte at the
same cycle delivered twice* for some bytes — a double-delivery that corrupts the
running FCS. Prime suspect: `sim_radio_bus_deliver_bytes`
(`src/sim/sim_radio_bus.c:436`) delivers a whole frame's bytes in a synchronous
`receive_byte()` loop (lines 464-479), which for a `SIM_RADIO_DELIVERY_PER_BYTE`
receiver (nRF54L15) can overlap the per-byte kernel `RX_BYTE` event path — i.e.
the frame is delivered by *both* mechanisms. Not yet root-caused; this is
Phase 1b.

**Open question (Phase 1a): why does the driver issue `TASKS_DISABLE`
mid-frame at all?** On real `nrf_802154`, after the header is parsed (BCMATCH)
the driver runs frame filtering and, for a valid frame (a broadcast DIO to
`ff02::1a` *is* for us), continues receiving — it does not disable. The model's
`rx_disable_pending` machinery exists specifically to absorb a mid-frame
DISABLE and still emit PHYEND so the driver's `psdu_being_received` flag clears
(otherwise the node can never TX again — see the `rx_disable_timeout_cb`
comment). We must confirm whether the DISABLE is (a) the driver *rejecting* the
frame because the model mis-set an address/PAN/CRC-config the filter reads,
(b) normal `nrf_802154` 54L flow that the model should ride through, or
(c) a reaction to a spurious event the model fired. The fix in Phase 2 differs
by case.

## 3. Plan

### Phase 0 — Harness (small)
- Keep `configs/test-2node-nrf54l15-dk.json` as the primary reproducer; add a
  pass/fail JSON validator (`"Received request"`/`"Received response"`) so it
  can join the gate once green.
- Capture a **known-good** reference from the nRF52840 2-node run under the same
  traces, to diff the RX event/timing sequence against 54L15.

### Phase 1 — Close the two unknowns (investigation)
- **1a. Why DISABLE mid-frame.** Instrument the point where `TASKS_DISABLE`
  arrives during RX: log `rx_phase`, the FCF/dest bytes parsed so far, and the
  driver's filter/PAN/address config registers. Compare against nRF52840, which
  does *not* disable mid-frame. Decide case (a)/(b)/(c) above. If (a), the real
  bug is upstream (address/PAN/CRCCNF wiring) and Bug A's timeout is a
  band-aid — fixing the filter may remove the mid-frame DISABLE entirely.
- **1b. Double-delivery.** Add a per-byte counter/assert in
  `nrf54l_radio_receive_byte`; confirm whether `sim_radio_bus_deliver_bytes`'s
  synchronous loop and the per-byte `RX_BYTE` event path both deliver to a
  54L15 (PER_BYTE) node. Establish which path is authoritative for PER_BYTE and
  which is the duplicate.

### Phase 2 — Fix Bug A (deferred disable must not fire mid-frame)
Pick per Phase 1a:
- **If the DISABLE is spurious/normal flow (case b/c):** replace the fixed 5 µs
  timeout with a **bytes-stopped watchdog** — reset the deadline on each RX byte
  received while `rx_disable_pending`, with a period **> 32 µs** (one byte time),
  e.g. 40–48 µs. A normally-progressing frame then completes naturally
  (`rx_remaining == 0` fires CRCOK/END and honours the pending disable — that
  path already exists, `nrf54l15_soc.c:1584`), and only a genuinely aborted
  frame (bytes actually stop) trips the watchdog. Reuse the bus-level RX-stall
  deadline if it already tracks "no byte for N ns" to avoid a second timer.
- **If the DISABLE is the driver rejecting the frame (case a):** fix the
  underlying filter/config mismatch so a valid broadcast/for-us frame is not
  rejected; the deferred-disable path then only runs for genuinely-rejected
  frames, where losing the frame is correct.
- Preserve the invariant the mechanism was built for: a mid-frame DISABLE that
  *is* honoured must still emit PHYEND so `psdu_being_received` clears (keep the
  `rx_disable_timeout_cb` PHYEND emit). Regression-guard: the 3-node chain's
  "node 3 can never TX" symptom the comment describes.

### Phase 3 — Fix Bug B (reception corruption / double-delivery)
- Based on 1b, make PER_BYTE receivers get each on-air byte **exactly once**.
  Likely: `sim_radio_bus_deliver_bytes` should not run its synchronous
  `receive_byte` loop for PER_BYTE-mode receivers (they are fed by the kernel
  `RX_BYTE` events), or vice-versa — one path owns delivery per mode.
- After the dedup, re-verify the FCS passes on a clean frame (the CRC compute
  itself was validated in the issue-#5 work; the failure is corrupted input,
  not convention). Keep the honest CRCSTATUS/CRCERROR behaviour from `6bc0402`.

### Phase 4 — Verification (broad gate + parity)
- 2-node `test-2node-nrf54l15-dk.json` routes end-to-end (request/response),
  then the 3-node `chain-3node-nrf54l15-dk.json`.
- **nRF52840 parity**: re-run 2-node nRF52840 RPL-UDP + TSCH + Zephyr echo to
  prove the shared bus/PER_BYTE change didn't regress the working model; if
  Phase 3 touched `sim_radio_bus.c`, also re-run cc2538 RPL-UDP/TSCH and MSP430
  multinode (all PER_BYTE/SYNC users).
- Re-check the nRF52840 4-node chains (`chain-4node-*`) — same receive-path
  class; note whether they now route or remain a separate item.
- Full broad gate (`release-0.1.1-hardening.md` §6) green; determinism check.

## 4. Risks & notes
- The `rx_disable_pending`/timeout machinery is the most heavily-commented,
  most fragile part of the 54L15 model, tuned around real `nrf_802154`
  choreography (critical sections, `psdu_being_received`, ACK turnaround).
  Every change must keep the single-node and FLPR paths green and preserve the
  PHYEND-on-disable invariant.
- Bug B's fix touches `sim_radio_bus.c`, shared by **all** PER_BYTE/SYNC
  platforms — the parity re-runs in Phase 4 are mandatory, not optional.
- If Phase 1a lands on case (a) (filter/config mismatch), the whole thing may
  reduce to a small wiring fix plus deleting the mid-frame-DISABLE band-aid —
  cheaper and cleaner than the watchdog. Do 1a before committing to Phase 2's
  shape.
- This is deep radio-timing work, **not** a 0.1.1 blocker; it can land as
  0.1.2. Recommended to consolidate the nRF52840/54L15 RX duplication
  (`release-0.1.1-hardening.md` D1) *after* T3, so the fix isn't duplicated.

## 5. Rough effort
Phase 1 (investigation) is the unknown — a day or two of tracing. Phases 2–3
are small once 1a/1b are answered (a watchdog reset + a one-line delivery-mode
guard, most likely). Phase 4 is mechanical but wide. Estimate: 2–4 focused days,
front-loaded on Phase 1.
