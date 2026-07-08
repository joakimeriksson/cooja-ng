# nRF multi-hop 802.15.4 forwarding — investigation & fix plan

Status: **RESOLVED (2026-07-08)** — nRF54L15 and nRF52840 multi-hop chains now
route end-to-end. Predecessor: [`t3-nrf54l15-rx-plan.md`](t3-nrf54l15-rx-plan.md)
(the single-hop nRF54L15 receive bug). The original investigation plan is kept
below for context; the actual root cause and fix are in the Resolution.

## Resolution (2026-07-08)

The failure was **not** in 6LoWPAN forwarding (as this plan first assumed) — it
was a radio-model bug shared in spirit by both nRF SoCs: **a driver-visible
`psdu_being_received` flag that never got cleared when a reception was aborted.**
Once set (on ADDRESS/FRAMESTART) but never cleared (needs a CRCOK/CRCERROR *with
its RX interrupt*), `nrf_802154_transmit_raw` returns `BUSY_CHANNEL` forever
(`can_terminate_current_operation → psdu_being_received_now`), so the router can
never TX/ACK/forward again. Single-hop never hit it (no collisions → no aborted
receptions); a multi-hop router hears two neighbours, so mid-frame aborts are
routine.

**nRF54L15** (`src/arm/nrf54l15_soc.c`, `nrf54l_radio_rx_stall`): a
collision-truncated frame stalled mid-payload and the bus RX-stall watchdog fired
only `PHYEND` (no IRQ) → flag stuck → router wedged in `DISABLED`. Fix: fire
`END+PHYEND+CRCERROR` on the stall so the driver's RX IRQ clears the flag.

**nRF52840** (`src/arm/nrf52840_soc.c`, `arm_elf_mote.c`) — two layers:
1. *psdu leak*: a collision-corrupted **invalid PHR after SFD** reset the parser
   silently after ADDRESS/FRAMESTART had already fired → same stuck flag (node
   could only send DIS, never joined). Fix: a `radio_abort_inflight_rx` helper
   (fires `END+PHYEND+CRCERROR`) called on invalid-PHR, on STOP/DISABLE
   mid-frame, and via a newly-wired `rx_stall` op (`armnrf_radio_ops`).
2. *spurious auto-ACK*: the fabricated auto-ACK had **no destination check**, so
   every neighbour that heard a unicast data frame ACKed it → two ACKs collided
   at the sender, which retransmitted forever and never established a route. Fix:
   fabricate the ACK only when the frame's extended-dest address matches this
   node's `FICR.DEVICEADDR0`.

Verified: `chain-3node-nrf54l15-dk`, `chain-3node-nrf52840-dk`,
`chain-4node-nrf52840-dk`/`-dongle` all PASS (node 4 routes 3 hops). No
regressions — nRF52840 2-node RPL, TSCH nRF52840, Zephyr echo, nRF54L15 2-node,
FLPR dual-core, cc2538/sky 4-node controls, and all unit suites stay green.

---

Original plan (for context):

Status: proposed (2026-07-08). Self-contained handoff — assumes no prior context.
Predecessor: [`t3-nrf54l15-rx-plan.md`](t3-nrf54l15-rx-plan.md) (the single-hop
nRF54L15 receive bug, **fixed**; this is the remaining multi-hop issue).

## 1. The bug (one paragraph)

On the nRF radio models (nRF52840 and nRF54L15), a 3+ node RPL-UDP **chain**
does not route past the first hop. The far node joins the DAG and sends UDP
requests, but they never reach the root, so it gets no responses. The *same*
Contiki-NG `rpl-udp` firmware routes fine over multiple hops on Tmote Sky
(CC2420) and CC2538 — so the kernel, radio medium, and RPL/6LoWPAN forwarding
are correct; the bug is in the **nRF radio model** (`src/arm/nrf52840_soc.c` and
`src/arm/nrf54l15_soc.c`, which share the deferred-PHYEND + driver-scheduled-ACK
machinery that the CC2538/CC2420 models do not have).

Single-hop (2-node) nRF works — that was a separate bug (frame double-emit),
already fixed by scheduling `tx_end_event` in cycles. This plan is only about the
multi-hop forwarding failure.

## 2. Reproducers (all committed)

| Config | Radio | Expected | Actual |
|---|---|---|---|
| `configs/test-2node-nrf54l15-dk.json` | nRF54L15 | PASS | **PASS** (single-hop, fixed) |
| `configs/chain-3node-nrf54l15-dk.json` | nRF54L15 | PASS | **FAIL** — node 3 gets 0 responses |
| `configs/chain-4node-nrf52840-dk.json` | nRF52840 | PASS | **FAIL** — same class |
| `configs/chain-4node-cc2538dk.json` | CC2538 | PASS | **PASS** (control — multi-hop works) |
| `configs/chain-4node-sky.json` | Sky/CC2420 | PASS | **PASS** (control) |

Run: `./build/test_runner test configs/<file>.json` and look for
`Validator [PASS]/[FAIL]` / `TEST PASSED/FAILED`. The 3-node nRF54L15 validators
are `"Received request" node=1` (passes — that's node 2's traffic) and
`"Received response" node=3` (**fails** — the 2-hop node never completes).

Topology of the chain: node 1 (server/root) at x=0, node 2 (client+router) at
x=50, node 3 (client) at x=100, `tx_range` ~55 so 1↔3 is out of range and node 3
must relay through node 2. A tighter-spacing variant (25 m, strong links) fails
identically, so it is **not** a marginal-link/UDGM topology artifact.

## 3. Established facts (do not re-derive)

1. **Platform-localized to nRF.** Sky and CC2538 multi-hop pass; nRF52840 and
   nRF54L15 fail. Shared kernel/medium/RPL/6LoWPAN are therefore exonerated.
2. **CC2538/CC2420 use "perfect reception."** `cc2538_rfcore_receive_byte`
   (`src/arm/cc2538_rfcore.c`) processes incoming frames *regardless* of radio
   state ("simulates perfect reception even when the firmware has nominally
   turned the radio off"). The nRF models instead **strictly drop** bytes when
   `state != RX`. This is the leading structural difference.
3. **The nRF router loses most of the far node's data frames.** In the 3-node
   nRF54L15 run, node 2 (the router) CRCOK-received node 3's UDP request (a
   `phr=56` frame) only **~1 time in 90 s** despite node 3 sending ~6. It
   receives node 3's DIOs/DAOs and sends its own UDP to the root reliably, and
   its forwarding *logic* works (it relays DAOs) — so the loss is on the radio
   RX side under concurrent load, not in routing.
4. **DISABLED-gap drops are a symptom, not the cause.** The nRF `receive_byte`
   drops bytes when not in RX: **389 drops in the 3-node run vs 0 in the working
   2-node run**, almost all `state=DISABLED`. These happen because the driver's
   RX re-arm is a DISABLE→RXEN cycle that leaves brief (~1 µs) DISABLED gaps that
   async third-node traffic lands in. **BUT** an experiment that processed bytes
   during the DISABLED/RXIDLE gap did **not** fix multi-hop — so closing the gap
   alone is insufficient. (Keep this in mind: the obvious "buffer during the gap"
   fix was tried and is not the whole answer.)
5. **A receive→forward-TX turnaround failure also exists.** The one node-3 UDP
   request node 2 *did* receive was **not** forwarded to the root — so beyond the
   reception loss there is a failure in the receive-then-immediately-transmit
   path (relevant nRF machinery: driver-scheduled ACK, deferred PHYEND / the
   `tx_end`/`rx_disable_pending` timing, DPPI TXEN/START).

## 4. Debug tooling (reuse this — it works)

Environment flags (stderr traces):
- `NRF54L_RADIO_TRACE=1` — nRF54L15 RADIO state/task/event trace. Add
  `NRF54L_RADIO_NODE=<cpu-tag>` to filter one node.
- `NRF54L_RX_DROP_TRACE=1` — logs each byte dropped because `state != RX`
  (`[radio cpu=... RX_DROP state=... byte=...]`).
- `CSIM_TRACE_RADIO=1` — bus-level per-byte TX + medium filter DELIVER/DROP.
- (nRF52840 equivalents: `NRF_RX_TRACE`, `NRF_RXBYTE_TRACE`.)

Node identification: the trace tags each radio by `cpu=0x%04x` (low 16 bits of
the CPU pointer). It changes per run but is stable within a run. **The busiest
node (most CRCOK events) is the middle router (node 2).** Get the mapping with:
`grep -oE "cpu=0x[0-9a-f]+" trace.log | sort | uniq -c | sort -rn`.

Frame fingerprint by on-air length (`phr`, = the `[PKT] Data #N <len>B` value):
- `phr=5` → ACK; `phr=48` → this node's own UDP request; `phr=56` → the far
  node's UDP request (the one that goes missing); `phr=85/93/94` → DAO/6LoWPAN;
  `phr=97/102` → DIO.

Temporary CRCOK trace (the one that produced fact #3) — add in the frame-complete
block of `nrf54l_radio_receive_byte` (`nrf54l15_soc.c`, just after `crc_ok` is
computed), gated on `getenv("MH_DEBUG")`, logging `cpu`, `rx_offset-1` (=phr),
`crc_ok`, `packetptr[1]` (FCF0), `packetptr[3]` (DSN), and `cpu->cycles`. Remove
it when done (it was fully reverted; the tree is clean).

The `[PKT]` / `[RF] ... -> receivers: N M` lines in normal stdout show every
frame's type, link src→dst (last 2 addr bytes), and which nodes the medium
delivered it to — the primary tool for following a packet hop by hop.

## 5. Investigation plan

### Phase 1 — Follow one node-3 request end to end
Instrument (or trace) a *single* node-3 UDP request through the whole path:
node 3 TX → does the medium deliver to node 2 (`receivers:` list)? → does node 2
CRCOK it? → does node 2 emit an ACK to node 3 **and** a forward TX to node 1? →
does node 1 CRCOK the forward? Pin the failure to exactly one edge. Fact #3/#5
say the first loss is node 2 not CRCOK-ing most copies, and the surviving copy
not being forwarded — confirm both and find which dominates.

### Phase 2 — Reception loss under load
Quantify node 2's RX availability: what fraction of wall-time is its radio in a
state that can receive (RX) vs not (DISABLED/RXIDLE/TX/RXRU), and does node 3's
TX consistently land in the not-RX windows? Compare against CC2538, which is
always-receptive. Determine *why* the driver leaves the nRF radio out of RX at
those moments — is it the post-TX (own-traffic / ACK) turnaround, the
DISABLE→RXEN re-arm, or the `rx_disable_pending` deferral holding a stale state?
Note fact #4: a naive "receive during the gap" leniency did **not** fix it, so
characterize precisely *which* not-RX windows the missed frames fall in before
choosing a fix.

### Phase 3 — Receive→forward-TX turnaround
For the request node 2 *does* receive, trace why the forward isn't emitted: does
the 6LoWPAN/CSMA layer even hand a frame to the radio, and if so does TASKS_TXEN/
START fire and `emit_tx` run? Suspect interaction between the driver-scheduled
ACK to node 3 and the immediately-following forward TX to node 1 (two TXs close
together), and the `tx_end`/PHYEND/`rx_disable_pending` state machine around it.

## 6. Candidate fixes (decide after Phase 1–3)

- **(a) Bounded perfect-reception / gap leniency**, à la CC2538: let the nRF RX
  parser accept bytes during the transient re-arm states (buffer + replay, or
  process in place) so async frames aren't lost. *Caveat:* the naive version was
  tried and didn't fix multi-hop by itself (fact #4) — it likely needs to be
  combined with the turnaround fix, and must not receive while the radio is
  genuinely off (guard against breaking any future nRF TSCH/duty-cycle path).
- **(b) Tighten the RX→TX turnaround** so node 2 can forward promptly after
  receiving, and returns to RX fast enough not to miss the next frame.
- **(c) Shrink/eliminate the DISABLE→RXEN gap** so the radio is effectively
  always-on under CSMA (closer to hardware and to CC2538). Necessary but per
  fact #4 not sufficient alone.

Most likely the fix is (a)+(b) together, or (c)+(b). Prove the hypothesis with a
throwaway experiment (as in fact #4) before writing the real change.

## 7. Verification (mandatory — the fix touches shared nRF radio code)

Must stay green (all pass today):
- `configs/test-2node-nrf54l15-dk.json` (single-hop nRF54L15 — the T3 fix)
- `nrf52840-dk` 2-node RPL-UDP; `test-tsch-nrf52840-dk.json` (TSCH assoc+sync)
- Zephyr echo (`nrf52840-dk` echo_server/client — nrf_802154-driven)
- FLPR dual-core (`mixed-multinode firmware/nrf54l15-dk/flpr-host.nrf54l15-dk`)
- cc2538 + sky RPL-UDP and TSCH (should be untouched, but the medium is shared)
- `arm-correctness` (153), `radio-medium` (241), `correctness` (83), `cc1200` (73)

Must now pass (the goal):
- `configs/chain-3node-nrf54l15-dk.json` — node 3 gets a response
- then `configs/chain-4node-nrf52840-dk.json` / `-dongle.json` (same class)

Determinism: same seed → identical event trace on the unchanged configs.

## 8. Code map

| What | Where |
|---|---|
| nRF54L15 RX byte parser + `state != RX` drop | `src/arm/nrf54l15_soc.c` `nrf54l_radio_receive_byte` (~1496) |
| nRF54L15 frame-complete (CRCOK/CRCERROR) | same fn, `if (r->rx_remaining == 0)` (~1571) |
| nRF54L15 TASKS_DISABLE / `rx_disable_pending` defer | `nrf54l_radio_trigger_task`, `R_TASKS_DISABLE` (~1280) |
| nRF54L15 TXEN/START/emit + `tx_end_event` (T3 fix) | same fn, `R_TASKS_TXEN`/`R_TASKS_START` (~1214–1275); `tx_end_cb` (~1360) |
| nRF52840 RX + `rx_incoming` buffering (RXRU/RXIDLE only) | `src/arm/nrf52840_soc.c` `nrf_radio_receive_byte` (~1325) |
| CC2538 perfect-reception (the working contrast) | `src/arm/cc2538_rfcore.c` `cc2538_rfcore_receive_byte` (~674) |
| Bus per-byte dispatch + RX-stall watchdog | `src/sim/sim_radio_bus.c` `dispatch_tx_byte` (~248), `sim_radio_bus_deliver_bytes` (~436) |
| Per-byte RX_BYTE kernel event delivery | `test/test_mixed_multinode.c` `deliver_rx_byte` (~578) |

## 9. Effort

Phase 1 (follow one packet) is a few hours and will likely reveal the dominant
failure edge. Phases 2–3 depend on that. The fix itself is probably small once
the mechanism is nailed, but touches shared nRF radio code, so Phase 7 parity is
the bulk of the wall-clock. Estimate: 2–4 focused days. Not a release blocker —
single-hop nRF and all other platforms work; this unblocks nRF *chains*.
