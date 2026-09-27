# Kernel and radio-path review, with a performance and refactoring plan

Status: **review complete** (2026-09-25, revised 2026-09-27 after review on
PR #62). Tier 0 item 3 and Tier 2 item 1 have landed (PRs #61 and #60); the
current sequencing is in §7.

Revisions: the §4 measurements and the Tier 0 prototype were taken on this
machine (Apple Silicon, stock `make`, GNU Lightning present) at `252fc0e`;
the PR #60/#61 figures on those branches, based on `22a66f0`. Every file:line
is anchored to `main` at `e4e3271` (after #60 and #61).
Companion to [`arm-performance-plan.md`](arm-performance-plan.md) (the ARM
interpreter, which this plan deliberately does not revisit) and
[`refactor-plan.md`](refactor-plan.md) (whose Phases 1–12 this plan assumes).

## Summary

**The architecture is sound and the kernel is used the way it should be.**
One unified `(time_ns, seq)` heap, one pump that sets `now_ns` from the popped
event before dispatch, generation-stamped events, every scheduler going
through the `sim_schedule_*` wrappers, and per-byte `RX_BYTE` kernel events
for every emulated chip. No wall-clock or host time on the radio path. The
Cooja invariants hold for chip-to-chip traffic.

**Radio delivery has no live shortcut between emulated chips, but three
things need fixing:**

1. A whole synchronous delivery subsystem inside `sim_radio_bus.c` (direct
   per-byte stepping of *another* mote's CPU, RXFIFO mini-steps, drain
   queues, a second auto-ACK model) is **dead for every chip today** — all
   chips are `PER_BYTE` — yet still compiled, still documented as the nrf52840
   path, and one registration flag away from coming back. It should go.
2. **JS app motes cannot hear emulated senders.** Their frames land in a
   queue nothing drains; `configs/cross-level-demo.json` shows `64 queued,
   0 drained, 3 dropped`.
3. The CC2538 hardware auto-ACK is emitted **synchronously at the last data
   byte with no turnaround**; three other timing constants are wrong or
   fudged (nRF52840 96 µs ACK, nRF54L15 fixed 100 µs PHYEND, CC1200 byte
   period on a node's first sub-GHz frame).

**Where time goes depends entirely on the ISA.** ARM workloads are ~90%
interpreter and are the ARM plan's problem. MSP430 workloads are dominated by
**kernel overhead per event**, because an active mote is sliced 1 µs at a
time (Cooja `execute(t, 1)`) and each slice is a full round trip through a
dispatcher that does **O(N) work per event**. On the 100-node grid that is
~1.6x real time (20 s simulated in 12.55 s) with only 9% of samples in the
interpreter.

**Two byte-identical changes measured in a scratch copy** (skip the runner's
per-wakeup all-nodes loops when they cannot matter; stop installing a debug
PC-trace hook on every MSP430 node):

| Workload | Stock | Prototype | |
|---|---|---|---|
| `udgm-100node-grid-sky`, 20 s sim | 12.55 s | **2.34 s** | **5.4x** |
| `chain-4node-sky`, 180 s sim | 0.685 s | **0.551 s** | 1.24x |
| `chain-4node-cc2538dk`, 180 s sim | 0.666 s | 0.633 s | 1.05x |

stdout (every timestamped console line) was identical on all three. The
PC-trace half has since landed on its own (PR #61) and measured no speedup,
so the gain is the loop skipping (Tier 0 items 1–2), which has not yet been
measured alone.

---

## 0. Method

- Read: `src/sim/sim_runtime.c`, `src/common/sim_event_queue.c`,
  `src/sim/sim_radio_bus.c` (+ header), `src/common/radio_medium.c` hot
  path, `include/common/event_queue.h`, the runner's loop/dispatch
  (`test/test_mixed_multinode.c`), both ELF mote modules, all five radio
  chips, the native/JS/external/Renode motes.
- Measured: `CSIM_PHASE_TIMING=1`, `sample(1)` self/inclusive profiles, and
  an instrumented scratch build that counts kernel events by kind and
  histograms per-CPU peripheral events by callback. The scratch build was
  also used for the prototype above; nothing in the tree was changed.
- Workloads: `chain-4node-sky.yaml`, `chain-3node-nrf52840-dk.json`,
  `chain-4node-cc2538dk.json`, `test-tsch-nrf52840-dk.json`,
  `udgm-100node-grid-sky.json`.

## 1. Architecture verdict — what is clean (verified)

| Property | Where | Verdict |
|---|---|---|
| Single unified queue, `(time, seq)` FIFO, `now_ns` set from the popped event before dispatch, generation drop, stop/pause honoured | `src/sim/sim_runtime.c:268–323` | clean |
| No `sim_eq_schedule*` caller outside the kernel; runner only `sim_eq_init` + read-only `peek_time` | grep; `test/test_mixed_multinode.c:2968, 2997, 3114` | clean |
| Past-time wakeups clamped, counted, warned once | `sim_runtime.c:105–120` | clean |
| Mote never writes `now_ns`; slice re-pins `sim_time_ns` to kernel time at both ends; overshoot ≤ 1 instruction and debited from the next slice | `src/motes/msp430_elf_mote.c:613–619`, `arm_elf_mote.c:438–449`, `msp430_cpu.c:906–942` | clean, MSPSim-faithful |
| Idle mote's wakeup derived from its CPU event-queue head; active mote sliced 1 µs (Cooja `MspMote.execute(t,1)`) | `msp430_cpu.c:973–984`, `arm_cpu.c:4801–4814`, `mote_impl.h:111–115` | clean (and the performance story, §4) |
| Every emulated chip registers `PER_BYTE`: one `SIM_EV_RX_BYTE` per on-air byte at `first_byte + n·period`, clamped ≥ now | `msp430_elf_mote.c:567–571`, `arm_elf_mote.c:395–400`, `sim_radio_bus.c:351–357` | clean |
| RX entry is the kernel: `execute(t,0)` → `receive_byte` → same-time wakeup | runner `deliver_rx_byte` :631–685 | clean, Cooja `MspMoteTimeEvent` |
| TX byte stamp is the kernel's `now`, never the mote-local clock | `sim_radio_bus.c:420–442` | clean, Cooja radio-callback semantics |
| No chip TX callback runs from inside a JIT block; `cpu->cycles` current at every IO callback | `msp430_jit.c:334–347`, `arm_jit.c:246–300, 664–694` | clean |
| No wall-clock/host time or libc `rand` on the radio path; medium RNG is seeded xorshift | grep; `radio_medium.c:58–63` | clean |
| Native→native frames queued at end-of-air-time through the kernel | `sim_radio_bus.c:957–1031`, `native_cooja_mote.c:393–416` | clean |
| Medium filter: link block, spectrum/channel pair, one probabilistic roll per (sender, receiver, frame) | `radio_medium.c:639–684` | clean |

## 2. Findings — radio data delivery

Severity: **A** = wrong behaviour observable in a supported scenario;
**B** = timing deviation from hardware that firmware can notice; **C** = dead
or misleading code that is a hazard, not a defect.

### F1 (C, large) — the synchronous chip-delivery subsystem is dead code

`sim_radio_bus_frame_complete()` (`src/sim/sim_radio_bus.c:673–924`) still
carries the pre-Phase-5 delivery model: `sim_radio_bus_deliver_bytes()`
(`:541–589`) steps *another* mote's CPU per byte through `rx_byte_sync`,
`frame_complete` mini-steps a receiver `+5000` cycles when its RXFIFO is
full (`:814–819`), `drain_rx` mini-steps (`:610–618`), the `emu_rx_queue`,
`rx_pre_sync`, the `executing_node` guard, the `RX_TICKING_STEP` /
`DRAIN_MINI_STEP` caps, and a second auto-ACK flush with its own turnaround
model (`:844–885`, `ack_start = tx_end + 192 µs`, `sender_ack_start = now +
192 µs`).

All of it only runs for `BATCH` receivers, and the only `BATCH` receivers
(JS, external, Renode) have no `rx_byte_sync`. The header still says
nrf52840 is `BATCH` (`include/sim/sim_radio_bus.h:161–164`); it is not
(`arm_elf_mote.c:395`). The CC2420's `rx_incoming[]` replay
(`src/chips/cc2420.c:398–406`) is equally dead: `rx_incoming_count` is only
ever zeroed. The CC2538's ISTXON-time direct-to-RXFIFO ACK injection
(`src/arm/cc2538_rfcore.c:175–235`) cannot fire under `PER_BYTE` either.

Why it matters: the re-entrant ACK path never feeds the assembler, so a
re-entrant ACK would be stamped preamble@now, SFD@now+32 µs with no
turnaround (`sim_radio_bus.c:420–436` + `:459–467`); and `deliver_bytes` runs one mote's
CPU from inside another's slice — exactly the invariant the kernel exists to
enforce. One `SIM_RADIO_DELIVERY_BATCH` registration on a chip mote brings
all of it back. The runner also spends real time on it (§4, F8).

### F2 (A) — JS app motes are deaf to emulated senders

`src/motes/js_app_mote.c:58–62` registers `BATCH` with `caps=0` and
`rxfifo_available` returning 0. In `frame_complete` that forces a
`step_until` on the JS mote and a `queue_frame` into `emu_rx_queue[js]`,
which `drain_rx` can never deliver (no `rx_byte_sync`). Measured on
`configs/cross-level-demo.json`: `Emu RX frames: 0 direct, 64 queued, 0
drained, 3 dropped`. The header already describes this failure mode
(`sim_radio_bus.h:188–195`). `js_mote_receive_frame` is only reached from
the frame-level path (native/JS/external senders). Fix: give JS
`SIM_RADIO_CAP_FRAME_CONSUMER` like external and Renode motes, and decide
whether the JS mote should see the frame at air-time start (today's
`receive_frame(now)` semantics, `js_app_mote.c:117–124`) or end.

### F3 (B) — CC2538 auto-ACK: synchronous, no turnaround, no TX state

`src/arm/cc2538_rfcore.c:828–850`: on the last payload byte the 11 ACK
bytes are pushed through `tx_callback` from inside the kernel `RX_BYTE`
dispatch. The bus stamps the ACK preamble at `now` = last data byte's air
time, so the ACK is on the air 32 µs *before the data frame ends* and
192 µs earlier than hardware; the chip stays in `SFD_WAIT` (`:853–855`)
during its own ACK, so it keeps receiving while "transmitting"; TXACKDONE
is raised at once. It works between two CC2538s because the sender's radio
is still in TX and buffers the early bytes (`:95–107`). It cannot work when
the data sender is a Sky: the ACK's preamble and SFD reach the CC2420 while it
is still in its TX states, and it drops bytes outside
`RX_SFD_SEARCH`/`RX_FRAME` (`cc2420.c:583`), so a **Sky→CC2538** unicast with
AR is never acknowledged. The other direction works: a CC2538→Sky frame is
acknowledged by the CC2420's own auto-ACK, which goes through
`TX_ACK_CALIBRATE` with a real turnaround (`cc2420.c:765–768`). Fix:
stage the ACK and emit it from a chip event at `+192 µs` after the last
byte, entering a TX state for its air time (the nRF52840 already does this;
`nrf52840_soc.c:900–908, 1564–1567`).

### F4 (B) — three timing constants

- nRF52840 auto-ACK fires at `cycles + 96·64` = **96 µs**
  (`nrf52840_soc.c:1564–1567`) while every comment says 192 µs (`:894,
  :1102, :1407, :2210`). Also hardcodes 64 MHz.
- nRF54L15 PHYEND is deferred a **fixed 100 µs** regardless of frame length
  (`nrf54l15_soc.c:1559–1571`). The comment's rationale ("the multinode
  harness delivers bytes … at synchronous (tx-start-anchored) timestamps")
  describes the pre-Phase-5 path that no longer exists; the per-byte path
  paces receivers correctly, so a full air-time PHYEND should now be
  re-tried. `nrf54l15-ack-gap.md` §"What remains approximate" already asks
  for real ramp durations.
- CC1200: `sender_byte_ns` is read *before* the assembler sees the byte
  (`sim_radio_bus.c:440–442`) and `subghz` only flips on the 4th sync byte
  (`:38–42`), so the first 8 bytes of a node's first sub-GHz frame (and of
  the first after a 2.4 GHz frame on a dual-radio Firefly — `tx_asm` is
  per node, not per radio) are stamped 32 µs apart, then jump ~1 ms.
  Harmless to reception (no `rx_stall` on those motes), wrong for the
  timeline and collision windows. Fix: per-(node, radio) assembler, or let
  the chip declare its byte period at registration.

### F5 (B) — emulated→native frames complete before their air time (**fixed by PR #56**)

At `252fc0e`, `SYNC` delivery fed each byte to the native assembler inside
the sender's TX call and queued the finished frame *as already ended* on the
receiver's stale clock, so a native mote could consume a frame ~4 ms before
it had finished arriving. PR #56 passes each byte's air time
(`receive_byte_at`, `sim_radio_bus.c:336–346`); the assembler queues the
frame so it ends on the bus clock (`src/native/native_radio.c:138–148`), the
native is woken at that end, and `native_dequeue_rx_frame` refuses a frame
whose end is still in the future (`native_node.c:278`).

### F6 (B) — CC2538 receives while RF is off; CCA never busy

`cc2538_rfcore.c:244–252` keeps the parser live after `ISRFOFF`
(`:691–693` "simulates perfect reception even when … turned the radio
off"); `ISTXONCCA` always transmits and `FSMSTAT1.CCA` is always clear
(`:68, :239–242, :381`); nRF52840 CCA is always idle (`nrf52840_soc.c:1186–1190`). CC1200
CCA does consult the bus's `tx_busy_until` (`runner :817–848`). Documented
modelling gaps; listed here because they are the remaining "receiver never
misses" shortcuts.

### F7 (C) — smaller items

- `deliver_rx_byte` schedules `if_earlier(next)` then `if_earlier(t)`
  (`runner :683–684`). `sync_to_time` returns a non-negative lead
  (`msp430_cpu.c:973–986`, `arm_cpu.c:4801–4814`), so `t ≤ next` and the
  second call always wins: the **first** one (`next_ns`) is redundant. The
  second is Cooja's same-time wakeup after `receivedByte()` and must stay.
- `frame_complete` uses `static` snapshot arrays (`sim_radio_bus.c:726–729`)
  — safe only because `tx_depth` prevents nesting; belongs on the bus.
- Per-process statics that break a second runtime: CC2420 stat counters
  (`cc2420.c:138–148`), CC2538 `rxfifo_overflow_count` (`cc2538_rfcore.c:23`), nRF54L15
  `trigger_task` depth guard (`nrf54l15_soc.c:1517–1519`), MSP430 PC-trace
  counters (`msp430_elf_mote.c:93–95`, live only under `CSIM_PC_TRACE=1`), `sim_service.c:74 dispatch_depth`,
  `sim_runtime.c:258, 270–271` spin diagnostics.
- `arm_elf_mote.c:376–377` says nRF54L15 has an `rx_incoming` buffer; it
  has none (`nrf54l15_soc.c:1822–1829`).
- `native_yield_callback` / `native_step_until_ns` (`runner :1315–1374`,
  `native_node.c:356–373`) would run other natives' ticks and hand an ACK
  back with zero air time — dead today (no caller reaches native
  `step_until`), a landmine.

## 3. Findings — event-queue usage and time-keeping

### F8 (perf, A-class for scale) — O(N) work on every `NODE_WAKEUP`

`dispatch_mote_wakeup` (`runner :1971–2040`) runs four loops over all
nodes on every wakeup: a native rx-count snapshot (`:1982–1991`), a
native got-frame scan (`:2008–2019`), `emu_rx_queue_drain(r)` for every
emulated node (`:2027–2030`, a bus call each — 16% of samples on the grid),
and `mixed_deliver_rf_bytes(r)` for every native (`:2033–2036`, dead work:
natives are `SYNC`, `rf_pending` is only staged for `BATCH`). With the
1 µs slicing (§4) this is `4·N` loop bodies per active microsecond per
mote — O(N²) in the node count. This is the single largest cost outside
the interpreters; the prototype removes it (Tier 0).

### F9 (perf) — one outer-loop iteration per distinct event time

Headless, the horizon is `min(end, next_event)` (`runner :3114–3138`), so
the pump drains only same-time events and the whole outer body (action
check, JS engine check, service-poll guard, console-injection scan,
radio-state guard, JSON step timeout, progress tick) runs once per event
timestamp: 10.2 M outer iterations for 11.8 M events on the Sky chain.
~6–11% of samples are `run_mixed_multinode_test` self time. The same horizon
is why Tier 1 cannot work on its own (see Tier 1's prerequisite).

### F10 (perf) — heap churn (**addressed by PR #60**)

At `252fc0e`, `sim_eq_schedule_gen` implemented "replace the pending wakeup"
as remove + sift-up + sift-down + insert; every `if_earlier` that fired paid
the same, and `sim_eq_pop` copied a 40-byte struct by value twice. Together
12% of Sky-chain samples, 13% on the grid. PR #60 reschedules in place and
sifts with a hole (`src/common/sim_event_queue.c`); the result and the
decision to stop there are under Tier 2 item 1.

### F11 (perf) — per-instruction debug hook on every MSP430 node (**fixed by PR #61**)

At `252fc0e`, `msp430_elf_mote_install_pc_trace` was called for every MSP430
node in every run, installing a callback the interpreter calls per
instruction (`msp430_cpu.c:1081–1082`). It compared against **hardcoded
firmware addresses `0xcb32` and `0xb138`** from one historical TSCH image and
fed three counters that appeared only in the end-of-run `FW
cc2420_transmit=…` line. PR #61 made it opt-in (`CSIM_PC_TRACE=1`,
`runner :2798`) and resolves the entry points from each node's own image
(`msp430_elf_mote.c:86–141`). The hook was 1.1% self time, but removing it
measured no speedup (Tier 0 item 3), so it contributes nothing measurable to
the prototype's gain.

### F12 (perf) — the 1 µs slice costs a function call per instruction

`msp430_step_until` (`msp430_cpu.c:906–942`) and `arm_step_until`
(`arm_cpu.c:4734–4767`) single-step (`steps = 1`) once fewer than 10
cycles remain, i.e. for essentially every active-mote slice (4 cycles at
4 MHz). The interpreter already stops at `cycle_limit` per instruction
(`msp430_cpu.c:1019–1020`), so the outer batching is redundant with it; the
ARM plan's §5.7b measured 79.8% of `arm_step` calls arriving with a budget
of 1. `msp430_step_until` self + `msp430_step` = 7% of Sky samples.

### F13 (design) — MSP430 JIT is switched off in multinode

`msp430_elf_mote.c:327–335` frees the JIT cache "because the scheduler
steps in ~1 µs increments". True today; it is a consequence of the slicing
model, not a fixed fact, and it means the only accelerated ISA in the tree
runs unaccelerated in every multinode simulation.

### F14 (correctness, minor) — `now_ns` pinned to the horizon before the pump

`runner :3140` sets `sim_rt.now_ns = sim_ns` (the horizon) before actions,
JS gen-msgs and service polls run, then the pump moves it back to the first
event. Headless the horizon *is* the next event; with the UI (+100 ms),
serial/pacing (+1 ms), the shell (+1 s) or a Renode quantum, anything that
reads `sim_runtime_now_ns` in that window (`sim_control_send` wake,
`native_cooja_mote.c:345–348`, shell wakes `shell_commands.c:921, 1876`,
Renode UART inject `renode_mote.c:84–85`) stamps a time later than every
pending event, and `now_ns` is non-monotonic within an iteration. The
header documents it as intended (`sim_runtime.h:315–317`); the consequence
(injected input up to a slice late in interactive modes) is not.

### F15 (correctness, minor) — MSP430 serial injection with clock deviation

`msp_mote_serial_input` (`msp430_elf_mote.c:743–781`) steps the CPU and
then sets `last_execute_us` from raw cycle time. With `clock_deviation ≠ 1`
raw cycle time ≠ kernel time, so the next slice's `jump_us` is wrong and
the mote over-steps. Only MSP430 + serial injection + deviation.

### F16 (labels) — `CSIM_PHASE_TIMING` mislabels

`time_step` brackets the whole `sim_runtime_run_until` including dispatch,
bus delivery and the O(N) loops (`runner :3295–3318`), but prints as "step
(CPU)"; "kernel/other" is only the outer loop. The grid reads 99% "CPU"
while 9% of samples are in the interpreter.

## 4. Where the time goes

### 4.1 Event counts (instrumented build, stock semantics)

| Workload | Nodes × sim s | Wall | Kernel events | of which RX_BYTE | Wakeups / node·s | Instr / wakeup | Peripheral events fired |
|---|---|---|---|---|---|---|---|
| `chain-4node-sky` | 4 × 180 | 0.68 s | 11.77 M | 30.6 k | **16.3 k** | **1.9** | 115 k (95 k Timer A CCR) |
| `chain-3node-nrf52840-dk` | 3 × 240 | 0.92 s | 2.31 M | 18.0 k | 3.2 k | 66 | 92 k (RTC) |
| `chain-4node-cc2538dk` | 4 × 180 | 0.68 s | 3.75 M | 30.1 k | 5.2 k | 21 | 92 k (SysTick) |
| `test-tsch-nrf52840-dk` | 2 × 90 | 1.36 s | 3.75 M | 1.0 k | 20.9 k | 49 | — |
| `udgm-100node-grid-sky`, 20 s | 100 × 20 | 12.4 s | 24.0 M | 53.6 k | 12.0 k | 3.2 | — |

Reading: peripheral events are cheap and rare (a 128 Hz clock tick per
node). The kernel event count is set by the **1 µs active-mote slicing**:
a Sky node that is active 1.6% of the time produces 16 k wakeups per second
and executes under two instructions per wakeup; an nRF52840 executes ~64
per wakeup (64 MHz × 1 µs). Every such wakeup is a heap pop, a dispatch
with four O(N) loops, an execute slice with a single-instruction
`step_until`, a heap insert, and an outer-loop iteration: **~58 ns per
event on the 4-node chain, ~515 ns on the 100-node grid.**

### 4.2 Profiles (`sample`, self time, share of samples)

`chain-4node-sky` (3416 samples): `msp430_step_interpreter` 32% ·
`mixed_dispatch_event` self 16% · `run_mixed_multinode_test` self 11% ·
`sim_eq_schedule_gen` 6% · `msp430_step_until` 6% · `sim_eq_pop` 6% ·
`sim_radio_bus_drain_rx` 6% · `msp_mote_execute` 5% · `sim_runtime_run_until`
4% · `sim_schedule_mote_wakeup_if_earlier` 2% · `srh_trace_cb` 1%.
**Interpreter 32%, per-event overhead ~62%.**

`udgm-100node-grid-sky` (4300 samples): `mixed_dispatch_event` self **52%** ·
`sim_radio_bus_drain_rx` **16%** · `msp430_step_interpreter` 9% ·
`sim_eq_pop` 7% · `sim_eq_schedule_gen` 6%. **Interpreter 9%.** ~1.6x real
time; `CSIM_PHASE_TIMING` reports 99% "step (CPU)" (F16).

`chain-3node-nrf52840-dk` (1100 samples): `arm_step_interpreter` 75% ·
`arm_step` 6% · `arm_jit_run` 3.5% · everything kernel-side < 8%.
`test-tsch-nrf52840-dk`: `arm_step_interpreter` 77%. **ARM is
interpreter-bound; see the ARM plan.**

### 4.3 What that means for prioritisation

- For MSP430 and for any simulation with more than a handful of nodes, the
  kernel/runner overhead is the problem and the wins are large and cheap.
- For ARM at 2–4 nodes the kernel is ≤ 10% and only the interpreter matters.
- Both meet in one place: the 1 µs slicing model sets the event count on
  every ISA (F12/F13/T3).

## 5. Optimisation plan

Ordered by measured evidence, then confidence, then cost. Every tier is
gated by `tools/check-determinism.sh` and `tools/check-baseline.sh` (all
nine workloads byte-identical except the lines its `FILTER` drops: wall-clock,
speed ratio, throughput and the opt-in PC-trace lines) plus the
radio-bus/radio-medium unit suites. Tiers 0 and 2 are semantics-preserving by
construction. Tier 1 is only once its prerequisite has been shown not to move
output (see there). Tier 3 is not, and gets its own gate.

### Tier 0 — remove O(N) work from the wakeup path (**prototype: 5.4x on 100 nodes, 1.24x on Sky chain, byte-identical**)

The prototype combined items 1–3 at `252fc0e`. Item 3 alone has since
measured no change, so the gain is items 1–2; re-measure them alone on
current `main`, whose stock baseline already includes #60.

1. `dispatch_mote_wakeup`: compute `have_native` once per topology change
   (add/remove/reboot) and skip the snapshot, got-frame and
   `mixed_deliver_rf_bytes` loops when false; delete the
   `mixed_deliver_rf_bytes` loop outright (dead — natives are `SYNC`).
   For the native case, replace the all-nodes snapshot with the sender's
   neighbour list from the medium.
2. Drain only receivers with a non-empty `emu_rx_queue` (keep a count or a
   small dirty list on the bus; the prototype tested `count == 0` inline).
   Once F1 lands, this loop disappears entirely.
3. ~~Make the PC trace opt-in~~ **DONE (PR #61)**: `CSIM_PC_TRACE=1`;
   the `FW cc2420_transmit=…` line goes with it. Not a measurable speedup
   alone (685 vs 689 ms Sky chain; 13.4 vs 13.1 s grid). The same PR made
   `check-baseline.sh` capture stdout and stderr separately, since the
   merged capture reported spurious diffs whenever an early stdout line
   moved a buffer-flush boundary.
4. Drop the redundant **first** `if_earlier` in `deliver_rx_byte`, the one
   with `next_ns` (F7). Not the second: that is the same-time wakeup after
   `receive_byte`, and without it every receiver reacts to a radio byte one
   slice late, which moves every radio workload.

Cost: a day. Risk: low. The prototype of items 1–3 diffed clean on 2686 + 173
output lines; item 4 was not in it and is gated like the rest.

### Tier 1 — pump-internal same-mote slice batching (est. 1.3–1.6x on MSP430 workloads, ~1.05–1.1x on ARM; needs F9 first)

An active mote reschedules itself to `now + 1 µs`; when that is the earliest
event in the heap, popping it is pure overhead. In `dispatch_mote_wakeup`
(or better, inside the pump with a "continue with the same mote" return
value), after `execute()` returns `next_ns`, loop `while (next_ns <
sim_eq_peek_time(q) && next_ns <= horizon && no stop/pause requested) {
now_ns = next_ns; next_ns = execute(m, next_ns); }` before touching the
heap. This is identical to pushing and immediately popping, because nothing
else can be scheduled between (single thread, and any event scheduled by
the slice itself is visible to `peek_time` on the next check — RX bytes for
other motes go through the heap and end the batch if earlier). It removes
a heap insert + pop, a generation check, an outer-loop iteration and a
dispatch per active microsecond. Expected to cut `sim_eq_*` +
`run_mixed_multinode_test` + `sim_runtime_run_until` self time (~25% on the
Sky chain) to a few percent.

**Prerequisite: a horizon past the next event (F9).** Headless, the runner
caps the pump's horizon at the next event (`runner :3129`), so inside the
pump the horizon *is* the time of the event being dispatched, and
`next_ns <= horizon` never holds for a mote's own next slice. Built as
above, the batch is a no-op and measures 0%. It only pays once the outer loop
lets the pump run past the next event, and that changes how often the
per-iteration duties run:

- timed actions (`runner :3146–3192`) and `CSIM_NODE*_INPUT` injection
  (`:3320–3349`) have due times and can bound the horizon exactly;
- the JS engine drain (`:3198`), `sim_service_poll_all` (`:3274`),
  radio-state tracking (`:3351–3361`) and the progress tick (`:3415`) run
  once per iteration, and output that depends on their cadence can move.

So Tier 1 is two steps. First F9: bound the horizon by the next due duty
rather than the next event, with a written list of the outputs it can move
and a `check-baseline.sh` run showing whether they do. Then the batch
itself, which is byte-identical relative to that. Until the first step has
been through the baseline, "byte-identical" for Tier 1 is a target, not an
argument.

Also required: `sim_control_note_event` (the `step N` budget) still counted
per slice.

### Tier 2 — cheaper primitives (est. 5–10% on MSP430 workloads; byte-identical)

1. ~~`sim_eq_schedule_gen` replace-in-place~~ **DONE (PR #60), and the
   queue is then left alone.** Reschedule rewrites the entry and sifts once;
   both sifts use a hole. Measured 685 → 674 ms on the Sky chain and
   13.4 → 12.5 s on the 100-node grid. The hope was 5–10% (pop + schedule
   were 12–13% of samples); the grid got 7% because its heap is deep
   enough for the per-level saving to matter, the chain got 1.6% because
   its heap is a handful of entries and the rest of the 12% is fixed
   per-call cost (wrapper, generation lookup, clamp, branches). No
   container fixes per-call cost; only calling it less does — that is
   Tier 1. **Decision (2026-09-26): no further event-queue work.** Not a
   radix heap (monotone keys would fit, change-key would not), not a
   calendar queue (bimodal times: active motes within µs of now, sleepers
   tens of ms out), not a 4-ary layout — unless a profile taken *after*
   Tier 1 puts `sim_eq_*` back above a few percent.
2. `msp430_step_until` / `arm_step_until`: call the interpreter once with a
   large count and let the existing per-instruction `cycle_limit` check
   stop it, instead of `steps = 1`. Keep the boundary event drain. Gate on
   the TSCH workloads (the comment cites a real desync the single-step
   fixed; the per-instruction limit check was added later and should make
   the outer batching redundant — verify, don't assume).
3. Per-CPU event queue (`include/common/event_queue.h`): the singly-linked
   insert is O(queue length). Queue lengths are small (< 10), so this is
   not measurable today; leave it, but note `cpu_set_frequency`'s full
   re-sort runs on every DCO write.
4. Runner outer loop: hoist the `getenv`/`snprintf` console-injection scan
   (`runner :3324–3349`) to a one-time table; skip the JSON `has_test` block when
   `action_count == 0`.

### Tier 3 — longer execute slices (the real ceiling; NOT byte-identical, needs its own gate)

The 1 µs slice is Cooja's, and it is what makes every MSP430 simulation a
kernel benchmark. The conservative alternative is to let an active mote A run
until `min(its next CPU event, the next kernel event of any kind, its first
radio emission, now + cap)`.

Single-threaded, nothing can influence A before the next kernel event
*except* reactions to A's own transmissions, and the kernel does not bound
how fast those come. The bus schedules a receiver's `RX_BYTE` at the byte's
own emission stamp, clamped to now (`sim_radio_bus.c:351–357`), so a
receiver can react at the instant A emitted. A chip with no modelled TX
ramp-up (the nRF54L15 puts the whole frame on the air at TXEN) or a channel
switch changes what A's in-slice CCA/RSSI reads should see with zero delay,
so a byte-period lookahead is not safe. The safe rule is to **end A's slice
at its first radio emission** (any call into the bus's TX path): until then
no other mote can have reacted to anything of A's. The cap is then only a
performance knob, not a causality argument. Running on past an emission
would need the smallest modelled TX ramp-up of any receiver in range as the
bound, and that is zero today.

Events A schedules inside the slice (TX bytes) must be stamped with A's
in-slice time (`anchor + cycles − anchor_cycles`), not the slice start. That
last point
changes byte timestamps by up to the slice length and therefore changes
every simulation's output — it is *more* accurate than Cooja, not less, but
it must be validated the way the ARM JIT was (paired runs, TSCH association
+ held sync, RPL convergence, `check-baseline.sh` with an explicit
"expected to move" sign-off).

Payoff: the Sky chain's 11.8 M events would drop toward the ~150 k
peripheral events plus radio bytes; the MSP430 JIT (F13) becomes worth
re-enabling; the ARM `arm_step` budget-of-1 tail (§5.7b of the ARM plan)
disappears. Realistically 3–10x on MSP430 multinode, 1.1–1.3x on ARM.
This is a project (a week plus validation), not a fix, and it should be
done after Tiers 0–2 have taken the cheap wins and after F1 has removed the
synchronous delivery code that would otherwise interact with it.

### What is *not* on this list

- The ARM interpreter: covered by `arm-performance-plan.md`; nothing here
  changes its conclusions.
- Multithreading: the whole point of Phase 5 was retiring `--threads`; the
  gains above come from doing less work, not from parallelism.
- The radio medium filter: `radio_medium_filter_byte_radio` does not
  appear in any profile above 0.1%.
- The event queue's data structure, after PR #60. The heap is small and
  cache-resident, the remaining cost is per-call, and Tier 1 removes the
  calls. See Tier 2 item 1 for the measurement and the decision.

## 6. Refactoring plan

Ordered so each step is reviewable on its own and leaves the tree
byte-identical on `check-baseline.sh` unless stated.

### R1 — delete the synchronous chip-delivery subsystem (F1)

Remove from `sim_radio_bus.c`: `sim_radio_bus_deliver_bytes`,
`sim_radio_bus_queue_frame`, `sim_radio_bus_drain_rx`,
`sim_radio_bus_set_executing`, `emu_rx_queue`, `emu_rx_end_ns`, the
RXFIFO-full `step_until`, `rx_pre_sync`, the ACK flush loop and the
`frame_snap` statics; from the mote ops: `rx_byte_sync`, `rx_pre_sync`,
`rx_busy`, `rxfifo_available` (chip side), `RX_TICKING_STEP`,
`DRAIN_MINI_STEP`; from the chips: CC2420 `rx_incoming[]` replay, CC2538
ISTXON-time RXFIFO injection. Keep `BATCH` only as "stage bytes; hand the
whole frame to a `FRAME_CONSUMER` at frame-complete" and **assert at
registration** that a `BATCH` receiver has `receive_frame` and no
`rx_byte_sync`. Fix the header comment about nrf52840. Retire the
`SIM_RADIO_RX_DIRECT`/`QUEUED` outcomes and the "Emu RX frames … direct
/ queued / drained" stats line. Expected: `sim_radio_bus.c` loses ~350
lines and the runner's per-wakeup drain loop with them (this subsumes half
of Tier 0).

Gate: byte-identical on all nine baseline workloads (the code is dead for
them) **except** the end-of-run `Emu RX frames: … direct, … queued, …
drained, … dropped, … collided` line (`runner :3534`). R1 retires three of
its counters, the runner prints it on every run, and `check-baseline.sh`
does not filter it, so "byte-identical" cannot pass as written. Land the
line change as its own first commit (drop the retired counters, or move the
line behind `-v`) with the baseline diff shown to be that line only; then
the deletion is byte-identical against it. Plus `test_radio_bus` rewritten
to the surviving surface.

### R2 — JS mote as a frame consumer (F2)

Register JS with `SIM_RADIO_CAP_FRAME_CONSUMER`; deliver at frame end (the
bus's `accurate_tx_end`) rather than `now`, and make the same choice for
external motes explicit in `external-nodes-plan.md`. Add
`cross-level-demo.json` to the checked scenarios with a validator that the
JS node prints a received frame. Not byte-identical for JS scenarios (they
are currently broken); byte-identical elsewhere.

### R3 — chip ACK and PHY timing (F3, F4)

- CC2538: stage the ACK, emit from a chip event at +192 µs with a proper
  TX state for its air time; make `ISRFOFF` actually stop the parser
  (behind a compatibility flag if any test depends on it). The regression
  test is a **Sky→CC2538** unicast with AR, the direction that fails today
  (F3).
- nRF52840: 96 → 192 µs, derive from `cpu_freq_hz`.
- nRF54L15: PHYEND at real air time; retire the 100 µs constant. If
  `nrf54l15-ack-gap.md`'s ordering problem reappears, model ramp-up/down
  durations as that document proposes.
- CC1200 / bus: per-(node, radio) `tx_asm`, or a byte period declared at
  registration.
- ~~Native `SYNC`: wake at true frame end (F5)~~ done by PR #56.

Each moves the simulation and needs the `check-baseline.sh` "expected to
move" sign-off with the TSCH, RPL-UDP and TrustZone-RPL scenarios as the
functional gate (all must still associate/converge/round-trip).

### R4 — runner hygiene

- `now_ns` handling around the pump (F14): keep `now_ns` at the last
  dispatched event and pass the horizon as a parameter; audit the readers
  listed under F14.
- `msp_mote_serial_input` (F15): re-pin `last_execute_us` from kernel time.
- Remove `native_yield_callback` / `native_step_until_ns`.
- Fix `CSIM_PHASE_TIMING` labels (F16), or replace it with the event-kind
  counters from §4.1 (cheap, always-on, far more informative than a
  wall-clock split).
- Move per-process statics (F7) onto the owning structs.

### R5 — kernel API for Tier 1/3

Give `sim_runtime_run_until` a dispatch return value ("next wakeup for the
same mote, or none") so slice batching lives in the kernel rather than the
runner, and give the bus a `stamp_ns` argument so Tier 3 can hand it the
in-slice time. Both are additive.

## 7. Sequencing

State on 2026-09-27: Tier 0 item 3 (PR #61) and Tier 2 item 1 (PR #60) have
landed. Tier 0 items 1–2 waited for PR #56 (native CCA), which touches the
same per-wakeup loops; it merged 2026-09-26, so they can start. R1 comes
after Tier 0, not in parallel with it, since both change the per-wakeup
drain loop.

1. Tier 0 (a day) — ship first; it is the 5.4x and it is low-risk.
2. R1 (two to three days) — deletes the hazard and most of the remaining
   per-wakeup runner work; rewrites `test_radio_bus`.
3. R2 (half a day) — fixes a broken supported scenario.
4. Tier 1 + R5 (two days) — F9's horizon change first, gated on its own,
   then the batch; the second-largest win.
5. Tier 2 (one to two days, item by item, each gated).
6. R3 + R4 (a week, each item gated separately; these move the simulation).
7. Tier 3 (a week plus validation) — only after 1–6, with its own gate and
   a written "expected to move" argument.

## 8. Reproducing the measurements

```sh
make
# phase split (mislabelled, see F16) and per-run summary
CSIM_PHASE_TIMING=1 ./build/test_runner test configs/chain-4node-sky.yaml -q
# profiles: run long, sample 4 s
./build/test_runner test configs/chain-4node-sky.yaml -t 1500000 -q & sample $! 4 1 -mayDie -file sky.txt
./build/test_runner test configs/udgm-100node-grid-sky.json -q & sample $! 5 1 -mayDie -file grid.txt
# event counts: add a per-kind counter in sim_runtime_run_until and a
# callback histogram (dladdr) in execute_events — the scratch patch used
# for §4.1 is ~30 lines and should become a permanent CSIM_EVENT_STATS=1
# (Tier 2 item 4 / R4).
# determinism gates for every change
tools/check-determinism.sh test configs/chain-4node-sky.yaml
tools/check-baseline.sh
```
