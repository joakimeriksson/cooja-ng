# Kernel and radio-path review, with a performance and refactoring plan

Status: **review complete; Tier 0 item 3 and Tier 2 item 1 are in PRs #61 and #60** (2026-09-26). Every number
below was measured on this machine (Apple Silicon, stock `make`, GNU Lightning
present) against `main` at `252fc0e`; every code claim carries a file:line.
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
1.0x real time with only 9% of samples in the interpreter.

**Two byte-identical changes measured in a scratch copy** (skip the runner's
per-wakeup all-nodes loops when they cannot matter; stop installing a debug
PC-trace hook on every MSP430 node):

| Workload | Stock | Prototype | |
|---|---|---|---|
| `udgm-100node-grid-sky`, 20 s sim | 12.55 s | **2.34 s** | **5.4x** |
| `chain-4node-sky`, 180 s sim | 0.685 s | **0.551 s** | 1.24x |
| `chain-4node-cc2538dk`, 180 s sim | 0.666 s | 0.633 s | 1.05x |

stdout (every timestamped console line) was identical on all three.

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
| No `sim_eq_schedule*` caller outside the kernel; runner only `sim_eq_init` + read-only `peek_time` | grep; `test/test_mixed_multinode.c:2961, 2990, 3107` | clean |
| Past-time wakeups clamped, counted, warned once | `sim_runtime.c:105–120` | clean |
| Mote never writes `now_ns`; slice re-pins `sim_time_ns` to kernel time at both ends; overshoot ≤ 1 instruction and debited from the next slice | `src/motes/msp430_elf_mote.c:589–595`, `arm_elf_mote.c:435–446`, `msp430_cpu.c:906–942` | clean, MSPSim-faithful |
| Idle mote's wakeup derived from its CPU event-queue head; active mote sliced 1 µs (Cooja `MspMote.execute(t,1)`) | `msp430_cpu.c:973–984`, `arm_cpu.c:4581–4594`, `mote_impl.h:111–115` | clean (and the performance story, §4) |
| Every emulated chip registers `PER_BYTE`: one `SIM_EV_RX_BYTE` per on-air byte at `first_byte + n·period`, clamped ≥ now | `msp430_elf_mote.c:543–547`, `arm_elf_mote.c:392–397`, `sim_radio_bus.c:280–286` | clean |
| RX entry is the kernel: `execute(t,0)` → `receive_byte` → same-time wakeup | runner `deliver_rx_byte` :618–676 | clean, Cooja `MspMoteTimeEvent` |
| TX byte stamp is the kernel's `now`, never the mote-local clock | `sim_radio_bus.c:349–370` | clean, Cooja radio-callback semantics |
| No chip TX callback runs from inside a JIT block; `cpu->cycles` current at every IO callback | `msp430_jit.c:334–347`, `arm_jit.c:246–300, 664–694` | clean |
| No wall-clock/host time or libc `rand` on the radio path; medium RNG is seeded xorshift | grep; `radio_medium.c:58–63` | clean |
| Native→native frames queued at end-of-air-time through the kernel | `sim_radio_bus.c:854–915`, `native_cooja_mote.c:366–389` | clean |
| Medium filter: link block, spectrum/channel pair, one probabilistic roll per (sender, receiver, frame) | `radio_medium.c:599–644` | clean |

## 2. Findings — radio data delivery

Severity: **A** = wrong behaviour observable in a supported scenario;
**B** = timing deviation from hardware that firmware can notice; **C** = dead
or misleading code that is a hazard, not a defect.

### F1 (C, large) — the synchronous chip-delivery subsystem is dead code

`sim_radio_bus_frame_complete()` (`src/sim/sim_radio_bus.c:590–821`) still
carries the pre-Phase-5 delivery model: `sim_radio_bus_deliver_bytes()`
(`:444–492`) steps *another* mote's CPU per byte through `rx_byte_sync`,
`frame_complete` mini-steps a receiver `+5000` cycles when its RXFIFO is
full (`:720–725`), `drain_rx` mini-steps (`:513–521`), the `emu_rx_queue`,
`rx_pre_sync`, the `executing_node` guard, the `RX_TICKING_STEP` /
`DRAIN_MINI_STEP` caps, and a second auto-ACK flush with its own turnaround
model (`:750–791`, `ack_start = tx_end + 192 µs`, `sender_ack_start = now +
192 µs`).

All of it only runs for `BATCH` receivers, and the only `BATCH` receivers
(JS, external, Renode) have no `rx_byte_sync`. The header still says
nrf52840 is `BATCH` (`include/sim/sim_radio_bus.h:139–142`); it is not
(`arm_elf_mote.c:392`). The CC2420's `rx_incoming[]` replay
(`src/chips/cc2420.c:398–406`) is equally dead: `rx_incoming_count` is only
ever zeroed. The CC2538's ISTXON-time direct-to-RXFIFO ACK injection
(`src/arm/cc2538_rfcore.c:175–235`) cannot fire under `PER_BYTE` either.

Why it matters: the re-entrant ACK path never feeds the assembler, so a
re-entrant ACK would be stamped preamble@now, SFD@now+32 µs with no
turnaround (`:349–364` + `:382–388`); and `deliver_bytes` runs one mote's
CPU from inside another's slice — exactly the invariant the kernel exists to
enforce. One `SIM_RADIO_DELIVERY_BATCH` registration on a chip mote brings
all of it back. The runner also spends real time on it (§4, F8).

### F2 (A) — JS app motes are deaf to emulated senders

`src/motes/js_app_mote.c:57–61` registers `BATCH` with `caps=0` and
`rxfifo_available` returning 0. In `frame_complete` that forces a
`step_until` on the JS mote and a `queue_frame` into `emu_rx_queue[js]`,
which `drain_rx` can never deliver (no `rx_byte_sync`). Measured on
`configs/cross-level-demo.json`: `Emu RX frames: 0 direct, 64 queued, 0
drained, 3 dropped`. The header already describes this failure mode
(`sim_radio_bus.h:166–173`). `js_mote_receive_frame` is only reached from
the frame-level path (native/JS/external senders). Fix: give JS
`SIM_RADIO_CAP_FRAME_CONSUMER` like external and Renode motes, and decide
whether the JS mote should see the frame at air-time start (today's
`receive_frame(now)` semantics, `js_app_mote.c:116–123`) or end.

### F3 (B) — CC2538 auto-ACK: synchronous, no turnaround, no TX state

`src/arm/cc2538_rfcore.c:828–850`: on the last payload byte the 11 ACK
bytes are pushed through `tx_callback` from inside the kernel `RX_BYTE`
dispatch. The bus stamps the ACK preamble at `now` = last data byte's air
time, so the ACK is on the air 32 µs *before the data frame ends* and
192 µs earlier than hardware; the chip stays in `SFD_WAIT` (`:853–855`)
during its own ACK, so it keeps receiving while "transmitting"; TXACKDONE
is raised at once. It works between two CC2538s because the sender's radio
is still in TX and buffers the early bytes (`:95–107`). It cannot work
toward a CC2420 sender, which drops bytes outside `RX_SFD_SEARCH`/`RX_FRAME`
(`cc2420.c:583`): CC2538→Sky unicast with AR is never acknowledged. Fix:
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
  (`sim_radio_bus.c:368–370`) and `subghz` only flips on the 4th sync byte
  (`:38–42`), so the first 8 bytes of a node's first sub-GHz frame (and of
  the first after a 2.4 GHz frame on a dual-radio Firefly — `tx_asm` is
  per node, not per radio) are stamped 32 µs apart, then jump ~1 ms.
  Harmless to reception (no `rx_stall` on those motes), wrong for the
  timeline and collision windows. Fix: per-(node, radio) assembler, or let
  the chip declare its byte period at registration.

### F5 (B) — emulated→native frames complete before their air time

`SYNC` delivery feeds each byte to the native assembler synchronously inside
the sender's TX call (`sim_radio_bus.c:271–275` →
`src/native/native_radio.c:90–145`); on the last byte the frame is queued
*as already ended* using the receiver's stale clock (`:135–140`), and the
runner wakes the native at the sender's event time. The bus meanwhile stamps
the same bytes on the air up to `(len+6)·32 µs` later. A native mote can
therefore consume a frame ~4 ms before it has finished arriving. The
comment shows the shortcut is deliberate; the timing gap is not documented.
Fix: schedule the native wakeup at the true frame end (the bus knows it),
as native→native already does.

### F6 (B) — CC2538 receives while RF is off; CCA never busy

`cc2538_rfcore.c:244–252` keeps the parser live after `ISRFOFF`
(`:691–693` "simulates perfect reception even when … turned the radio
off"); `ISTXONCCA` always transmits and `FSMSTAT1.CCA` is always clear
(`:68, :239–242, :381`); nRF52840 CCA is always idle (`:1186–1190`). CC1200
CCA does consult the bus's `tx_busy_until` (`runner :804–835`). Documented
modelling gaps; listed here because they are the remaining "receiver never
misses" shortcuts.

### F7 (C) — smaller items

- `deliver_rx_byte` schedules `if_earlier(next)` then `if_earlier(t)`
  (`runner :674–675`); the second always wins.
- `frame_complete` uses `static` snapshot arrays (`sim_radio_bus.c:633–636`)
  — safe only because `tx_depth` prevents nesting; belongs on the bus.
- Per-process statics that break a second runtime: CC2420 stat counters
  (`cc2420.c:138–148`), CC2538 `rxfifo_overflow_count` (`:23`), nRF54L15
  `trigger_task` depth guard (`:1517–1519`), MSP430 PC-trace counters
  (`msp430_elf_mote.c:82–85`), `sim_service.c:74 dispatch_depth`,
  `sim_runtime.c:258, 270–271` spin diagnostics.
- `arm_elf_mote.c:373–374` says nRF54L15 has an `rx_incoming` buffer; it
  has none (`nrf54l15_soc.c:1822–1829`).
- `native_yield_callback` / `native_step_until_ns` (`runner :1302–1362`,
  `native_node.c:345–362`) would run other natives' ticks and hand an ACK
  back with zero air time — dead today (no caller reaches native
  `step_until`), a landmine.

## 3. Findings — event-queue usage and time-keeping

### F8 (perf, A-class for scale) — O(N) work on every `NODE_WAKEUP`

`dispatch_mote_wakeup` (`runner :1958–2036`) runs four loops over all
nodes on every wakeup: a native rx-count snapshot (`:1969–1978`), a
native got-frame scan (`:1997–2015`), `emu_rx_queue_drain(r)` for every
emulated node (`:2023–2026`, a bus call each — 16% of samples on the grid),
and `mixed_deliver_rf_bytes(r)` for every native (`:2029–2032`, dead work:
natives are `SYNC`, `rf_pending` is only staged for `BATCH`). With the
1 µs slicing (§4) this is `4·N` loop bodies per active microsecond per
mote — O(N²) in the node count. This is the single largest cost outside
the interpreters; the prototype removes it (Tier 0).

### F9 (perf) — one outer-loop iteration per distinct event time

Headless, the horizon is `min(end, next_event)` (`runner :3107–3126`), so
the pump drains only same-time events and the whole outer body (action
check, JS engine check, service-poll guard, console-injection scan,
radio-state guard, JSON step timeout, progress tick) runs once per event
timestamp: 10.2 M outer iterations for 11.8 M events on the Sky chain.
~6–11% of samples are `run_mixed_multinode_test` self time.

### F10 (perf) — heap churn

`sim_eq_schedule_gen` (`src/common/sim_event_queue.c:93–131`) implements
"replace the pending wakeup" as remove + sift-up + sift-down + insert; every
`if_earlier` that fires pays the same. `sim_eq_pop` copies a 40-byte struct
by value twice. Together 12% of Sky-chain samples, 13% on the grid.

### F11 (perf) — per-instruction debug hook on every MSP430 node

`msp430_elf_mote_install_pc_trace` is called for every MSP430 node in every
run (`runner :2791–2797`), installing `srh_trace_cb`
(`msp430_elf_mote.c:87–98`) which the interpreter calls per instruction
(`msp430_cpu.c:1081–1082`). The callback compares against **hardcoded
firmware addresses `0xcb32` and `0xb138`** from one historical TSCH image
— it is meaningless for any other firmware — and feeds three counters that
appear only in the end-of-run `FW cc2420_transmit=… eb_process=…` line.
1.1% self time plus the indirect-call cost inside the hot loop; part of the
1.24x in the prototype. Should be opt-in (`CSIM_PC_TRACE=1`) or removed.

### F12 (perf) — the 1 µs slice costs a function call per instruction

`msp430_step_until` (`msp430_cpu.c:906–942`) and `arm_step_until`
(`arm_cpu.c:4514–4547`) single-step (`steps = 1`) once fewer than 10
cycles remain, i.e. for essentially every active-mote slice (4 cycles at
4 MHz). The interpreter already stops at `cycle_limit` per instruction
(`msp430_cpu.c:1019–1020`), so the outer batching is redundant with it; the
ARM plan's §5.7b measured 79.8% of `arm_step` calls arriving with a budget
of 1. `msp430_step_until` self + `msp430_step` = 7% of Sky samples.

### F13 (design) — MSP430 JIT is switched off in multinode

`msp430_elf_mote.c:305–312` frees the JIT cache "because the scheduler
steps in ~1 µs increments". True today; it is a consequence of the slicing
model, not a fixed fact, and it means the only accelerated ISA in the tree
runs unaccelerated in every multinode simulation.

### F14 (correctness, minor) — `now_ns` pinned to the horizon before the pump

`runner :3126` sets `sim_rt.now_ns = sim_ns` (the horizon) before actions,
JS gen-msgs and service polls run, then the pump moves it back to the first
event. Headless the horizon *is* the next event; with the UI (+100 ms),
serial/pacing (+1 ms), the shell (+1 s) or a Renode quantum, anything that
reads `sim_runtime_now_ns` in that window (`sim_control_send` wake,
`native_cooja_mote.c:318–321`, shell wakes `shell_commands.c:921, 1876`,
Renode UART inject `renode_mote.c:84–85`) stamps a time later than every
pending event, and `now_ns` is non-monotonic within an iteration. The
header documents it as intended (`sim_runtime.h:315–317`); the consequence
(injected input up to a slice late in interactive modes) is not.

### F15 (correctness, minor) — MSP430 serial injection with clock deviation

`msp_mote_serial_input` (`msp430_elf_mote.c:734–753`) steps the CPU and
then sets `last_execute_us` from raw cycle time. With `clock_deviation ≠ 1`
raw cycle time ≠ kernel time, so the next slice's `jump_us` is wrong and
the mote over-steps. Only MSP430 + serial injection + deviation.

### F16 (labels) — `CSIM_PHASE_TIMING` mislabels

`time_step` brackets the whole `sim_runtime_run_until` including dispatch,
bus delivery and the O(N) loops (`runner :3288–3311`), but prints as "step
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
`sim_eq_pop` 7% · `sim_eq_schedule_gen` 6%. **Interpreter 9%.** 1.0x real
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
nine workloads byte-identical except the three wall-clock lines) plus the
radio-bus/radio-medium unit suites. Tiers 0–2 are semantics-preserving by
construction; Tier 3 is not and gets its own gate.

### Tier 0 — remove O(N) work from the wakeup path (**measured 5.4x on 100 nodes, 1.24x on Sky chain, byte-identical**)

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
4. Drop the redundant second `if_earlier` in `deliver_rx_byte`.

Cost: a day. Risk: nil (prototype diffed clean on 2686 + 173 output lines).

### Tier 1 — pump-internal same-mote slice batching (est. 1.3–1.6x on MSP430 workloads, ~1.05–1.1x on ARM; byte-identical)

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

Requires: `sim_control_note_event` (`step N` budget) still counted per
slice; `radio_state_tracking` / UI polling still run per outer iteration —
bound the batch to the outer horizon so those cadences are unchanged.

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
   (`:3317–3342`) to a one-time table; skip the JSON `has_test` block when
   `action_count == 0`.

### Tier 3 — longer execute slices (the real ceiling; NOT byte-identical, needs its own gate)

The 1 µs slice is Cooja's, and it is what makes every MSP430 simulation a
kernel benchmark. The conservative alternative is to let an active mote run
until `min(its next CPU event, the next kernel event time for any other
mote, now + lookahead)`. Single-threaded, nothing can influence mote A
before the next kernel event *except* A's own transmissions, whose replies
(ACKs, CCA-visible bytes) cannot arrive earlier than one byte period after
A's byte leaves — so `lookahead = 32 µs` (one 2.4 GHz byte period, 160 µs
sub-GHz) is safe for radio causality, while events A schedules inside the
slice (TX bytes) must be stamped with A's in-slice time
(`anchor + cycles − anchor_cycles`), not the slice start. That last point
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
them), plus `test_radio_bus` rewritten to the surviving surface.

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
  (behind a compatibility flag if any test depends on it).
- nRF52840: 96 → 192 µs, derive from `cpu_freq_hz`.
- nRF54L15: PHYEND at real air time; retire the 100 µs constant. If
  `nrf54l15-ack-gap.md`'s ordering problem reappears, model ramp-up/down
  durations as that document proposes.
- CC1200 / bus: per-(node, radio) `tx_asm`, or a byte period declared at
  registration.
- Native `SYNC`: wake at true frame end (F5).

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

1. Tier 0 (a day) — ship first; it is the 5.4x and it is risk-free.
2. R1 (two to three days) — deletes the hazard and most of the remaining
   per-wakeup runner work; rewrites `test_radio_bus`.
3. R2 (half a day) — fixes a broken supported scenario.
4. Tier 1 + R5 (two days) — second-largest byte-identical win.
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
