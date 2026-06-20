# Plan: Zephyr 802.15.4 communication on Cooja-NG

Goal: **two Zephyr nodes exchange a UDP packet over 802.15.4** on csim's
emulated nRF52840 — single-hop echo (`echo_client` → `echo_server` → reply),
no Thread/mesh. Thread (OpenThread) is a later step on top.

## Where we are (verified this session)

Working:
- Stock Zephyr **boots and runs** on csim's nRF52840 — kernel, timers (RTC1),
  threads, PendSV context switch (single- and multi-thread samples print).
- **Performance is solved** (commit `1ab5278`): tickless RTC + WFI horizon cap →
  Zephyr net firmware runs ~**119× real-time** (echo_server 3 s sim in 25 ms
  wall). Iteration is now instant.

Two gates remain:
1. **Console** — `printf` samples print via `uart_poll_out` (modeled), but
   LOG/net firmware (echo, OpenThread) emits 0 bytes: their console uses the
   **interrupt/async UARTE TX** path csim doesn't model. We're blind on net
   firmware output.
2. **Radio** — echo touches `RADIO` 0 times; net init blocks before it. The
   `nrf_802154` driver needs csim's RADIO to satisfy its hardware choreography.

## Strategy: behavioural emulation, not register-accurate PPI

csim's existing nRF RADIO model (for Contiki) is **behavioural**: it shortcuts
the hardware state machine and directly sets the events firmware waits for
(instant TX, synchronous auto-ACK). We follow the same philosophy for
`nrf_802154` rather than emulating PPI/DPPI + TIMER cycle-accurately:

> For each thing the driver *waits on*, make csim satisfy it — set the event,
> complete the transfer, deliver the frame — special-casing the specific
> event→task links the driver wires up (exactly how UARTE's `ENDTX→STOPTX` PPI
> was handled: csim sets `TXSTOPPED` directly instead of modelling PPI).

This trades fidelity (real ACK-turnaround timing) for tractability. The risk is
timing-sensitive driver loops (mitigations below). Generic PPI modelling stays
a fallback for cases the behavioural shortcut can't cover.

## Phases (each ≈ one focused session)

### Phase 0 — Console visibility (prerequisite, smallest)
Without output we're blind. Two options:
- **Fast (config):** build echo with the console forced to polling
  (`CONFIG_UART_INTERRUPT_DRIVEN=n`, or a UART-poll log backend). If that makes
  output appear, use it for all bring-up debugging — no emulator work.
- **Proper (model):** model the interrupt/async UARTE TX path. Trace the UART
  page (`0x40002xxx`) with `ARM_MMIO_TRACE`/`NRF_RADIO_TRACE`-style logging to
  see the exact sequence (likely FIFO fill + `TASKS_STARTTX` + `ENDTX` IRQ
  chaining, possibly multi-buffer via `TXD.PTR` list). Extend the UARTE model
  (deliver bytes, raise `ENDTX` IRQ, handle the next buffer).
- **Verify:** echo prints its boot banner + `net`/`802154` debug logs.
- **Recommendation:** do the config workaround first to unblock Phases 1–4;
  circle back to the proper async-UART model before shipping a demo.

### Phase 1 — `nrf_802154` init reaches the RADIO
With console visible, find where net init blocks (debug logs + spin PC). Model
the peripherals the driver polls during init, found one-at-a-time via
`ARM_MMIO_TRACE`. Expected suspects:
- **TEMP** (`0x4000C000`) — `nrf_802154` does periodic temperature measurement
  for radio calibration; `TASKS_START` → set `EVENTS_DATARDY` + a plausible
  `TEMP` value.
- A **dedicated TIMER** for the driver's timeslot/ACK timing (TIMER0–4 — already
  modelled; confirm the driver's instance works).
- Any clock/`HFCLK` status the driver waits on (HFCLKSTAT already modelled).
- **Verify:** the driver progresses to configuring the RADIO (first RADIO
  register writes appear under `NRF_RADIO_TRACE`); net interface comes up.

### Phase 2 — Single-node TX (a frame goes on-air)
The driver arms the RADIO and triggers a transmit. csim already emits an IEEE
802.15.4 frame from `PACKETPTR` on `TASKS_START` (Contiki path). For
`nrf_802154`:
- Trace the **SHORTS** and **PPI** (`0x4001f000`) writes the driver makes for
  TX, plus the task/event order (`TXEN`→`READY`/`TXREADY`→`START`→`PHYEND`).
- Behaviourally complete the TX and fire the events/SHORTS the driver chained
  (e.g. `PHYEND`, `PHYEND→DISABLE`), and the specific PPI event→task links
  (special-cased like UARTE).
- **Verify:** node 1's `radio_tx_cb` fires; bytes reach the radio medium
  (`Total RF bytes` > 0).

### Phase 3 — Single-node RX + hardware auto-ACK
csim's medium delivers bytes to node 2's RADIO (`nrf_radio_receive_byte`,
already wired). `nrf_802154` expects:
- `FRAMESTART`/`END`/`CRCOK` events (modelled) + **hardware address filtering**
  (the driver programs the FFSM-equivalent; may need the filter check).
- **Hardware/enhanced ACK**: `nrf_802154` relies on the radio auto-generating
  the ACK (the imm/enh-ACK generator). csim has a *software* auto-ACK in the RX
  path (for Contiki) — adapt it to what `nrf_802154` expects (ACK frame built
  from the radio's RAM / the enh-ACK data).
- **Verify:** node 2's L2 receives the frame, 6LoWPAN/IPv6/UDP decode, the ACK
  returns; node 1 sees the ACK (no retransmit storm).

### Phase 4 — Two-node echo end-to-end
`echo_client` UDP → `echo_server` → reply, including neighbour discovery
(NS/NA) over 802.15.4 and 6LoWPAN.
- **Verify:** both consoles show the request/echo round-trip (the success
  string the sample prints). Bundle the two ELFs as
  `firmware/nrf52840-dk/zephyr-echo-{server,client}.nrf52840-dk` + a doc, the
  way the hello_world/synchronization examples are bundled.

### Later — Thread (OpenThread)
Mostly "more radio + lots of compute that already works", plus the **QSPI**
settings backend (`0x40029000`) for Thread credentials (model `EVENTS_READY` on
task; JEDEC-ID can fail gracefully → RAM-backed settings). Build verified;
gated on Phases 1–3.

## The PPI question
`nrf_802154` leans on PPI/DPPI (event→task routing for ACK-turnaround timing).
Order of preference:
1. **Behavioural shortcut** (default): when csim sets an `EVENTS_*` that the
   traced PPI config links to a task, trigger that task directly. Cheap,
   matches the existing model.
2. **Minimal generic PPI** (fallback): store `CH[n].EEP`/`TEP`; on any event
   publish, fire the subscribed task. ~50 lines, reusable, only if the
   behavioural shortcut proves too brittle.

## Tooling (have / add)
- Have: `ARM_MMIO_TRACE` (unmapped peripherals), `NRF_RADIO_TRACE` (radio regs),
  `ARM_EXC_TRACE` (ISR/context), the `program_counter` op + `addr2line`.
- Add: a **PPI trace** (which event→task links are configured) and a UART-TX
  trace for Phase 0.

## Risks & mitigations
- **ACK-turnaround timing** — `nrf_802154` expects the ACK within ~192 µs via
  TIMER+PPI; csim's instant model may arrive "too early/late" for the driver's
  state machine (same class as the nrf54l15 deferred-PHYEND issue). *Mitigation:*
  reuse the deferred-event trick (fire PHYEND/ACK a fixed sim-delay later so the
  driver's critical section exits first).
- **Hardware auto-ACK format** (imm vs enh-ACK). *Mitigation:* start with
  ACK-not-required frames (no ACK path) to prove the data round-trip, then add
  ACK.
- **Perf once the radio is active** (more events). *Mitigation:* the tickless
  RTC + WFI cap already landed; watch the radio event rate.
- **Two-node medium routing for nRF** — csim's radio_medium already routes
  nRF52840 Contiki nodes, so this should carry over.

## Milestones
| M | Deliverable | Verify |
|---|---|---|
| 0 | echo prints over UART | boot banner + debug logs visible |
| 1 | `nrf_802154` reaches RADIO | first RADIO writes; iface up |
| 2 | node TX | `Total RF bytes > 0` |
| 3 | node RX + ACK | frame decoded; no retx storm |
| 4 | two-node UDP echo | request/echo round-trip in both consoles |
| 5 | OpenThread single node | `ot state` shell responds |

Estimated 2–4 sessions to M4 (the basic-comms goal); M5 a further effort.
