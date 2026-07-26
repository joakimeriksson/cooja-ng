# Changelog

All notable changes to Cooja-NG (`csim`) are documented here. The format is
loosely based on [Keep a Changelog](https://keepachangelog.com/); this project
uses [Semantic Versioning](https://semver.org/) once it reaches 1.0 — until
then, 0.x minor releases may adjust the CLI, config, and plugin ABI.

## [0.1.0] — 2026-07-25

First public release. A fast, multi-architecture C re-implementation of the
parts of Cooja and MSPSim needed to run the upstream Contiki-NG test suite
headlessly, with a focus on simulation speed, deterministic timing, and
faithful peripheral behaviour.

The feature set is described under *Emulation* / *Simulation kernel* /
*Plugins* / *Tooling & tests* below; the `Hardening` sections record a full
pre-release subsystem audit (correctness, memory-safety, robustness) whose
fixes are included in this release. See
[`docs/design/release-0.1.1-hardening.md`](docs/design/release-0.1.1-hardening.md)
for that audit and its plan.

### Emulation
- **MSP430 / MSP430X** CPU (computed-goto interpreter + optional GNU Lightning
  JIT), MCU configs F149 / F1611 / F2617 / F5437 / CC430F5137 / FR5969, with
  GPIO, USART/eUSCI, Timer A/B, BCS/UCS/CS clocks, ELF loader, and the CC2420
  radio.
- **ARM Cortex-M3/M4/M33** interpreter (Thumb-2 + M4 DSP/VFP) with CC2538
  (on-chip RF Core), Zolertia Firefly (+ CC1200 sub-GHz), and Nordic nRF52840 /
  nRF54L15 (on-chip 802.15.4) platforms.
- **RISC-V (RV32EMC)** via the nRF54L15 FLPR coprocessor: the M33 loads the FLPR
  blob into shared SRAM and releases it through the VPR `CPURUN` register, after
  which the RV32EMC core runs **unmodified Contiki-NG** dual-core alongside the
  M33 over one address space. ISA `rv32emc_zicsr_zifencei` (base + M + C + CSR +
  `fence.i`); both cores idle in WFI. See
  [`docs/design/riscv-vpr-plan.md`](docs/design/riscv-vpr-plan.md).
- **Native Cooja motes** (`dlopen`) and **JS app motes** (QuickJS).
- **Multi-RTOS**: Contiki-NG is the primary, fully-validated target; csim also
  boots stock **Zephyr OS** (incl. 802.15.4 `echo_server`/`echo_client` over
  UDP) and **RIOT OS** (`gnrc_networking` forming a 2-node RPL DODAG) on the
  nRF52840 — experimental / best-effort. The nRF52840 model grew the fidelity
  these need: UARTE EasyDMA RX, the TEMP sensor, RADIO/TIMER/RTC behaviour, and
  Cortex-M4 ops (PLD/PLI hint, parallel UADD8/SADD8 + SEL with APSR.GE). See
  [`docs/zephyr.md`](docs/zephyr.md) and [`docs/riot.md`](docs/riot.md).

### Simulation kernel
- Single-threaded, event-driven kernel (`sim_runtime_t`): ns-precise clock,
  unified `(time, seq)` event queue, per-radio multi-channel medium, per-byte
  RF delivery, and a Cooja-compatible execution model.
- **Ports-and-adapters architecture**: CPUs, radio chips, propagation policies,
  observation features, and plugins are adapters behind small vtable ports; the
  kernel never branches on a concrete node type.
- Pluggable **radio medium** policy (UDGM / NONE / plugin), and **power-aware
  range** matching Cooja UDGM (range scales with the transmitter's output
  power; byte-identical for firmware that holds max PA).

### Plugins
- `dlopen` plugin ABI (`csim_plugin.h`), additive/version-gated:
  v1 register a service, v2 register a radio medium, v3 draw a live web-UI panel
  (`publish_panel`).
- Plugins may also be **compiled in** as built-in services and selected by
  config name (`"plugins": ["energest"]`), Cooja's built-in-plugin style.
- Bundled examples: `packet_sink` (service `.so`), `lossy_medium` (medium
  `.so`), and **energest** — a compiled-in energy estimator (per-mote radio
  duty cycle + CPU/LPM/TX/LISTEN energy, with a live UI panel).

### Tooling & tests
- JSON simulation configs (v1 + v2), a live WebSocket UI, PCAP capture,
  activity timeline, per-mote GDB stub, and a JS/JSON test engine.
- Passes the upstream Contiki-NG Cooja test suite via `tools/run-cooja-tests.sh`
  — **93 / 93**, 0 failed / 0 skipped: all 85 headless tests plus all 8
  TUN/border-router cases (those need `--with-tun` and root). Plus instruction,
  firmware, radio-medium/bus, chip-driver, and plugin unit suites.
- CI on Linux (gcc) and macOS (clang) gating the unit suites plus nRF52840
  networking (Contiki RPL-UDP on DK + Dongle, stock Zephyr 802.15.4 echo) and
  determinism / config-equivalence guards.

### Hardening — memory safety
- **WebSocket server**: validate the frame payload length before the buffer
  arithmetic. A 64-bit attacker-controlled length folded into a signed `int`
  truncated negative, passed the "have we buffered the whole frame?" guard, and
  drove a `memmove` out of bounds — a crash triggerable by any connected
  client. Oversized/unmasked frames are now rejected.
- **ELF loader**: bound-check segment routing without adding, so a crafted
  `p_paddr` near `UINT32_MAX` can't wrap past the region end and return a wild
  destination pointer (heap corruption on load). `elf_find_symbol` validates
  `sh_link` and caps the strtab allocation against a malformed-symtab DoS.
- **Native Cooja motes**: clamp every firmware-controlled / medium-supplied
  frame length to the 128-byte radio buffers (the direct RX fast path had no
  clamp), and guard the firmware-set log length.
- **Config loader**: guard an unchecked `ftell` (a directory path gave
  `malloc(0)` + `fread(SIZE_MAX)`).
- **Native dlopen**: use `mkstemp` for the per-node temp copy instead of a
  predictable `/tmp` name opened without `O_EXCL` (symlink / planted-library
  vector on shared hosts); release the handle and temp file on load failure.

### Hardening — correctness
- **MSP430 `DADD`**: real per-nibble BCD addition with the carry flag, replacing
  a plain binary add that produced wrong sums and never set carry (shared helper
  so the interpreter and decoded paths can't diverge).
- **MSP430 `RRCM`**: carry-out now comes from the last bit rotated out (was off
  by two vs the sibling rotate instructions).
- **MSP430 Timer Up/Down (MC=3)**: compare and overflow events scheduled on the
  real 0→CCR0→0 triangle instead of the continuous-mode wrap.
- **MSP430 JIT**: exclude `ADDC` from inlining (its carry-in left no register to
  compute the overflow flag — a warm-block-only divergence); self-modifying-code
  cache invalidation now frees every block whose actual byte span covers the
  write, not just a fixed 6-byte window.
- **ARM NVIC**: PendSV and SysTick no longer share a single pending slot (a
  SysTick firing before a pended PendSV was taken silently dropped the context
  switch); NVIC IPR word reads are clamped at the array end.
- **CC2538 GPTimer**: timeouts raise the NVIC interrupt and periodic mode
  reloads (was poll-only, so a WFI waiting on a GPTimer IRQ wedged).
- **RISC-V (nRF54L15 FLPR)**: WFI resumes on a pending enabled interrupt
  regardless of `mstatus.MIE` (spec behaviour; the old code could deadlock the
  coprocessor); machine-interrupt priority corrected to MEI > MSI > MTI.
- **nRF54L15**: per-node DPPI/timer/EGU binding storage — three file-scope
  tables were shared across SoC instances, so in a multi-node run one node's
  callback could be routed to another's radio/timer.
- **Radio medium / event queue**: clamp `node_count` to the array bound; a
  full-queue reschedule now replaces the node's wakeup instead of dropping it.

### Hardening — robustness
- Service dispatch re-entrancy guard enforced in release builds (was
  assert-only); pcap writer checks every write and stops on a short write
  instead of emitting a corrupt capture; energy-panel JSON never truncates
  mid-structure; firmware→board detection keys on the basename's extension;
  external-command service cleans up on `fork` failure. The firmware test
  harness no longer reports success for a firmware that never self-reports —
  a hung or instruction-starved run printed `WARN` and returned 0, making it
  indistinguishable from a pass in the suite's exit status.

### Hardening — nRF54L15 radio (T3)
- **Two-node nRF54L15 802.15.4 now routes end-to-end.** The radio's TX-completion
  event was scheduled off the lagging `sim_time_ns`, so PHYEND fired ~1 cycle
  after START instead of ~100 µs later; the whole TX collapsed into one cycle and
  the driver's DPPI TXEN/START fan-out emitted each frame twice — two SFDs on air,
  so a per-byte receiver mis-latched the second SFD as the PHR and every frame
  failed CRC. Scheduling `tx_end_event` in cycles fixes it. Regression test:
  `configs/test-2node-nrf54l15-dk.json`.

### Hardening — nRF multi-hop 802.15.4 (nRF54L15 + nRF52840)
- **3+ node RPL-UDP chains now route end-to-end on both nRF radios.** The bug was
  in the radio model, not 6LoWPAN forwarding: when a reception was aborted
  mid-frame (routine on a multi-hop router that hears two neighbours and gets
  collision-truncated frames), the abort paths fired only a non-interrupting
  `PHYEND`, so the `nrf_802154` driver's `psdu_being_received` flag — set on
  ADDRESS/FRAMESTART, cleared only by a CRCOK/CRCERROR with its RX IRQ — stayed
  set forever. Every later `nrf_802154_transmit_raw` then returned
  `BUSY_CHANNEL` (`psdu_being_received_now`) and the router could never
  TX/ACK/forward again. Single-hop never hit this (no collisions → no aborts).
  - **nRF54L15** (`nrf54l15_soc.c`): the RX-stall watchdog now fires
    `END+PHYEND+CRCERROR` instead of bare `PHYEND`.
  - **nRF52840** (`nrf52840_soc.c`, `arm_elf_mote.c`): a `radio_abort_inflight_rx`
    helper fires the terminal CRCERROR on an invalid-PHR-after-SFD, on a
    STOP/DISABLE that interrupts a frame, and via a newly-wired `rx_stall` op.
    Additionally, the fabricated auto-ACK now checks the frame's extended
    destination address against this node's `FICR.DEVICEADDR0` — previously every
    neighbour ACKed every unicast, so two ACKs collided at the sender and it
    retransmitted until it gave up.
  - Regression tests: `configs/chain-3node-nrf54l15-dk.json`,
    `configs/chain-3node-nrf52840-dk.json`,
    `configs/chain-4node-nrf52840-dk.json` (+ `-dongle`; node 4 relays 3 hops).
    No regressions to nRF52840 2-node RPL, TSCH, Zephyr echo, nRF54L15 2-node,
    FLPR dual-core, or the cc2538/sky controls.

### Hardening — serial socket (native border router)
- **`17-tun-rpl-br/09-native-border-router-cooja-frag` now passes; the Cooja
  suite is 93/93.** A 1200-byte ping through `border-router.native` used to get
  5 transmitted / 0 received, with the router dying on `slip_send overflow`.
  The cause was a race against a 31 ms window that csim lost by being too fast.
  SLIP has no flow control, so Contiki substitutes a fixed
  `SLIP_DEV_CONF_SEND_DELAY` of `CLOCK_SECOND/32` = 31 ms and drains one SLIP
  packet per `slip_flushbuf()`. That timer is a passive `struct timer` posting
  no event, and the native platform gives `select()` a flat 1 s
  `SELECT_TIMEOUT` when idle — so inside the window nothing is scheduled to
  wake the router, and its only early wake-up is one of our bytes. A host-side
  1200-byte ping fragments into 13 SLIP packets queued in one unpaced burst; a
  reply landing inside the window consumed that wake-up while flushing was
  still forbidden, so the loop slept a full second per fragment and the
  2048-byte queue overflowed fatally. Measured flush→reply latency: csim
  0.2–26.5 ms (0 of 13 above the threshold) vs Cooja 0.0–47.3 ms (5 of 14
  above) — Cooja is not correct here, only lucky, and stalls a second whenever
  it loses. Protocol traffic is identical in both (one 7–8 byte `!R`
  confirmation per fragment), so this is not a throughput or framing
  difference. Fixed by modelling the USB-CDC host link a real slip-radio sits
  behind (default 40 ms wall-clock, `CSIM_SERIAL_TX_LATENCY_MS`, `0` disables)
  — the latency that makes Contiki's 31 ms constant work on hardware, and
  which neither simulator modelled. Result: 5/5 replies, 0% loss, versus
  Cooja's 3/5–5/5. Removal criteria are documented in `sim_serial_bridge.c`.

### Known limitations
See [README "Known issues"](README.md#known-issues). Notably: a default
circular topology with many nodes (`-n 16`) does not converge (one collision
domain, not a regression); the Firefly sub-GHz chain has an ACK-turnaround
residual after sustained traffic; native host-process scheduling is a
documented deferral; the energest ARM CPU current is MSP430-class (indicative).
**SVC is a no-op** (no SVCall exception) and **MSP430 CS HFXT** returns the DCO
frequency — unmodeled; only affects firmware that uses them. The serial socket
carries a deliberate 40 ms wall-clock host-link latency (see below) that a
future flow-controlled link should remove.

[0.1.0]: https://github.com/joakimeriksson/cooja-ng/releases/tag/v0.1.0
