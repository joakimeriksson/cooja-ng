# Changelog

All notable changes to Cooja-NG (`csim`) are documented here. The format is
loosely based on [Keep a Changelog](https://keepachangelog.com/); this project
uses [Semantic Versioning](https://semver.org/) once it reaches 1.0 — until
then, 0.x minor releases may adjust the CLI, config, and plugin ABI.

## [0.1.1] — unreleased

Stabilization release: correctness, memory-safety, and robustness fixes from a
full subsystem audit. No CLI, config, or plugin-ABI changes. See
[`docs/design/release-0.1.1-hardening.md`](docs/design/release-0.1.1-hardening.md)
for the audit and plan.

### Fixed — memory safety
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

### Fixed — correctness
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

### Fixed — robustness
- Service dispatch re-entrancy guard enforced in release builds (was
  assert-only); pcap writer checks every write and stops on a short write
  instead of emitting a corrupt capture; energy-panel JSON never truncates
  mid-structure; firmware→board detection keys on the basename's extension;
  external-command service cleans up on `fork` failure.

### Known issues
- **nRF54L15 two-node radio and nRF52840 multi-hop chains do not route.**
  The FLPR single-node dual-core feature works; two-node nRF54L15 reception is
  broken by a stack of radio-timing bugs in the receive model (a deferred-disable
  timeout that fires mid-frame, plus reception corruption failing CRC) —
  pre-existing, unrelated to the 0.1.1 changes, root-caused in the plan (item
  T3). nRF52840 4-node chains fail similarly (T1/T2).
- **SVC is a no-op** (no SVCall exception) and **MSP430 CS HFXT** returns the
  DCO frequency — unmodeled; only affects firmware that uses them.

## [0.1.0] — 2026-06-22

First public release. A fast, multi-architecture C re-implementation of the
parts of Cooja and MSPSim needed to run the upstream Contiki-NG test suite
headlessly, with a focus on simulation speed, deterministic timing, and
faithful peripheral behaviour.

### Emulation
- **MSP430 / MSP430X** CPU (computed-goto interpreter + optional GNU Lightning
  JIT), MCU configs F149 / F1611 / F2617 / F5437 / CC430F5137 / FR5969, with
  GPIO, USART/eUSCI, Timer A/B, BCS/UCS/CS clocks, ELF loader, and the CC2420
  radio.
- **ARM Cortex-M3/M4/M33** interpreter (Thumb-2 + M4 DSP/VFP) with CC2538
  (on-chip RF Core), Zolertia Firefly (+ CC1200 sub-GHz), and Nordic nRF52840 /
  nRF54L15 (on-chip 802.15.4) platforms.
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
- Passes the upstream Contiki-NG Cooja test suite (**89 / 89**, including the
  TUN/border-router cases) via `tools/run-cooja-tests.sh`, plus instruction,
  firmware, radio-medium/bus, chip-driver, and plugin unit suites.
- CI on Linux (gcc) and macOS (clang) gating the unit suites plus nRF52840
  networking (Contiki RPL-UDP on DK + Dongle, stock Zephyr 802.15.4 echo) and
  determinism / config-equivalence guards.

### Known limitations
See [README "Known issues"](README.md#known-issues). Notably: a default
circular topology with many nodes (`-n 16`) does not converge (one collision
domain, not a regression); the Firefly sub-GHz chain has an ACK-turnaround
residual after sustained traffic; native host-process scheduling is a
documented deferral; the energest ARM CPU current is MSP430-class (indicative).

[0.1.0]: https://github.com/joakimeriksson/cooja-ng/releases/tag/v0.1.0
