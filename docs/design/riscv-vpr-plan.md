# RISC-V on csim: the nRF54L15 FLPR (RV32E VPR coprocessor)

**Status:** plan, not started (2026-06-22).
**Goal:** run Contiki-NG's `nrf-vpr` port — the
[contiki-ng#3168](https://github.com/contiki-ng/contiki-ng/pull/3168) FLPR
coprocessor work — inside csim, so a single emulated nRF54L15-DK runs **two
ISAs at once**: the Cortex-M33 (`flpr-host`) and the RV32E FLPR (`hello-vpr`),
communicating through shared SRAM.

This is *not* a generic RISC-V networking node (no radio, no RPL). It is a
**dual-core, cross-ISA, shared-memory** scenario that extends csim's
"cross-level / multi-OS" story to "cross-ISA / multi-core" — and it is far more
tractable than a from-scratch RISC-V SoC because the radio stays on the M33 we
already emulate.

## Why it is tractable

The thing csim loads is **one ordinary `nrf54l15-dk` ARM ELF** (`flpr-host`),
which already boots today. `flpr-host` itself:

1. `memcpy`s the FLPR blob (embedded via `flpr-blob.h`, ~8 KB) into shared SRAM,
2. flips an SPU permission bit,
3. writes the VPR start PC, and
4. releases the VPR from reset.

So csim needs no separate RISC-V ELF loader for the demo — the M33 writes the
RV32E code into SRAM at run time. csim's job: **when the M33 writes the VPR
`CPURUN` register, spin up an RV32E interpreter pointed at that same SRAM and
step it on the same ns clock.** Both cores share `cpu->sram`, so the shared
counter and the blob copy "just work".

## The boot dance csim must intercept

From `examples/flpr-host/flpr-host.c` (mirrors Zephyr's
`nordic_vpr_launcher`). All four writes use **Secure** addresses; Contiki's M33
runs Secure by default.

| Step | What `flpr-host` does | Address | csim action |
|------|----------------------|---------|-------------|
| 1 | `memcpy(0x20028000, blob, len)` | SRAM | none — normal SRAM writes |
| 2 | `SPU00_S PERIPH[12].PERM \|= (1<<4)` (SECATTR=Secure) | `0x50040530` | model the bit; **gate** launch on it (HW won't fetch without it) |
| 3 | `VPR_S->INITPC = 0x20028000` | `0x5004C808` | latch FLPR entry PC |
| 4 | `VPR_S->CPURUN = 1` | `0x5004C800` | **trigger**: instantiate + start the RV32E core at INITPC |

`VPR00_S` base is `0x5004C000`; `CPURUN` at `+0x800`, `INITPC` at `+0x808`.

## Memory map (all M33-view addresses, inside the M33 SRAM block)

Per `arch/cpu/nrf-vpr/nrf-vpr-sram.ld` and the Zephyr cpuflpr DTS. The FLPR's
96 KB block lives inside the M33 SRAM csim already backs with `cpu->sram`, so
sharing is automatic.

| Region | Address | Size | Use |
|--------|---------|------|-----|
| FLPR exec (code/.rodata) | `0x20028000` | 32 KB | INITPC target |
| FLPR data/.bss/stack | `0x20030000` | 60 KB | RV32E RAM |
| **Shared counter** | `0x2003F000` | 4 B | FLPR writes tick `N`; M33 polls. Also: `_start` writes `0xA0000001` on entry, `0xA000FFFF` on `main()` return; trap handler writes `0xFA1100\|mcause` |
| Trap mepc dump | `0x2003F004` | 4 B | faulting PC on exception |

Peripherals the FLPR touches (Secure apertures, already modeled for the M33 —
just need to be reachable from the RV32E core's bus):

- **GRTC** `0x500E2000`, `SYSCOUNTER[0]` L/H at `+0x720/+0x724`. csim already
  returns the **live 1 MHz counter** for any SYSCOUNTER index
  (`nrf54l15_soc.c` ~line 530). The FLPR's `clock_time()` divides this by
  `1MHz/CLOCK_SECOND`, so the tick advancing 2/sec is driven by GRTC, **not**
  by RV32E MIPS — FLPR CPU clock accuracy is not critical.
- **GPIO** `P2_S` (gpio2, LED0 = pin 9, FLPR) and `P1_S` (gpio1, LED1 = pin 10,
  M33). DIRSET/OUTSET/OUTCLR — already modeled.

## RV32EMC instruction surface

**Confirmed** from `arch/cpu/nrf-vpr/Makefile.nrf-vpr`:
`VPR_MARCH = rv32emc_zicsr`, `VPR_MABI = ilp32e`.

- **Base:** RV32E — RV32I with 16 GP registers (x0–x15), `ilp32e` ABI.
- **Extensions:** **M** (hardware mul/div/rem) + **C** (compressed, 2-byte
  encodings) + **Zicsr** (CSR instructions); csim additionally implements
  **Zifencei** (`fence.i`). No F/D. Instructions are therefore a mix of 4-byte
  base and 2-byte compressed encodings — the decoder must handle both.
- **Hardware mul/div:** with **M** in `VPR_MARCH`, Contiki's `clock_time()`
  division etc. compile to native `mul`/`div`/`rem` rather than the libgcc
  soft-routines (`__mulsi3`, `__udivdi3`, …), so the interpreter must implement
  the M extension. Wider 64-bit helpers may still come from `-lgcc`.
- **CSR/trap:** `csrw mtvec`, `csrr mcause`, `csrr mepc`. On any exception:
  vector to `mtvec`, set `mcause`/`mepc`. `startup-stubs.c` reroutes `mtvec` to
  a handler that stamps `0x2003F000`, so a faithful trap path makes RV32E
  crashes visible from the M33 console exactly as on hardware.
- **No F/D** (no float), no supervisor mode, no MMU, no A (atomics).

## Staged plan

### Stage 0 — Boot-dance validation with a 5-instruction core (~½ day)
De-risk the integration before building a full CPU. Implement just `lui`,
`addi`, `sw`, `j` (the doc's RV32E "stamp" blob writes `0xCAFEBABE` to
`0x2003F000`), plus the VPR `CPURUN` trigger and shared-SRAM aliasing. Load
`flpr-host` (committed blob temporarily swapped for the stamp), confirm the M33
reads back `0xCAFEBABE`. Proves: SPU SECATTR gating, INITPC/CPURUN intercept,
shared-memory visibility, and co-stepping plumbing — independent of decoder
completeness.

### Stage 1 — RV32E core + correctness suite (~2–3 days)
New `src/riscv/`:
- `riscv_cpu.c` — RV32EMC + Zicsr interpreter, trap (mtvec/mcause/mepc),
  `riscv_step` / `riscv_step_until` matching the arch-CPU API shape used by
  `arm_cpu.c`.
- `riscv_elf.c` — `EM_RISCV=243` route over the shared `elf_loader.c` (for
  standalone FLPR-ELF testing; not on the demo path).
- `include/riscv/riscv_cpu.h`.
- `test/test_riscv_correctness.c` + a `riscv-correctness` subcommand in
  `test_main.c`. Mirror the MSP430/ARM correctness suites; seed from known
  RV32 test vectors and the `hello-vpr` disassembly.

Self-contained and reusable regardless of how dual-core integration lands —
this is the foundation and cannot be wasted.

### Stage 2 — VPR + SPU + shared bus in `nrf54l15_soc.c` (~1–2 days)
- `arm_register_io` regions for `VPR00_S` (`0x5004C800` CPURUN / `0x5004C808`
  INITPC) and `SPU00_S PERIPH[12]` (`0x50040530`).
- Model SECATTR: only launch when the bit is set (matches HW "writes succeed
  but no fetch" behavior — worth reproducing for fidelity).
- On `CPURUN=1`: construct the RV32E core, `pc = INITPC`, and wire its
  memory/IO bus to **share** the M33's `cpu->sram` and route `0x500xxxxx`
  peripheral reads/writes through the existing nrf54l15 IO handlers (GRTC, GPIO).

### Stage 3 — Dual-core stepping (~1–2 days) — the one real design question
Today a node = one CPU. The FLPR is a **second core inside one node** that
starts mid-run. Extend `arm_mote_execute` (`src/motes/arm_elf_mote.c`): after
the M33's slice, if the FLPR is running, advance it on the same ns budget.
Pick a nominal FLPR clock (exact value non-critical — GRTC drives timing).
Reproduce trap visibility (`0xFA1100|mcause` → `0x2003F000`). Keep it
single-threaded and deterministic, consistent with the rest of the runtime.

**Decision to settle here:** model the FLPR as (a) a co-core stepped inside the
M33 node's execute tick (simplest, recommended), or (b) a first-class second
mote sharing a memory view (more general, more plumbing). Recommendation: (a)
for this demo; revisit (b) only if a second cross-ISA SoC appears.

### Stage 4 — End-to-end demo + regression (~1 day)
- Build `flpr-host.nrf54l15-dk` (needs the RISC-V Zephyr SDK + arm-none-eabi;
  or use the committed `flpr-blob.h`). Add it under `firmware/nrf54l15-dk/`.
- Load in csim; assert `[FLPR] tick N` advancing **~2/sec** and LED0 (1 Hz) /
  LED1 (2 Hz) GPIO toggles — mirroring the hardware-validated result.
- Add an `nrf54l15-dk` VPR regression to the test runner + CI.
- Update `docs/architecture.md`, `CLAUDE.md`, and the platform table.

## Effort

~1 week end-to-end. Front-loaded risk is retired in Stage 0 (½ day). Stage 1
(the RV32E core) is the bulk and is independently useful. RV32E is one of the
simplest ISAs to interpret — markedly less work than the Thumb-2/IT-block ARM
core already in the tree.

## Available artifacts (no toolchain needed for the demo)

Prebuilt binaries already exist in the local Contiki-NG checkout
(`../contiki-ng`):

- `examples/flpr-host/build/nrf/nrf54l15/dk/flpr-host.nrf` — **ARM ELF**
  (EABI5, statically linked) with the FLPR blob embedded. csim loads this
  as a normal `nrf54l15-dk` image today; it *is* the Stage-4 demo input.
- `examples/hello-vpr/build/nrf-vpr/hello-vpr.bin` — the raw **8 KB RV32E
  blob** (and `hello-vpr.nrf-vpr`, the RISC-V ELF). Use it as the golden
  reference for the Stage-1 decoder and the Stage-0 boot validation.

So Stages 0/1/4 can proceed against real, hardware-validated binaries without
installing the RISC-V Zephyr SDK or arm-none-eabi. (Rebuild only if we want the
M33 image to embed the very latest FLPR blob — the committed `flpr-host.nrf`
predates the newest `hello-vpr.bin`.)

## Open questions / risks

1. ~~**Exact `-march`**~~ — **resolved:** `rv32emc_zicsr`, `ilp32e` (M + C, to
   match contiki-main's FLPR build). See *RV32EMC instruction surface*.
2. **SPU/SECATTR depth** — model only enough to gate the VPR launch; not a full
   TrustZone/SPU implementation.
3. **FLPR clock rate** — approximate; timing correctness comes from GRTC.
4. **Determinism** — co-stepping must stay deterministic under the existing
   `tools/check-determinism.sh` harness.
5. **No FLPR console** — output is the shared counter polled by the M33; no
   UART model needed on the RV32E side.

## References

- Contiki-NG PR: <https://github.com/contiki-ng/contiki-ng/pull/3168>
- `doc/platforms/nrf-vpr.md` (boot sequence, register addresses, bring-up stamp)
- `examples/flpr-host/flpr-host.c`, `examples/hello-vpr/hello-vpr.c`
- `arch/cpu/nrf-vpr/{startup-stubs.c,clock-arch.c,nrf-vpr-sram.ld}`
- Zephyr `drivers/misc/nordic_vpr_launcher/nordic_vpr_launcher.c`,
  `dts/vendor/nordic/nrf54l15_cpuflpr.dtsi`
- csim anchors: `src/arm/nrf54l15_soc.c` (GRTC SYSCOUNTER, IO regions),
  `src/motes/arm_elf_mote.c` (`arm_mote_execute` tick), `src/common/elf_loader.c`
