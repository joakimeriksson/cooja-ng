# nRF52840 USB Dongle Port — Status & Direction

> Strategic doc: where the port is, where it's going, and what's
> explicitly out of scope. For the device contract see
> [`SPEC.md`](SPEC.md).

## Current state — short version

**Pre-L0 scaffolding started.** The contract — [`SPEC.md`](SPEC.md)
and this file — defines what the port covers, how it splits across
commits, and where the risks sit.

Concrete progress so far:

- `arm_config_t nrf52840_config` added to `src/arm/arm_config.{c,h}`:
  64 MHz, 1 MiB flash @ `0x00000000`, 256 KiB SRAM @ `0x20000000`,
  48 IRQs. Compiles clean; existing 68 MSP430 + 33 ARM correctness
  tests still pass.

This is the smallest first commit. It introduces no architectural
commitment (no nRF entry in `arm_platform_t` yet — that struct is
hard-wired to CC2538 peripherals and needs deliberate redesign before
nRF peripherals can land alongside it).

## Why this port, why this board

- **Why nRF52840:** large installed base (Particle Argon, Adafruit
  Feather, Nordic DK + Dongle, many third-party boards). Multi-protocol
  radio (802.15.4 + BLE) is a unique capability vs the existing
  CC2420/CC1200/CC2538 lineup. Single-chip — no off-SoC radio to
  emulate.
- **Why the Dongle (PCA10059) and not the DK (PCA10056):** smaller
  surface. No SEGGER, no LCD, no SD card, no QSPI flash, fewer LEDs
  and buttons. Same SoC = same heavy lifting on the radio + PPI;
  fewer board peripherals to model around it. Trade-off documented in
  [`SPEC.md`](SPEC.md): default console is USB-CDC, which we'll
  side-step in L0–L4 by building with UART console on the dongle's
  exposed pads.

## Scope split

The port has two independent risk areas. Both must land for L6, but
they can be developed in parallel after L0–L1 is green.

### Track A — Cortex-M4F support (CPU-side)

The existing ARM interpreter is Cortex-M3. M4 adds:
- DSP/SIMD instructions (`SADD16`, `SMLAxy`, …)
- VFPv4 single-precision FPU
- Bit-band aliases (also exist on M3, may differ in detail)
- M4-specific MPU bits

Realistic estimate based on past experience: **Contiki networking
firmware does not exercise the FPU or DSP/SIMD instructions.** The
existing M3 interpreter probably runs nrf52840 firmware end-to-end on
day one, with M4-specific opcodes added only if firmware actually
traps on them. Empirical, not assumed — we'll know on L1.

### Track B — nRF52840 peripherals

In dependency order (each its own commit):

1. **Boot path**: `arm_config.c` entry, vector table, NVMC stub,
   POWER stub. Firmware halts at first peripheral access if these
   aren't there.
2. **Tick source**: CLOCK + RTC. Without RTC, Contiki's main loop
   never advances.
3. **Output**: GPIO + GPIOTE + UARTE0. L2 banner depends on this.
4. **RADIO** in 802.15.4 mode + EasyDMA. L4 onwards.
5. **PPI**. Required for TSCH. Not required for plain CSMA.
6. **Multinode glue**: per-node IEEE address patching (FICR.DEVICEADDR),
   register the radio with the medium, fan-out byte delivery.

The RADIO peripheral is the dominant risk — it has hardware shorts
(RXREADY → START → END → DISABLE chains), EasyDMA frame buffers, and
participates in PPI-driven TSCH timing. Expect this single item to
take as long as everything else combined, mirroring the CC1200
experience on Firefly.

## Out of scope

Explicitly **not** part of this port:

- **USB-CDC console.** The factory dongle uses USB-CDC for serial.
  Modeling enough USBD for CDC enumeration is real work
  (endpoint config, SETUP transfers, EasyDMA → bulk-IN endpoint).
  L0–L4 will run firmware built with UART-console-on-pads.
  USBD modeling is a separate follow-on, tracked as its own SPEC
  appendix if we get there.
- **BLE.** The nRF52840 radio is multi-protocol. We model only
  802.15.4 mode. BLE timing, LL state machine, advertising channels
  — not in scope. (csim's medium model is single-protocol.)
- **SoftDevice.** Nordic's protocol stack. Contiki on nRF runs
  bare-metal, no SoftDevice.
- **Bootloader / DFU.** csim loads ELFs directly into flash; the
  Open Bootloader region (the top of flash on factory dongles) is
  not exercised.
- **Cryptographic accelerators (CCM/AAR/ECB).** Not needed for
  RPL/UDP without link-layer security. If TSCH-encrypted firmware
  is requested, revisit.
- **External chips.** None on the Dongle. If a future board (e.g.
  Particle Argon = nRF52840 + ESP32 coprocessor) is requested, it
  becomes a separate SPEC.

## Risks

1. **PPI accuracy.** TSCH on nRF uses PPI for sub-microsecond timing
   between RADIO events and TIMER captures. csim's event queue is
   integer-ns; PPI dispatch must happen *before* the next CPU
   instruction step or TSCH slips. Plan: write a small PPI unit test
   harness early to pin the timing semantics before TSCH firmware
   sees the model.
2. **EasyDMA semantics.** Real hardware reads/writes the
   `*PACKETPTR`-pointed RAM at peripheral-controlled times. Firmware
   that mutates the buffer mid-transaction has implementation-defined
   behavior. csim should mirror "buffer is owned by the peripheral
   between START and END" or expose a clear violation diagnostic.
3. **M4-only opcodes encountered late.** If a single `VLDR` or
   `SMUL` shows up deep in a library function, debugging is annoying
   because the trap is far from the cause. Mitigation: surface
   `undef` with full PC + insn dump from day one.
4. **Contiki-NG nrf52840 dongle build path may not be vanilla.** The
   DK is the well-tested target; the Dongle has historically required
   board-specific patches in Contiki-NG. Expect to spend the first
   day on the build path before any csim work. If the build is
   broken upstream, file PRs (Firefly precedent — fix Contiki-NG, not
   csim).

## Files

- [`SPEC.md`](SPEC.md) — device contract, definition of done
- [`STATUS.md`](STATUS.md) — this file
- [`../../docs/porting-a-device.md`](../../docs/porting-a-device.md) —
  the L0–L6 process this port follows

(`HARDWARE-TEST.md`, `DATASHEET-FINDINGS.md`, `archive/` are left to
be created if and when the port reaches the corresponding stage.
Don't pre-create empty docs.)

## Next concrete step

Now that `nrf52840_config` exists, the next step is the **architecture
question for nRF peripherals** — before writing any peripheral code.

`arm_platform_t` (in `include/arm/arm_platform.h`) currently embeds
CC2538-specific peripheral structs by value:

```c
typedef struct arm_platform {
    arm_cpu_t           cpu;
    arm_nvic_t          nvic;
    arm_systick_t       systick;
    cc2538_uart_t       uart0;       /* ← CC2538-specific */
    cc2538_gpio_t       gpio;        /* ← CC2538-specific */
    /* ... more cc2538_* ... */
} arm_platform_t;
```

Adding nRF peripherals here would either bloat the struct with unused
CC2538 fields (and vice versa) or force every consumer to know which
SoC it's running on.

Three options worth weighing before any peripheral code lands:

1. **Polymorphic platform via `sim_host_t`-style vtable.**
   Generalize `arm_platform_t` so the SoC-specific peripheral state
   lives behind a void pointer + ops table. Biggest one-time cost,
   cleanest end state. Likely the right answer.
2. **Parallel `nrf_platform_t`.** Mirror the existing struct for
   nRF, accept the `test_mixed_multinode` plumbing duplication.
   Faster to first L0, more cleanup debt.
3. **Single fat union.** Put both peripheral sets in `arm_platform_t`
   behind a `soc_kind` discriminator. Pragmatic, ugly, doesn't scale
   beyond two SoCs.

Recommendation: do option 1, scoped to *just* the platform struct
(don't refactor `sim_host_t` itself — that interface is already SoC-
agnostic). Then write the first peripheral (CLOCK stub) against the
new shape so the boundary is exercised before there's much code to
move.

Once that decision lands, the L0 path is: minimal `nrf_clock` +
`nrf_power` stubs → load any nRF ELF → step until first peripheral
access → log every IO touch → see how far we get. Same empirical
loop the Sky and Firefly ports used.
