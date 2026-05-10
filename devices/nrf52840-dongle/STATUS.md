# nRF52840 USB Dongle Port — Status & Direction

> Strategic doc: where the port is, where it's going, and what's
> explicitly out of scope. For the device contract see
> [`SPEC.md`](SPEC.md).

## Current state — short version

**Pre-L0 scaffolding in progress; the architectural decision needed
before peripheral code lands has been resolved.** The contract —
[`SPEC.md`](SPEC.md) and this file — defines what the port covers,
how it splits across commits, and where the risks sit.

Concrete progress so far:

- `arm_config_t nrf52840_config` added to `src/arm/arm_config.{c,h}`:
  64 MHz, 1 MiB flash @ `0x00000000`, 256 KiB SRAM @ `0x20000000`,
  48 IRQs. Per nRF52840 PS v1.7.
- **`arm_platform_t` is now SoC-polymorphic.** SoC-specific peripherals
  live behind `plat->soc` + an `arm_soc_ops_t` vtable (init / destroy /
  set_console). CC2538 peripherals migrated into `cc2538_soc_t`
  (`include/arm/cc2538_soc.h`, `src/arm/cc2538_soc.c`); consumers reach
  them via the `arm_platform_cc2538(plat)` accessor. The new
  `arm_platform.h` no longer mentions any CC2538 type. nRF peripherals
  will plug in via the same shape with their own `nrf52840_soc_t` +
  `nrf52840_soc_ops` once they land.

Verification of the polymorphism refactor (zero regressions):
- 68/68 MSP430 + 33/33 ARM correctness
- ARM Firefly bringup firmware (Zolertia banner) PASS
- 2-node cc2538dk nullnet 15.2× real-time
- 2-node cc2538dk RPL-UDP 11.2× real-time, 26 direct RX
- 73/73 CC1200 mock-host, 21/21 CC2420 mock-host
- 235/235 radio_medium

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

Architecture decided and landed: option 1 (vtable polymorphism). The
boundary the nRF peripherals will plug into looks like this:

```c
/* arm_platform.h */
typedef struct arm_soc_ops {
    const char *name;
    void (*init)(struct arm_platform *plat);
    void (*destroy)(struct arm_platform *plat);
    void (*set_console)(struct arm_platform *plat,
                        arm_uart_tx_callback cb, void *user_data);
} arm_soc_ops_t;

typedef struct arm_platform {
    arm_cpu_t           cpu;          /* common */
    arm_nvic_t          nvic;         /* common */
    arm_systick_t       systick;      /* common */
    sim_host_t          host;         /* populated by soc_ops->init */
    void               *soc;          /* per-SoC state */
    const arm_platform_config_t *config;
} arm_platform_t;
```

The L0 path is now unblocked. Order:

1. **`include/arm/nrf52840_soc.h` + `src/arm/nrf52840_soc.c`** — the
   SoC bundle. Initially: empty struct, `nrf52840_soc_ops` with init
   that does nothing useful and a stub `set_console` (writes to
   stderr). Mirrors `cc2538_soc.{c,h}` shape so the pattern stays
   uniform.
2. **`platform_nrf52840_dongle`** entry in `arm_platform.c`'s
   registry, pointing at `nrf52840_config` + `nrf52840_soc_ops`.
3. **Pick the first nrf ELF**: the simplest Contiki-NG nrf52840
   firmware that builds clean. Run it through `arm_load_elf` →
   `arm_cpu_reset` → `arm_step` and see what address the first
   peripheral access lands on. That tells us which peripheral to
   model first (almost certainly CLOCK or NVMC).
4. Iterate: stub the next peripheral, step further, repeat until we
   reach `main()`. That's L1.

Same empirical loop the Sky and Firefly ports used.
