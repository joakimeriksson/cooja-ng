# Device SPEC — `nRF54L15-DK (PCA10156)`

> Initial draft, written from Contiki-NG's freshly merged nrf54l15 port
> (`arch/cpu/nrf/nrf54l15/`, `arch/platform/nrf/nrf54l15/dk/`). Items
> still to verify against the nRF54L15 Product Specification are marked
> `TODO:` rather than guessed (per `docs/porting-a-device.md` §3 —
> guessing is the #1 source of port bugs).

> **The point of this port.** nRF54L15 is Nordic's first ARMv8-M / Cortex-M33
> chip with both BLE LE 5.4 and IEEE 802.15.4. csim already runs nRF52840
> through `nrf52840_soc.c`, so this port reuses the SoC-polymorphism
> scaffolding (vtable-driven peripheral bundles, per-instance memory
> layout, SoC-aware VTOR) and writes new peripheral models for the
> nrf54l15's substantially different peripheral set. It is **not** an
> incremental extension of `nrf52840_soc.c` — register addresses,
> peripheral lineup, and IRQ map are all different.

## Identity

- **SoC**: `nRF54L15` (Nordic Semiconductor, ARM Cortex-M33 + 2.4 GHz
  multi-protocol radio: BLE 5.4, IEEE 802.15.4, proprietary)
- **Board name**: `nRF54L15 Development Kit` (Nordic PCA10156)
- **Contiki-NG `TARGET`**: `nrf`
- **Contiki-NG `BOARD`**: `nrf54l15/dk`
- **csim platform string**: `nrf54l15-dk` (used as `.nrf54l15-dk` ELF
  extension and `--platform` argument)
- **Reference docs**:
  - **nRF54L15 product page** (datasheet + PS + errata + user guides):
    <https://www.nordicsemi.com/Products/nRF54L15>
  - **PCA10156 dev kit page**: <https://www.nordicsemi.com/Products/Development-hardware/nrf54l15-dk>
  - **ARMv8-M Architecture Reference Manual** (Cortex-M33 ISA) — ARM DDI 0553,
    <https://developer.arm.com/documentation/ddi0553/latest>
  - **Contiki-NG port**:
    - CPU: `arch/cpu/nrf/nrf54l15/`
    - Board glue: `arch/platform/nrf/nrf54l15/dk/`
    - 802.15.4 driver: `arch/cpu/nrf/nrf54l15/nrf-ieee-driver-nrf54l15.c`
      (thin shim over Nordic's `nrf_802154` SDK driver — see
      `nrf_802154_platform_*.c` in the same dir)
  - **nrfx 3.x** (vendor HAL + linker scripts + MDK):
    <https://github.com/NordicSemiconductor/nrfx>, submodule at
    `arch/cpu/nrf/lib/nrfx/`. Contiki-NG needs nrfx v3.12.1+ for
    nrf54l15 support; the submodule is pinned accordingly.
  - **sdk-nrfxlib** (Nordic 802.15.4 binary driver):
    <https://github.com/nrfconnect/sdk-nrfxlib>, submodule at
    `arch/cpu/nrf/lib/sdk-nrfxlib/`. Must be initialised with
    `git submodule update --init` after a fresh Contiki-NG clone.

## CPU

- **Architecture**: `arm-cortex-m33` (NEW — current csim has
  `arm-cortex-m3` and `arm-cortex-m4f` (`arm_cpu.c` interpreter +
  `arm_vfp.c`). M33 is **ARMv8-M Mainline**, not ARMv7-M.)
- **Frequency**: 128 MHz (HFXO 32 MHz × 4 PLL — TODO: verify against PS)
- **RAM**: 192 KiB @ `0x20000000` (TODO: confirm — XIAO README claims
  256 KiB, port headers say 192 KiB. nRF54L15 vs nRF54L15M variant?)
- **Flash**: 1536 KiB @ `0x00000000`
- **Reuses existing emulator?**: **partially.**
  - **Thumb-2 ISA**: Most M33 instructions are the same as M3/M4 →
    the existing `arm_cpu.c` interpreter should run the common path.
  - **ARMv8-M additions to consider**:
    - **Stack-limit registers** (`MSPLIM`, `PSPLIM`) — M33 traps if the
      stack pointer crosses the limit. Contiki may not actually program
      these; verify empirically.
    - **BTI (Branch Target Identification)** — ARMv8.1-M extension,
      generally not on M33 base.
    - **TrustZone-M security extensions** (`SG`, `BXNS`, `BLXNS`,
      `SAU/IDAU`, secure/non-secure register banking) — the Contiki port's
      Makefile has a `TRUSTZONE_SECURE_BUILD` option (default OFF), so the
      L0–L6 path is non-secure only and we ignore the secure side.
    - **VFP**: M33 supports FPv5-SP-D16 (single + half precision).
      **The Contiki nrf54l15 build is soft-float ABI** (verified in the
      built ELF flags) so the existing `arm_vfp.c` may not be exercised
      at all on the networking path. Add on demand.
  - Realistic plan: load an nrf54l15 ELF, surface any unimplemented
    M33-only opcodes via the existing "loud trap" path, add handlers
    only when firmware actually traps. Same empirical approach the
    nrf52840 port used.

## Console

- **Peripheral**: `UARTE20` — instance 20 of the UARTE peripheral. nrf54l15
  has multiple UARTE instances (00, 20, 21, 22, 30 — TODO: verify list);
  the DK board ties UARTE20 to the SEGGER J-Link VCP.
- **Base address**: TODO — confirm against PS. Likely in the
  0x500x_xxxx peripheral range (see Memory Map below).
- **Baud**: 115200 bps (Contiki-NG default)
- **TX pin**: `P1.4`
- **RX pin**: `P1.5`
- **EasyDMA-only.** Unlike nRF52840's UARTE which has a legacy register
  window we used for csim (TXD at 0x51C), nrf54l15 firmware drives UARTE
  via EasyDMA exclusively (`TXD.PTR`, `TXD.MAXCNT`, `TASKS_STARTTX`).
  csim's nrf54l15 UART model must implement EasyDMA-style TX walking
  the buffer in RAM (`uarte_write` in
  `arch/cpu/nrf/nrf54l15/uarte-arch.c` is the canonical reference).

## LEDs

| Index | Name  | Port | Pin  | Polarity |
|-------|-------|------|------|----------|
| 0     | LED1  | 2    | 9    | TODO: verify (likely active-low) |
| 1     | LED2  | 1    | 10   | TODO |
| 2     | LED3  | 2    | 7    | TODO |
| 3     | LED4  | 1    | 14   | TODO |

Source: `arch/platform/nrf/nrf54l15/dk/nrf54l15-dk-def.h`.

## Buttons

| Index | Name     | Port | Pin  | Polarity |
|-------|----------|------|------|----------|
| 0     | BUTTON1  | 1    | 13   | active-low, internal pull-up |
| 1     | BUTTON2  | 1    | 9    | active-low, internal pull-up |
| 2     | BUTTON3  | 1    | 8    | active-low, internal pull-up |
| 3     | BUTTON4  | 0    | 4    | active-low, internal pull-up |

## Off-SoC chips

**None.** Single-chip SoC. The SEGGER J-Link onboard the dev kit is a
flashing/debug interface, not visible to firmware.

## Clock tree

- **Source**: HFXO 32 MHz crystal (BOM populated on PCA10156). LFXO
  32.768 kHz crystal optional; otherwise LFRC internal RC.
- **Effective core clock**: 128 MHz (HFXO × 4 PLL).
- **What the firmware actually configures**: nrf54l15 introduces
  **GLOBAL_CLOCK** (replaces nRF52's CLOCK peripheral). Different
  register layout, different task/event set. TODO: extract specifics
  from `arch/cpu/nrf/nrf54l15/clock-arch.c` and the PS.

## Memory map — peripheral region

**This is the big delta from nRF52.** nRF54L15 moves all peripherals
from the 0x40000000 base used on nRF52 to the **0x50000000 base** and
uses tighter address windows (sub-4 KB slots per peripheral instead of
4 KB blocks).

Observed accesses in `hello-world.nrf54l15-dk` boot path
(`arm-none-eabi-objdump -d ... | grep -E '0x5[0-9a-f]{6}'`):

| Address (top hits) | Likely peripheral |
|---|---|
| 0x5008_xxxx (00, 20, 50, 70, A0, A1) | GLOBAL_CLOCK / GLOBAL_POWER region |
| 0x500C_A00 | TODO: identify |
| 0x500E_200 / 0x500E_210 | TODO: identify (clock-related?) |
| 0x5010_E00 / 0x5010_E01 / 0x5010_E10 | TODO: identify (UARTE20 ?) |
| 0x5012_000 | TODO |
| 0x4000_000 | NVIC / Cortex-M system bus (common region) |

Filling this table in is the **first concrete pre-L0 step** —
correlating each base against the nRF54L15 PS so we know what to model.

## Required csim infrastructure (new code)

Ordered by dependency. Each item is its own commit.

1. **Cortex-M33 baseline** in `arm_cpu.c` — `MSPLIM`/`PSPLIM`,
   ARMv8-M exception behaviour deltas. Add only as firmware actually
   exercises them.
2. **`src/arm/arm_config.c` — `nrf54l15_config`**: 128 MHz, 1536 KiB
   flash, 192 KiB SRAM, ~50 IRQs (TODO: count). VTOR=0.
3. **`src/arm/nrf54l15_soc.c` + `include/arm/nrf54l15_soc.h`** — new
   SoC bundle. Cannot reuse `nrf52840_soc.c` (different memory map).
   Initial stubs: GLOBAL_CLOCK, GRTC, UARTE20 (EasyDMA), FICR/UICR,
   GPIO (P0/P1/P2), NVMC. Models grow as L0–L4 firmware traps on each
   peripheral.
4. **GRTC (Global RTC)** — replaces nRF52's RTC0/RTC1/RTC2. Different
   register layout: 52-bit counter, multiple CCs, separate task and
   event domains. Drives Contiki's tick on nrf54l15
   (see `arch/cpu/nrf/nrf54l15/rtimer-arch.c`).
5. **UARTE20 EasyDMA** — `TXD.PTR` / `TXD.MAXCNT` / `TASKS_STARTTX`,
   bytes walked from RAM and pushed to the platform console callback.
6. **RADIO** — at a new address with a new register layout. Contiki
   uses Nordic's `nrf_802154` SDK driver here, which is significantly
   bigger than the nrf52840 Contiki driver: PPI/DPPI-based ACK
   choreography, address filtering, RX buffer management, BLE / 15.4
   protocol multiplexing inside the chip. Compare to `nrf52840_soc.c`
   for the structural template; the implementation is mostly fresh.
7. **DPPI (Distributed PPI)** — required if RADIO timing matters
   (it does — the Nordic 802.15.4 driver programs DPPI channels for
   tight ACK turnaround). Different programming model from nRF52's
   PPI.
8. **Multinode harness wiring** — `test_mixed_multinode.c` recognises
   `.nrf54l15-dk` / `.nrf54l15-xiao` extensions, branches `init_arm_node`
   on a new `nrf54l15_soc` accessor, fans RX bytes through the new
   `nrf54l15_radio_receive_byte`.

## Known firmware quirks

- **nrfx 3.x submodule required**. Contiki-NG's nrfx submodule is
  pinned to v3.12.1+ for nrf54l15 (nrf54l15 MDK files live in
  `lib/nrfx/mdk/nrf54l15_xxaa_application.ld`, `system_nrf54l.c`,
  `gcc_startup_nrf54l15_application.S` — these only exist in nrfx
  v3.x). Run `git submodule update --init --recursive` after pulling
  Contiki-NG.
- **`sdk-nrfxlib` must be initialised** — the nrf_802154 driver lives
  there as a binary blob. `git submodule status` shows a `-` prefix
  until you `git submodule update --init arch/cpu/nrf/lib/sdk-nrfxlib`.
- **soft-float ABI**. Verified from the built ELF flags
  (`arm-none-eabi-readelf -h` reports `soft-float ABI`). Networking
  firmware doesn't emit hard-FPU instructions, so the existing
  `arm_vfp.c` may not be exercised.
- **NVIC IRQ map is different**. nRF54L15 has its own IRQ vector
  numbering — must be derived from `nrf54l15_application.h` (in nrfx
  MDK). Don't reuse nrf52840's `irq_num` constants.
- **EasyDMA pointers must be in RAM.** Same constraint as nRF52840 —
  PACKETPTR / TXD.PTR / RXD.PTR pointing at flash silently fails.

## Reference firmware

Pre-built ELFs under `firmware/nrf54l15-dk/`:

- `hello-world.nrf54l15-dk` (293 KiB) — banner + idle; for L0–L4.
- `udp-server.nrf54l15-dk` (294 KiB) — RPL Lite root + UDP echo.
- `udp-client.nrf54l15-dk` (294 KiB) — RPL Lite leaf + UDP client.

Build command (from PROVENANCE.md):

```sh
tools/build-device-firmware.sh --target nrf --board nrf54l15/dk \
    --example examples/rpl-udp --source-file udp-server \
    --output firmware/nrf54l15-dk/udp-server.nrf54l15-dk
```

## Test ladder

| Level | Test                                                          | Wall time |
|-------|---------------------------------------------------------------|-----------|
| L0    | ELF loads cleanly, reset vector at 0x0, MSP/PC sane           | <100 ms |
| L1    | Reset_Handler runs to `main()` without faulting                | <1 s |
| L2    | UARTE20 prints `"nRF54L15 DK"` banner                          | <2 s |
| L3    | LEDs toggle in a known sequence                                | <5 s |
| L4    | `Starting Contiki-NG-…` + RADIO probe succeeds via nrf_802154  | <10 s |
| L5    | 2-node `nullnet`/RPL: ≥1 RX per node                           | <30 s sim |
| L6    | 2-node RPL-UDP: ≥1 hello/response                              | <60 s sim |

No `L−1` row — there's no off-SoC chip to mock-host test.

## Definition of done

- [ ] `make clean && make` builds with no new warnings.
- [ ] All M33-specific opcodes Contiki networking firmware emits are
      handled (or surfaced as loud `undef` traps).
- [ ] `./build/test_runner nrf54l15-dk-multinode firmware/nrf54l15-dk/udp-server.nrf54l15-dk firmware/nrf54l15-dk/udp-client.nrf54l15-dk -t 60000`
      exchanges ≥1 hello/response (L6).
- [ ] No `arm_cpu_t` / nRF peripheral types leak into other peripheral
      drivers — peripherals use `sim_host_t` per the porting guide.
- [ ] `docs/architecture.md` Platforms table includes the
      `nrf54l15-dk` entry.
- [ ] `.github/workflows/test.yml` runs the new subcommand on PR.
- [ ] No regressions on existing platforms (still 471 cross-platform
      tests pass).
