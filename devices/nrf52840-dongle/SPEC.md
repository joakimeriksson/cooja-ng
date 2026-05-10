# Device SPEC — `nRF52840 USB Dongle (PCA10059)`

> Initial draft. Most fields are filled in from Nordic's PCA10059
> schematic + the nRF52840 Product Spec v1.7. Items still to verify
> against Contiki-NG `arch/cpu/nrf52840` and `arch/platform/nrf52840`
> are marked `TODO:` rather than guessed (per `docs/porting-a-device.md`
> §3 — guessing is the #1 source of port bugs).

> **The point of this port is the SoC, not a board chip.** Unlike
> `zoul-firefly`, the nRF52840 has no off-SoC radio — the 802.15.4
> radio is on-die. The work is therefore (a) extending csim's ARM
> backend from Cortex-M3 → Cortex-M4F where Contiki firmware needs
> M4 features, and (b) writing nRF52840 peripheral models. The Dongle
> form factor is chosen over the DK (PCA10056) because it has fewer
> board peripherals to model — same SoC, smaller surface.

> **Why "dongle easier than DK":** no LCD, no SD card, no SEGGER, one
> RGB + one mono LED, one button. The trade-off is that the default
> console is **USB-CDC** instead of UART; for L0–L4 we plan to build
> the firmware with UART console enabled (Nordic's nrf52840-dongle
> support in Contiki-NG exposes a UART option for SWD pads), and
> revisit USBD modeling only if a real-world firmware run requires
> the dongle's USB enumeration path.

## Identity

- **SoC**: `nRF52840` (Nordic Semiconductor, ARM Cortex-M4F + 2.4 GHz
  multi-protocol radio: 802.15.4, BLE, proprietary)
- **Board name**: `nRF52840 USB Dongle` (Nordic PCA10059)
- **Contiki-NG `TARGET`**: `nrf52840` (per the current Contiki-NG tree;
  TODO: verify exact target name — historically Contiki-NG has had
  both `nrf52` and `nrf52840` targets at different times)
- **Contiki-NG `BOARD`**: `dongle` (TODO: verify — DK is typically the
  default; dongle support may need a board-name option or live in a
  branch / separate examples)
- **csim platform string**: `nrf52840-dongle` (used as `.nrf52840-dongle`
  ELF extension and `--platform` argument)
- **Reference docs**:
  - **nRF52840 Product Specification v1.7** (Nordic doc number 4397_734
    v1.7) — primary source for every peripheral.
  - **PCA10059 schematic & PCN** (Nordic) — board pinout, LED/button
    assignments, USB pads.
  - **Nordic Open Bootloader** documentation — for the DFU flash path
    (relevant only if csim needs to skip the bootloader on reset, which
    we do not expect).
  - Contiki-NG: `arch/cpu/nrf52840/`, `arch/platform/nrf52840/`,
    `arch/dev/ble/nrf52840/` — TODO: verify these paths against the
    current Contiki-NG main.

## CPU

- **Architecture**: `arm-cortex-m4f` (NEW — current csim has only
  `arm-cortex-m3` from CC2538)
- **Frequency**: 64 MHz (HFXO 32 MHz × 2 PLL, internal). Contiki tick
  source runs off LFXO/LFRC at 32 768 Hz.
- **RAM**: 256 KiB @ `0x20000000`
- **Flash**: 1 MiB @ `0x00000000`
- **Reuses existing emulator?**: **partially.** The Thumb-2 ISA M4
  uses is a strict superset of M3's, so the existing `arm_cpu`
  interpreter handles the *common* path (every instruction Contiki
  networking firmware actually executes is almost certainly already
  implemented). M4-only additions to consider:
  - **DSP/SIMD instructions** (`SADD16`, `SSUB16`, `SMUL.x.y`,
    `SMLAxy`, `QADD`, `QSUB`, …) — used in DSP/audio code, not in
    Contiki's networking stack. Add only when firmware traps an
    `undef`.
  - **VFPv4-SP (FPU)** — same. Contiki doesn't use float on the
    network path.
  - **Bit-band aliases** for SRAM / peripheral regions — TODO: confirm
    Contiki nrf driver code doesn't rely on them.
  - **MPU** — Cortex-M3 and M4 MPUs are similar; Contiki rarely
    enables MPU at all.

  Realistic plan: build with the existing M3 interpreter and surface
  any unimplemented opcodes via the same "fail loudly with PC + insn"
  path used during the MSP430X work. Add M4-specific handlers
  on-demand.

## Console

- **Peripheral**: TODO — primary depends on firmware build option:
  - **L0–L4 plan**: `UART0` (`UARTE0` in nRF parlance, base
    `0x40002000`). Build the firmware with `NRF_LOG_BACKEND_UART = 1`
    and console routed to UARTE0 on dongle pads `P0.13` (RX) and
    `P0.15` (TX), or whichever pins the dongle exposes via the
    soldered SWD pad area. **TODO: verify pinout.**
  - **Stretch**: `USBD` USB-CDC ACM. The dongle's "factory" console
    path. Modeling enough USBD for CDC enumeration is meaningful work
    (endpoint config, SETUP transfers, EasyDMA → bulk-IN). Defer to
    after L6.
- **Base address**: `0x40002000` (UARTE0)
- **Baud**: 115200 bps
- **TX pin**: TODO (`P0.15` likely, verify on PCA10059 schematic)
- **RX pin**: TODO (`P0.13` likely)

> Implication: until USBD lands, the firmware ELFs we ship under
> `firmware/nrf52840-dongle/` are *not* drop-in replacements for what
> the dongle runs out of the factory — they're a "console-on-pads"
> variant. This is fine for csim-side correctness; document it
> prominently in `firmware/nrf52840-dongle/README.md`.

## LEDs

> PCA10059: 1 mono green LED (LD1) + 1 RGB LED (LD2). All
> active-low (common-anode RGB).

| Index | Name             | Port | Pin | Polarity     |
|-------|------------------|------|-----|--------------|
| 0     | LD1 (Green mono) | 0    | 6   | active-low   |
| 1     | LD2 Red          | 0    | 8   | active-low   |
| 2     | LD2 Green        | 1    | 9   | active-low   |
| 3     | LD2 Blue         | 0    | 12  | active-low   |

(Verify against PCA10059 schematic — the above is the published Nordic
pinout; double-check polarity in `arch/platform/nrf52840/dongle/leds-arch.c`
or equivalent.)

## Buttons

| Index | Name        | Port | Pin | Polarity |
|-------|-------------|------|-----|----------|
| 0     | SW1 (USER)  | 1    | 6   | active-low, internal pull-up |

(RESET is not a normal button — it pulses RST_N on `P0.18`, handled
inside the chip's reset block, not by firmware GPIO IRQ.)

## Off-SoC chips

**None.** The nRF52840 is single-chip — radio, flash, RAM, USB are
all on-die. This is the key simplification vs `zoul-firefly`.

## Clock tree

- **Source**: HFXO 32 MHz crystal (BOM populated on PCA10059) →
  internal HFCLK 64 MHz. LFXO 32.768 kHz crystal (also populated) →
  LFCLK for RTC. Optional internal HFRC 64 MHz / LFRC 32 768 Hz when
  crystals not started.
- **CPU clock divider**: none. CPU runs at the full HFCLK rate
  (64 MHz).
- **What the firmware actually configures**: TODO — read
  `arch/cpu/nrf52840/clock.c` (or whatever the Contiki-NG file is
  called). Generally: HFCLK is started on radio use (`NRF_CLOCK->TASKS_HFCLKSTART`
  → wait for `EVENTS_HFCLKSTARTED`); LFCLK is started early in boot.
  csim can skip the wait-loop by always reporting "started" via the
  EVENTS register.

## Required csim infrastructure (new code)

Ordered by dependency. Each item is its own commit (per the porting
guide's discipline).

1. **`src/arm/arm_config.c` — add `nrf52840_config`.** Address space,
   RAM/flash regions, vector table size (~48 IRQs), default frequency
   (64 MHz). Reuse the Cortex-M3 interpreter for now.
2. **Cortex-M4 ISA delta — opt-in.** Surface unimplemented opcodes
   loudly. Implement *only* the M4-only instructions that real
   firmware traps on (expected list: empty for L0–L6 RPL/UDP/TSCH
   firmware; populate as we discover gaps).
3. **`src/arm/nrf_clock.{c,h}` — CLOCK peripheral.** Stub `HFCLKSTART`
   / `LFCLKSTART` to immediately set the corresponding `EVENTS_*`
   register. Most firmware just spin-waits on these.
4. **`src/arm/nrf_rtc.{c,h}` — RTC0/RTC1/RTC2.** 24-bit counter at
   32 768 Hz, COUNTER register, CC[0..3] compare events, prescaler.
   This is Contiki's tick source on nRF — must be accurate.
5. **`src/arm/nrf_timer.{c,h}` — TIMER0..4.** 16/32-bit timer with
   capture-compare. TIMER0 is owned by the RADIO/SoftDevice on real
   hardware; in Contiki bare-metal it's available but Contiki tends
   to use TIMER1+ for higher-res timing.
6. **`src/arm/nrf_gpio.{c,h}` + `src/arm/nrf_gpiote.{c,h}` — GPIO
   ports P0/P1 + GPIOTE.** Two 32-pin ports. GPIOTE is the IRQ +
   PORT/IN event source — analogous to CC2538's GPIO module but with
   Nordic's task/event abstraction layered on top.
7. **`src/arm/nrf_uarte.{c,h}` — UARTE0** (and UARTE1 for symmetry).
   EasyDMA: TX writes are pointed at a RAM buffer + length, peripheral
   reads RAM and emits bytes. csim TX callback fires per-byte (same
   pattern as `cc2538_uart`).
8. **`src/arm/nrf_radio.{c,h}` — RADIO peripheral in 802.15.4 mode.**
   The big one. EasyDMA PACKETPTR for frame buffer; hardware shorts
   (`SHORTS`) chain RXREADY → START → END → DISABLE etc.; PPI
   integration for tight timing (TSCH). This is the work-equivalent
   of CC1200 in the Firefly port — expect it to dominate.
9. **`src/arm/nrf_ppi.{c,h}` — Programmable Peripheral Interconnect.**
   Routes events on one peripheral to tasks on another with no CPU
   involvement. Contiki TSCH on nRF uses PPI for slot timing
   (TIMER → RADIO TASKS_TXEN, RADIO EVENTS_END → TIMER CAPTURE).
   Without it, TSCH won't work.
10. **`src/arm/nrf_power.{c,h}` — POWER + reset reason.** Mostly a
    stub: report POR on first boot.
11. **`src/arm/nrf_nvmc.{c,h}` — flash controller.** Read-only for
    csim. Firmware reads UICR (`0x10001000`) for the IEEE EUI-64;
    multinode runner must patch this per-node (same hook pattern as
    CC2538's IEEE address).
12. **`src/arm/nrf_rng.{c,h}` — RNG.** Trivial: return values from
    a deterministic PRNG (seed per-node so multinode is repeatable).
13. **(Stretch) `src/arm/nrf_usbd.{c,h}` — USBD.** Only needed if we
    want to run unmodified factory firmware whose console is USB-CDC.
    Out of L0–L6 scope; track separately.
14. **Platform glue** in `src/arm/arm_platform.c`: `platform_nrf52840_dongle`
    entry, board pinout (LED/button output callbacks), default
    bringup state.

The platform glue (#14) is small. Items #4–9 are the bulk.

## Known firmware quirks

> Filled in as we discover them during bring-up. Initial known items:

- **CLOCK START/STARTED handshake.** Firmware writes `1` to
  `TASKS_HFCLKSTART` then spins on `EVENTS_HFCLKSTARTED`. csim must
  set the events register synchronously (or one event-tick later) or
  boot will hang. Same pattern for LFCLK.
- **EasyDMA pointers must be in RAM.** Real hardware enforces this —
  pointing PACKETPTR at flash silently fails. csim should mirror this
  (or warn loudly) so that firmware bugs don't masquerade as csim
  bugs.
- **PPI events fire on the same cycle as the originating event.** No
  CPU stall, no IRQ priority. csim PPI dispatch should be *before*
  the next CPU instruction step, otherwise TSCH timing slips.
- **IEEE EUI-64 is in FICR, not UICR.** Read at `0x10000080..0x10000088`
  (`FICR.DEVICEADDR[0..1]`). Multinode patching writes there per node.
  TODO: confirm Contiki-NG nRF reads from FICR vs a Nordic-defined
  `IEEE_ADDR` location elsewhere.
- **USBD power detection.** If USB cable is detected, factory
  firmware tries to enumerate. Avoid by: (a) building without USB
  console, or (b) having csim's USBD stub report "not connected".

## Reference firmware

Build with `tools/build-device-firmware.sh --target nrf52840 --board
dongle --example <name>`. **TODO:** the script may need an
`nrf52840` target added.

- `firmware/nrf52840-dongle/bringup.nrf52840-dongle` — bring-up:
  prints `"nRF52840 USB Dongle"` banner over UART, blinks LD1 in a
  fixed pattern, halts. Used for L0–L4. Build with UART console
  (NOT USB-CDC) until USBD lands.
- `firmware/nrf52840-dongle/nullnet-broadcast.nrf52840-dongle` —
  802.15.4 broadcast on the on-chip radio. Used for L5.
- `firmware/nrf52840-dongle/udp-server.nrf52840-dongle` and
  `udp-client.nrf52840-dongle` — RPL-UDP. Used for L6.

## Test ladder

| Level | Test                                                            | Wall time |
|-------|-----------------------------------------------------------------|-----------|
| L0    | ELF loads cleanly, reset vector points into flash                | <100 ms |
| L1    | Reset handler runs to `main()` without faulting                  | <1 s |
| L2    | UART0 prints `"nRF52840 USB Dongle"` banner                      | <2 s |
| L3    | LED LD1 toggles in a known sequence                              | <5 s |
| L4    | `Starting Contiki-NG-…` and on-chip RADIO probe succeeds         | <10 s |
| L5    | 2-node `nullnet-broadcast`: ≥1 RX per node                       | <30 s sim |
| L6    | 2-node RPL-UDP: ≥1 hello/response                                | <60 s sim |

No `L−1` row — there's no off-SoC chip to mock-host test.

## Definition of done

- [ ] `make clean && make` builds with no new warnings (and the new
      ARM-M4 / nRF peripheral source files compile under the same flags)
- [ ] All M4-specific opcodes Contiki-NG firmware actually executes
      are implemented (verified by running L4 firmware to completion
      with no `undef` traps)
- [ ] `./build/test_runner nrf52840-dongle-firmware` passes (covers
      L0–L4)
- [ ] `./build/test_runner nrf52840-dongle-multinode firmware/nrf52840-dongle/nullnet-broadcast.nrf52840-dongle -t 20000 -q`
      shows ≥1 RX per node (L5)
- [ ] `./build/test_runner nrf52840-dongle-multinode firmware/nrf52840-dongle/udp-server.nrf52840-dongle firmware/nrf52840-dongle/udp-client.nrf52840-dongle -t 60000`
      exchanges ≥1 hello/response (L6)
- [ ] `.github/workflows/test.yml` runs the new subcommands on PR
- [ ] No `arm_cpu_t` / nRF peripheral types leak into other peripheral
      drivers — peripherals use `sim_host_t` (per the porting guide)
- [ ] PPI is wired well enough for TSCH to work (Cooja regression:
      TSCH-on-nRF tests, if any, pass)
- [ ] `docs/architecture.md` Platforms table includes the
      `nrf52840-dongle` entry
- [ ] No regressions in existing Cooja-NG suite (still 81/81 pass)
