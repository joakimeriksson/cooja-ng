# Device SPEC — `Zolertia Firefly`

> Filled in from Contiki-NG `arch/platform/zoul/firefly/board.h` (BOARD_STRING
> `"Zolertia Firefly platform"`) and `arch/cpu/cc2538/`. Polarities verified
> against `arch/platform/zoul/dev/leds-arch.c` and `dev/board-buttons.c`.
> CC1200 wiring + driver expectations from
> `arch/dev/radio/cc1200/{cc1200.c,cc1200-const.h,cc1200-arch.h}` and
> `arch/platform/zoul/dev/cc1200-zoul-arch.c`.

> **The point of this port is the CC1200.** Without it, the Firefly is
> indistinguishable from the existing `cc2538dk` platform — the SoC,
> on-chip 2.4 GHz radio, UART, GPIO, and timers are all already
> emulated. The new work is: a CC2538 SSI peripheral, a CC1200 chip
> driver, an 802.15.4g-aware frame profile, and per-node fan-out so
> the medium delivers bytes to the chip whose channel matches.

> **Radio medium strategy.** We do *not* need a true dual-band /
> dual-radio refactor of `radio_medium`. The CC1200 sub-GHz channels
> are modeled as a non-overlapping extension of the channel space
> (e.g. 11–26 = 2.4 GHz IEEE 802.15.4, 100+ = sub-GHz CC1200). The
> existing channel-match-drops-bytes filter then prevents cross-talk
> between the two radios for free. The only multi-radio piece left
> is a per-node fan-out in the multinode runner: each node registers
> two receive callbacks (cc2538_rfcore + cc1200), and bytes are
> dispatched to whichever chip's current channel matches the
> sender's. The `radio_medium_t` itself stays single-radio.

## Identity

- **SoC**: `CC2538` (TI, ARM Cortex-M3 + on-chip 2.4 GHz IEEE 802.15.4 radio)
- **Board name**: `Zolertia Firefly` (rev B; the original revA is a separate
  Contiki-NG board `firefly-reva` and is out of scope here)
- **Contiki-NG `TARGET`**: `zoul`
- **Contiki-NG `BOARD`**: `firefly`
- **csim platform string**: `zoul-firefly` (used as `.zoul-firefly` ELF
  extension and `--platform` argument)
- **Reference docs**:
  - Contiki-NG: `arch/platform/zoul/firefly/board.h`,
    `arch/platform/zoul/platform.c`, `arch/cpu/cc2538/`,
    `arch/dev/radio/cc1200/`
  - CC2538 datasheet (SWRS096, TI)
  - CC1200 datasheet (SWRS123, TI) — not in tree; needed for any
    behavior the Contiki driver doesn't already exercise
  - Zolertia wiki: <https://github.com/Zolertia/Resources/wiki/Firefly>

## CPU

- **Architecture**: `arm-cortex-m3`
- **Frequency**: 32 000 000 Hz (32 MHz, sourced from external XOSC after
  Contiki's `sys_ctrl_init()`)
- **RAM**: 32 KiB @ `0x20000000`
- **Flash**: 512 KiB @ `0x00200000`
- **Reuses existing emulator?**: yes — same `cc2538_config` and
  `arm_cpu`/peripheral set as `cc2538dk` and `openmote`. No new CPU
  code required.

## Console

- **Peripheral**: `UART0` (CC2538 UART0)
- **Base address**: `0x4000C000` (already wired in
  `src/arm/cc2538_uart.c`)
- **Baud**: 115200 bps (Contiki-NG default)
- **TX pin**: `PA1`
- **RX pin**: `PA0`

> UART0 is wired to a CP2104 USB-to-serial bridge — natural console
> choice. UART1 (PC0/PC1) is exposed on JP3 but unused by the boot
> path.

## LEDs

| Index | Name         | Port | Pin | Polarity     |
|-------|--------------|------|-----|--------------|
| 0     | LED1 (Red)   | D    | 5   | active-high (`negative_logic = false`) |
| 1     | LED2 (Green) | D    | 4   | active-high |
| 2     | LED3 (Blue)  | D    | 3   | active-high |

## Buttons

| Index | Name      | Port | Pin | Polarity |
|-------|-----------|------|-----|----------|
| 0     | USER (S1) | A    | 3   | active-low, internal pull-up (`negative_logic = true`, `GPIO_HAL_PIN_CFG_PULL_UP`) — also shared with the bootloader |

## Off-SoC chips

### `CC1200` — sub-GHz radio (**primary deliverable**, not deferred)

The Firefly is a dual-RF board: the on-chip CC2538 2.4 GHz radio is
always present, and a CC1200 sub-GHz transceiver shares the board over
SSI0. The whole reason to add this platform is the CC1200 — Contiki
firmware that targets `zoul/firefly` typically uses the sub-GHz radio
for long-range / 802.15.4g use cases that the cc2538dk cannot exercise.

- **Datasheet ref**: TI SWRS123 (CC1200 Low-Power High-Performance RF
  Transceiver, 2013). Plus the Contiki driver at
  `arch/dev/radio/cc1200/cc1200.c` — for emulation we only need to
  satisfy what real firmware actually does, and that is exactly what
  this driver exercises (every register read/write the firmware
  performs is in this file).
- **Bus**: `SSI0` on CC2538 (`CC1200_SPI_INSTANCE = 0`). SSI0 base
  address `0x40008000`. **Not yet emulated in csim** — see "Required
  csim infrastructure" below.
- **CSn pin**: `PB5`, active-low (`CC1200_SPI_CSN_*`). Driven by
  firmware as a normal GPIO output, *not* by the SSI controller's
  hardware FSS, so the platform can hook CSn via the existing
  GPIO output-callback mechanism (same pattern as MSP430 → CC2420).
- **Status pins** driven by CC1200 back to MCU:

  | Signal | Port | Pin | Polarity | Maps to / role |
  |--------|------|-----|----------|----------------|
  | GDO0   | B    | 4   | edge     | `CC1200_GDO0_*`. Configured by firmware via `IOCFG0` register. Default Contiki use: PKT_SYNC_RXTX (rising = SFD detected / TX started; falling = packet end). Fires `GPIO_B_IRQn`. |
  | GDO2   | B    | 0   | edge     | `CC1200_GDO2_*`. Optional — driver only wires it if `CC1200_USE_GPIO2` is set. Same IRQ vector. |
  | GDO3   | —    | —   | —        | Not wired on Firefly. The driver only reads GDO3 in test modes. |

- **Reset pin**: `PC7` (active-low, `CC1200_RESET_*`). Firmware
  pulses it during `cc1200_arch_init()`.
- **Power/enable pin**: shared 3V3 rail, gated externally by R10
  (0 Ω resistor). No software control. Emulator can ignore — the
  chip is always powered.
- **Interrupt routing**: GDO0 (and optionally GDO2) → `GPIO_B_IRQn`.
  The Contiki driver re-purposes the GPIO IRQ as the radio's main
  interrupt — see `cc1200_arch_gpio0_setup_irq()` /
  `cc1200_rx_interrupt()`.
- **Notes**:
  - The driver expects standard CC120x SPI command framing: byte 0 =
    `R/W | BURST | addr[5:0]`. Extended-address registers use
    `0x2F` as the first byte then a second address byte.
  - Strobes (SRES, SFRX, STX, SRX, …) return the chip status byte
    (state + FIFO bytes available). Status byte must reflect
    state machine progression for the driver to advance.
  - The driver heavily depends on the chip's *state machine* (IDLE
    → RX → TX → IDLE etc., readable via `MARCSTATE` `0x2F73`).
    Mock-host unit tests should pin every state transition.
  - Frame format on-air is **not** standard 802.15.4 (2.4 GHz). It
    is 802.15.4g / SUN-FSK: longer preamble (`PREAMBLE_CFG1`),
    32-bit sync word `0x930B51DE` (default for 50 kbps 2-FSK
    Contiki config), variable-length packet (`PKT_CFG0` MODE = 1),
    payload, CRC-16. The radio medium's frame tracker needs a
    second SFD profile.

## Clock tree

- **Source**: external 32 MHz crystal (XOSC) and 32.768 kHz crystal
  (XOSC32K) on the Zoul module.
- **CPU clock divider**: `SYS_DIV = 32 MHz` (no division). 16 MHz IO
  clock (`IO_DIV = 16 MHz`) per the CC2538 default that
  `sys_ctrl_init()` programs.
- **What the firmware actually configures**:
  `arch/cpu/cc2538/dev/sys-ctrl.c` → `sys_ctrl_init()` selects 32 MHz
  XOSC as the system source (`SYS_CTRL_O_CLOCK_CTRL`), enables RF
  clock on RUN/SLEEP, gates IRQs. csim already pegs the CPU at
  `default_cpu_freq = 32 MHz`, which matches.
- **CC1200 clock**: 40 MHz crystal local to the CC1200 chip. Not
  visible to the SoC; relevant only for CC1200 internal symbol-rate
  computations during emulation.

## Required csim infrastructure (new code)

Unlike the existing `cc2538dk` port, this one needs new emulator
infrastructure beyond the platform glue. List ordered by dependency:

1. **`src/arm/cc2538_ssi.{c,h}` — CC2538 SSI controller.** Two
   instances at `0x40008000` (SSI0) and `0x40009000` (SSI1). Memory-
   mapped TX/RX FIFOs, `SSI_CR0`/`SSI_CR1` config, `SSI_SR` status,
   `SSI_DR` data. Exposes a host-side hook
   `cc2538_ssi_set_exchange_callback(ssi, cb, ctx)` that the
   platform fills in to route bytes to chip drivers (analogous to
   `msp430_usart_set_spi_exchange`). Only SSI0 is required for the
   Firefly, but build both for symmetry — costs nothing.
2. **`src/arm/cc1200.{c,h}` — CC1200 chip driver.** Takes
   `const sim_host_t *host` only — never `arm_cpu_t` /
   `cc2538_gpio_t` directly. Exposes:
     - `cc1200_init(chip, host)`
     - `cc1200_spi_exchange(chip, byte) -> byte`
     - `cc1200_set_csn(chip, level)` (called from platform GPIO
       output-callback for PB5)
     - `cc1200_set_reset(chip, level)` (PC7)
     - `cc1200_set_rf_listener(chip, cb, data)` for outbound bytes
     - `cc1200_receive_byte(chip, byte)` for inbound bytes
   Drives GDO0/GDO2 back via `host->set_input_pin(...)`; uses
   `host->force_irq_edge(...)` for edge IRQs.
3. **`test/test_cc1200.c` — mock-host unit tests.** Pattern from
   `test/test_mock_host.c`. Cover: chip reset, register read/write
   (single + burst + extended-address), strobe state-machine
   progression, SFD detection on RX, GDO0 edge generation,
   TX→RX turnaround. **Must pass before integrating into the
   platform.** Per the pitfalls catalog, state-machine bugs that
   slip past unit tests show up as "RPL doesn't converge after
   60 s" — six layers away.
4. **Channel-space extension + per-node fan-out.** No
   `radio_medium` API change required (see the strategy note at
   the top). Concretely:
     - Allocate a sub-GHz channel range (e.g. 100+) and reserve it
       for CC1200. The existing `radio_medium_set_channel()` and
       channel-match filter handle isolation between the 2.4 GHz
       and sub-GHz nodes for free.
     - In `test/test_mixed_multinode.c`, each Firefly node owns
       *two* receive endpoints: cc2538_rfcore and cc1200. The TX
       listener is registered on each chip; the per-node receive
       dispatch picks the chip whose currently-tuned channel
       matches the sender. (This is logic in the test runner, not
       the medium.)
     - The `TODO(dual-radio)` markers in `radio_medium.h` can stay
       — they describe a deeper refactor for a future port that
       has two radios *on the same band*. We don't need it.
5. **802.15.4g frame profile in the medium's frame tracker.** The
   current `frame_tracker_t` assumes 4× `0x00` preamble + the
   CC2420/CC2538 SFD byte. CC1200 frames use a longer programmable
   preamble + 32-bit sync word `0x930B51DE` (default for the
   Contiki 50 kbps 2-FSK config). Add a profile field on
   `radio_node_state_t` (or key it off the channel range) and
   parameterize the tracker. Per-frame loss decisions then keep
   working for sub-GHz traffic.
6. **Platform glue** in `src/arm/arm_platform.c`:
   `platform_zoul_firefly` entry, `arm_host_*` vtable (already
   exists for cc2538dk — reuse), CSn/RESET output-callback wiring,
   SSI exchange callback installation.

The platform glue (#6) is the smallest piece. Everything above it is
the actual work.

## Known firmware quirks

- **CC1200 boot probe.** `cc1200_init()` reads the part number from
  `EXT_PARTNUMBER` (`0x2F8F`) — must return `0x20` for the driver to
  decide "this is a CC1200" (vs. CC1201 = `0x21`). Then it reads
  `EXT_PARTVERSION` (`0x2F90`) — anything non-zero will do.
- **`linkaddr_node_addr` patching.** Like the cc2538dk port, the
  Firefly derives its IEEE address from on-chip flash
  (`ieee-addr.c` reads `IEEE_ADDR_LOCATION` at `0x00280028`). The
  multinode runner must patch this per-node — same hook the
  cc2538dk port uses.
- **User button shares PA3 with the bootloader.** Driving PA3 low
  at reset enters the ROM bootloader on real hardware. In
  simulation this has no effect, but firmware that drives PA3 as
  output may surprise later.
- **Dual-RF default.** `REMOTE_DUAL_RF_ENABLED = 1` by default. The
  firmware build path *will* try to initialise both radios. Until
  CC1200 emulation lands, build with the 2.4 GHz radio explicitly
  selected to avoid the SSI0 probe (see `firmware/zoul-firefly/
  README.md`).
- **Strobe timing.** CC1200 returns the chip status byte on every
  SPI transaction, but the *new* state after a strobe is only
  guaranteed visible after the strobe completes. Real firmware
  polls `MARCSTATE` rather than relying on the strobe response —
  the emulator should mirror that semantic (don't update state
  synchronously inside `cc1200_spi_exchange`; schedule the
  transition on the event queue using ns timing, like CC2420).

## Reference firmware

Build with `tools/build-device-firmware.sh --target zoul --board
firefly --example <name>`. Each ELF gets a sibling `PROVENANCE.md`
written by the script.

- `firmware/zoul-firefly/bringup.zoul-firefly` — bring-up: prints
  `"Zolertia Firefly platform"` banner, blinks LEDs in a fixed
  pattern, halts. Used for L0–L4. Build with the 2.4 GHz radio
  driver to keep the dependency surface minimal until CC1200 lands.
- `firmware/zoul-firefly/nullnet-broadcast-subghz.zoul-firefly` —
  802.15.4g broadcast on the **CC1200**. Used for L5 *of the CC1200
  port*. Build with `MAKE_RADIO=cc1200`.
- `firmware/zoul-firefly/udp-server-subghz.zoul-firefly` and
  `udp-client-subghz.zoul-firefly` — RPL-UDP over CC1200. Used
  for L6 of the CC1200 port.
- *(Optional)* equivalents on the on-chip 2.4 GHz radio — useful as
  a reference baseline to confirm the SoC + platform glue work
  before debugging CC1200 issues.

## Test ladder

Standard L0–L6 for the platform, plus chip-driver checkpoints
*below* L0 because the off-SoC chip is the headline:

| Level | Test                                           | Wall time |
|-------|------------------------------------------------|-----------|
| L−1   | `test_cc1200` mock-host: register R/W + strobe state machine | <1 s |
| L−1   | `test_cc1200` mock-host: SFD detect + GDO0 edge generation   | <1 s |
| L0    | ELF loads cleanly, reset vector points into flash            | <100 ms |
| L1    | Reset handler runs to `main()` without faulting              | <1 s |
| L2    | UART0 prints `"Zolertia Firefly platform"` banner            | <2 s |
| L3    | LEDs toggle in a known sequence                              | <5 s |
| L4    | `Starting Contiki-NG-…` and `cc1200: detected, part=0x20`    | <10 s |
| L5    | 2-node `nullnet-broadcast-subghz`: ≥1 RX per node            | <30 s sim |
| L6    | 2-node RPL-UDP over CC1200: ≥1 hello/response                | <60 s sim |

## Definition of done

- [x] `make clean && make` builds with no new warnings
- [x] `./build/test_runner cc1200-mock-host` passes (chip-driver
      unit tests; 57/57 PASS — added strobe transition timing
      assertions when SIDLE/SRX/STX moved to event-driven MARCSTATE
      transitions in `src/arm/cc1200.c`)
- [x] L0–L4 bringup tests pass — `./build/test_runner arm-firmware`
      runs `bringup.zoul-firefly` and asserts the
      `"Zolertia Firefly platform"` banner. (No dedicated
      `zoul-firefly-firmware` subcommand was added; the existing
      `arm-firmware` harness covers it.)
- [x] **L5 — `./build/test_runner zoul-firefly-multinode firmware/zoul-firefly/nullnet-broadcast-subghz.zoul-firefly -t 20000 -q`** shows ≥1 RX per node. Passing.
- [ ] **L6 — `./build/test_runner zoul-firefly-multinode firmware/zoul-firefly/udp-server-subghz.zoul-firefly firmware/zoul-firefly/udp-client-subghz.zoul-firefly -t 60000`** exchanges ≥1 hello/response.
      Status: **does not converge.** Latest measurement (`-d 200`,
      60 s, post commits `7b9b26d` + `5260786`):
      `Total RF bytes: 101988, Emu RX frames: 80 direct + 214 queued + 150 drained + 672 dropped`
      → 444 frames reach Node 1's chip, 230 reach firmware via the
      ISR chain (per the chain audit at
      [`CC1200-RX-ACK-CHAIN.md`](CC1200-RX-ACK-CHAIN.md)), 50 ACKs
      emitted, but RPL DAG still doesn't form within 60 s.
      Tactical work-list: [`L6-PLAN.md`](L6-PLAN.md). Project status
      and decision context: [`STATUS.md`](STATUS.md).
- [x] `.github/workflows/test.yml` runs the new subcommands on PR
- [x] CC1200 driver takes `sim_host_t` only — no `arm_cpu_t` /
      `cc2538_gpio_t` types leak in
- [x] CC2538 SSI driver covers SSI0 *and* SSI1 (Firefly only uses
      SSI0, but symmetry costs nothing and prevents a "second port
      adds SSI1 separately" scenario)
- [x] Per-radio dispatch in the multinode runner lands without
      regressing the existing `cc2538dk` and `sky` multinode tests
      (verified: cc2538dk nullnet still 3 RX in 20 s, cc2538dk
      RPL-UDP `-d 100` converges, sky multinode still exchanges
      packets, full Cooja regression at 81/81 non-skipped).
      `radio_medium_t` is now per-radio: see
      [`docs/radio-medium.md`](../../docs/radio-medium.md). The
      original "reserved sub-GHz channel range + `cross_band_drop`"
      hack is preserved as the legacy fallback for unregistered slots.
- [x] `docs/architecture.md` Platforms table includes the
      `zoul-firefly` entry with CC1200 noted under off-SoC chips.
