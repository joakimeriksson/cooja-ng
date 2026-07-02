# nRF52840 USB Dongle Port — Status & Direction

> Strategic doc: where the port is, where it's going, and what's
> explicitly out of scope. For the device contract see [`SPEC.md`](SPEC.md).
> The L0–L6 narrative — what each milestone unlocked, what bugs were
> caught, the surprises — lives in `git log devices/nrf52840-dongle/
> include/arm/nrf52840_soc.h src/arm/nrf52840_soc.c src/arm/arm_vfp.c`.

## Current state — short version

**The port is complete. L6 RPL-UDP converges between two nrf52840
nodes**, exchanging `hello N` messages every ~18 s at ~1× real-time
wall on the on-chip 2.4 GHz radio. Everything works end-to-end:

- L0–L4 single-node bring-up: green
- L5 — radio bytes flow between two nodes via the on-chip RADIO
  (verified through L6's RPL exchange, which exercises every TX/RX path)
- L6 — RPL Lite forms a DODAG, DAO routes, UDP request/response over
  the on-chip 2.4 GHz radio
- 74/74 ARM correctness tests (33 base Thumb-2 + 15 M4 DSP halfword
  multiply + 26 FPv4-SP-D16 VFP)
- All 471 cross-platform tests still pass on every existing target
  (no regressions on Sky / Z1 / cc2538dk / openmote / zoul-firefly)

## Architectural deltas this port introduced

Beyond the port itself, this work surfaced and fixed csim
infrastructure that benefits all platforms:

1. **ARM SoC vtable polymorphism.** `arm_platform_t` was hard-wired
   to CC2538 peripherals; now SoC-specific state lives behind
   `plat->soc` + an `arm_soc_ops_t` vtable
   (`include/arm/arm_platform.h`). CC2538 peripherals migrated into
   `cc2538_soc_t`; nrf52840 plugs in as `nrf52840_soc_t`. New SoCs are
   a config + an ops table away.
2. **Per-instance memory layout in `arm_cpu_t`.** `flash_base`,
   `flash_end`, `sram_base`, `sram_end`, `rom_size` populated from the
   SoC config at init. Every load/store consults these instance
   fields instead of the CC2538-hardcoded macros. The macros stay in
   `arm_cpu.h` only as test-fixture constants.
3. **SoC-aware vector-table discovery.** `arm_config_t::vtor_default`:
   non-zero → use directly; zero → fall back to CC2538 CCA convention.
   Lets boards with a bootloader region (PCA10059's 0x0..0xfff Open
   Bootloader) point VTOR past it.
4. **WFI fix in `arm_cpu.c`.** Cortex-M `wfi` wakes on any pending IRQ
   regardless of PRIMASK; previous code only cleared `cpu_off` if the
   IRQ was actually taken (which `arm_nvic_check_pending` skips when
   PRIMASK=1). Affected every ARM platform; CC2538 just didn't use
   that idle pattern. Now correct.
5. **Cortex-M4 DSP halfword multiply** (`arm_cpu.c`) — 16 opcodes
   (SMULBB/BT/TB/TT, SMLABB/BT/TB/TT, SMULWB/WT, SMLAWB/WT, SMLALBB/
   BT/TB/TT) with 15 correctness tests. Latent gap before — any M4
   firmware would have trapped on these.
6. **Cortex-M4F single-precision VFP** (`arm_vfp.c`) — FPv4-SP-D16
   subset that real firmware emits, with 26 correctness tests. The
   tests caught 4 latent bugs in the implementation (bit-position
   masks, signed/unsigned discriminators) that wouldn't have shown up
   in networking firmware but would have silently corrupted any
   firmware that uses the FPU for arithmetic.

## Multinode scaling + ACK timing

Two-node RPL-UDP runs clean (`hello N` exchange every ~20 s,
~2× real-time). 5-node converges in ~30 s sim with every client
exchanging messages with the root. 10-node converges in ~60 s sim and
all 9 clients exchange messages — but with `CSMA_CONF_ACK_WAIT_TIME`
at its stock Contiki default of `RTIMER_SECOND/2500` (~0.4 ms), the
contention causes a heavy retx storm (~50 % collision rate on the
medium).

The default is tuned for radios with hardware auto-ACK. Although
nRF52840 *can* auto-ACK via PPI + TIMER + SHORTS choreography
(Nordic's own 802.15.4 driver does this), the Contiki nrf driver
doesn't program that path and relies on CSMA's software ACK
(`CSMA_SEND_SOFT_ACK` in `csma.c`). csim models the chip's
hardware-equivalent ACK in `nrf52840_soc.c::nrf_radio_receive_byte`
(same convention as cc2538), but bytes between nodes still cross a
1 ms scheduler-tick boundary, so the sender's CSMA busy-wait expires
before the ACK can return.

**Recommended firmware override for clean multinode runs**
(`project-conf.h`):

```c
#define CSMA_CONF_ACK_WAIT_TIME (RTIMER_SECOND / 40)   /* 25 ms */
```

This mirrors the upstream Firefly/CC1200 fix (`fix/zoul-cc1200-ack-wait`).
On real hardware with software ACK it's also the right value — the
0.4 ms stock default leaves no slack for ISR latency. Measured on 10
nodes:

| Default 0.4 ms | Bumped to 25 ms |
|---|---|
| 4 649 collisions (50 % of attempts) | 227 collisions (5.4 %) |
| Up to 8 retx per frame | 1 attempt per frame, no retx |

## Out of scope

Explicitly **not** part of this port:

- **USB-CDC console.** The dongle's factory console runs over USB-CDC
  ACM. Modeling enough USBD for CDC enumeration (endpoint config,
  SETUP transfers, EasyDMA → bulk-IN endpoint) is real work. The L0–L6
  ladder is run with `NRF52840_NATIVE_USB=0` so console routes to the
  legacy UART register window, which csim already models.
- ~~**PPI for TSCH.**~~ **DONE (2026-07-02).** TSCH works on nRF52840:
  PPI timestamping (FRAMESTART/END → TIMER0 CAPTURE) is modeled, the
  radio moved to per-byte delivery with start-of-air TX emission, and
  the TIMER 32-bit compare wrap + FICR node-id seeding were fixed.
  2-node association holds indefinitely at −7 ppm drift with all
  keepalives enhanced-ACKed. Regression:
  `./build/test_runner test configs/test-tsch-nrf52840-dk.json`.
- **BLE.** The nRF52840 RADIO is multi-protocol; csim only models
  802.15.4 mode. BLE timing, LL state machine, advertising channels —
  not in scope.
- **SoftDevice.** Nordic's protocol stack. Contiki on nRF runs
  bare-metal.
- **Bootloader / DFU.** csim loads ELFs directly; the Open Bootloader
  region (top of flash on factory dongles) isn't exercised. VTOR is
  set to 0x1000 from the start so the application vector table is
  found without bootloader emulation.
- **Cryptographic accelerators (CCM/AAR/ECB).** Not needed for
  RPL-UDP without link-layer security. Add when TSCH-encrypted
  firmware is requested.
- **External chips on derivative boards.** None on PCA10059. Other
  nRF52840 boards (Particle Argon = nRF52840 + ESP32 coprocessor;
  Adafruit Feather + extras) become separate SPECs.
- **Real double-precision arithmetic.** M4F has no D registers; the
  VFP interpreter rejects double-precision data-processing opcodes.
  GCC may emit coproc=B for VPUSH/VPOP/VLDM/VSTM as the "two singles
  atomically" alias — those are accepted because they don't perform
  arithmetic.
- **Verifying M4 DSP outside the halfword multiply family.** SIMD
  parallel add/sub (`SADD16`/etc.), saturating arithmetic
  (`QADD`/etc.), packing (`PKHBT`/`PKHTB`), and `SMLAD`/`SMLSD`/
  `SMMUL`/`SMMLA`/`SMMLS`/`USAD8` are still on the unimplemented
  side. Contiki networking doesn't emit them; loud trap on first
  encounter so we'll know.

## Files

- [`SPEC.md`](SPEC.md) — device contract, definition of done (all
  green except PPI, deliberately out of scope)
- `../../docs/porting-a-device.md` — the L0–L6 process this port
  followed; §10 (closing out a port) drove this STATUS rewrite.
- `../../docs/architecture.md` — Platforms table includes the
  nrf52840-dongle row.
- `../../firmware/nrf52840-dongle/PROVENANCE.md` — build commands for
  the reference ELFs (hello-world UART + USB variants, udp-client,
  udp-server).

## How to run

```sh
# Build
make

# L0–L4 single-node UART hello-world (manual; no test_runner subcommand)
# Reference ELF in firmware/nrf52840-dongle/hello-world-uart.nrf52840-dongle

# L6 — two-node RPL-UDP
./build/test_runner nrf52840-dongle-multinode \
    firmware/nrf52840-dongle/udp-server.nrf52840-dongle \
    firmware/nrf52840-dongle/udp-client.nrf52840-dongle \
    -t 60000

# Regression against existing platforms
./build/test_runner correctness        # 68 MSP430
./build/test_runner arm-correctness    # 74 ARM (incl. M4 DSP + VFP)
./build/test_runner radio-medium       # 235 medium
./build/test_runner cc1200-mock-host   # 73 CC1200
./build/test_runner mock-host          # 21 CC2420
```

## What the next contributor should know

- The SoC bundle in `nrf52840_soc.c` is intentionally one file. RTC
  alone is ~150 lines; RADIO alone is ~400. If anyone adds USBD or
  PPI it'll grow past a sensible single-file size — split then,
  following the `cc2538_*.c` per-peripheral pattern.
- `radio_emit_tx` and `nrf_radio_receive_byte` are the public seams
  the multinode harness wires into. A new nrf-derivative board (DK,
  Feather, Argon, …) reuses both; only board-glue (LEDs, button
  pinout, console pad selection) differs.
- The 4 VFP bugs the unit tests caught (see commit 70c79d4) are a
  reminder that VFP encoding is fiddly. Adding new VFP opcodes? Add
  the correctness test in the same commit.
- ~~TSCH on nRF needs PPI~~ — resolved 2026-07-02; TSCH works (see
  above). Radio timing is per-byte faithful now (start-of-air TX
  emission, per-byte RX).
