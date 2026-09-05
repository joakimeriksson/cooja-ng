# nRF54L15 (Seeed XIAO) — emulator vs. silicon

Board: Seeed Studio XIAO nRF54L15, CMSIS-DAP, console on `/dev/ttyACM4` at
115200. Flashed with OpenOCD through the board's config:

```sh
make -C examples/platform-specific/nrf/trustzone BOARD=nrf54l15/xiao upload
```

Firmware: the bundled TrustZone example (secure world + minimal-platform
normal world) from `contiki-ng-nrf54l15`, branch `nrf54l15-trustzone`,
commit `0076a9779`. The same two ELFs run in the emulator through
`configs/test-tz-boot-nrf54l15-xiao.yaml`.

## Boot and hand-off

Identical line sequence on both: the secure world reports its reset reason,
initialises TrustZone, enables the fault handlers, hands off to the normal
world, and the normal world registers the TrustZone API through the secure
gateway veneers.

## Clock rate

The normal world prints one iteration per ten seconds with Contiki's
`clock_time()` value. Tick rate matches exactly.

| | iteration interval (clock ticks) | wall/sim seconds |
|---|---|---|
| Hardware | 1280 | 10.0 |
| Emulator | 1280 | 10.0 |

Absolute values differ by a constant six ticks (hardware iteration 1 at 1286,
emulator at 1280): about 47 ms more boot time before the first tick on
silicon. Not modelled, not significant.

## Watchdog

Built with `DEFINES=NORMAL_WORLD_CONF_HANG_AT_ITERATION=2`, so the normal
world stops feeding the watchdog after its second iteration. The secure world
owns WDT30 and the normal world feeds it through a secure-gateway veneer.

| | timeout | reset reason reported |
|---|---|---|
| Hardware | 2 s | `watchdog0 (0x00000002)` |
| Emulator | 2 s | `watchdog0 (0x00000002)` |

Hardware measures 2.475 s from the hang message to the reset-reason line;
the extra 0.475 s is secure-world boot and console output at 115200 baud.
The emulator reaches the same line 2.003 s after the hang because its UARTE
model transmits with no baud-rate delay. Both then reboot cleanly and the
normal world comes back up, looping.

## Cycle counter

The secure world's shell exposes the cycle-measurement commands the firmware
branch added, driven the same way on both sides: `call-perf 1000` typed at the
console. The emulator's console accepts input, so this is a like-for-like
comparison.

| | clock the firmware measures | local call, mean | min | max |
|---|---|---|---|---|
| Hardware | 128 MHz | 17 cycles | 14 | 22 |
| Emulator | 128 MHz | 13 cycles | 12 | 12 |

The calibration loop agrees to three parts in a hundred thousand: 6400210
cycles against 6400015, both over 3125 clock ticks. So the cycle counter and
the clock that drives it are faithful.

The per-call figure is about a quarter lower in the emulator, and has no
spread at all. Both follow from the same thing: the emulator charges a fixed
cost per instruction and models no memory system, so there are no flash wait
states, no cache misses and no bus contention to pay for or to vary. Use it
for comparing one change against another, not as an absolute cycle count.

## Known difference: radio driver assertion at startup

On hardware the first boot after flashing hits
`A! nrf_802154_trx.c:360` once, which is the
`NRF_802154_ASSERT(radio_is_disabled)` at the end of
`wait_until_radio_is_disabled()`, and the node resets with reason
`software`. It then runs stably. The emulator does not reproduce this
one-shot startup assertion.

## Hardware-only acknowledgement

Firmware on this branch sets `CSMA_CONF_SEND_SOFT_ACK 0`, so link-layer
acknowledgements come solely from the Nordic 802.15.4 driver's own
timer-and-interconnect path. That path now completes in the emulator, and the
two-node RPL-UDP test over TrustZone passes. The investigation that got it
there, including a real double-acknowledgement bug in the older firmware that
the emulator had been hiding, is in `docs/design/nrf54l15-ack-gap.md`.
