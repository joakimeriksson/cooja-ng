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

## Security unit (SPU) and Non-secure peripheral access

Measured 2026-09-15 with a scratch probe firmware (secure world with shell
commands that dump and poke the four SPU instances, normal world with raw
read/write commands reached through the secure shell's `ns`; a hook in the
secure BusFault handler reports SPU state at the moment of a fault). The
emulator's model was changed to match every row.

| | Hardware | Emulator (after) |
|---|---|---|
| Non-secure read or write of a Secure peripheral alias | precise BusFault into the Secure world: CFSR `0x8200` (PRECISERR, BFARVALID), BFAR = the `0x4...` address; the load or store does not complete | same: the frame names the faulting instruction and the destination register is untouched |
| Security-unit state at the fault | owning instance `EVENTS_PERIPHACCERR=1`, `PERIPHACCERR.ADDRESS` = low 16 bits of the address; MPC00 `EVENTS_MEMACCERR=1` | same (MPC address registers not modelled) |
| Order of handlers | BusFault handler runs first; the SPU interrupt stays pending behind it (equal priority) | same: the refused transaction only marks the SPU/MPC line pending, the core enters the synchronous BusFault first, and the line is then taken by priority (a higher-priority line would preempt the handler) |
| SPU interrupt | level: `INTENSET` after a latched event pends the NVIC line at once | same |
| Clearing the event | `EVENTS_PERIPHACCERR = 0` also zeroes `PERIPHACCERR.ADDRESS` | same |
| `PERIPH[n].PERM` at reset | every present slot `SECATTR=1` (Secure) except SPU00 slots 12-15 (`0x8001000a`: slot 12 is VPR00, slots 13-15 have no peripheral in the MDK memory map); fixed-Secure (`SECUREMAPPING=1`): SPU00 0,1,5,8,9,11,23,24; SPU10 0,30; SPU20 0,28,39,63; SPU30 0,8,30,31,63; split (`=3`): SPU00 2,16,17; SPU10 2; SPU20 2,24-26,34; SPU30 2,10-12; bit 16 set on every present slot | table copied verbatim (`nrf54l_spu_perm_reset`) |
| `PERM` writes | `SECATTR` and `LOCK` writable; `DMASEC` only where the DMA field is non-zero; fixed-Secure slots ignore `SECATTR`; `PRESENT`, `SECUREMAPPING`, DMA field, bit 16 read-only | same |
| Split peripherals with `SECATTR=1` | GPIO P2 readable from Non-secure (the minimal normal world does it at boot); a GPIOTE30 channel `CONFIG` write faults (`BFAR=4010c51c`) | per-feature attributes stored, not enforced: split slots open to both worlds, so the first fault of the full-platform normal world is the CLOCK/POWER/RESET read (`BFAR=4010e600`) rather than GPIOTE30 |

The checked-in `test-tz-spu-violation-nrf54l15-xiao.yaml` images were flashed
as well: the board prints `BF! PC=... CF=00008200 HF=00000000 BFAR=4010c51c
MMFAR=4010c51c` and resets, in a loop, and never reaches the SPU handler's
"Reboot caused by SPU violation" line. The emulator prints the same line with
`BFAR=4010e600` for the reason in the last row. The `PC=`/`LR=` fields are not
comparable on either side: the fault is taken from the Non-secure thread, so
its frame is on the Non-secure stack, while Contiki's BusFault handler reads
the Secure MSP and prints whatever stale words are there (the emulator's
`ARM_EXC_TRACE=1` shows the real stacked PC, the `ldr.w` at `0x909e6`).

Not measured on the board, modelled from the architecture: the Non-secure
view of CFSR/HFSR/BFAR (RAZ/WI while `AIRCR.BFHFNMINS` is clear; MMFAR banked)
and escalation of a BusFault to HardFault when its priority cannot preempt
the current execution priority.

