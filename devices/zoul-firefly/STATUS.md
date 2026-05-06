# Zoul Firefly Port — Status & Direction

> Strategic doc: where the port is, where it's going, and what's
> explicitly out of scope. For the actual operational task list see
> [`L6-PLAN.md`](L6-PLAN.md). For the device contract see
> [`SPEC.md`](SPEC.md).

## Current state — short version

**The port is complete. L6 RPL-UDP converges in csim with corrected
Contiki-NG firmware (2026-05-06).** Everything works end-to-end:

- L0–L4 single-node bring-up: green
- L5 nullnet broadcast over CC1200: 16 RX in 20 s, both nodes converse
- All 73 CC1200 chip mock-host unit tests pass
- All 21 CC2420 chip mock-host unit tests pass
- 235 radio_medium unit tests pass
- 81/81 Cooja regression tests pass (matches the pre-port baseline)
- cc2538dk RPL-UDP `-d 100` converges in ~10 s
- No regressions vs pre-port state on any platform

**Beyond the port itself**, this work surfaced and fixed substantial
csim infrastructure debt that benefits all platforms (not just
Firefly):

- 2 pre-existing CC2538 GPIO IRQ bugs (`f8b2a7e`, `fc78288`) — also
  benefits cc2538dk and openmote.
- Real per-radio multi-channel model with synchronous chip-side push —
  closes the "TSCH is fake" gap from before the port. Cooja-ng tests
  now actually use channel state instead of a disabled-check
  workaround.
- Architectural pitfall doc on event-driven chip drivers
  ([`docs/porting-a-device.md`](../../docs/porting-a-device.md) §8).
- Reference doc for the medium itself
  ([`docs/radio-medium.md`](../../docs/radio-medium.md)).
- A 235-test radio_medium safety net to make future refactors
  bisectable.

## L6 RPL-UDP — resolved 2026-05-06

After a hardware investigation on real Zolertia Firefly boards
(see [`HARDWARE-TEST.md`](HARDWARE-TEST.md) and
[`L6-PLAN.md`](L6-PLAN.md) for the full trail), **L6 was a Contiki-NG
firmware bug, not a csim emulation gap.** Two upstream Contiki-NG
fixes resolve it both on hardware *and* in csim:

1. **`zoul: bump CSMA_CONF_ACK_WAIT_TIME for CC1200 sub-GHz`**
   The Zoul platform default `RTIMER_SECOND/200` (5 ms) was far below
   the actual CC1200 ACK round-trip on the SUN FSK 50 kbps PHY
   (measured ~12.5 ms for a 25-byte data frame on Firefly hardware).
   Raised to `RTIMER_SECOND/40` (25 ms), with `#ifndef` guards so
   project-conf.h can override.

2. **`cc1200: throttle pending_packet() to avoid SPI starvation of RX IRQ`**
   CSMA's `RTIMER_BUSYWAIT_UNTIL` polled `pending_packet()` so tightly
   that the rapid `LOCK_SPI` / `single_read` cycle starved the cc1200
   RX IRQ chain — the ACK was received over the air but never
   delivered to MAC. Race threshold ~200–300 µs at the CC2538 SoC's
   32 MHz; `clock_delay_usec(300)` resolves it. The race had been
   masked for years by the `INFO("RF: Pending ...")` printf above the
   throttle, which provided the same throttle as a side effect of
   UART blocking when `DEBUG_LEVEL >= 3`.

Both fixes are scoped to upstream Contiki-NG; csim required no
changes. With the fixed firmware:

- 6/6 RPL-UDP hello cycles complete in 60 s in csim (was 0/6)
- Total RF bytes 2,242 (was 101,988 — ~50× less, no retx storm)
- 26 direct RX, 0 dropped, 0 collided (was 80 direct + 672 dropped)
- Speed 9.4× real-time
- Same firmware also converges on real Firefly hardware (the
  motivating hardware test)

**This means csim's CC1200 emulation was correct enough to faithfully
expose two real upstream firmware bugs.** The items in
[`L6-PLAN.md`](L6-PLAN.md) (rx_incoming buffering, Node 1 starvation,
queue overflow) were *symptoms* of the firmware bugs amplified through
csim's emulation, not gaps in the simulator. With both firmware bugs
fixed, the symptoms disappear naturally.

The two fixes are staged as upstream Contiki-NG PR branches:
- `fix/zoul-cc1200-ack-wait` (`53d219af5`)
- `fix/cc1200-pending-packet-race` (`de8f711e9`)

## What hardware wouldn't help with

- The chip mock-host suites (94 tests) — these are datasheet-driven,
  hardware re-confirmation adds nothing.
- The radio_medium model (235 tests) — pure simulator infrastructure,
  no physical analog.
- Port architecture decisions (multi-radio model, IOCFG multiplexing,
  event-driven strobes) — these are right per spec.

## Out of scope

These are explicitly NOT going to be tackled as part of this port:

- **Real PHY modeling.** csim and Cooja both treat radio as
  byte-stream routing. No demodulator, no SNR-based decoding. The
  byte-level filter + per-frame loss probability is the whole model.
- **Dual-band collision physics.** Two transmitters on the same
  channel collide; that's it. No SINR, no capture effect, no
  near-far problem modeling.
- **Real-world mesh sizes.** The architecture supports up to
  `RADIO_MEDIUM_MAX_NODES = 128` but anything beyond ~10 nodes is
  likely to hit other csim performance walls before becoming useful.
- **CC1200 in non-Contiki configurations.** The chip driver is
  tested against what Contiki's cc1200.c actually exercises. Other
  firmware that pokes obscure registers may need driver extensions.

## Files
- [`SPEC.md`](SPEC.md) — device contract, definition of done
- [`L6-PLAN.md`](L6-PLAN.md) — tactical work-list for the L6 gap
- [`CC1200-RX-ACK-CHAIN.md`](CC1200-RX-ACK-CHAIN.md) — datasheet + code
  audit of the 10-step RX→ACK chain (the diagnostic source for L6-PLAN)
- [`DATASHEET-FINDINGS.md`](DATASHEET-FINDINGS.md) — citations from
  CC1200 SWRU346B and CC2538 SWRU319C user guides
- [`../../docs/radio-medium.md`](../../docs/radio-medium.md) — how the
  radio medium routes bytes between nodes
