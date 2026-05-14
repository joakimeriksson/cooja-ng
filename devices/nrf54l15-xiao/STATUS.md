# Seeed Studio XIAO nRF54L15 Port — Status

> Companion to [`SPEC.md`](SPEC.md). For the full SoC narrative and
> architectural deltas, see [`../nrf54l15-dk/STATUS.md`](../nrf54l15-dk/STATUS.md)
> — the XIAO shares everything except board glue and the external RF
> switch.

## Current state — short version

**L−1 planning.** Firmware ELFs build cleanly under
`firmware/nrf54l15-xiao/`. No csim code yet — waiting on the DK port
to land the SoC peripheral models.

## Plan

Same model as `nrf52840-dk` after `-dongle`: once `nrf54l15_soc.c` is
working for the DK board, adding the XIAO is a single
`platform_nrf54l15_xiao` entry in `arm_platform.c`. Expected delta:

  - LED at P2.0, button at P0.0, UART pads at P1.9/P1.8.
  - RF-switch GPIO writes (P2.3 PWR, P2.5 SEL) accepted as no-ops via
    the existing platform GPIO output-callback path.
  - No new peripherals to model.

## What hardware testing tells us

The Contiki-NG XIAO port has a `README.md` next to it listing
working / pending features on real silicon. As of the merge:

**Working on real XIAO hardware**:
  - Build system integration
  - GRTC-based clock and rtimer (using GRTC_0)
  - GPIO HAL with nrfx v3.x API
  - UART console on UART20
  - User LED on P2.0, button on P0.0
  - RF front-end switch control (PWR=P2.3, SEL=P2.5)
  - 802.15.4 radio via Nordic `nrf_802154` driver (CSMA, ACKs)
  - IPv6 networking stack (RPL + UDP examples)

**Pending on real hardware** (deferred for csim too):
  - Low-power modes
  - Watchdog integration
  - Temperature sensor

So we have a known-good firmware reference. csim's job is to faithfully
emulate the SoC so the same firmware runs unmodified.

## Files

- [`SPEC.md`](SPEC.md) — board contract; lists only the deltas from
  the DK SPEC.
- [`../nrf54l15-dk/SPEC.md`](../nrf54l15-dk/SPEC.md) — SoC contract.
- `../../firmware/nrf54l15-xiao/PROVENANCE.md` — build commands.

## Next concrete step

Wait until the DK port reaches L4 (RADIO probe succeeds via
nrf_802154). Then add `platform_nrf54l15_xiao` + `.nrf54l15-xiao`
detection in the multinode harness in one commit, run RPL-UDP at 2
nodes, done.
