# Zoul Firefly Port — Status & Direction

> Strategic doc: where the port is, where it's going, and what's
> explicitly out of scope. For the actual operational task list see
> [`L6-PLAN.md`](L6-PLAN.md). For the device contract see
> [`SPEC.md`](SPEC.md).

## Current state — short version

**The port is functionally usable.** Everything except L6 RPL-UDP
convergence works:

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

## What's not done

**L6 RPL-UDP over CC1200**. Stack progresses far enough that 230
frames per minute reach Node 1's firmware and 50 ACKs go back, but RPL
DAG doesn't form. Detailed work-list at [`L6-PLAN.md`](L6-PLAN.md).

## Decision: do we keep chasing L6?

Three options:

### Option A — Keep going on simulation alone

Land L6-1 (CC1200 `rx_incoming[]` buffer mirror of CC2420's pattern)
and re-test. Predicted to unblock or substantially advance L6. ~25
lines of code. But: it asks us to commit to a fidelity-vs-convenience
tradeoff (the CC2420 emulator already makes this same tradeoff but
unverified) without independent confirmation.

**Cost**: ~1 focused agent session (~1 hour wall time).
**Risk**: low if the SWRU346B p. 32-34 reading checks out; medium if
we're just papering over a real-world tuning gap.
**Gates after**: re-run L6, see if DAG converges. If yes: tick the
SPEC box, port complete. If no: probably need physical hardware.

### Option B — Get physical hardware first

Two Zolertia Firefly boards (~$50 each from Mouser/Crowd Supply when
available) + a CC1200-capable sniffer (third Firefly running Sensniff
works) + USB cables. ~half-day of setup.

**Cost**: $100–200 + half-day setup.
**Benefit**: short-circuits weeks of speculative simulation
investigation. We can directly compare simulator behavior against
real hardware behavior — Wireshark captures, GDO0 timing on a logic
analyzer, etc. Also lets us answer "does Contiki RPL-UDP actually
converge on stock Firefly hardware?" — if NO, the simulator is
already more correct than the firmware build, and our L6 chase has
been chasing a phantom.
**Risk**: low. Worst case we learn something useful regardless.

### Option C — Ship as-is

Port is functionally complete except L6. Document L6 as ongoing,
move on. The architectural work delivered substantial value beyond
the immediate port. Picking L6 back up later — with hardware, with a
fresh agent, or both — costs little because the simulator state is
well-documented and well-tested.

**Cost**: minimal. Cosmetic SPEC update.
**Risk**: zero. Worst case L6 stays open indefinitely.

### My recommendation

**Option C now, Option B in parallel if hardware is easy to get.**

Reasoning:
- We've spent a lot of session time on L6. Each fix has revealed the
  next layer; each layer has been smaller than the last; we may or
  may not be near the bottom.
- The rest of the port + the infrastructure improvements are
  immediately useful to anyone doing csim work, regardless of L6.
- L6 RPL-UDP with stock Contiki on real CC1200 hardware is known to be
  finicky in the wild. Continuing to grind on it in simulation without
  a hardware reference point is increasingly speculative.
- L6-1 (Option A) is a small, well-understood fix that we *could*
  land any time and then re-evaluate. It doesn't need to be a
  decision-point now — it's a tactical item that can sit on the
  L6-PLAN list.

## What hardware would tell us

Specifically:

1. Whether the firmware ELFs we built (`udp-server-subghz` /
   `udp-client-subghz`, both `MAKE_RADIO=cc1200`) actually converge
   on real Firefly boards with default settings. If not, our
   simulator is correct and the issue is firmware-side; if yes, the
   simulator has a remaining gap we can compare against.
2. Wireshark / Sensniff capture of the actual on-air frame sequence.
   Compare against csim's `CSIM_TRACE_RADIO=1` output line-by-line.
3. Logic analyzer trace of GDO0 / SPI during a typical RX→ACK cycle.
   Compare against csim's `cc2420 node=N state X -> Y` traces.
4. Real CCA / RSSI behavior over time during RX.
5. Real strobe transition timing (we currently use 50–720 µs
   approximations from the Contiki driver source rather than measured
   silicon values).

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
