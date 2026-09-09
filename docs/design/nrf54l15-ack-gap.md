# nRF54L15: the driver's own acknowledgement path

## Status: resolved

The Nordic 802.15.4 driver on the nRF54L15 acknowledges received frames
itself, in hardware time, so Contiki-NG firmware for this CPU turns CSMA's
software acknowledgement off (`CSMA_CONF_SEND_SOFT_ACK 0`). Such firmware
relies entirely on the driver to build and transmit link-layer
acknowledgements. That path never completed in the emulator: a receiver never
acknowledged a unicast frame, the sender retried until it gave up, and
two-node RPL-UDP never formed a DAG. It now works, and
`configs/test-2node-nrf54l15-dk.json` gates it.

Three model defects were found on the way. The first two were unambiguous
bugs against the vendor's register descriptions; the third is a timing
constant whose correct value was only discoverable once the first two were
fixed, and whose previous value had been hiding a real firmware bug.

## What the driver does

There is no chip-level automatic acknowledgement to model on this part. The
driver transmits the acknowledgement itself, timed off TIMER10 and the
distributed interconnect:

1. The frame ends. `EVENTS_END` triggers the disable shortcut, and
   `EVENTS_DISABLED` starts TIMER10 through the interconnect.
2. The receive interrupt handler runs, waits for the radio to report
   disabled, tears down the receive ramp-up chain, and calls
   `nrf_802154_trx_transmit_ack()`.
3. That function programs the acknowledgement ramp-up time into TIMER10 CC1,
   publishes CC1 to the channel the radio's TXEN task subscribes to, then
   captures the live timer into CC3 and sanity-checks it:

   ```c
   if ((timer_cc_now < timer_cc_ramp_up_start) &&
       ((timer_cc_fem_start >= timer_cc_ramp_up_start) ||
        (timer_cc_now > timer_cc_fem_start)))
   ```

   If the captured value is not strictly between the front-end start value
   (CC0, set to 2) and the ramp-up target (CC1, 131), the driver concludes it
   is too late, tears the acknowledgement down, and returns to receive.

## Defect 1: interconnect channel groups were not modelled

The driver puts its ramp-up channel in a channel group and subscribes that
group's disable task to the channel itself, so the chain that starts a
receive or transmit fires exactly once and disarms itself. Without groups the
chain stayed armed: the disabled event at the end of every operation
re-triggered it, and every transmission went on air twice. The model had
been compensating with an eight microsecond delay on the disabled event,
which gave the firmware time to tear the chain down by hand first.

Groups are now modelled, including the rule that a group task raised from
inside a publish is applied after that publish has reached every subscriber,
which is what makes the self-disable a one-shot rather than a race.

## Defect 2: the timer's compare-stop shortcut was read from the wrong bit

The model checked bit 8+n, the layout of older nRF5x parts. On this SoC it is
bit 16+n. TIMER10, armed to stop on compare 0, therefore never stopped, and
the timestamp the driver sampled in step 3 came back in the millions of ticks.
That alone was enough to abandon every acknowledgement.

A related fix landed first: GRTC and TIMER deadlines were converted to a fire
cycle against `sim_time_ns`, a time mirror the mote layer pins only at slice
boundaries, so every deadline computed from the cycle clock landed late by
the gap. They are now converted against the cycle clock they were computed
from. The window below was measured with that in place.

## Defect 3: the disabled-event delay, and what it was hiding

With the first two fixed, the driver's check still failed: the timer had not
started when it was sampled, because the eight microsecond delay on the
disabled event put the timer's start after the sample.

The delay cannot simply be removed. On silicon the CRC result is known a few
cycles before the frame's end event, so the driver's receive interrupt handler
is already running when the disabled event arrives. The emulator raises the
CRC event, the end event and the disable in a single instant, and can only
enter the handler at the next instruction boundary. The delay stands in for
that head start, and it has to satisfy both sides of the handler:

- not before about 2.5 µs, so the handler reaches the point where it tears
  down the ramp-up chain before the disabled event fires, or the chain
  re-arms the receiver under it;
- not after about 4 µs, so the timer the disabled event starts has advanced a
  few ticks by the time the handler samples it, roughly 6.6 µs after the CRC
  event.

Measured with two-node RPL-UDP: 2 µs fails on the first side, 5 µs on the
second, 2.5 to 4 µs pass. The default is now 3 µs.

That sweep produced an apparent contradiction: every value that made the
driver's acknowledgement work made the previously-passing two-node DK test
fail. The reason turned out to be a genuine bug in the firmware that test
used. Once the driver's acknowledgement worked, that firmware acknowledged
every unicast frame **twice**, once from the driver and once from its CSMA
layer, and the sender's acknowledgement-timeout logic asserted on the second,
unexpected one. That is the bug Contiki-NG's "nrf54l15: acknowledge received
frames in hardware only" fixes. The old test had only ever passed because the
emulator's driver path was broken, leaving the CSMA acknowledgement as the
only one on air.

The checked-in nRF54L15-DK RPL-UDP images are therefore rebuilt from a
Contiki-NG that carries that fix (see `firmware/nrf54l15-dk/PROVENANCE.md`).
The older images are not wrong to keep as history, but they are not a valid
gate for a radio model that acknowledges correctly.

## What remains approximate

The delay is a proxy for an ordering, not a physical latency, and a firmware
whose handler reaches the two points above at very different times would
need a different value. The faithful model would give the radio's ramp-up and
ramp-down real duration, so that `STATE` reads what silicon reports while the
handler runs, and the disable-to-ramp-up chain cannot complete inside one
instruction. That would make the firmware's own polling absorb the timing and
remove the constant. Nothing in the tree needs it today.

## Reproducing

```sh
./build/test_runner test configs/test-2node-nrf54l15-dk.json
NRF54L_DISABLED_DEFER_NS=8000 ./build/test_runner test configs/test-2node-nrf54l15-dk.json   # the old value: fails
```

Useful traces: `NRF54L_RADIO_TRACE=1`, `NRF54L_RADIO_TASK_TRACE=1`,
`NRF54L_TIMER_TRACE=1`, `NRF54L_DPPI_TRACE=1`.
