# nRF54L15-DK Port — Status & Direction

> Strategic doc: where the port is, where it's going, and what's
> explicitly out of scope. For the device contract see
> [`SPEC.md`](SPEC.md).

## Current state — short version

**L3 reached — full Contiki banner prints on emulated nrf54l15-dk.**

```
[INFO: Main      ] Starting Contiki-NG-release/v5.1-164-gf15d82e66
[INFO: Main      ] - Routing: RPL Lite
[INFO: Main      ] - Net: sicslowpan
[INFO: Main      ] - MAC: CSMA
[INFO: Main      ] - 802.15.4 PANID: 0xabcd
[INFO: Main      ] - 802.15.4 Default channel: 26
[INFO: Main      ] Node ID: 0
[INFO: Main      ] Link-layer address: f4ce.3600.0000.0000
[INFO: Main      ] Tentative link-local IPv6 address: fe80::f6ce:3600:0:0
Hello, world
```

End-to-end: hello-world.nrf54l15-dk runs through libc init, nrfx
clock startup, GRTC, Nordic 802.15.4 driver init, Contiki netstack
init, autostart, and into the `hello-world` process — all
peripherals it touches modelled or accepted as no-ops, all bytes
delivered via UARTE20 EasyDMA.

### The BFI bug that gated this

The biggest blocker turned out to be a Thumb-2 interpreter bug,
NOT a missing peripheral.

Nordic's `nrf_802154` driver stores the IEEE 802.15.4 channel as a
**5-bit bitfield** (`uint8_t channel : 5`).  The setter compiles to

    bfi r2, r0, #3, #5

`BFI`'s op-code in the ARMv7-M plain-binary-immediate group is
`op[4:0] = 10110 = 0x16`.  Our interpreter routed it with
`(op & 0x1C) == 0x18`, a mask that captures 0x18..0x1B but **misses
0x16** — so every BFI fell through to the SBFX path (0x14..0x17,
since 0x16 & 0x1C = 0x14), which extracted bits instead of
inserting them.  The store-back wrote garbage to the bitfield byte.

Result: `nrf_802154_pib_channel_set(26)` silently left
`m_data.channel = 0` in SRAM.  Later, `nrf_802154_trx_enable` read
`nrf_802154_pib_channel_get() = 0` and called
`channel_set(0)`, tripping
`NRF_802154_ASSERT(channel >= 11U && channel <= 26U)`.

Fix: route the plain-binary-immediate sub-opcodes by *exact* op
value (BFI=0x16, SBFX=0x14, BFC=0x16+Rn=PC, UBFX=0x1C), with a
mask of `& 0x1E` for the SSAT/USAT pairs.  Locked in by four
regression tests in `test/test_arm_correctness.c::test_bit_field_ops`.

This silently broke any C code on csim that touched bitfields
wider than 1 bit at a non-zero offset.  It just happened to be
the nrf54l15 port that hit it first because Nordic's 802.15.4
driver packs PIB fields tightly.

The Thumb-2 interpreter has run >10 M instructions of
`hello-world.nrf54l15-dk` end-to-end without a single `undef`. Track
A (ARMv8-M ISA gaps) stays burnt down — no M33-specific opcodes hit
through libc init, nrfx setup, GRTC start, and into `main()`.

**Boot stages cleared by GLOBAL_CLOCK alone:**

  1. **Reset_Handler → libc init** — flat memory + Thumb-2 interpreter.
  2. **`nrfx_clock_start`** — HFXO start handshake (task @ 0x010,
     event @ 0x108).
  3. **`clock_init`** — domain-running status bit at 0x44c (set when
     enable bit at 0x440 is written).
  4. **`nrfx_grtc_init` / `_syscounter_start`** — pass through without
     blocking (GRTC not modelled yet; no spin trigged).
  5. **`nrf_802154_clock_init`** — HFCLK + LFCLK start (canonical
     tasks @ 0x000/0x008, events @ 0x100/0x104).  This driver uses
     **different offsets within GLOBAL_CLOCK** than nrfx does; both
     pairs are needed.
  6. **`main()` reached** — firmware enters the Contiki main loop and
     attempts to printf the boot banner.

**Where the firmware parks now**: boot path is fully complete; the
Contiki main process is alive, the radio process is alive, but
nothing on-air happens.  The radio init writes ~127 RADIO config
registers at `0x5008_A000`, configures DPPI subscriptions
(`*[0x104] = 0x80000017` — channel 23 subscribing to RADIO event
4), then **waits for DPPI channels to publish events**.  No
`TASKS_TXEN` / `TASKS_RXEN` ever fires.

### Address aliasing — the 0x5xxx window

Contiki's nrf54l15 build addresses every peripheral via the
**`0x5xxx_xxxx` alias**, NOT the `0x4xxx_xxxx` non-secure base
that the SVD lists first.  Confirmed by literal-pool dumps of the
firmware:

| Peripheral        | csim base       | SVD "_NS"       | SVD "_S"        |
| ----------------- | --------------- | --------------- | --------------- |
| GLOBAL_CLOCK      | `0x5010_E000`   | `0x4010_E000`   | `0x5010_E000`   |
| UARTE20           | `0x500C_6000`   | `0x400C_6000`   | `0x500C_6000`   |
| GRTC              | `0x500E_2000`   | `0x400E_2000`   | `0x500E_2000`   |
| RADIO             | `0x5008_A000`   | `0x4008_A000`   | `0x5008_A000`   |

Every new peripheral added to this port must use the 0x5xxx base.

### Why RADIO needs DPPI before it engages

Nordic's `nrf_802154` driver is built around the **DPPI** fabric:
peripheral A *publishes* an event on a DPPI channel, peripheral B
*subscribes* that channel to one of its tasks.  The actual radio
ramp-up / TX kick is driven by GRTC compare → DPPI channel →
`TASKS_TXEN`, **never by the CPU writing the task register
directly**.

That's why even with the GRTC modelled and ticks firing, RADIO
sees no new task triggers: the GRTC compare event has nowhere to
go without a DPPI channel routing it.

**DPPI fabric is now in place** (collapsed to one 32-channel global
state aliased across DPPIC00/10/20/30 at `0x5004_2000`, `0x5008_2000`,
`0x500C_2000`, `0x5010_2000`).  Confirmed via instrumented run:
udp-server's boot path writes `CHENSET=0x000045f8` and
`CHENSET=0x00800000` to DPPIC10 — enabling channels 3..8, 10, 14,
plus channel 23 which RADIO subscribed to via `SUBSCRIBE_RXEN`.

GRTC compare-fire now publishes on its configured channel via
`PUBLISH_COMPARE[n]` (bit 31 = EN, bits 4..0 = channel id) →
`nrf54l_dppi_publish`.  No publishes log yet because MPSL only
programs `PUBLISH_COMPARE` when an active radio operation is
requested, and that's gated on a RADIO model existing.

Subscribers (e.g. future RADIO SUBSCRIBE_TXEN/_RXEN) register via
`nrf54l_dppi_subscribe(soc->dppi, channel, callback, user)`.

### Order of work for real RX/TX

1. ~~**DPPI**~~ ✅ — fabric with publish/subscribe.
2. ~~**RADIO**~~ ✅ — state machine + EasyDMA + DPPI integration.
3. ~~**EGU**~~ ✅ — software-trigger source for the DPPI fabric.
   Discovered while tracing: Nordic's nrf_802154 driver uses
   EGU10.TASKS_TRIGGER[n] → EGU10.PUBLISH_TRIGGERED[n] → DPPI ch →
   RADIO.SUBSCRIBE_RXEN as its "kick" point for radio ramp-up.
4. ~~**Multinode harness wiring**~~ ✅ — `.nrf54l15-dk` extension
   detection, TX listener install, byte delivery to
   `nrf54l_radio_receive_byte`.
5. ⚠️  **Tick rate** — much improved, but still blocked.  Multiple
   GRTC modelling bugs fixed this iteration, in order of how badly
   they broke things:

   - **`CPSIE i` / `MSR PRIMASK` didn't re-evaluate pending IRQs.**
     A bug in the Thumb-2 interpreter, not GRTC: when PRIMASK
     cleared, our interpreter set the flag but never called
     `arm_nvic_check_pending`, so any IRQ that pended during a
     critical section stayed deferred forever (until the next
     unrelated `set_pending`).  Fixed in src/arm/arm_cpu.c.
   - **Contiki reads SYSCOUNTER[2] (application core), not [0].**
     The cluster is at offset 0x720 with 4 entries (one per CPU
     view) of stride 0x10.  Our model only handled offset 0x724
     (SYSCOUNTER[0].H).  Contiki reads 0x744 (SYSCOUNTER[2].H).
     Now collapses all 4 entries onto the same logical counter.
   - **SYSCOUNTERH.LOADED is bit 29, not bit 31.**  Setting bit
     31 (OVERFLOW) instead made nrfx think the counter overflowed
     every read.
   - **GRTC INTPEND2 not modelled.**  The dispatcher at
     `GRTC_2_IRQHandler` reads `INTPEND2` (offset 0x32C) to find
     which CC fired.  Returning 0 made the dispatcher exit
     without calling the user handler; returning the correct
     pending&enabled bitmap fixes dispatch.

   Despite all these fixes, the IRQ handler **still does not
   re-arm the CC after the first tick** — `nrfx_grtc_syscounter_cc_relative_set`
   apparently still returns an error.  Likely some nrfx-internal
   SRAM state we don't initialise (channel allocator bitmap,
   active-flag bookkeeping, etc.).  Next session: instrument
   the actual return value of nrfx_grtc_syscounter_cc_relative_set
   to see which error branch it takes.

Once tick rate is fully fixed, RPL-UDP should work end-to-end.

The Contiki-NG nrf54l15 port (`arch/cpu/nrf/nrf54l15/` +
`arch/platform/nrf/nrf54l15/dk/`) merged recently — author has
confirmed it runs on real PCA10156 hardware. We have a known-good
firmware baseline to emulate against.

### Empirically discovered GLOBAL_CLOCK register map (so far)

Reverse-engineered from `objdump -d` of the Contiki nrf54l15
hello-world. Each TASKS_* sets the matching EVENTS_* immediately on
write of 1 (real HW takes hundreds of µs; csim shortcuts to zero
latency for polling firmware).

| Offset  | Direction | Name (inferred)        | Purpose                                              |
| ------- | --------- | ---------------------- | ---------------------------------------------------- |
| 0x000   | W         | TASKS_HFCLKSTART       | nrf_802154 driver kicks main HFCLK                   |
| 0x008   | W         | TASKS_LFCLKSTART       | nrf_802154 driver kicks LFCLK                        |
| 0x010   | W         | TASKS_HFXOSTART        | nrfx_clock_start kicks HFXO crystal                  |
| 0x100   | R/W       | EVENTS_HFCLKSTARTED    | latched on TASKS_HFCLKSTART                          |
| 0x104   | R/W       | EVENTS_LFCLKSTARTED    | latched on TASKS_LFCLKSTART                          |
| 0x108   | R/W       | EVENTS_HFXOSTARTED     | latched on TASKS_HFXOSTART                           |
| 0x440   | W         | DOMAIN_ENABLE          | clock-domain enable; gates the 0x44c status bit      |
| 0x44c   | R         | DOMAIN_STATUS          | bit 16 = "domain running"; polled by clock_init      |

## Why this port

- **nRF54L15 is Nordic's first ARMv8-M / Cortex-M33 part** with BLE 5.4
  + 802.15.4 — covers a different ISA generation than the M3/M4F we
  already support, and the platform's first ARMv8-M port forces us to
  flesh out cm33-specific cpu state we haven't needed before.
- **The chip uses Nordic's full `nrf_802154` SDK driver**, not a
  Contiki-native driver like nrf52840-ieee.c was. That driver depends
  on **DPPI** (Distributed PPI) + **GRTC** (Global RTC) — both new
  peripherals we'll need to model.
- **Two reference boards already supported by Contiki-NG**: PCA10156
  (Nordic dev kit) and Seeed Studio XIAO nRF54L15. DK is the natural
  primary target; XIAO is a follow-up that should be cheap once the
  SoC is done (same pattern as `nrf52840-dk` after `nrf52840-dongle`).

## Scope split

Same shape as the nRF52840 port — two parallel tracks after a small
shared baseline.

### Track A — ARMv8-M / Cortex-M33 support (CPU side)

Mostly empirical: extend `src/arm/arm_cpu.c` to handle ARMv8-M deltas
**only when firmware actually traps**. Expected delta surface:

  - **Stack-limit registers** (`MSPLIM`, `PSPLIM`). M33 traps if SP
    crosses the limit. Add as `cpu->msplim` / `cpu->psplim` fields and
    check on stack writes — but only if firmware programs them. Many
    Contiki configs leave them at 0 (no-op).
  - **VFP** — FPv5-SP, superset of M4F's FPv4-SP-D16. Contiki nrf54l15
    builds with **soft-float ABI** (verified in the ELF flags) so this
    is likely a no-op for the networking path. Existing `arm_vfp.c`
    covers what we'd need anyway.
  - **TrustZone-M** (`SG`, `BXNS`, `BLXNS`, `SAU/IDAU`, secure/non-secure
    register banking) — Contiki-NG's nrf54l15 Makefile has a
    `TRUSTZONE_SECURE_BUILD` option but defaults to non-secure-only.
    L0–L6 ignores the secure side entirely.

### Track B — nrf54l15 peripherals

This is the bulk of the work. Cannot reuse `nrf52840_soc.c` — register
addresses, peripheral set, and IRQ map are all different.

In rough dependency order:

  1. **Boot path** — `nrf54l15_config` + minimal `nrf54l15_soc_ops`
     + platform_nrf54l15_dk entry. Just enough to load an ELF and
     reset.
  2. **GLOBAL_CLOCK** — replaces nRF52's CLOCK + POWER (different
     register layout, task/event split). Firmware spins on
     HFCLKSTARTED early in boot.
  3. **GRTC** (Global RTC) — Contiki's tick source. 52-bit counter,
     multiple CCs, event group masks. Replaces RTC0/1/2.
  4. **UARTE20 (EasyDMA)** — Console output. nrf54l15 firmware does
     NOT use the legacy register window we shortcut for nrf52840;
     must implement TXD.PTR / TXD.MAXCNT / TASKS_STARTTX correctly.
  5. **FICR / NVMC** — per-node IEEE EUI-64 derivation. Address space
     is different from nRF52840 — verify against the PS.
  6. **GPIO / GPIOTE** — port count is 3 on nrf54l15 (P0, P1, P2), vs
     nRF52840's 2. Single-channel TASKS_OUT events for LED writes.
  7. **RADIO** — at a new base, new register layout, programmed by
     Nordic's `nrf_802154` driver rather than a hand-rolled Contiki
     one. **The dominant work item.**
  8. **DPPI** — needed for the radio driver's ACK timing
     choreography. Different programming model from PPI (channels are
     globally addressable, peripherals subscribe/publish via channel
     IDs rather than discrete connection objects).
  9. **Multinode harness** wiring (`test_mixed_multinode.c`).

## Risks

1. **DPPI accuracy.** Nordic's 802.15.4 driver depends on DPPI to fire
   `TASKS_TXEN` from a timer compare exactly at the right inter-frame
   boundary. csim's event queue is integer-ns; DPPI dispatch must
   happen *before* the next CPU instruction step or ACK timing slips.
   This is the equivalent of nrf52840's PPI risk but more acute,
   because the driver actually uses it (vs Contiki's nrf52840 driver
   leaving the radio's ACK timing unconfigured).
2. **Nordic's binary 802.15.4 driver internals.** The driver lives in
   `arch/cpu/nrf/lib/sdk-nrfxlib/nrf_802154/` — most of it is C source,
   but it's a bigger codebase than the Contiki nrf52840 driver and
   programs more registers in more sequences. Following its register
   choreography during L4–L6 debug will be slow.
3. **ARMv8-M instruction gaps** discovered late. If a rare M33-only
   opcode shows up deep in a library call, the trap is far from the
   cause. Mitigation: same `undef` + PC dump approach we used during
   the MSP430X RPT and M4 DSP work.
4. **Memory-map gaps.** The PS pages we haven't read yet may put some
   peripherals at addresses we haven't seen in the boot path. Discover
   empirically as L4 progresses.

## Out of scope

Explicitly **not** part of this port:

- **BLE.** Single-protocol (802.15.4) model in csim's medium. BLE
  timing, LL state machine, advertising channels — not in scope.
- **TrustZone-M (secure world)**. Contiki nrf54l15 runs non-secure;
  `TRUSTZONE_SECURE_BUILD=0` is the default. Modelling SAU/IDAU,
  secure stack pointer banking, and the secure-call instructions is a
  separate, much bigger port.
- **MPU**. Contiki rarely enables the MPU; if it does on nrf54l15 we
  add only what firmware programs.
- **Crypto accelerator (CRACEN)**. Not needed for plain RPL-UDP. Add
  if TSCH-encrypted firmware shows up later.
- **External chips on derivative boards**. The XIAO has an RF
  front-end switch (PWR=P2.3, SEL=P2.5) which is just a GPIO write —
  csim's existing GPIO output callback path absorbs it without a real
  switch model.

## Files

- [`SPEC.md`](SPEC.md) — device contract, memory map deltas, definition
  of done.
- [`../nrf54l15-xiao/SPEC.md`](../nrf54l15-xiao/SPEC.md) — sister
  board, deltas-only.
- `../../firmware/nrf54l15-dk/PROVENANCE.md` — how the reference ELFs
  were built.
- `../../docs/porting-a-device.md` — the L0–L6 process this port
  follows.

## Next concrete step

L0 and L1 of the guide §4 experiment are **done** — see "Current
state" above. Outcome:

  - **Track A green for the boot path.** No M33-only opcode hit in 5M
    instructions. Adding `nrf54l15_config` was the only CPU-side
    change needed to reach `nrfx_clock_start`.
  - **First peripheral known**: GLOBAL_CLOCK at `0x5010e000`, polling
    `EVENTS_HFXOSTARTED` at offset `0x100`.

**Next commit unit**: fix the GRTC compare-fire interval so
Contiki etimers actually fire fast enough for RPL Trickle.  The
RADIO model is in place and gets TASKS_RXEN triggered correctly
via the EGU → DPPI chain, so once etimers advance at the expected
~128 Hz, the netstack should start TXing DIO packets and the
medium delivery + auto-ACK paths take over.

Diagnosis hint: the symptom is `[grtc compare[6] fired]` at 7 ms,
406 ms, 800 ms, 1195 ms (~395 ms intervals on 2 nodes; ~200 ms
per node).  CCADD=7813 should produce ~7.8 ms intervals.  Either
the sim_time_ns at the IRQ handler's re-arm point has drifted
forward (multinode time-coordination issue), or the
`arm_schedule_event_ns` event-queue path is clamping minimum
delay.
