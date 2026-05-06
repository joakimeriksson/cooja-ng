# L6 RPL-UDP — RESOLVED 2026-05-06

> **L6 is now passing in csim.** Resolution came from two upstream
> Contiki-NG firmware fixes, not from csim changes. See
> [`STATUS.md`](STATUS.md) §"L6 RPL-UDP — resolved" for the full
> narrative; this file is kept for the historical investigation trail.
>
> The original tactical items below (L6-1 through L6-6) were
> *symptoms* of the firmware bugs amplified through csim's faithful
> emulation. With the firmware fixed, the symptoms vanish without any
> csim-side change.

## Resolution summary

Two Contiki-NG PR branches:

1. `fix/zoul-cc1200-ack-wait` — Zoul `CSMA_CONF_ACK_WAIT_TIME`
   `RTIMER_SECOND/200` (5 ms) → `RTIMER_SECOND/40` (25 ms).
   The 5 ms default is below the cc1200 driver's actual ACK
   round-trip (~12.5 ms measured). Per IEEE 802.15.4-2015 §6.4.5.4,
   spec macAckWaitDuration on SUN FSK 50 kbps is ~3.6 ms, but
   Contiki's cc1200 software auto-ACK path is much slower than spec.
2. `fix/cc1200-pending-packet-race` — `clock_delay_usec(300)` after
   the SPI register read in `pending_packet()`. Without it, CSMA's
   `RTIMER_BUSYWAIT_UNTIL` polls so tightly that the rapid
   `LOCK_SPI`/`single_read`/`RELEASE_SPI` cycle starves the cc1200
   RX IRQ chain — ACK is received over the air but never delivered
   to MAC. Race had been masked for years by the `INFO("RF: Pending")`
   printf, which provided the same throttle as a UART-blocking side
   effect when `DEBUG_LEVEL >= 3`.

## Current L6 baseline with fixed firmware (2026-05-06)

```
$ ./build/test_runner zoul-firefly-multinode \
    firmware/zoul-firefly/udp-server-subghz-fixed.zoul-firefly \
    firmware/zoul-firefly/udp-client-subghz-fixed.zoul-firefly \
    -t 60000 -d 200

Total RF bytes:    2 242            (was 101 988 — ~50× less)
Emu RX frames:     26 direct, 1 queued, 1 drained, 0 dropped, 0 collided
Node 1 (server):   961 M cycles, 10.9 M instructions
Node 2 (client):   961 M cycles, 11.8 M instructions
RPL-UDP:           6/6 hello request/response cycles complete
Speed:             9.4× real-time
```

Same firmware also converges on real Zolertia Firefly hardware (see
[`HARDWARE-TEST.md`](HARDWARE-TEST.md)).

## Original baseline (pre-fix) for comparison

```
Total RF bytes:    101 988
Emu RX frames:     80 direct + 214 queued + 150 drained + 672 dropped
Node 1 (server):   1 010 M cycles,    50 M instructions   ← mostly idle WFI
Node 2 (client):   1 702 M cycles, 1 216 M instructions   ← active retx storm
Firmware-level:    "Not reachable yet" every ~9 s, no DAG, no UDP exchange
```

230 frames per minute reached Node 1's chip; 50 ACKs emitted; RPL
never bootstrapped — exactly the failure mode the firmware bugs
produce. The csim emulation was correct; it was reporting a real bug.

## Original investigation items (historical, all resolved)

### L6-1: CC1200 `rx_incoming[]` buffer for transient MARC≠RX windows

- **Status**: open, blocked on user policy decision.
- **Evidence**: [`CC1200-RX-ACK-CHAIN.md`](CC1200-RX-ACK-CHAIN.md)
  §"Cumulative diagnosis". Of 19 977 air bytes presented to Node 1's
  chip in a 30 s L6 run, **3 569 (18%)** are dropped at
  `src/arm/cc1200.c:282` — the `if (marcstate != CC1200_MARC_RX) return;`
  gate at the top of `cc1200_receive_byte`. The chip is in
  IDLE/SETTLING during the firmware's own `idle()` → `SIDLE` →
  `prepare()` → `idle_tx_rx()` ACK transmit window (~3 ms), and Node 2's
  CSMA retransmit storm hits during that window.
- **Hypothesis**: real CC1200 silicon (per SWRU346B p. 32-34) keeps
  the AGC + demodulator pipeline running on the RX channel during
  RX→TX→RX turnaround; only the in-flight frame is lost, not the next
  frame's preamble. Mirror the `rx_incoming[]` pattern from
  `src/msp430/cc2420.c` — buffer up to ~32 bytes when MARC is in a
  transient state with `marc_pending == CC1200_MARC_RX`, replay them
  through the air decoder when MARC transitions back.
- **Fix sketch** (~25 lines):
  ```c
  // include/arm/cc1200.h:
  uint8_t rx_incoming[32];
  int     rx_incoming_count;

  // src/arm/cc1200.c:cc1200_receive_byte
  if (c->marcstate != CC1200_MARC_RX) {
      if (c->marc_pending == CC1200_MARC_RX &&
          c->rx_incoming_count < 32) {
          c->rx_incoming[c->rx_incoming_count++] = byte;
      }
      return;
  }

  // src/arm/cc1200.c:marcstate_event_cb (when transitioning to RX):
  if (c->rx_incoming_count > 0) {
      int n = c->rx_incoming_count; c->rx_incoming_count = 0;
      for (int i = 0; i < n; i++) cc1200_receive_byte(c, c->rx_incoming[i]);
  }
  ```
- **Verify**: `./build/test_runner zoul-firefly-multinode .../udp-server-subghz.zoul-firefly .../udp-client-subghz.zoul-firefly -t 60000 -d 200` — expect Emu RX direct count to rise substantially; RPL convergence likely (predicted ≥80% ACK success).
- **Open question (gating decision)**: The user's "no cheating" policy
  applies. We need to either (a) re-verify SWRU346B p. 32-34 ourselves
  to confirm AGC-during-turnaround is documented behavior, or (b)
  acknowledge this is a fidelity-vs-convenience tradeoff matching what
  cc2420.c already does (which also has `rx_incoming[]` per CLAUDE.md).
  cc2420's RPL-UDP works in csim because of this buffer; CC1200 doesn't
  because it lacks one. Before implementing, decide explicitly.

### L6-2: Node 1 (receiver) CPU starvation symptom

- **Status**: open, mitigated but not resolved.
- **Evidence**: Node 1 retires 50 M instructions in 60 s (~1.5 s of
  CPU time at 32 MHz) while Node 2 retires 1 216 M (~38 s). Node 1's
  cycle counter advances to 1 010 M cycles — so the CPU IS being
  stepped — but most cycles are spent in WFI (cycles count without
  retiring instructions).
- **Hypothesis**: when L6-1 lands, more frames will reach firmware,
  ISR will run more often, instruction count should rise. Node 1 isn't
  fundamentally starved by the harness (commit `7b9b26d` already fixed
  the `schedule_emulated_wakeup` cycle-vs-time issue that caused
  `tsch-drift-z1` to hang); it's idle because there's nothing to do.
- **Fix**: probably resolves with L6-1. If not after L6-1 lands,
  re-investigate.
- **Verify**: post-L6-1, Node 1 should retire ≥500 M instructions in
  60 s if it's actually processing frames.

### L6-3: Sub-GHz collision / queue accounting

- **Status**: open, lower priority.
- **Evidence**: 672 dropped frames in current L6 baseline. Drops are
  at `emu_rx_queue_push()` queue-full path, even at
  `EMU_RX_QUEUE_SIZE = 64`.
- **Hypothesis**: with L6-1 the queue should drain faster (firmware
  reads frames promptly), so the queue-full pressure should drop. If
  the queue is still saturating at 64, bump to 128 and/or investigate
  whether the per-byte deliver path can avoid queueing in the
  synchronous-deliver case.
- **Fix sketch**: trivial size bump if needed, or a deeper look at
  whether we should be queueing at all when the synchronous deliver
  branch is available.
- **Verify**: L6 stats show "dropped" near zero post-L6-1. If not,
  bump queue size and recheck.

### L6-4: ACK turnaround timing

- **Status**: open, **promoted to high-priority** after 2026-05-05
  hardware test (see [`STATUS.md`](STATUS.md) §Hardware test result).
- **Evidence**: Audit step 10. csim emits ACKs ~1 byte-period after
  the RX_END event; real CC1200 ACK turnaround is closer to the chip's
  `tx_rx_turnaround` window (typically ~10 ms per
  `cc1200-802154g-863-870-fsk-50kbps.c`). Contiki's
  `CSMA_CONF_ACK_WAIT_TIME` on Zoul is 5 ms (override of the
  csma.h default 400 µs).
  Hardware run on 2026-05-05 shows **server→client unicast takes 8 MAC
  retxs per packet** (server: `status 2, tx 8`; client: 7×
  `drop duplicate link layer packet` per seqno). Data arrives every
  time, but auto-ACK doesn't land in the sender's wait window —
  exactly the failure mode this item describes, on real silicon.
- **Hypothesis (revised)**: csim's synchronous auto-ACK (zero
  turnaround latency) makes csim *too forgiving* — 1-tx delivery
  where hardware shows 8-tx. To reproduce hardware faithfully, model
  CC1200 TX→RX→ACK turnaround as a scheduled event ~160–200 µs after
  RX_END, not synchronous. Also worth confirming whether Contiki's
  `CSMA_CONF_ACK_WAIT_TIME = 5 ms` is actually being honored on the
  sender side — if the sender closes its wait window earlier, that's
  a Contiki bug independent of csim.
- **Verify**: after modeling turnaround latency, csim should
  reproduce a non-zero retx rate on unicast frames. Compare
  retx-per-packet in csim trace vs hardware logs (currently 0 vs 7).
- **Cross-ref**: this is now arguably a more important fidelity gap
  than L6-1, because hardware proves the firmware *does* converge —
  the simulator's job is to faithfully reproduce that path, including
  its inefficiencies.

### L6-5: Sub-GHz CCA — RSSI / CARRIER_SENSE settling time

- **Status**: open, fidelity item.
- **Evidence**: Commit `076402a` added basic CCA via `RSSI0`. Real
  CC1200 has a CCA settling time after entering RX (~tens of µs to a
  few hundred µs depending on AGC config — SWRU346B §6.9.1). csim
  asserts `CARRIER_SENSE_VALID` immediately on entering RX.
- **Hypothesis**: not a convergence blocker. CSMA backoff with
  CCA-always-clear is too aggressive and contributes to retransmit
  storms, but the specific L6 failure is at a different layer.
- **Fix sketch**: schedule a `host->schedule_ns()` event ~200 µs after
  entering RX that sets `c->sig_cs_valid = true` and propagates.
- **Verify**: CCA reads return "invalid" briefly after SRX, then
  "clear" or "busy" depending on neighbor TX state.

### L6-6: Firmware-level `CSMA_CONF_MAX_FRAME_RETRIES` tuning

- **Status**: **resolved by hardware test 2026-05-05.** Stock
  Contiki firmware with default `CSMA_CONF_MAX_FRAME_RETRIES` *does*
  converge on real Firefly hardware (RPL DAG forms, UDP exchange
  works). The retx storm is real (8 tx per unicast — see L6-4) but
  the default retry limit is sufficient to push frames through. So
  the L6 csim failure is not a Contiki tuning gap; it's csim
  emulation gaps. The productive direction is closing those gaps
  (L6-1, L6-4 first), not retuning the firmware.

## Closed items (kept for context)

### L6-C1: Sub-GHz RX FIFO accounting (fixed in `164f6e4`)

`emu_rx_queue_drain` was reading length from `data[5]` (correct for
802.15.4 SFD-relative offset) for sub-GHz frames where `data[5]` is
the second byte of the 32-bit sync word. Fixed by routing through
`frame_fifo_bytes()` per profile + bumping `EMU_RX_QUEUE_SIZE` 16→64.
Result: 5 → 80 direct deliveries.

### L6-C2: Past-time delivery deadlock (fixed in `7a28708`)

`accurate_tx_start` was 0 for sub-GHz frames because `first_byte_ns`
never armed on `0x55` preamble. `emu_deliver_bytes` then scheduled
receiver wakeups in the past (~3 s), the inner event loop thrashed,
Node 2 CPU starved. Fixed by anchoring sub-GHz delivery to
`current_sim_ns + frame_air_dur`.

### L6-C3: Startup-delay double-count (fixed in `4ebc68b`)

`node_start_ns[i] = node_sim_time_ns(i) + delay_ns` after
`cpu->sim_time_ns += delay_ns` doubled the delay for ARM nodes.

### L6-C4: TSCH drift-z1 regression (fixed in `7b9b26d`)

Not L6, but same root cause class as L6-2: `schedule_emulated_wakeup`
read `cpu->sim_time_ns` (rolled back inside chip event callbacks by
`execute_events`) to compute next sim_eq wakeup. Sender wakeups
landed in the past, sim_eq replaced legitimate forward-looking
entries, Node 1 trapped in a backward-time loop. Fixed by anchoring
to `current_sim_ns`. Restored `tsch-drift-z1` and the full Cooja
suite to 81/81 non-skipped.

### L6-C5: CC1200 IOCFG multiplexing (refactor commits `8a2d03b` →
`72665bb`)

15 commits implementing the per-radio model + chip-driver channel
push. Restored TSCH channel matching for emulated chips. See
[`docs/radio-medium.md`](../../docs/radio-medium.md).

## How to update this doc

Treat each open item as a checkbox. When you investigate or fix one:

1. Add empirical evidence (trace excerpt, file:line, exact stat
   numbers) to the existing section.
2. If you fix it: move the section to "Closed items" with a one-line
   summary + commit SHA.
3. If you discover a new blocker: add it as a new section, citing the
   evidence that surfaced it.
4. Update the "Current L6 baseline" block with the new measurement.

Goal: this doc + `STATUS.md` together should be enough that a fresh
contributor (human or agent) can pick up L6 work without re-reading
the entire chat history.
