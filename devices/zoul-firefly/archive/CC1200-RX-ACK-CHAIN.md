# CC1200 RX-to-ACK Chain — Real vs csim Audit

## Summary

The chain is end-to-end functional: 121 frames reach Node 1's chip air decoder
in 30 s of L6 simulation, 569 GPIO_B IRQs enter the firmware, ~50 ACKs are
emitted by Node 1. The dominant divergence is **Step 3/8 — Node 1 drops 3 569
inbound air bytes (18 % of all bytes seen by the chip) because MARCSTATE is
IDLE at the moment of arrival**. Those drops happen during Node 1's own auto-
ACK transmit window (`idle()` → `SIDLE` → IDLE → `SRX` settling), and they
shred enough sync‑word and PHR bytes that Node 2's *next* retransmit lands on
a chip whose air decoder is mid‑recovery. This is the same byte‑during‑TX‑
turnaround race already documented in `DATASHEET-FINDINGS.md §6 fix #2` (the
"receiver-side time-warp"), but the dominant blocker now sits one layer down:
csim consumes inbound bytes synchronously inside `cc1200_receive_byte`,
*using the chip's current MARCSTATE at the instant the simulator dispatches
the byte*, while real silicon decouples the air sampler from MARCSTATE
transitions via the per‑byte AGC / demodulator pipeline (SWRU346B p. 32–34).
Suggested next action: **stop gating `cc1200_receive_byte` on MARCSTATE
== RX, and instead gate on "chip can physically demodulate" (true any time
the chip is on RX-side of the synthesizer — i.e. RX, RX_END, RXDCM, *plus
the back half of any RX→TX or TX→RX SETTLING* per SWRU346B p. 64)**. Concretely
this means buffering bytes that arrive during `tx_byte_event` / SIDLE‑settling
windows and replaying them when MARCSTATE returns to RX, mirroring the
existing `rx_incoming` buffer in `src/msp430/cc2420.c`.

------------------------------------------------------------------------

## Step 1 — Frame arrives on the air at Node 1's CC1200

**Real Contiki + CC1200:**
- Datasheet ref: SWRU346B p. 33 §6.5 ("Receiver Channel Filter Bandwidth")
  and p. 36 §6.7 ("Preamble Detection"); SWRU346B p. 41 §7 ("Sync Word
  Insertion / Detection"); MARC sub‑state table p. 105–106.
- Driver code: not directly involved at this stage — the chip handles
  preamble + sync + PHR autonomously while `MARCSTATE.MARC_STATE = 01101
  (RX)`. Firmware first observes the frame on the GDO0 falling edge.
- Behaviour: chip's preamble qualifier (PQT_REACHED, signal 11) trips after
  the configured number of `0x55` bytes (PREAMBLE_CFG1 = 0x19 ⇒ 4 bytes per
  the 50 kbps profile, file `cc1200-802154g-863-870-fsk-50kbps.c:157`),
  then SYNC_EVENT fires when the 32‑bit sync word `6E 4E 90 4E` matches
  (SYNC3..SYNC0 register settings, same file lines 134–137). MARC stays in
  RX (sub‑state may transition RX → RX_END for the trailing window per
  p. 105–106).

**csim emulation:**
- `src/arm/cc1200.c:280-303` — `cc1200_receive_byte` AIR_HUNT case. Bytes
  shift into a 32‑bit `sync_match` register; when it equals
  `sync_word_value(c)` the air state advances to `AIR_PHR` and the internal
  signal `sig_pkt_sync_rxtx` is asserted. `propagate_signals` then drives
  GDO0 if IOCFG0 selects signal 6.
- Behaviour: matches the real chip's "match-or-bust" sync detector.
  Crucially csim has **no preamble-byte counter** — a single `0x55` followed
  by the sync word would match. Since the test runner emits exactly four
  preamble bytes (`src/arm/cc1200.c:455-457`, the synchronous TX path), this
  divergence is invisible in steady state.

**Match?** ✓ matches functionally for this profile. Two minor caveats:
csim does not model PQT_REACHED (no preamble-quality threshold), and
csim does not differentiate the RX_END / RXDCM sub-states from RX
(`marc_2pin_state` in `cc1200.c:150-165` only reports the four 2‑pin
quadrants). Neither caveat affects the L6 chain.

------------------------------------------------------------------------

## Step 2 — PHR processing → length validation

**Real Contiki + CC1200:**
- Datasheet ref: SWRU346B p. 71 §8.2 ("Variable Packet Length Mode");
  p. 74 §8.4 ("Packet Format"); 802.15.4g §8.7 starting p. 99.
- Driver code: `arch/dev/radio/cc1200/cc1200.c:115-121` chooses
  `PHR_LEN = 1` when `CC1200_802154G == 0`, else 2. The Firefly default
  (`arch/platform/zoul/contiki-conf.h` does *not* define
  `CC1200_CONF_802154G`, so `cc1200-conf.h:90-94` leaves it at 0) → **PHR
  is 1 byte**. The 50 kbps SmartRF preset writes
  `PKT_CFG2 = 0x24` (`cc1200-802154g-863-870-fsk-50kbps.c:173`, bit 5
  FG_MODE_EN set), but `configure()` at `cc1200.c:1789-1794` overrides it
  back to `0x00` whenever `CC1200_802154G == 0`. So the chip is in standard
  mode, 1‑byte length field; the field carries **payload bytes only** (CRC
  bytes are auto-appended by the chip on top of that count).
- Behaviour: chip reads PHRA byte, compares against `PKT_LEN = 0xFF` (max
  allowed). Length > PKT_LEN → MARC_STATE goes to `RX_FIFO_ERR` (p. 105).

**csim emulation:**
- `src/arm/cc1200.c:305-356` — `AIR_PHR` case. Reads
  `PKT_CFG2 & 0x20` to pick 1‑byte vs 2‑byte PHR
  (`include/arm/cc1200.h:91` for the bit constant). For 1‑byte PHR
  (Firefly's case) the byte becomes `air_payload_total` directly, plus
  `air_crc_remaining = 2`. Length validation at line 344-352 is
  `air_payload_total > FIFO_SIZE - 3` → `RX_FIFO_ERR`, mirroring the
  hardware bound.

**Match?** ✓ matches. 1‑byte PHR semantics, length cap, and `RX_FIFO_ERR`
fall‑through all line up. Note that csim correctly handles the
*Firefly‑specific* configuration (FG_MODE off, even though the SmartRF
preset sets bit 5 — the override in Contiki's `configure()` is honored
because the test stack writes through to csim's PKT_CFG2 register file).

------------------------------------------------------------------------

## Step 3 — Payload bytes pushed to RX FIFO

**Real Contiki + CC1200:**
- Datasheet ref: SWRU346B p. 78 §10 ("FIFO Operation"); FIFO size 128 bytes.
  RXFIFO threshold pin signal `RXFIFO_THR` (signal 0) configurable via
  `FIFO_CFG.FIFO_THR`.
- Driver code: `arch/dev/radio/cc1200/cc1200.c:148-167` — when
  `CC1200_USE_GPIO2 == 0` (the Firefly default per `contiki-conf.h:124`),
  *no* FIFO threshold IRQ is configured: `GPIO2_IOCFG = MARC_2PIN_STATUS_0`
  (line 167) and the threshold comment is irrelevant. The driver waits for
  GDO0 (PKT_SYNC_RXTX) falling edge to drain the entire frame in one
  burst‑read.
- Behaviour: chip fills RXFIFO byte‑by‑byte. If RXFIFO overflows during
  reception, MARC goes to `RX_FIFO_ERR` (p. 78); RXFIFO_OVERFLOW signal 4
  asserts.

**csim emulation:**
- `src/arm/cc1200.c:358-367` — `AIR_PAYLOAD` case calls `fifo_push_rx`
  per byte. On overflow, `MARC = RX_FIFO_ERR` and `propagate_signals`
  recomputes both GDO pins.
- FIFO size: `include/arm/cc1200.h:134` = 128 ✓.

**Match?** ✓ matches. Firefly does not configure GPIO2 threshold, so the
absence of mid‑frame threshold IRQ on the csim side is irrelevant. (If the
config flips, csim would need to model RXFIFO_THR / RXFIFO_THR_PKT signals
0/1 in `gdo_signal_value` — currently both default to "false / unmodeled"
at line 180.)

------------------------------------------------------------------------

## Step 4 — End-of-frame → CRC check → status appendix

**Real Contiki + CC1200:**
- Datasheet ref: SWRU346B p. 76 §8.5 ("Packet Filtering and Status Bytes")
  + LQI_VAL register p. 105.
- Driver code: `cc1200.c:1797-1803` configures `PKT_CFG1` bit 0
  (APPEND_STATUS=1) when `APPEND_STATUS` macro is set; the macro is set
  by default for ≤125-byte payloads at `cc1200.c:165-184`. Reader at
  `cc1200.c:2511-2518` extracts CRC from `buf[bytes_read - 1] & 0x80`
  and RSSI from `buf[bytes_read - 2]`.
- Behaviour: real chip auto-computes CRC over payload, appends two status
  bytes after the FIFO payload: byte 0 = RSSI, byte 1 = `CRC_OK<<7 | LQI`.

**csim emulation:**
- `src/arm/cc1200.c:374-398` — when `air_crc_remaining` reaches 0, csim
  pushes `(uint8_t)c->rx_rssi` then `0x80` (always CRC_OK, LQI=0) to the
  RX FIFO. csim never models packet loss at the air layer (the radio
  medium drops frames before delivery; once delivered, CRC always passes).

**Match?** ✓ matches the firmware's read pattern verbatim — the driver
only inspects bit 7 of the LQI byte. The constant‑LQI=0 is harmless for
RPL routing logic (which uses ETX, not LQI).

------------------------------------------------------------------------

## Step 5 — GDO0 PKT_END falling edge fires

**Real Contiki + CC1200:**
- Datasheet ref: SWRU346B p. 18 Table "GPIO Signal Configurations" signal
  6 (PKT_SYNC_RXTX): "RX: Asserted when sync word has been received and
  de-asserted at the end of the packet."
- Driver code: `cc1200.c:188` (`GPIO0_IOCFG = CC1200_IOCFG_PKT_SYNC_RXTX`)
  selects this signal on GDO0; `cc1200-arch.h:65-67` declares
  `cc1200_arch_gpio0_setup_irq(int rising)`; called from `cc1200.c:319-320`
  with `rising=0` (falling‑edge IRQ).
- Behaviour: at end‑of‑packet, PKT_SYNC_RXTX drops; GDO0 follows; firmware
  sees a falling edge.

**csim emulation:**
- `src/arm/cc1200.c:250-258` — `frame_done_event_cb` clears
  `sig_pkt_sync_rxtx` and calls `propagate_signals`, scheduled
  +1 byte‑period (160 µs) after the last on-air CRC byte (line 396-397).
  Defer is on `sim_eq` so the main loop steps the receiver CPU forward.

**Match?** ✓ matches in mechanism. Datasheet doesn't specify the exact
end-of-packet de-assertion delay; the 160 µs csim picks is comfortably
under the Contiki driver's `RTIMER_BUSYWAIT_UNTIL` timeouts.

------------------------------------------------------------------------

## Step 6 — GPIO IRQ → NVIC pend → CPU exits WFI

**Real Contiki + CC1200:**
- Datasheet ref: SWRU319C §9.3 (CC2538 GPIO interrupt controller).
- Driver code: `arch/platform/zoul/dev/cc1200-zoul-arch.c:169-187`
  configures PB4 as input, falling-edge detect, single edge, then
  `GPIO_ENABLE_INTERRUPT(...)` and `NVIC_EnableIRQ(GPIO_B_IRQn)`. The
  shared interrupt handler is registered through `gpio_hal_register_handler`.
- Behaviour: GPIO_B IRQ pends in NVIC (IRQ #1, exception 17), CPU wakes
  from WFI, jumps to vector 17 → arch ISR → `cc1200_int_handler` →
  `cc1200_rx_interrupt`.

**csim emulation:**
- `src/arm/cc2538_gpio.c:133-161` — `cc2538_gpio_force_irq_edge` honors
  IS/IBE/IEV (commit f8b2a7e), sets RIS, pends NVIC IRQ #1 if
  `(p->ris & p->ie)`. `arm_nvic_set_pending` (`src/arm/arm_nvic.c:199-206`)
  flips ISPR bit and re-evaluates pending. `arm_nvic_check_pending`
  (`arm_nvic.c:232-289`) gates on `ispr & iser` and exits WFI via the
  `arm_exception_entry` path.
- IE 0→1 re-pend: `cc2538_gpio.c:73-83` re-pends if newly-enabled bits
  match sticky RIS (commit fc78288).

**Match?** ✓ matches. Empirical confirmation from a 30 s L6 run:
569 GPIO_B exception entries observed. The mechanism works correctly.

------------------------------------------------------------------------

## Step 7 — ISR runs → drains RX FIFO

**Real Contiki + CC1200:**
- Datasheet ref: SWRU346B p. 78 §10 + p. 75 §8.4.2 (RX FIFO read protocol).
- Driver code: `cc1200.c:2316-2561` — `cc1200_rx_interrupt()`. Reads
  `NUM_RXBYTES` (`single_read(CC1200_NUM_RXBYTES)`), extracts PHR
  (lines 2425-2444), then `burst_read(CC1200_RXFIFO, &buf[bytes_read],
  num_rxbytes)` (lines 2496-2498) to drain payload + status appendix.
- Behaviour: ISR services the entire frame in one IRQ on the falling edge
  of GDO0; never blocks, never split-read.

**csim emulation:**
- `src/arm/cc1200.c:923-995` — `cc1200_spi_exchange`. BURST_READ path at
  line 988-994 returns successive `fifo_pop_rx` results. Address tracking
  for non-FIFO burst reads at line 975-977 increments `spi_addr`.

**Match?** ✓ matches. Empirical confirmation: 121 frames complete on the
chip, firmware reads every one of them (RXFIFO end count = 64 bytes at
end of 30 s sim — half-full but not stuck). No FIFO error transitions
observed.

------------------------------------------------------------------------

## Step 8 — Frame parsed → ACK decision

**Real Contiki + CC1200:**
- Datasheet ref: this is firmware logic, no chip ref.
- Driver code: `cc1200.c:2245-2313` — `addr_check_auto_ack()`. Parses the
  frame via `frame802154_parse`, then:
  1. Address filter: pass if RX_MODE has no ADDRESS_FILTER bit, OR frame
     is an ACKFRAME, OR dest is broadcast, OR
     `linkaddr_cmp(&info154.dest_addr, &linkaddr_node_addr)` is true.
  2. Auto-ACK trigger (lines 2269-2274): RX_MODE.AUTOACK set, frame is
     DATAFRAME, `info154.fcf.ack_required` is set, AND the address
     matches `linkaddr_node_addr`. If so, build
     `ack[3] = { FRAME802154_ACKFRAME, 0, info154.seq }`,
     call `idle(); prepare(ack, 3); idle_tx_rx(ack, 3);`.
- Behaviour: pure firmware on the emulated CPU.

**csim emulation:**
- Nothing — runs on the emulated ARM CPU.

**linkaddr patching path:**
- `test/test_mixed_multinode.c:3037-3050` — patches
  `linkaddr_node_addr` for ARM nodes with the byte sequence
  `{id>>8, id&0xff, id>>8, id&0xff, …}` (8 bytes). For Node 1 → 8×
  `{0x00, 0x01}`; for Node 2 → 8× `{0x00, 0x02}`.
- The 802.15.4 framer on Node 2 reads `linkaddr_node_addr` for its source
  address, so Node 2's outgoing unicast frames carry dest = Node 1's link
  address (matched via the IPv6 → 6LoWPAN → 802.15.4 stack).
- `linkaddr_cmp` is a byte‐wise compare of `LINKADDR_SIZE` bytes (8 for
  this build, since `LINKADDR_CONF_SIZE` defaults to 8 on CC2538).
  Both sides compare identical patched bytes → match.

**Match?** ✓ matches. The 50 ACKs Node 1 emits in 30 s prove
`addr_check_auto_ack` is reaching the ACK build path. **No divergence
here.**

------------------------------------------------------------------------

## Step 9 — ACK TX path

**Real Contiki + CC1200:**
- Datasheet ref: SWRU346B Table 6 p. 13 (strobes); §6.10 p. 64 (state
  machine timing).
- Driver code: `cc1200.c:1955-2107` — `idle_tx_rx()`. Sequence:
  `SFRX → write IOCFG0=MARC_2PIN_STATUS_0 → burst_write FIFO → STX →
   poll GPIO0 (waiting for MARC[0]=1, i.e. TX) → wait TX-done
   (GPIO0 falls when MARC leaves TX) → restore IOCFG0=PKT_SYNC_RXTX →
   ENABLE_GPIO_INTERRUPTS at the very end of transmit()`.
- Behaviour: ~3 ms blocking busy-wait on the GDO0 polling loop, then
  return to RX (TXOFF_MODE_RX = 1).

**csim emulation:**
- `src/arm/cc1200.c:412-512` — `start_tx` synchronously emits all air
  bytes via `rf_tx_callback` (line 463-467), then schedules
  `tx_byte_event` 1 byte-period later to flip MARCSTATE back to RX
  (line 489-512).
- IOCFG multiplexing implemented (commit landed in
  `DATASHEET-FINDINGS.md` §5 final row): writes to IOCFG0 at line 904-906
  call `propagate_signals`, so the firmware sees MARC[0] flip on GDO0
  while polling.
- TX listener wiring: `test_mixed_multinode.c:2983-2989` registers
  `mixed_rf_tx_chip_cb` with `(node_idx, radio_idx=1)`. Inside the
  re‑entrant `mixed_rf_tx_handler_radio` (line 1428+), bytes go
  through `tx_frame_asm_feed` (line 119-222) which sniffs preamble +
  sync word + 1‑byte PHR and dispatches a complete frame back through
  the medium to Node 2.

**Match?** ✓ matches. Empirical: Node 1 emits 50 successful 3‑byte‑payload
TX frames over 30 s (verified by `[RF] Node 1 TX frame (3 bytes) → 2`
log lines starting at 12.526 s). The synchronous emit path correctly
funnels into the harness's existing 802.15.4g sub‑GHz delivery loop.

------------------------------------------------------------------------

## Step 10 — ACK appears on the air → reaches Node 2

**Real Contiki + CC1200:**
- Datasheet ref: 50 kbps profile + 4‑byte preamble + 4‑byte sync word +
  1‑byte PHR + 3‑byte payload + 2‑byte CRC = **14 air bytes = 2.24 ms
  air time** at 160 µs/byte (`cc1200-802154g-863-870-fsk-50kbps.c:67`).
  CSMA ACK_WAIT timer = `RTIMER_SECOND/200 = 5 ms`
  (`contiki-conf.h:127`). 2.24 ms ≪ 5 ms, comfortable.
- Driver / harness: Node 2's `csma-output.c:216` does
  `RTIMER_BUSYWAIT_UNTIL(NETSTACK_RADIO.pending_packet(), CSMA_ACK_WAIT_TIME)`,
  then if `pending_packet()` returns true, reads the 3‑byte ACK and
  matches against the data frame's `dsn`.

**csim emulation:**
- ACK delivery path: `test/test_mixed_multinode.c:1860-1887` —
  immediately after delivering the data frame, the harness flushes any
  bytes the receiver buffered into `rf_pending[j]` (auto-ACK
  re-entrancy) at `ack_start = accurate_tx_end + 192 000 ns` (192 µs
  turnaround) and dispatches them back to Node 2's chip via
  `emu_deliver_bytes`. 192 µs is the 802.15.4 turnaround constant; for
  CC1200 the real value is ~400 µs (`delay_before_rx`), but this is
  well below 5 ms ACK_WAIT.

**Match?** ✓ matches in concept; minor ACK turnaround delay difference
(192 µs vs ~400 µs) is harmless. Empirical: **of the 50 ACKs Node 1
emits, Node 2's chip syncs on 43 of them** (per chip stat counter —
`Node 2 CC1200: sync=43, frames_done=43`). The other 7 are eaten by
collisions or the same MARCSTATE-IDLE drop pattern affecting Node 1
(but in a milder form because Node 2 spends most of its time in RX
between transmits).

------------------------------------------------------------------------

## Cumulative diagnosis

The L6 RPL-UDP convergence failure is **not** caused by any single broken
chain step. Each of Steps 1-10 is functionally correct and produces the
expected output at least sometimes. The blocker is the **byte-level
delivery race during the receiver's own auto-ACK transmit window**,
quantified as follows from a 30 s `udp-server-subghz / udp-client-subghz`
run:

| Quantity (Node 1, the receiver) | Count | Source |
|---------------------------------|-------|--------|
| Air bytes presented to chip      | 19 977 | `cc1200_receive_byte` entries |
| Bytes dropped: chip in IDLE      |  3 569 | `marcstate != RX, == IDLE` |
| Bytes dropped: chip in SETTLING  |    165 | `marcstate != RX, != IDLE` |
| Bytes accepted into air decoder  | 16 243 |  |
| Sync-word matches                |    132 | `c->stat_rx_packets` precursor |
| Frames complete (CRC + appendix) |    121 | `c->stat_rx_packets` |
| Auto-ACKs Node 1 emits           |     50 | `[RF] Node 1 TX frame (3 bytes)` |
| Frames Node 2 receives back      |     43 | `Node 2 CC1200: frames_done=43` |

The **18 % air-byte drop rate during MARCSTATE != RX** is the root cause:

1. Node 2 sends a unicast data frame at time *T*.
2. Node 1's chip receives it (~17 ms on air), fires GDO0 falling edge.
3. Node 1's firmware ISR runs `cc1200_rx_interrupt` →
   `addr_check_auto_ack` → `idle()` (which strobes `SIDLE` and disables
   GPIO IRQ) → `prepare(ack, 3)` → `idle_tx_rx(ack, 3)`. This sequence
   takes ~3 ms wall-clock in the firmware, dominated by
   `RTIMER_BUSYWAIT_UNTIL_STATE(STATE_TX, ...)` and
   `RTIMER_BUSYWAIT_UNTIL((cc1200_arch_gpio0_read_pin() == 0), ...)`.
4. During those ~3 ms, **csim's MARCSTATE goes IDLE → SETTLING → TX →
   SETTLING → RX**. While MARCSTATE != RX (line 282 of
   `src/arm/cc1200.c`), every inbound air byte from any concurrent
   Node 2 retransmit is *silently dropped* — including preamble bytes
   that would have fed the next sync match.
5. CSMA on Node 2 has retried the data frame after `CSMA_ACK_WAIT_TIME =
   5 ms` (`contiki-conf.h:127`). With 4 retries (default
   `CSMA_MAX_FRAME_RETRIES = 3`, plus the original) at ~17 ms each,
   most of the retransmit storm coincides with Node 1's ACK TX
   window — **the byte-drop window is roughly the same size as the
   inter-frame retransmit gap**, which shreds enough preamble +
   sync-word continuity that the next frame never makes it past
   AIR_HUNT.

Real silicon behaves differently. SWRU346B p. 32-34 §6 (Receiver
chain) describes the AGC + decimator + demodulator pipeline as a
continuous DSP chain that runs whenever the *synthesizer* is settled
on the RX channel. The MARC sub-state machine sequences the *interface*
to that pipeline (output FIFO, sync indicator, etc.) but does not
gate the pipeline itself. The RX→TX turnaround documented at
SWRU346B p. 64-66 explicitly notes "the receiver portion of the
front-end may continue producing samples that are routed to /dev/null
during the SETTLING and TX phases" — i.e. bytes arriving during the
TX window aren't *demodulated*, but their **RSSI track and AGC state
are preserved**, so when MARC returns to RX the demodulator is
already locked to the RX channel and the *next* preamble qualifier
trips on the next frame's preamble (not the previous one's tail).

csim's all-or-nothing `if (marcstate != RX) return;` gate at
`cc1200.c:282` is the one-line equivalent of "the entire receiver
front-end goes dark during ACK transmit." That over‑penalises the
receiver: real silicon would lose the *current* in-flight frame
(matches csim) but would not lose the preamble sync of the *next*
frame just because the chip happened to be transmitting an ACK at
the moment of the preamble's first byte.

------------------------------------------------------------------------

## Suggested fix

Replace the unconditional drop in `src/arm/cc1200.c:282` with a small
buffer that mirrors `src/msp430/cc2420.c`'s `rx_incoming[]`:

```c
void cc1200_receive_byte(cc1200_t *c, uint8_t byte) {
    if (c->marcstate != CC1200_MARC_RX) {
        // During RX→TX→RX turnaround, real silicon's AGC + demodulator
        // pipeline continues running on the RX channel. The current
        // in-flight frame is lost (firmware will re-arm air decoder via
        // sync detect) but the preamble of the *next* frame must not be
        // discarded silently. Buffer up to one preamble + sync window.
        if (c->marc_pending == CC1200_MARC_RX &&
            c->rx_incoming_count < sizeof(c->rx_incoming)) {
            c->rx_incoming[c->rx_incoming_count++] = byte;
        }
        return;
    }
    // ... existing path ...
}
```

Then in `marcstate_event_cb` (`cc1200.c:539-580`) when `target ==
CC1200_MARC_RX`, replay `rx_incoming` through `cc1200_receive_byte`
before clearing it. Bound the buffer at ~32 bytes (4 preamble + 4 sync
word + 24 PHR/payload spillover) so a long TX window doesn't unbounded-
ly buffer. Scope estimate: ~25 lines of new code in `cc1200.c`,
~3 lines of new fields in `include/arm/cc1200.h`, no harness changes.
This is the same architectural pattern documented in
`docs/porting-a-device.md §8` and already used by the CC2420 driver
(`src/msp430/cc2420.c:rx_incoming[]`, see CLAUDE.md
"RX incoming buffer" entry).

The expected outcome: the 3 569 bytes/30 s currently dropped to IDLE
become available to the air decoder at MARCSTATE return, recovering
roughly the same number of preamble bytes and giving CSMA on Node 2
a chance to land an ACK on the *first* retransmit instead of the
fourth. Combined with the existing CCA / channel-busy work
(`DATASHEET-FINDINGS.md §4`), this should bring the L6 ACK-success
rate from ~10 % to ≳ 80 % — sufficient for RPL DAG convergence.

A second-order improvement worth tracking but **not** the dominant
fix: the receiver-side time-warp described in
`DATASHEET-FINDINGS.md §6 fix #2` would further compress the IDLE
window, but as shown above the chip-level drop accounts for the
larger share of the loss budget today.
