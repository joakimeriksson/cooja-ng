# Datasheet-Anchored Findings — Zoul Firefly Port

> Working notes derived from the **CC1200 User's Guide (SWRU346B)** and
> the **CC2538 User's Guide (SWRU319C)**. Everything here is cited to a
> page or section so future contributors don't have to re-derive
> behavior empirically.
>
> Local datasheet copies (downloaded from ti.com):
> - `/tmp/claude/cc1200/cc1200-userguide.pdf` (115 pages, SWRU346B)
> - `/tmp/claude/cc1200/cc2538-userguide.pdf` (770 pages, SWRU319C)
>
> Plus the smaller summary datasheets (35 pages each):
> - `/tmp/claude/cc1200/cc1200-datasheet.pdf` (SWRS123D)
> - `/tmp/claude/cc1200/cc2538-datasheet.pdf` (SWRS096D)
>
> These should be re-fetched from `https://www.ti.com/lit/` before
> committing changes that depend on them — they are not in the repo
> and the original local paths are temporary.

## 1. CC1200 GDO signal multiplexing — the L6 architectural blocker

**Citation: SWRU346B page 18-19, Table "GPIO Signal Configurations"**

The CC1200 GDOx pins are **multiplexed**: each pin has an independent
`IOCFGx.GPIOx_CFG` register that selects *which internal signal* drives
the pin. The chip drives GDOx according to the **currently-selected**
signal's value at every moment. Writing `IOCFGx` instantly changes
which signal is on the pin — and therefore can cause an instant level
change on GDOx, which can fire an edge interrupt.

The two signals relevant to L6:

- **PKT_SYNC_RXTX (signal 6, IOCFG default)**:
  - RX: asserted when sync word received; de-asserted at end of packet
    (or on bad address/length, RX FIFO over/underflow).
  - TX: asserted when sync word sent; de-asserted at end of packet.
- **MARC_2PIN_STATUS_0 (signal 38)**: bit 0 of MARC state. Per
  page 19 table:
  - `00` SETTLING → MARC[0]=0
  - `01` TX → MARC[0]=1
  - `10` IDLE → MARC[0]=0
  - `11` RX → MARC[0]=1

**Contiki uses both** (`arch/dev/radio/cc1200/cc1200.c:1960`):
the firmware reconfigures `IOCFG0` to `MARC_2PIN_STATUS_0` inside
`transmit()` so it can poll GDO0 to detect when MARC enters TX state.
Then sets `IOCFG0` back to `PKT_SYNC_RXTX` after TX completes.

### What csim currently gets wrong

`src/arm/cc1200.c` does **not** implement IOCFG multiplexing. It only
calls `drive_gdo0()` from a few hard-coded chip-state events (sync
match, frame_done, TX start) and gates each on
`c->regs[CC1200_REG_IOCFG0] == CC1200_IOCFG_PKT_SYNC_RXTX`.

Two consequences:

1. **Frames RX'd while `IOCFG0 != PKT_SYNC_RXTX` generate no edges at
   all.** csim's chip stays silent on GDO0. Real silicon would still
   not generate `PKT_SYNC_RXTX` edges (correctly), but it would update
   GDO0 to reflect MARC[0] in real time — including an edge when MARC
   transitions out of RX.

2. **Commit `1a694cd` is wrong per spec.** It made csim *always* drive
   GDO0 with PKT_SYNC_RXTX semantics regardless of IOCFG0. That
   produces edges real silicon would not produce. It does happen to
   be a no-op for L5 (which leaves IOCFG0 alone) and for cc2538dk
   regression (which doesn't use CC1200 at all), but it's not
   datasheet-correct and it didn't unblock L6 anyway.

### The right fix

Implement IOCFG-driven multiplexing in `src/arm/cc1200.c`:

- For each GDOx pin, track the current selected signal in
  `c->regs[CC1200_REG_IOCFGx]`.
- Maintain internal "current value" for each signal csim cares about
  (PKT_SYNC_RXTX, MARC_2PIN_STATUS_0, MARC_2PIN_STATUS_1, RSSI_VALID,
  CARRIER_SENSE_VALID, CARRIER_SENSE).
- Whenever an internal signal value changes (chip state transition,
  sync detect, frame end, RSSI update), propagate to GDOx pins whose
  IOCFG selects that signal.
- Whenever firmware writes `IOCFGx`, immediately update the
  corresponding GDOx pin level to reflect the newly-selected signal's
  current value. Edge generation falls out automatically.

Exposed signals csim needs to model for the Firefly to work:
6 (PKT_SYNC_RXTX), 17 (CARRIER_SENSE), 16 (CARRIER_SENSE_VALID),
13 (RSSI_VALID), 38 (MARC_2PIN_STATUS_0), 37 (MARC_2PIN_STATUS_1).
HIGHZ (e.g. signal 0x30) → pin floats; treat as no driving.

## 2. CC1200 MARC state machine

**Citation: SWRU346B page 105-106, MARCSTATE register; pages 62-66,
state machine descriptions**

22 distinct internal sub-states, mapped down to 4 visible states via
`MARC_2PIN_STATE`:

| MARC code | Symbolic name | MARC_2PIN_STATE |
|-----------|---------------|-----------------|
| 0x01 | IDLE | IDLE |
| 0x0D | RX | RX |
| 0x0E | RX_END | RX |
| 0x0F | RXDCM | RX |
| 0x13 | TX | TX |
| 0x14 | TX_END | TX |
| 0x10/0x11/0x15/0x16/0x17 | TXRX_SWITCH / RX_FIFO_ERR / RXTX_SWITCH / TX_FIFO_ERR / IFADCON_TXRX | SETTLING |
| 0x03..0x0C, 0x12 | various calibration / settling sub-states | SETTLING |

**Strobe semantics (Table 6, page 13):**
- `SRES`: chip reset. CSn must stay low until SO drops (datasheet
  page 13). Initial calibration runs as part of this — substantial
  delay (~1 ms in csim's current model).
- `SCAL`: calibrate frequency synthesizer, then turn it off. Goes
  through MANCAL → BIAS_SETTLE_MC → REG_SETTLE_MC → ... → IDLE.
- `SRX` / `STX`: enables RX/TX; if `SETTLING_CFG.FS_AUTOCAL = 1`,
  performs calibration first (takes longer).
- `SIDLE`: forces IDLE from any state. Always works.
- `SFRX`: flush RX FIFO. **Only valid in IDLE or RX_FIFO_ERR.**
  Issuing it from RX leads to undefined behavior on real silicon.
- `SFTX`: flush TX FIFO. **Only valid in IDLE or TX_FIFO_ERR.**

### What csim currently models

Strobe MARCSTATE transitions are event-driven (commit `739cbef`),
with timing values drawn from a mix of datasheet calibration delays
and Contiki driver `RTIMER_BUSYWAIT_UNTIL` constants. Specifically:
SIDLE ~50 µs, SRX/STX ~150-720 µs depending on calibration state.
This is reasonable but worth re-validating against page 62-66 of the
datasheet — the actual values depend on `SETTLING_CFG.FS_AUTOCAL`
and CIC decimation factor.

### Open question — does csim correctly handle "SRX while in RX"?

Per page 63 footnote 20: "When an SRX strobe is issued in RX state,
RX is restarted (the modulator starts searching for a sync word). If
the radio was in the middle of a packet reception, part of the 'old'
packet will remain in the RX FIFO." csim should preserve this
behavior; verify in `cc1200.c` strobe handler.

## 3. CC2538 GPIO interrupt semantics

**Citation: SWRU319C section 9.2.2.2 (page 234) and §9.3.1.x (pages
242-244)**

Per-pin interrupt configuration is controlled by 4 registers:

| Reg | Field | Semantic |
|-----|-------|----------|
| `GPIO_IS`  | `IS[7:0]`  | 0 = edge-triggered, 1 = level-triggered |
| `GPIO_IBE` | `IBE[7:0]` | (when IS=0) 1 = both edges, 0 = use IEV polarity |
| `GPIO_IEV` | `IEV[7:0]` | (when IBE=0) 1 = rising edge / high level, 0 = falling edge / low level |
| `GPIO_IE`  | `IE[7:0]`  | 1 = unmasked (NVIC pend allowed), 0 = masked |
| `GPIO_RIS` | `RIS[7:0]` | **sticky** raw interrupt status: "Bits set: Requirements met by corresponding pins." Cleared only by writing 1 to `GPIO_IC`. |
| `GPIO_MIS` | `MIS[7:0]` | computed: `MIS = RIS & IE`. NVIC IRQ is asserted whenever MIS != 0. |
| `GPIO_IC`  | `IC[7:0]`  | write-1-to-clear corresponding RIS bit |

**Critical datasheet warnings:**

1. Page 234, section 9.2.2.2: "When programming the interrupt
   control registers (GPIO_IS, GPIO_IBE, or GPIO_IEV), the
   interrupts must be masked (GPIO_IM cleared). Writing any value to
   an interrupt control register can generate a spurious interrupt
   if the corresponding bits are enabled."
   → csim should re-evaluate NVIC pend whenever IS/IBE/IEV is written
   while IE is non-zero. Currently it does not.
2. Page 234: "For edge-triggered interrupts, software must clear the
   interrupt to enable any further interrupts. For a level-sensitive
   interrupt, the external source must hold the level constant for
   the interrupt to be recognized by the INTC."
3. Page 243, GPIO_RIS description: "Bits read high in RIS reflect
   the status of interrupts trigger conditions detected (raw, before
   masking), indicating that all the requirements are met, before
   they are finally allowed to trigger by IE."

### What csim currently does

`src/arm/cc2538_gpio.c`:
- `cc2538_gpio_force_irq_edge()` (commit `f8b2a7e`): correctly checks
  IS/IBE/IEV; sets RIS only on edges matching the configuration. ✓
- `gpio_write` GPIO_IE handler (commit `fc78288`): re-pends NVIC if
  `(newly_enabled & RIS) != 0`. ✓ Matches datasheet semantics for
  MIS = RIS & IE.
- Level-triggered interrupts (`IS=1`): not modeled — early-return.
  Acceptable for now (chip drivers all use edge-triggered GDO pins),
  but document the gap.

### What csim is still missing

- **Spurious-interrupt-on-IS/IBE/IEV-write.** Per page 234 warning,
  writing these registers while IE is non-zero can fire spurious IRQs.
  Real firmware is supposed to mask first; the datasheet documents
  this as expected hardware behavior. Should csim model it for
  fidelity, or is current behavior (silently ignore the spurious
  generation possibility) sufficient? Probably sufficient — it's a
  firmware bug if it happens.
- **Power-up interrupt path.** `SYS_CTRL_IWE`, `GPIO_P_EDGE_CTRL`,
  `GPIO_PI_IEN`. Used for sleep-mode wake. Not relevant for L6 but
  worth noting for completeness.

## 4. CC1200 CCA (carrier sense) timing

**Citation: SWRU346B sections 6.9.1 and 9.5.2 (pages 32-37, 64)**

`CARRIER_SENSE` (signal 17) is asserted "if RSSI level is above
threshold." It becomes valid (`CARRIER_SENSE_VALID`, signal 16) after
a settling time given by Equation 19 — function of
`AGC_SETTLE_WAIT`, BB_CIC_DECFACT, decimation factor, and fxosc. With
default Contiki 50 kbps config, this is roughly tens of µs to a few
hundred µs after entering RX.

### What csim currently does

Commit `076402a` added basic CCA support: `cc1200_t` tracks
CARRIER_SENSE state, drives `CC1200_RSSI0` register reads.
Effectiveness limited by the previous diagnosis — the test runner's
medium-busy timestamp uses a stale `first_byte_ns = 0` anchor for
sub-GHz frames (only 802.15.4 preamble bytes `0x00` arm
`first_byte_ns`, not CC1200's `0x55`), so the CCA query
(`node_tx_busy_until_ns[j]`) ends up in the distant past and
CARRIER_SENSE essentially never asserts.

### What csim should do

Two pieces:
1. Honor real CARRIER_SENSE_VALID settling time (~tens of µs after
   entering RX). Currently csim asserts it immediately.
2. Fix the `first_byte_ns` anchor for sub-GHz frames (or eliminate
   the anchor in favor of a per-channel "currently transmitting"
   query on the medium). This is the agent's earlier punt.

## 5. Existing csim commits — datasheet-correctness audit

| Commit | Change | Datasheet verdict |
|--------|--------|-------------------|
| `f8b2a7e` | `cc2538_gpio_force_irq_edge` honor IS/IBE/IEV | ✓ matches §9.3.1 IS/IBE/IEV semantics |
| `fc78288` | `cc2538_gpio` IE 0→1 re-pend with sticky RIS | ✓ matches "MIS = RIS & IE" + sticky-RIS semantics |
| `739cbef` | CC1200 event-driven MARCSTATE strobe transitions | ✓ matches state-machine model on pages 62-66; specific delay values worth validating |
| `076402a` | CC1200 CCA via RSSI0/CARRIER_SENSE | ✓ direction; needs settling-time + medium-busy fix |
| `1a694cd` | CC1200 always-drive GDO0 on sync match regardless of IOCFG0 | ✗ **WRONG per page 18-19**; ~~revert when proper IOCFG multiplexing lands~~ **reverted** as part of the IOCFG multiplexing landing — sync match now sets `sig_pkt_sync_rxtx`, `propagate_signals()` drives only the GDO pins whose IOCFG selects signal 6 |
| _(this commit)_ | CC1200 IOCFG/GDO multiplexing model + signal tracking | ✓ matches §1 above. GDOx pin level recomputed on every IOCFGx write and on every internal-signal change; modeled signals are 6 PKT_SYNC_RXTX, 13 RSSI_VALID, 16 CARRIER_SENSE_VALID, 17 CARRIER_SENSE, 37/38 MARC_2PIN_STATUS_1/0; GPIOx_INV bit honoured. L5 still green (16 direct RX); L6 RF byte volume jumps from 1581 → 55821 over 60 s, but DAG convergence still blocked — see `DATASHEET-FINDINGS.md` §6 fix #2 (receiver-side time-warp). |

## 6. The path to L6 convergence — datasheet-grounded

Three fixes, ordered by dependency:

1. **Implement CC1200 IOCFG multiplexing** (§1 above). Revert
   `1a694cd` as part of this. Frames RX'd during the firmware's
   `transmit()` window will then trigger the natural MARC[0]
   transitions on GDO0 that real silicon produces; combined with the
   already-in-place `cc2538_gpio` IE-rise re-pend (`fc78288`),
   the firmware ISR will pick up buffered frames after `transmit()`
   ends and ENABLE_GPIO_INTERRUPTS is called.

2. **Fix the receiver-side time-warp.** `test_mixed_multinode.c:1169`
   rolls back `cpu->sim_time_ns` for byte delivery but leaves chip
   register state at present. This creates an artificially extended
   "transmit-prep window" where IOCFG0 has been set to
   MARC_2PIN_STATUS_0 ages ago in csim's wall clock, but bytes are
   still arriving as if the frame were in flight. On real silicon
   this race window is microseconds; in csim it's milliseconds. Fix
   = make byte delivery and firmware register writes share a
   consistent time axis.

3. **Tighten CCA semantics** (§4 above). Less critical but needed
   for collision-avoidance realism. CSMA backoff currently has no
   real medium-busy signal to back off on.

L5 (broadcast nullnet) works because Contiki doesn't reconfigure
IOCFG0 outside of normal RX in the broadcast-only flow, so csim's
incomplete IOCFG model is never exercised. L6 hits the
reconfiguration path on every unicast TX, so it cannot succeed
without #1.

## 7. References — re-fetch URLs

If/when these notes need updating:

- CC1200 User's Guide: https://www.ti.com/lit/ug/swru346b/swru346b.pdf
- CC2538 User's Guide: https://www.ti.com/lit/ug/swru319c/swru319c.pdf
- CC1200 Datasheet (summary): https://www.ti.com/lit/ds/symlink/cc1200.pdf
- CC2538 Datasheet (summary): https://www.ti.com/lit/ds/symlink/cc2538.pdf

The user guides are the authoritative source for register-level
semantics. The summary datasheets give electrical characteristics
and pinout but not register details.
