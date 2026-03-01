# ARM Multinode RF Plan (Merged)

## Goal
Upgrade `arm-multinode` from byte-forwarding to a minimal but firmware-viable CC2538 802.15.4 radio model with explicit packet boundaries, realistic interrupts, and deterministic timing. Keep it simple, but correct enough for real stacks.

## Current State
- `arm-multinode` runs multiple CC2538 nodes and forwards RF bytes immediately.
- RF core has basic FIFOs, strobes, and a few status registers.
- No frame parsing, CRC, interrupts, CCA, collisions, or timing.

## Desired Capabilities (Minimal-Useful)
- Packet boundaries with `SFD` → byte stream → `RXPKTDONE`.
- Deterministic timing and delivery of RF bytes based on simulation time.
- Correct RF core interrupts and FIFO thresholds.
- CRC handling and RX status byte (CRC_OK) as CC2538 expects.
- Basic CCA and collision behavior sufficient for real firmware drivers.

## Plan

1. **Target Firmware Expectations**
   - Choose target firmware(s) for multinode validation (e.g., Contiki-NG nullnet-broadcast for CC2538DK).
   - Document required RF features from those stacks: SFD/FIFOP/RXPKTDONE use, AUTOCRC, AUTOACK, CCA.

2. **Frame Delivery First, Then Medium Model**
   - **Phase A: Frame-based delivery using existing byte forwarding**
     - Implement frame parsing in the RF core while keeping the current direct byte-delivery path.
     - This proves SFD/length parsing and interrupt behavior before introducing the medium model.
   - **Phase B: Deterministic RF medium model**
     - Replace direct forwarding with a shared medium queue:
       - TX enqueues a frame with start time, duration, channel, and node id.
       - RX nodes consume frames based on simulation time.
     - Add collision/CCA logic:
       - Overlapping TX on same channel corrupts frame or marks CRC bad.
       - CCA reports busy if any overlapping transmission in window.

3. **Frame-Based RX/TX in RF Core (Details)**
   - Interpret TXFIFO as 802.15.4 frame: length byte + payload + FCS.
   - **SFD/Length parsing state machine**
     - Count preamble bytes (0x00). When `zero_symbols >= 4` and byte == `0x7A`, raise `SFD`.
     - Next byte is `length`, then `length` bytes of payload (including FCS).
     - Extract `FCF0`, `FCF1`, `DSN` at bytes 0/1/2 of payload for ACK logic.
   - On TX start, emit SFD, then stream bytes with timing.
   - On RX, parse SFD and length, fill RXFIFO, then set RXPKTDONE.

4. **Interrupt Masks and NVIC Wiring**
   - Add `rfirqm0`, `rfirqm1`, `rferrm` handling with register offsets:
     - `RFIRQM0 = 0x08C`, `RFIRQM1 = 0x090`, `RFERRM = 0x094`.
   - Pend RFCORE IRQs through NVIC when masked flags set.
   - Re-check interrupts when flags are cleared (W1C).
   - **NVIC plumbing prerequisite**:
     - Add `arm_nvic_t *nvic` to `cc2538_rfcore_t`.
     - Update `cc2538_rfcore_init()` signature.
     - Pass `&plat->nvic` from `arm_platform.c`.

5. **CRC + RX Status Byte**
   - Implement CCITT CRC-16 with bit-reversal (same as CC2420).
   - On TX: if AUTOCRC enabled, compute and append FCS.
   - On RX: verify CRC, replace last 2 bytes with RSSI + CRC_OK byte.

6. **Basic Auto-ACK (Optional but Useful)**
   - If AUTOACK enabled and RX frame requests ACK and CRC OK:
     - Emit minimal ACK frame (length=5, FCF=0x0002, DSN).
     - Set TXACKDONE interrupt.

7. **Timed State Transitions**
   - Model RX calibration delay for `ISRXON` (e.g., 12 symbols).
   - Keep TX timing simple for now unless firmware needs accurate delays.

8. **Multi-Node Uniqueness**
   - Ensure each node has a unique IEEE address.
   - Patch `linkaddr_node_addr` or RF core EXT_ADDR registers after ELF load.

9. **Tests and Verification**
   - Unit tests for RF core:
     - TX→RX frame delivery
     - CCA busy detection
     - Collision CRC bad
     - Auto-ACK path
   - Integration test:
     - `arm-multinode` with nullnet-broadcast should show send/receive on both nodes.

## Milestones

Milestones are ordered so each one can be tested and stabilized independently before adding the next layer. Each milestone produces a working system — never a half-broken intermediate state.

- **M1**: Frame parsing + interrupts using existing byte forwarding (steps 2A, 3, 4).
  - *Why first:* Frame parsing and interrupt generation are the foundation everything else depends on. The simple byte-forwarding path is already working, so we can validate SFD detection, RXPKTDONE, FIFOP threshold, and NVIC interrupt delivery in isolation — without any medium model complexity. If the interrupt wiring or frame state machine has bugs, they're easy to diagnose here because the delivery path is trivial.

- **M2**: RF medium model + CCA + collisions (step 2B).
  - *Why second:* With frame parsing proven correct in M1, we can swap out the delivery layer and know that any new failures come from the medium model, not the parser. CCA and collision logic are only meaningful once frames have explicit boundaries and timing — both established in M1.

- **M3**: CRC + RX status byte + optional auto-ACK + timed transitions (steps 5-7).
  - *Why third:* CRC and auto-ACK are correctness features that sit on top of frame parsing (M1) and realistic delivery (M2). Adding them earlier would make M1/M2 debugging harder because CRC failures mask frame parsing bugs and auto-ACK generates additional RF traffic that complicates medium model validation. Timed state transitions (RX calibration delay) belong here because they affect event ordering — safer to add once the event-driven interrupt path is stable.

- **M4**: Multi-node uniqueness + test firmware + verification (steps 8-9).
  - *Why last:* MAC address patching and integration tests exercise the full stack. They can only validate correctly once all lower layers (parsing, delivery, CRC, interrupts) are solid. Running integration tests against a broken stack wastes time chasing symptoms instead of root causes.

## Risks / Open Questions
- Which firmware stacks are the priority targets?
- Required fidelity for PHY timing and CRC details.
- Performance impact of detailed RF modeling.

## Next Step (if approved)
- Confirm target firmware(s) and required RF features.
- Implement M1 first to validate frame parsing and interrupt correctness.
