# Archived investigation trail — Zoul Firefly L6

These docs were written *during* the L6 RPL-UDP investigation in May 2026 and are kept here for historical reference only. **All items they describe are resolved.** For the current state of the port see [`../STATUS.md`](../STATUS.md); for the device contract see [`../SPEC.md`](../SPEC.md).

| File | What it was | Disposition |
|---|---|---|
| [`L6-PLAN.md`](L6-PLAN.md) | Operational task list for the L6 convergence gap (items L6-1 through L6-6). | All items closed. The convergence failure was not a csim bug — it was two upstream Contiki-NG firmware bugs (`CSMA_CONF_ACK_WAIT_TIME`, `pending_packet()` SPI starvation), both staged on PR branches. The csim "fixes" listed in this file (rx_incoming buffer, queue depth, ACK turnaround) would have *masked* the firmware bugs. |
| [`CC1200-RX-ACK-CHAIN.md`](CC1200-RX-ACK-CHAIN.md) | 10-step datasheet+code audit of the RX→ACK path, written to localise the L6 failure. | The audit's "suggested fix" (mirror cc2420's `rx_incoming[]` buffer in cc1200) is the canonical example of a fidelity hack that would have hidden a real firmware race. See `docs/porting-a-device.md §8` "Don't add fidelity buffers to mask firmware races." |

The lessons themselves live in [`docs/porting-a-device.md`](../../../docs/porting-a-device.md) §8 (pitfalls) and §10 (closing out a port). If you're starting a new port and looking for examples, read that doc first — these archived files are the raw investigation trail, not the lessons.
