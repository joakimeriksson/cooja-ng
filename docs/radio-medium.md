# Radio Medium

This document describes how the radio medium routes RF byte streams
between nodes in csim. It is intentionally precise — implementing a new
chip driver, adding a new platform, or debugging a TSCH/CSMA test all
depend on understanding what the medium does and what it deliberately
does not do.

Reference files:

- API: [`include/common/radio_medium.h`](../include/common/radio_medium.h)
- Implementation: [`src/common/radio_medium.c`](../src/common/radio_medium.c)
- Harness integration: [`test/test_mixed_multinode.c`](../test/test_mixed_multinode.c) (search for `mixed_rf_tx_handler`, `mixed_node_radio_set_channel`, `sync_native_node_channel`)
- Unit tests: [`test/test_radio_medium.c`](../test/test_radio_medium.c) — 235 assertions pinning the API contract

## Mental model

The medium is a **policy oracle**, not a dispatcher. Given a candidate
delivery `(sender_node, sender_radio, receiver_node, receiver_radio, byte)`
it answers one question: *should this byte be delivered?* It holds **no
chip pointers** and **no byte-delivery callbacks**. The harness
(`test_mixed_multinode.c`) owns dispatch and chip routing — the medium
just gates.

This separation is important: chip drivers (`cc2420`, `cc2538_rfcore`,
`cc1200`) never include `radio_medium.h`. They communicate with the
medium only through the `sim_host_t` vtable (one-way push of channel
state). Conversely, the medium knows nothing about chip state machines.

## Data model

```c
typedef enum {
    RADIO_SPECTRUM_NONE         = 0,   /* slot unregistered */
    RADIO_SPECTRUM_2_4GHZ_15_4  = 1,   /* CC2420 / cc2538_rfcore — channels 11–26 */
    RADIO_SPECTRUM_868MHZ_15_4G = 2,   /* CC1200 EU sub-GHz */
    RADIO_SPECTRUM_915MHZ_15_4G = 3,   /* CC1200 NA sub-GHz */
} radio_spectrum_t;

typedef struct {
    radio_spectrum_t spectrum;     /* NONE = unregistered */
    int              channel;      /* band-local index, -1 = unknown */
    bool             rx_enabled;   /* receiver side: chip currently in RX */
} radio_t;

typedef struct {
    double  x, y;                                   /* shared by all radios on this node */
    radio_t radios[RADIO_MEDIUM_MAX_RADIOS_PER_NODE];  /* typically 1; Firefly = 2 */
    int     radio_count;
    int     channel;   /* legacy alias = radios[0].channel, kept in lockstep */
} radio_node_state_t;

typedef struct {
    radio_medium_type_t type;             /* NONE = pass-all, UDGM = distance + loss */
    udgm_config_t       udgm;
    int                 node_count;
    radio_node_state_t  nodes[MAX_NODES];
    frame_tracker_t     frame_track[MAX_NODES][MAX_RADIOS];   /* per (sender, radio) */
    rx_decision_t       rx_decisions[MAX_NODES][MAX_NODES];   /* per (sender, receiver) cached loss roll */
    neighbor_list_t     neighbors[MAX_NODES];                 /* precomputed in-range */
    neighbor_list_t     interference_neighbors[MAX_NODES];
    uint32_t            next_frame_id;
    uint32_t            rng_state;
} radio_medium_t;
```

`MAX_RADIOS_PER_NODE` is 2 today (Firefly is the only multi-radio
device). The model is extensible — bump the constant if a future device
needs more.

## Channel propagation

Channels reach the medium via four independent paths, all converging on
`radio_medium_set_radio_channel(rm, node, radio_idx, channel)`. The
medium is always told the channel synchronously, the moment it changes,
not at periodic sweep boundaries. This is what makes TSCH-style channel
hopping accurate.

| Source | Mechanism |
|---|---|
| **CC2420** (sky/z1/esb/...) | `set_reg(FSCTRL, value)` decodes channel from `FSCTRL[9:0]` (`channel = (freq − 357)/5 + 11`) and fires `host->radio_set_channel(host->radio_user_data, 0, channel)` via the `sim_host_t` vtable. |
| **CC2538 RFCore** (cc2538dk/openmote/zoul-firefly slot 0) | `FREQCTRL.FREQ` register write decodes `channel = (FREQ − 11)/5 + 11` and fires the `cc2538_rfcore_set_channel_callback` observer. |
| **CC1200** (zoul-firefly slot 1) | `FREQ0/1/2` 24-bit register write computes the band-local channel index from frequency and fires `host->radio_set_channel(host->radio_user_data, 1, channel)`. |
| **Native (Cooja) motes** | No chip emulator. Harness reads `simRadioChannel` *inline* before each filter call via `sync_native_node_channel(idx)`. Catches TSCH mid-tick hops at the byte boundary. |

The `sim_host_t.radio_set_channel` callback is registered per-node in
`init_msp430_node` / `init_arm_node` to point at
`mixed_host_radio_set_channel` → `mixed_node_radio_set_channel`. That
adapter does exactly two stores (per-radio channel, legacy alias) — no
other side effects. Chip drivers never know about `radio_medium_t`.

## Filter pipeline

Per-byte path is `radio_medium_filter_byte_radio(rm, sender,
sender_radio, receiver, receiver_radio, byte)`:

1. **NONE-type fast path**: if `rm->type == RADIO_MEDIUM_NONE`, pass all
   bytes. This is the default until `radio_medium_configure_udgm` runs.
2. **Bounds check** sender/receiver/radios.
3. **Advance the frame tracker** for `(sender, sender_radio)` on this
   byte. This always runs, even if the byte is later dropped — the
   tracker has to stay in sync with the sender's actual byte stream so
   future per-frame decisions don't misalign.
4. **`radio_pair_match()`** — see "pair match" section below. Drops on
   mismatch.
5. **UDGM distance + per-frame probabilistic loss decision**. One roll
   per frame, cached in `rx_decisions[sender][receiver]` and keyed off
   the frame tracker's current `frame_id`.

`radio_medium_filter_frame_radio` is the same path minus step 3 — used
by frame-level callers (e.g. medium-busy queries).

The legacy single-radio `filter_byte` / `filter_frame` (no `_radio`
suffix) forward to `(sender_radio = 0, receiver_radio = 0)` for
backward compat with single-radio platforms.

### Pair match

`radio_pair_match()` (in `src/common/radio_medium.c`) is the heart of
the routing decision. Three cases based on whether each side has been
registered (spectrum != NONE):

| Sender registered | Receiver registered | Behavior |
|---|---|---|
| Yes | Yes | Spectrum must match (different bands → drop). Channel must match (`-1` on either side passes). |
| Yes | No | Drop. Stops e.g. a Firefly's CC1200 (slot 1, 868 MHz registered) from leaking onto a cc2538dk's empty slot 1. |
| No | Yes | Drop, same reason. |
| No | No | Legacy fallback: cross-band drop using the channel-base heuristic (`channel >= RADIO_MEDIUM_SUBGHZ_CHANNEL_BASE = 100` means sub-GHz, otherwise 2.4 GHz). Within-band channel match is also applied. Keeps platforms that never call `register_radio` working. |

Then a final `rx_enabled` gate: receiver radio must currently be in RX.

**Channel matching is enforced.** The earlier "TSCH hopping makes stale
values unreliable" workaround that disabled the within-band match has
been removed. TSCH works because:

1. Channels are **accurate** at the instant the medium consults them
   (chip-side push, not periodic sweep).
2. The harness's `schedule_emulated_wakeup` uses `current_sim_ns`
   rather than stale `cpu->sim_time_ns`, so chip events fire on a
   coherent global timeline (commit `7b9b26d` — see git log for the
   exact diagnosis).

## Frame tracker

Per `(sender, sender_radio)` slot. State machine recognizes one of two
on-air formats:

| Profile | Detection | Length |
|---|---|---|
| **802.15.4** | 4× `0x00` preamble → `0x7A` SFD | 1-byte length |
| **802.15.4g** | Any preamble, then 32-bit sync word `0x6E4E904E` slid through a rolling shift register | 2-byte PHR (PHRA + PHRB) |

The profile is set via `register_radio` based on spectrum:
sub-GHz spectra (`868MHZ_15_4G`, `915MHZ_15_4G`) → 802.15.4g profile,
otherwise standard 802.15.4.

When a tracker enters `FRAME_DATA` it bumps `frame_id`. That
invalidates any cached per-receiver loss decision for this sender and
forces a new probabilistic roll on the next byte that reaches step 5.

## Per-frame loss caching

`rx_decisions[sender][receiver]` holds `(frame_id, decided, drop)`.
When the first byte of a new frame reaches the loss step:

1. `udgm_reception_prob(sender, receiver)` returns a probability based
   on distance vs `tx_range`/`interference_range`.
2. The medium rolls once via `rng_next(rm)` (deterministic xorshift32
   seedable via `radio_medium_set_seed`).
3. Result is cached in `rx_decisions[sender][receiver]` keyed off
   `frame_id`.
4. Every subsequent byte of the same frame uses the cached decision.

Result: an entire frame is either delivered or dropped. No
half-frames, no per-byte coin flips.

## Cross-band isolation

Two layers compose:

1. **Spectrum tag** (registered radios): `spectrum_a != spectrum_b` →
   drop. This is the right answer when both sides have called
   `register_radio` (or auto-registered via the legacy
   `radio_medium_set_channel` API).
2. **Legacy channel-base heuristic** (both sides unregistered):
   `channel >= 100` means sub-GHz, otherwise 2.4 GHz. Mismatch → drop.

Concrete example (Firefly + cc2538dk in the same simulation):

- cc2538dk slot 0 (cc2538_rfcore, 2.4 GHz registered) ↔ Firefly slot 0
  (cc2538_rfcore, 2.4 GHz registered) → spectrum match, **delivered**
- cc2538dk slot 0 ↔ Firefly slot 1 (CC1200, 868 MHz registered) →
  spectrum mismatch, **dropped**
- Firefly slot 0 ↔ Firefly slot 1: same node, harness loops past self

## Harness integration

The medium decides "should this byte be delivered." The harness in
`test_mixed_multinode.c` does the actual dispatch:

1. Each chip's TX listener carries an `(node_idx, radio_idx)` context
   (`rf_listener_ctx_t`, one per slot).
2. When a chip emits a byte, `mixed_rf_tx_chip_cb` looks up the slot
   tag and calls `mixed_rf_tx_handler_radio(sender_idx, sender_radio,
   byte)`.
3. For each candidate receiver and each of its registered radios, the
   handler calls
   `radio_medium_filter_byte_radio(rm, sender, sender_radio, receiver,
   receiver_radio, byte)`.
4. If the filter returns true, the byte is dispatched to the matching
   chip's `*_receive_byte()` (cc2420, cc2538_rfcore, cc1200, or native
   `simInDataBuffer`), picked per slot.

For dual-radio Firefly nodes this routes:

- A `cc2538_rfcore`-emitted byte → only delivered to other nodes' slot 0
  chips that are 2.4 GHz on a matching channel.
- A `cc1200`-emitted byte → only delivered to other nodes' slot 1 chips
  on a matching sub-GHz channel.

There is no per-node fan-out hack. No "feed every byte to both chips
and let them ignore." The medium gates per-(sender_radio,
receiver_radio) explicitly.

## Backward compatibility

Single-radio platforms (sky, esb, z1, wismote, exp5438, cc430,
cc2538dk, openmote) require zero changes:

- Their chip driver pushes via the same `sim_host_t.radio_set_channel`
  callback (CC2420 or cc2538_rfcore).
- The legacy `radio_medium_set_channel(rm, node, ch)` (no `_radio`
  suffix) auto-registers slot 0 with the spectrum implied by the
  channel range, bumps `radio_count` to 1, then calls
  `radio_medium_set_radio_channel(rm, node, 0, ch)`.
- Legacy `filter_byte` / `filter_frame` forward to
  `radio_idx == 0` on both sides.
- Existing tests still see the same delivery semantics.

## Debugging — `CSIM_TRACE_RADIO`

Set env var `CSIM_TRACE_RADIO=1` before running `test_runner` to log
every radio event from `test_mixed_multinode.c`. Output is one line per
event, parseable, low overhead when disabled (one TLS bool check).

| Line | Meaning |
|---|---|
| `[t=N.Ns] ch_set node=N radio=N ch=N (was N)` | A chip changed channel |
| `[t=N.Ns] tx_byte node=N radio=N ch=N byte=0xNN state=N zc=N` | One TX byte from a sender |
| `[t=N.Ns] tx_frame_complete node=N radio=N ch=N subghz=N expected_len=N payload_count=N` | Sender finished assembling a frame |
| `[t=N.Ns] filter sender=N/N receiver=N/N ch=N/N -> DELIVER/DROP (channel_mismatch)` | Per-filter decision |
| `[t=N.Ns] cc2420 node=N state X -> Y` | CC2420 chip state transition (from `src/msp430/cc2420.c`) |

Useful diagnostic patterns:

- Many `filter ... -> DROP (channel_mismatch)` between two nodes that
  should be on the same channel: chip-side channel push is missing or
  fired with the wrong value.
- `ch_set` events for one node but never another: that node's chip
  driver isn't wired to push channel via `sim_host_t`.
- Non-monotonic timestamps in `cc2420 state ...` lines: the CPU's
  `sim_time_ns` is being read out of sync with the global wall clock.
  See `7b9b26d` for the canonical example.

## Tests

- **Unit**: `./build/test_runner radio-medium` — 235 assertions in
  `test/test_radio_medium.c` covering channel match (within-band,
  cross-band, legacy mode), UDGM distance + probabilistic loss, frame
  tracker for both 802.15.4 and 802.15.4g, native mid-tick channel
  changes, multi-radio register + dispatch + isolation, RX-disabled
  gating, TSCH-style 50-hop stress (50 hops → exactly 10/50 deliver on
  matching channel), spectrum mismatch, RSSI computation, edge cases.
- **Chip-level (mock-host)**: `cc1200-mock-host` (73), `mock-host` (21
  for CC2420). Exercise chips in isolation, no harness involvement.
- **Integration**:
  - sky 2-node nullnet (`./build/test_runner multinode -n 2 -t 5000 -q`)
  - cc2538dk 2-node nullnet (`./build/test_runner arm-multinode firmware/cc2538dk/nullnet-broadcast.cc2538dk -n 2 -t 20000 -q`)
  - cc2538dk RPL-UDP convergence (`-d 100 -t 60000`) — gates the
    startup-delay mechanism end-to-end
  - Firefly L5 sub-GHz nullnet
- **Cooja regression**: 81 of 88 tests pass via `tools/run-cooja-tests.sh`
  (the 7 skipped require host TUN setup). All TSCH tests included.

## Anti-patterns

These are explicitly NOT allowed in chip-driver or per-byte delivery
code, per established project rules (see
[`docs/porting-a-device.md`](porting-a-device.md) §8):

- **Calling `step_node_until` from chip-driver code or per-byte
  delivery callbacks.** Indicates the chip driver is missing a
  `host->schedule_ns()` call. The simulator is event-driven; chips
  must drive their own state machine via scheduled events.
- **Importing `radio_medium.h` from a chip driver.** Chip drivers stay
  CPU/medium-agnostic. Channel state goes through `sim_host_t`.
- **Reading `cpu->sim_time_ns` to schedule cross-node events.** Use
  `current_sim_ns` (the harness's monotonic wall clock) for cross-node
  scheduling. `cpu->sim_time_ns` can be rolled back by
  `execute_events` to align with the firing event's cycle time. See
  `7b9b26d` for an example regression caused by this.
