# External data-driven nodes for Cooja-NG — plan (Phase 13)

Status: **proposal, for team review.** No code written except the worked
example `examples/ext/jammer.py` (§5.5). Approved internally 2026-09-03;
awaiting team OK on the §10 decisions before implementation starts.

## 0. Short answer: how small, how long, how much to maintain

**Minimal cut: about 2 working days, ~450 new lines of C in one new file,
~80 lines of Python, zero changes to the runner or kernel.** Everything in
§4–§9 beyond this is optional and additive.

The minimal cut is:

| Piece | Size | Notes |
|---|---|---|
| `src/motes/external_mote.c` | ~350 lines | Copy of `js_app_mote.c`'s shape (142 lines) + a stdio lockstep source (fork/exec/pipes, like the 122-line `sim_external_command.c`) + NDJSON parse with the in-tree cJSON |
| Replay-file source | ~60 lines | Same file: read `.ndjson` lines into the outbound queue at boot, `step` is a no-op |
| Registration | ~15 lines | One `sim_board_kind_t` value, two extension rows in `sim_board.c` (`.py` → live process, `.ndjson` → replay), one row in `mote_kinds.c`, one union arm in `mote_impl.h` |
| `tools/csim_ext.py` | ~80 lines | stdlib only: read `step`, call `on_rx`/`on_serial`, write `done` |
| Config/test | 1 JSON config + 1 line in the regression list | v1 config: `"firmware": "examples/ext/echo.py"` — no schema change, the firmware path *is* the executable |

What makes it this small:

- **No runner change.** TX reuses the existing `env->js_rf_frame` hook
  (`mixed_js_rf_handler`, `test/test_mixed_multinode.c:1333`): it takes
  `(node, frame, len)` and is not JS-specific, so the external mote calls it
  as-is. The `app_rf_frame` rename in §5.1 is cosmetic and deferred.
- **No config schema change.** Extension-driven kind selection exactly like
  `.js`/`.cooja`. Arguments go to the peer via `hello` (`id`, `x`, `y`, `seed`)
  and the environment; a config `external` block is a later nicety.
- **Protocol v1 = 5 message types**: `hello`, `step`, `done`, inputs `rx`/
  `serial`, outputs `tx`/`log`. LED, radio-state, move, TCP source, recorder
  service, pcap import are all later and additive (§9).

Maintenance: the burden is that of one more `js_app_mote.c`. That file has
had 5 commits in its life and `js_node.c` 1, through the entire Phase 1–10
refactor, because they sit behind `sim_mote_ops_t` and nothing else. The
external mote touches the same three seams only (mote ops, `env->js_rf_frame`,
`env->uart_byte`); a kernel or bus change that keeps the JS mote working keeps
this one working. The protocol is versioned in `hello` and only ever gains
fields. The one recurring cost is the Python helper, which is stdlib-only and
has no build step.

Recommended sequencing: ship the minimal cut (M2+M3 collapsed, §6) first;
decide on the recorder, TCP and config block afterwards based on use.

## 1. Context

Cooja-NG today has four mote kinds: emulated MSP430 ELF, emulated ARM ELF,
native Cooja (`.cooja` dlopen) and JS app motes (QuickJS). Every node's
behaviour is therefore produced *inside* the csim process. The refactor plan
(`docs/design/refactor-plan.md` §5.4) already lists the missing kinds:
"future: external process mote, record/replay mote, hardware-in-loop mote".

We want **external data-driven nodes**: nodes whose behaviour comes from
*outside* the emulator but that still live in simulation time, transmit and
receive on the shared radio medium, print to the console, show in the web UI,
and drive JSON/JS tests unchanged. Concrete use cases:

- A Python (or any language) model, digital twin, ML agent or protocol
  prototype acting as a node in a simulated Contiki-NG/Zephyr network.
- Replaying recorded node behaviour (traces) so a heavyweight or flaky node
  can be replaced by its recording in regression tests, or so field data can
  be injected into a simulation deterministically.
- Feeding sensor/dataset values into a simulation as a node that "reports"
  them over the air.

**Assumed scope (to confirm with the team):** the first delivery is (a) a
*live external-process* mote and (b) a *record/replay* mote, both built on
one shared "external event" vocabulary. Hardware-in-the-loop and feeding
sensor values into *emulated* firmware are listed as follow-ups (§9), not in
scope now.

## 2. What exists that we build on (facts from the tree)

| Need | Existing piece | Where |
|---|---|---|
| Non-emulated mote template | JS app mote: `execute` processes queued events ≤ now and returns next wakeup; `receive_frame` queues a whole frame; radio registered BATCH with stub byte ops | `src/motes/js_app_mote.c`, `src/native/js_node.c` |
| Whole-frame TX from a non-chip mote | JS handler sends the frame twice: `sim_radio_bus_tx_frame` (frame consumers) **and** PHY-wrapped bytes via `sim_radio_bus_tx_byte` (emulated receivers) | `test/test_mixed_multinode.c:1333-1345` (`mixed_js_rf_handler`), `src/native/native_radio.c` (`native_frame_to_bytes`) |
| Mote vtable | `sim_mote_ops_t`: `execute`, `receive_frame`, `serial_input`, `ui_radio_state`, `ui_leds`, `reset_time`, `destroy` … | `include/sim/sim_mote.h:53-233` |
| Kind registry row | `sim_mote_kind_t { name, banner_label, node_type, boot, register_radio, ops }`, static table indexed by `sim_board_kind_t` | `src/motes/mote_impl.h`, `src/motes/mote_kinds.c:17-50`, `src/sim/sim_board.c` (extension → kind) |
| Per-receiver RSSI without a bus API change | `radio_medium_get_rssi(&sim->radio_medium, sender, receiver)`; mote has `env->sim` | `include/common/radio_medium.h`, `src/sim/sim_radio_bus.c:648` |
| Spawning a child process | fork/exec + SIGTERM/SIGKILL teardown in the external-command service | `src/sim/sim_external_command.c:66-116` |
| Non-blocking loopback TCP | serial bridge listener/accept/read pattern | `src/sim/sim_serial_bridge.c:150-286` |
| Serial input into a mote | `ops->serial_input`, runner `inject_serial`, test action `send` | `test/test_mixed_multinode.c:1190-1200`, `docs/test-format.md` |
| Observer stream for a recorder | `SIM_OBS_MOTE_LOG_LINE`, `LED_CHANGED`, `RADIO_STATE`, `PACKET_FRAME`, `RADIO_TX_START/END` (frame bytes + channel + rssi in `ev->radio`) | `include/sim/sim_observer.h:29-86` |
| Service host | `sim_service_ops_t { init, destroy, on_event, poll }`; built-in services registered by name | `include/sim/sim_service.h`, `src/sim/sim_registry.c:29-38` |
| Config v2 | `mote_types[] { name, kind, firmware … }` — `kind` is parsed **but ignored** today; nodes reference a type by name | `src/sim/sim_config.c:458-531`, `include/sim/sim_config.h:90-97` |

Gaps that matter for the design:

- **No mote-kind plugin ABI.** `csim_registry_ops_t` has only `register_service` and `register_radio_medium`; the header states `register_mote_type` is intentionally absent. So the new kind must be **built in** (like `js-app`), not a `.so`.
- **`receive_frame` carries no RSSI/channel/air-time** (only `frame, len, now_ns, sender_idx`). The mote can compute RSSI itself via the medium; channel comes from the sender's medium state.
- **A csim pcap is not replayable**: it records MAC bytes and TX-start time only, no sender, channel or direction. A dedicated record format is needed.
- **The kernel never blocks** and services' `poll` only runs when a serial socket / external command is active. The external mote must therefore do its I/O inside its own `execute` slice, not rely on `poll`.
- **No per-node free-form parameters** in config (`sim_node_config_t` has firmware/id/x/y/clock_deviation/type_name only). The external mote needs a small config block (command, transport, trace path).

## 3. Design overview

One new built-in mote kind, **`external`** (`SIM_BOARD_KIND_EXTERNAL`,
banner `EXT`), with a pluggable **event source** behind it:

```
                 ┌──────────────── csim process ────────────────┐
 config          │  external mote (src/motes/external_mote.c)    │
 ─────────►      │   ├─ source: file    (replay NDJSON trace)    │
                 │   ├─ source: process (spawn child, stdio)     │
                 │   └─ source: tcp     (connect/listen socket)  │
                 │  same ops table, same event vocabulary        │
                 └───────────────────────────────────────────────┘
                                   ▲   NDJSON, sim-time stamped, lockstep
                                   ▼
                    python/rust/… peer  (tools/csim_ext.py reference lib)
```

Design rules (mirroring the kernel invariants in refactor-plan §3.14):

1. **csim owns the clock.** Peers never advance time. Every exchange is
   stamped with simulation nanoseconds. Wall-clock never enters the protocol.
2. **Lockstep by default.** The peer only speaks when csim asks it to run a
   slice, and csim waits for the reply. Output is a function of (inputs, time)
   only, so `CSIM_ARM_JIT=0/1`-style byte-identical determinism holds as long
   as the peer is deterministic. A free-running/real-time mode is a later
   opt-in (§9).
3. **One vocabulary for replay and live.** A replay file is literally the
   peer's side of the conversation, recorded. A recorder service can produce
   it from *any* node kind.
4. **No fake CPU.** Like JS/native motes: pseudo-cycles 1 µs = 1 cycle,
   `freq_hz` 1 MHz, no `sched_hint_ns`/`sync_to_time`.
5. **Built in, not a plugin.** Same reason `js-app` is built in; §9 notes what
   a later mote-kind plugin ABI would need.

## 4. Event vocabulary (protocol v1)

Line-delimited JSON (NDJSON), one object per line, UTF-8, frames as lowercase
hex. Chosen over CBOR/binary for debuggability and because a trace file must
be diff-able and hand-editable; a binary framing can be added behind the same
source abstraction later if profiling demands it.

**csim → peer**

| `type` | fields | when |
|---|---|---|
| `hello` | `proto:1, id, slot, x, y, seed, args{…}` | once, before the first slice |
| `step` | `t` (ns) plus an `in:[…]` array of inputs that arrived since the last step | every execute slice |
| `stop` | `t, reason` | end of run / node removed |

Inputs inside `step.in`:

| `type` | fields |
|---|---|
| `rx` | `t, from (node id), ch, rssi, frame (hex, MAC incl. FCS)` |
| `serial` | `t, data (hex)` |
| `move` | `t, x, y` (test action `move`, UI drag) |

**peer → csim**, exactly one `done` reply per `hello` and per `step` (the
reply to `hello` is what carries the peer's first `wake`; `out` may be empty):

```
{"type":"done","t":T,"wake":T_next_or_null,"out":[ …events… ]}
```

Output events (all stamped `t ≥ step.t`; csim rejects earlier stamps):

| `type` | fields | csim action |
|---|---|---|
| `tx` | `t, ch, frame (hex)` | whole-frame TX at `t` via the existing JS dual path (frame consumers + PHY-wrapped bytes to emulated receivers); `t > now` is queued as a future wakeup |
| `log` | `t, line` | console line through `env->uart_byte` → `SIM_OBS_MOTE_LOG_LINE` (tests, UI, timeline unchanged) |
| `led` | `t, leds[3]` | `ui_leds` |
| `radio` | `t, state ("off"/"on"/"rx"/"tx"), ch` | medium `rx_enabled`/channel (so UDGM per-channel filtering and energest work), `ui_radio_state` |
| `wake` | `t` | additional wakeup request (same as `wake` in `done`) |

Three choices made now so the same protocol can later run with the roles
swapped (an external coordinator stepping csim, §9):

- every `rx`, `tx`, `log` and `serial` event accepts an optional `node`
  field; in node mode it is omitted and implied;
- `hello` carries a `nodes:[…]` array (one entry in node mode);
- `step` reserves a `stop_on_tx` flag (ignored in node mode).

Rules: `wake` is the *earliest* time csim will call `step` again; csim also
calls `step` whenever an input arrives (an `rx` or `serial`). A peer that
sets `wake:null` and gets no input is never stepped again (idle).

**Frame size: 152 bytes, not 2047** (corrected against the implementation).
The reused TX hook PHY-wraps into the runner's `uint8_t bytes[160]`
(4 preamble + SFD + length + frame + 2 CRC), and `native_frame_to_bytes`
returns 0 — *silently dropping the whole frame* — for anything larger. The
engine therefore rejects an oversize `tx` loudly rather than inheriting that
silent drop. Raising the cap means a second TX path, not a constant.

**The PHY wrap appends a valid CRC.** A `tx` frame is delivered to receivers
as a well-formed frame whose *contents* are whatever the peer sent, so a
jammer disturbs by occupying air time and colliding, not by putting corrupt
bits on the air. That is the realistic model for an interferer on-channel;
a peer cannot currently emit a CRC-invalid frame through this path.

**Replay file** = the sequence of peer output events only, sorted by `t`:

```
{"type":"log","t":1000000000,"line":"Hello from replay"}
{"type":"tx","t":1500000000,"ch":26,"frame":"41c8...."}
```

Inputs are ignored by a file source (a trace does not react). Recording keeps
inputs too (as comment lines / `rx` events) so a trace can be inspected.

## 5. Components

### 5.1 `src/motes/external_mote.c` (new, built-in kind)

- Add `SIM_BOARD_KIND_EXTERNAL` (`include/sim/sim_board.h`), extensions
  `.ndjson` → replay file source; for process/tcp sources the config block
  (§5.4) selects the source, firmware path is the trace/command.
- `mixed_node_t.plat` gains an `external_node_t` union arm
  (`src/motes/mote_impl.h`): source vtable pointer, RX queue (reuse the
  `js_node.c` queue shape), pending outbound events sorted by `t`, next
  wakeup, radio state/channel, leds, line buffer, child pid / fd.
- Ops (copy the shape of `js_app_mote.c`):
  - `boot`: create the source (spawn / connect / open file), send `hello`,
    print `Node %d [EXT] initialized`.
  - `register_radio`: `sim_radio_bus_register(..., SIM_RADIO_DELIVERY_BATCH, 0)`
    with stub byte ops **plus a real `current_channel`** (returns the peer's
    declared channel, `-2` when radio off) so channel filtering is correct —
    an improvement over the JS mote, which has none.
  - `receive_frame(m, frame, len, now, sender)`: compute
    `rssi = radio_medium_get_rssi(&env->sim->radio_medium, sender, slot)`,
    sender's channel from the medium, queue `rx`, and
    `sim_schedule_mote_wakeup_if_earlier(sim, slot, now)`.
  - `execute(m, now)`: (1) apply queued output events with `t ≤ now` (tx/log/
    led/radio); (2) if inputs pending or `now ≥ wake`: source `step(now, inputs)`
    → append its `out` to the queue, set `wake`; apply those with `t == now`
    immediately; (3) return min(wake, earliest queued output `t`, earliest
    queued rx `t`) or `INT64_MAX`.
  - `serial_input`: queue `serial`, schedule wakeup at `now`, return `len`.
  - `ui_radio_state`, `ui_leds`, `reset_time`, `destroy` (source close +
    child teardown as in `sim_external_command.c:109-116`).
- TX: call the existing `env->js_rf_frame(node, frame, len)` hook
  (`mixed_js_rf_handler`, `test/test_mixed_multinode.c:1333`) unchanged — it
  is not JS-specific. Renaming it to `app_rf_frame` is cosmetic and deferred,
  so the minimal cut makes **no runner change**.

### 5.2 Event sources (`src/motes/ext_source_{file,process,tcp}.c`)

Common vtable: `open(cfg)`, `hello()`, `step(now, inputs, out_events, *wake)`,
`stop()`, `close()`.

- **file**: reads the NDJSON once at boot into the outbound queue; `step` is a
  no-op. Zero I/O at run time, fully deterministic, no timeouts.
- **process**: fork/exec (reuse the pattern in `sim_external_command.c`),
  two pipes, blocking `write` of the `step` line then blocking `read` until a
  `done` line (loop with `poll()` and a wall-clock timeout,
  `CSIM_EXT_TIMEOUT_MS`, default 5000). Peer stderr passes through. Child
  exit or timeout → run fails with a clear error (kernel error policy,
  refactor-plan §3.14.2), never silently skipped.
- **tcp**: `connect` to `host:port` (peer already running, e.g. a notebook) or
  `listen` and wait for one connection before the first step (loopback only,
  like the serial bridge). Same lockstep exchange as process.

Blocking inside `execute` is acceptable: the kernel is single-threaded, a
slow slice is indistinguishable from a slow emulated slice, and services
(UI, serial bridge) are polled between slices as today. UI pacing already
rebases its wall-clock start after pauses; a slow peer just lowers the
achieved speed ratio.

### 5.3 Recorder service (`src/services/record_service.c`, built-in name `record`)

- Subscribes to `SIM_OBS_MOTE_LOG_LINE`, `LED_CHANGED`, `RADIO_STATE`,
  `RADIO_TX_START` (frame bytes + channel), and RX events for context;
  writes `<outdir>/node-<id>.ndjson` per selected node in the §4 format.
- Selected via config `"plugins": ["record"]` (compiled-in plugin, same style
  as `energest`) with `CSIM_RECORD_DIR` / `CSIM_RECORD_NODES` env knobs,
  matching how the Gilbert-Elliott medium takes its knobs.
- Requires `sim_runtime_set_radio_state_tracking(true)` — any plugin attach
  already turns that on.

### 5.4 Config

v2 `mote_types[]` entry gains an `external` block; `kind` becomes **honoured**
for this kind (today it is parsed and ignored — we keep ignoring it for the
other kinds to stay byte-identical):

```json
{ "version": 2,
  "mote_types": [
    { "name": "sky-client", "kind": "emulated-elf", "firmware": "firmware/sky/udp-client.sky" },
    { "name": "py-server",  "kind": "external",
      "external": { "source": "process",
                    "command": "python3 examples/ext/echo_server.py",
                    "args": { "port": 5678 } } },
    { "name": "replayed-br", "kind": "external",
      "external": { "source": "file", "path": "traces/node-1.ndjson" } }
  ],
  "nodes": [ { "type": "py-server", "id": 1, "x": 0, "y": 0 },
             { "type": "sky-client", "id": 2, "x": 20, "y": 0 } ] }
```

`sim_node_config_t`/`sim_mote_type_config_t` get `ext_source[8]`,
`ext_path_or_command[512]`, `ext_args_json[256]` (passed verbatim in `hello`).
v1 configs: a `firmware` ending in `.ndjson` is a replay node (extension map
in `sim_board.c`), nothing else changes.

### 5.5 Reference client and examples

- `tools/csim_ext.py` (single file, stdlib only, ~150 lines): `class ExtNode`
  with `on_hello`, `on_rx`, `on_serial`, `on_move`, `step(t)`, helpers
  `send(frame, ch, t=None)`, `log(line)`, `led(...)`, `radio(...)`,
  `wake_at(t)`; `run_stdio()` / `run_tcp()`. Includes a tiny 802.15.4 header
  builder (the peer builds MAC headers itself, exactly like JS motes do).
- `examples/ext/jammer.py` — **written, and now runnable** (see §12): a disturber node
  in ~30 lines of logic and no library, which puts a burst of junk on the air
  every `JAM_PERIOD_MS` of sim time and so collides with anything in flight.
  It is the smallest useful external node (it never listens) and doubles as
  the protocol's worked example. Knobs are environment variables
  (`JAM_PERIOD_MS`, `JAM_LEN`, `JAM_CHANNEL`), so it needs no config-schema
  change. Verified against a hand-written mock of csim's side of the
  conversation; it cannot be run for real until M1 exists.
- `examples/ext/broadcast.py` (twin of `firmware/js/broadcast.js`),
  `examples/ext/sniffer.py` (logs every frame heard: shows RSSI/channel
  plumbing), `examples/ext/csv_sensor.py` (reads a CSV of timestamped
  readings and broadcasts them at those sim times — the "data-driven" case).

### 5.6 Docs

- `docs/design/external-nodes-plan.md` (this document, with decisions).
- `docs/external-nodes.md`: protocol reference, config, Python quickstart.
- `CLAUDE.md`: new kind in the mote-module table, test commands.
- `refactor-plan.md` §5.4: tick "external process mote" and "record/replay".

## 6. Milestones

| # | Deliverable | Gate | Est. |
|---|---|---|---|
| **M1 (minimal cut, §0)** | `external` kind: stdio lockstep **process** source + **file** replay source, extension-driven (`.py`, `.ndjson`), `tools/csim_ext.py`, `examples/ext/broadcast.py`, one config in the regression list, `docs/external-nodes.md` | run twice → identical stdout; Python node ↔ Sky and ↔ cc2538 exchange frames; peer crash/timeout → non-zero exit with message; all existing suites unchanged | **2 d** |
| M2 (optional) | Recorder service (`"plugins": ["record"]`) so any node can be replayed | record 2-node `nullnet-broadcast.cc2538dk`, replace node 2 by its `.ndjson`, node 1's console output identical | 1 d |
| M3 (optional) | Config v2 `external` block (command args, tcp connect/listen), `led`/`radio`/`move` events, UI radio state + LEDs | `configs/test-ext-tcp.json` with a listener peer; `--ui` shows the node | 1.5 d |
| M4 (optional) | `csv_sensor.py`, `sniffer.py`, pcap→ndjson import, CLAUDE.md, smoke script | docs reviewed | 1 d |

Minimal cut alone: 2 days. Everything: ≈ 5.5 days. Each row is a separately
mergeable PR; M2–M4 can be dropped or reordered without touching M1.

## 7. Verification

- **Determinism:** same config + seed twice → `diff` of stdout/stderr (the
  existing Phase gate). Applies to file and process sources.
- **Round trip:** record → replay equivalence on `nullnet-broadcast` (M2) and
  on `udp-server/udp-client` cc2538dk (stretch; ACK timing must reproduce).
- **Cross-kind interop:** Python node ↔ Sky (CC2420 per-byte), ↔ cc2538,
  ↔ nRF52840, ↔ native Cooja, ↔ JS in one config (extend
  `configs/cross-level-demo.json`).
- **Failure paths:** peer exits, peer replies with `t < step.t`, malformed
  line, oversize frame, timeout → each produces one clear error and exit ≠ 0.
- **Existing suites unchanged:** Cooja regression 93/93 (jftest4 reference),
  `arm-correctness`, `radio-medium`, plugin smoke.

## 8. Risks and trade-offs

- **Lockstep latency**: one round trip per slice. A chatty Python peer at
  ~50 µs/round-trip is still ≫ real time for typical IoT traffic; TSCH-grade
  timing is *not* a goal (BATCH delivery is frame-complete based, same as JS).
- **Peer non-determinism** (Python `random`, wall clock) breaks csim's
  determinism guarantee only for that config; the `hello.seed` field and the
  docs make the contract explicit.
- **Blocking in `execute`** stalls the UI while waiting on a slow peer.
  Acceptable for v1; a `poll`-based asynchronous source is the §9 follow-up.
- **No auto-ACK / no address filter** in the external node: it sees every
  frame in range (like a sniffer) and must ACK itself if it wants
  CSMA-with-ACK peers to be happy. `csim_ext.py` provides an `auto_ack`
  helper; it is a software ACK with frame-level timing, good enough for
  nullnet/UDP over CSMA (same as native Cooja motes today).
- **Runner coupling**: the `app_rf_frame` refactor touches
  `test/test_mixed_multinode.c`; kept minimal and gated by byte-identical JS
  output.

## 9. Explicitly out of scope (follow-ups)

- **Sensor values into emulated firmware** (ADC/GPIO/I2C per chip): reuse
  the same NDJSON vocabulary as a `sensor` input to a chip-level hook; needs
  per-peripheral work, separate plan.
- **Coordinator (co-simulation) mode**, i.e. PR #1's use case where another
  simulator owns the clock and the channel model: not needed now, but likely
  the moment Cooja-NG has to integrate with other emulators. It is the same
  protocol with the roles swapped: the coordinator sends `step`, csim answers
  `done` once every node reached `t`, `rx` entries carry `node`, `tx`/`log`
  come back with `node`, `stop_on_tx` gives PR #1's run-until/continue cycle
  and `done.wake` is the `node_idle` report. It is a *service* plus a
  "who supplies the next horizon" runner hook (the second
  `sim_scheduler_ops_t` policy reserved in refactor-plan §3.13), on top of
  `"medium": {"type": "none"}`, `SIM_OBS_RADIO_TX_START` and
  `sim_radio_bus_deliver_bytes`. Estimate 2–3 days; PR #1's transport and
  mock-coordinator test port over. First thing to prototype: the auto-ACK
  round trip through the coordinator inside one paused window.
- **Hardware-in-the-loop / real-time free-running peers**: needs a
  non-lockstep source that polls a socket between slices and injects at
  "now", with `speed: 1.0` pacing. Determinism is inherently lost; document
  as such.
- **Mote-kind plugin ABI** (`register_mote_type`): once external nodes exist,
  most "custom node" needs are covered by the process protocol, which is a
  much smaller and language-neutral surface than a C ABI over
  `mixed_node_t`. Revisit only if a C-level in-process kind is needed.
- **Binary/CBOR framing** if profiling shows NDJSON parsing dominates.

## 10. Decisions the team should confirm

1. Scope = live process + replay first; HIL and sensor-into-firmware later.
2. Lockstep, csim-owned sim time, no wall-clock in the protocol.
3. NDJSON with hex frames as the v1 wire and trace format.
4. Built-in kind (`external`), not a `.so`; config v2 `external` block and
   `.ndjson` extension for v1 configs.
5. Python stdlib reference client shipped in `tools/`, examples under
   `examples/ext/`.
6. Keep the door open for PR #1's coordinator mode at zero cost now: optional
   `node` field, `hello.nodes[]`, reserved `stop_on_tx` (§4, §9, §11). Not
   built in this plan.

## 11. Relationship to the open co-simulation PR (#1)

PR #1 "Add co-simulation support" (opened 2026-04-23, last updated
2026-06-14, no reviews yet) solves the **inverse** problem: an external
*coordinator* drives csim's clock (`time_advance` / `step_to` / `run_until` +
`continue`) and routes **all** radio traffic through an external channel model
(csim emits `tx`, the coordinator injects `rx`). csim is a component inside
someone else's simulation. This plan keeps csim as the master clock and makes
external processes *nodes* inside csim's medium.

Overlap worth exploiting:

- Nearly identical message fields: cosim `tx`/`rx`/`console` carry node id,
  ns timestamp, channel, RSSI/TX power and a raw 802.15.4 frame; this plan's
  `tx`/`rx`/`log` carry the same. **Align the names and field spelling** so
  one Python helper can speak both.
- `src/cosim/cosim.c` (394 lines: length-prefixed JSON over TCP, blocking
  wait-for-command) is a reusable transport for this plan's later `tcp`
  source.

State of PR #1 (measured against `main` today): merge-base 2026-03-27, 470
commits behind, `CONFLICTING` in 30 files. Of its 20 commits, 18 are
pre-refactor history that already landed in `main` separately (JS motes,
GDB stub, pcap, Z1…). The unique payload is two commits: `cosim.h/.c`
(~520 lines), 656 lines of runner integration in `test_mixed_multinode.c`,
and ~900 lines of tests. The runner part predates Phases 1–10 and would have
to be redone as a second `sim_scheduler_ops_t` policy (refactor-plan §3.13
already reserves that slot) rather than rebased; the transport and tests
carry over.

Recommendation: treat the two as one "external interfaces" theme with a
shared message vocabulary, land this plan's minimal cut first (it needs no
runner changes), then revive PR #1's coordinator mode as a scheduler policy
on the post-refactor kernel. Neither blocks the other.

## 12. Status: what is built

The M1 minimal cut's transmit half is **implemented and merged-ready** (PR
#23), sized at ~530 lines against the ~380 estimated:

| File | Lines | |
|---|---|---|
| `src/native/ext_node.c` + `include/native/ext_node.h` | ~390 | Engine: fork/exec + pipes, bounded line reader, NDJSON via the in-tree cJSON, `tx`/`log`/`wake` |
| `src/motes/external_mote.c` | ~150 | Mote ops, deliberately the `js_app_mote.c` shape |
| Registration + build | 13 | One enum value, one `.py` extension row, one `mote_kinds.c` row, one union arm, two Makefile lines |

**No runner change, no kernel change, no config-schema change**, as predicted.

Measured, on `configs/test-ext-jammer-sky.json` (two Sky nullnet nodes 5 m
apart, `examples/ext/jammer.py` between them at 2 m):

| | sends | app-level receives | CC2420 |
|---|---|---|---|
| Baseline, no jammer | 4 | 4 | `crc_ok=4 crc_fail=0` |
| `JAM_PERIOD_MS=2` (64% duty) | 4 | **0** | `crc_ok=20137 crc_fail=4 dropped=2492 overflow=10` |

The link goes to 100% loss. The large `crc_ok` is the jam frames themselves
arriving as valid-CRC garbage (§4); `crc_fail=4` is the four real packets,
destroyed by collision.

Verified: byte-identical across two runs (108 lines compared, host-timing
lines excluded); `correctness` 87/87, `arm-correctness` 230/230,
`radio-medium` 241/241, `cc1200-mock-host` 73/73 unchanged; the JS mote
unaffected (`cross-level-demo.json`).

Failure paths, each with a specific message: peer exits before replying,
peer replies with non-JSON, `tx` frame malformed or over 152 bytes, reply
timeout (`CSIM_EXT_TIMEOUT_MS`, default 5000), output stamped before the
step time.

**RX is now built too.** A delivered frame reaches the peer as an `rx`
input on its next step, carrying the sender's node id, the sender's channel
and the per-receiver RSSI the medium computed (asked of the medium rather
than carried on the frame: two nodes at different distances hear the same
transmission at different strengths). `examples/ext/sniffer.py` decodes the
802.15.4 header from it — see `configs/test-ext-sniffer-sky.json`:

```
rx #2  from=1  rssi=-65 dBm  ch=26  len=17
    DATA seq=86  ver=1  dst=abcd/ffff  src=(same)/0101.0100.0174.1200
    hdr  41d8 56cd abff ff00 1274 0100 0101 01
    data 0000
```

The decoded source address matches what Contiki prints on the sending
node's own console, and the PAN matches its boot banner.

**This needed one bus change, contradicting the "no kernel change"
prediction.** A BATCH receiver's frame goes to `deliver_bytes` or
`queue_frame`, and both end in `rx_byte_sync`, which a non-emulated mote does
not have — so `receive_frame` was only ever called for native/JS senders, and
nothing from an emulated chip could reach a frame-consuming mote. The fix is
one opt-in capability, `SIM_RADIO_CAP_FRAME_CONSUMER`: at frame-complete the
bus hands such a receiver the MAC frame directly, PHY wrap and FCS stripped.
Opt-in means no existing receiver's delivery changes.

**Discovered while doing it:** JS app motes have the same gap. Their
`receivedPacket` handler in `firmware/js/broadcast.js` never fires from an
emulated sender — verified: 0 receives in `configs/cross-level-demo.json`
despite two neighbours in range. Giving the JS kind the same capability
would fix it, but it would also change existing JS console output, so it is
left as a separate decision rather than a silent side effect.

**Both examples are gated in CI**, not just shipped as demos
(`.github/workflows/test.yml`, ubuntu + macOS):

- **sniffer** — validators assert the decoded *source addresses*, so it fails
  if the peer dies, if the bus stops delivering, or if the MAC decode drifts.
  Verified by mutation: removing `SIM_RADIO_CAP_FRAME_CONSUMER` turns it red
  (`"rx #" matched 0/4`).
- **jammer** — a negative test: `fail_on` trips if any application message
  gets through, and validators require that the Sky pair actually sent and
  that the jammer actually transmitted, so a jammer that does nothing cannot
  pass vacuously. Verified by mutation: at the 100 ms default period the link
  survives and the test fails, naming the packet that got through.

These validators are also what makes the exit-code gap above harmless in
practice: a dead peer produces no log lines, so the validator fails even
though the mote failure itself cannot set the exit code.

Measured dose-response, which is the evidence the interference model behaves
sensibly rather than just "on/off":

| `JAM_PERIOD_MS` | duty | app messages delivered (of 4) |
|---|---|---|
| none (baseline) | 0% | 4 |
| 100 | ~1% | 4 |
| 20 | ~7% | 3 |
| 5 | ~26% | 1 |
| 2 | ~64% | 0 |

**Still not built:** `serial` input into the peer, the replay-file source,
`tools/csim_ext.py`, the config-v2 block, and `led`/`radio`/`move` events.
The config-v2 block has a concrete motivation now: `JAM_PERIOD_MS=2` has to be
passed as an environment variable in the CI step because a config cannot yet
carry per-node arguments.

**Known rough edge:** a peer that dies *mid-run* logs to stderr and stops the
run immediately via `sim_runtime_request_stop()`, but the process still exits
0 — refactor-plan §3.14.2 explicitly forbids a mote failure from aborting the
sim, and a non-zero exit needs a runner change. Boot-time failures already
exit 1. Until that is resolved, a CI job using an external node should assert
liveness through a test validator rather than trusting the exit code.
