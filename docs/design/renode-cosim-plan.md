# Renode as clock master for csim (co-simulation slave mode)

**Status:** implemented. Unit + end-to-end tests green, gated in CI.

## 1. What this is

Everywhere else in csim, csim owns the simulation clock: the runner picks a
horizon and the kernel pump runs to it. This inverts that for one case.
Renode's `CoSimulationPlugin` drives an external simulator with fixed-quantum
`tickClock` messages, and this feature turns each of those into the runner's
next horizon, so Renode's virtual time and csim's advance together.

From Renode's side, csim is an ordinary co-simulated peripheral: one line in
a `.repl` file, no Renode-side code, no plugin to build. Behind that
peripheral's register window sits csim's entire 802.15.4 network — every
emulated node, the radio medium, the per-receiver RSSI. Firmware running in
Renode can transmit into that network, receive from it, and read a csim
node's console, without knowing csim exists.

This is the inverse of [`external-nodes-plan.md`](external-nodes-plan.md),
where csim is the master and an external *process* is a node inside csim's
medium. The two do not interact and use different protocols, deliberately:
see §7.

## 2. Renode's protocol

Read from Renode's sources (`src/Plugins/CoSimulationPlugin`). Everything
here is Renode's design; csim implements the peer side of it.

**Transport.** Renode listens on two TCP ports and the peer connects, main
port first, then async. Renode can also spawn the peer, passing the two ports
and the address on its command line. The main socket carries Renode's
requests and the peer's replies, strictly one for one; the async socket
carries the peer's unsolicited events.

**Message.** One fixed 24-byte little-endian packed struct, every time:

| Field | Type |
|---|---|
| action | int32 |
| addr | uint64 |
| value | uint64 |
| peripheralIndex | int32 (-1 = none) |

**Handshake.** Renode sends `handshake` on main; the peer echoes it.

**Time.** The peripheral is declared with a `frequency` in hertz and a
`limitBuffer` tick count. Renode installs a timer on the machine's clock
source, and every `limitBuffer` ticks it sends `tickClock` (with the tick
count in `value`) and **blocks its whole emulation** until the peer replies
with its own `tickClock` **on the async socket**. A `timeout`, 3000 ms by default, guards the wait.
The message carries only the count, not the frequency, so the peer must be
told the frequency out of band — hence `--renode-freq`.

**Bus.** A guest access inside the peripheral's window becomes
`readRequestByte/Word/DoubleWord/QuadWord` or `writeRequest…`, with the
offset in `addr`. A read is answered with the generic `readRequest` action
carrying the data; a write with `ok`. Renode's CPU stalls until the reply.

**Which socket replies on.** This is the one part that is easy to get wrong
and is not guessable from the message format: **bus read and write replies go
back on the main socket; the tick confirmation, `interrupt`, `logMessage` and
the `disconnect` acknowledgement all go on the async socket.** Renode's own
integration library is the authority (`plugins/IntegrationLibrary/src/renode_bus.cpp`:
`readFromBus`/`writeToBus` use `sendMain`, `case tickClock` and
`handleDisconnect` use `sendSender`). Answering a tick on the main socket
deadlocks — the peer believes it has replied while Renode waits out its
entire tick timeout.

**Async events.** `interrupt` (index in `addr`, level in `value`) drives one
of the peripheral's GPIO outputs, which the `.repl` wires to an interrupt
controller. `logMessage` (length in `addr`, level in `value`) is followed by
the raw text and appears in Renode's log.

**Lifecycle.** `resetPeripheral` on machine reset, no reply expected.
`disconnect` ends the session; the peer answers `ok` on the async socket and
closes.

**Not implemented here.** Actions 11–20 and 29–31 (`isHalted`,
`registerGet`/`Set`, `singleStepMode`, `step`, the `push*`/`get*` family)
exist for co-simulating an HDL CPU *core* inside Renode. A network
peripheral never receives them. csim logs and ignores them rather than
replying: a reply Renode is not waiting for would desynchronise the main
stream for every message after it.

## 3. Architecture

```
Renode (master)                          csim
  LimitTimer ─tickClock(N)─▶ main ─▶ renode_cosim_service ─next_horizon()─▶ runner loop
  CPU bus   ─read/write────▶ main ─▶   (sockets, protocol,   ◀─ quantum done ─┘ sim_runtime_run_until
            ◀─reply/ok────── main ◀──   horizon, log/irq buffer)
            ◀─interrupt/log─ async ◀──      │ get_interface(SIM_MOTE_IFACE_RENODE_DEV)
                                            ▼
                                     renode_dev_t  (register window + FIFOs; no kernel, no sockets)
                                            ▲ owned by
                                     renode_mote.c (mote kind "RENODE": a node in csim's medium)
```

Four pieces, each testable on its own:

| File | Role |
|---|---|
| `include/common/renode_proto.h`, `src/common/renode_proto.c` | The 24-byte wire codec. No sockets, no state. |
| `include/native/renode_dev.h`, `src/native/renode_dev.c` | The register window and its FIFOs. No kernel, no sockets: every simulation-facing action goes through a hook. |
| `src/motes/renode_mote.c` | The mote kind. Gives the device a place in csim's medium (position, channel, RSSI) and fills in the device's hooks. Deliberately the `external_mote.c` shape. |
| `include/sim/renode_cosim_service.h`, `src/services/renode_cosim_service.c` | The sockets, the protocol loop, and the horizon the runner asks for. |

**Why the device is a separate object from the mote.** A service cannot see
`mote_impl.h`, which is private to `src/motes` and the runner. The service
finds the device through the documented escape hatch,
`sim_mote_ops_t.get_interface(m, SIM_MOTE_IFACE_RENODE_DEV)`, which also
means the device model can be unit-tested with no mote at all.

**Why there is no scheduler vtable.** `docs/design/refactor-plan.md` §3.13
reserves a `sim_scheduler_ops_t` for a second scheduling policy, but no such
type exists in the tree; the runner owns the horizon inline. Introducing a
vtable for one policy would have been a larger change than the feature, so
the seam is a single gated branch in the runner's loop with the logic behind
two service calls. That is the "who supplies the next horizon" runner hook
`external-nodes-plan.md` §9 asks for, without the refactor.

## 4. The runner seam

Two changes in `test/test_mixed_multinode.c`, both gated on a master being
attached:

```c
while (sim_ns < end_ns || ui_service_active(&ui_svc) ||
       (sim_serial_bridge_active(&serial_bridge) && ss_has_command) ||
       renode_cosim_active(&renode_svc)) {
    ...
    if (renode_cosim_active(&renode_svc)) {
        int64_t h = renode_cosim_next_horizon(&renode_svc, sim_ns);
        if (h < 0) break;          /* disconnect, peer loss or timeout */
        sim_ns = h;
    } else {
        /* the existing three-way horizon computation, untouched */
    }
    sim_rt.now_ns = sim_ns;
    ...
    sim_runtime_run_until(&sim_rt, sim_ns, mixed_dispatch_event, NULL);
```

The horizon is deliberately **not** capped to the next queued event, the way
the normal path caps it. Stopping early would report a quantum as finished
when it was not.

`-t` never decides the run in slave mode: the loop stays alive while a master
is attached, and ends when the master disconnects, dies, or the wait times
out. Attach failure is fatal rather than a warning — a run that quietly
free-runs without the master meant to drive it would look like a pass.

### Two ordering rules that matter

**Bus accesses happen at the quantum boundary.** After the pump returns,
`sim->now_ns` is the last *dispatched event's* time, which is at or before
the horizon. Frame injection stamps with `sim_runtime_now_ns`, so the service
pins `sim->now_ns` to the boundary before servicing any access. Without that,
a frame the guest injects would go on the air at whatever event csim happened
to dispatch last, inside a quantum Renode already considers finished.

**Observer callbacks do not touch the socket.** `sim_observer.h` forbids
blocking in them. Console lines and interrupt-level changes are buffered and
flushed on the async socket at the quantum boundary, immediately before the
`tickClock` reply. That is also the right semantics: Renode only observes csim
at quantum boundaries anyway.

## 5. The register window

32-bit registers in a 0x100 window. The two FIFO data registers are byte
streams: an access of any width transfers that many bytes. Every other
register is a 32-bit value, and a narrow access sees its low bytes. The map
lives in `include/native/renode_dev.h` and is mirrored for guest firmware in
`examples/renode/csim_dev.h`.

| Off | Name | R/W | Meaning |
|---|---|---|---|
| 0x00 | ID | R | `0x4353494D` ("CSIM") |
| 0x04 | VERSION | R | register-map version (1) |
| 0x08 | CTRL | RW | b0 IRQ_RX_EN, b1 IRQ_UART_EN, b2 RX_FLUSH (W1), b3 TX_RESET (W1) |
| 0x0C | STATUS | R | b0 RX_AVAIL, b1 RX_OVERFLOW (sticky), b2 UART_AVAIL, b3 UART_OVERFLOW, b4 UART_IN_PENDING |
| 0x10 | CHANNEL | RW | 11..26, or -1 for "not selected"; pushed to the medium |
| 0x14 | TXPOWER | RW | 0..31, the CC2420 PA scale; the medium scales range by it |
| 0x18 | TX_LEN | RW | MAC frame length, excluding FCS |
| 0x1C | TX_DATA | W | appends the access's own width in bytes |
| 0x20 | TX_CTRL | W | write 1 to transmit at the current simulation time |
| 0x24 | TX_COUNT | R | frames transmitted |
| 0x28 | RX_COUNT | R | frames queued |
| 0x2C | RX_LEN | R | head frame length |
| 0x30 | RX_DATA | R | next bytes of the head, zero-padded past the end |
| 0x34 | RX_POP | W | write 1 to drop the head |
| 0x38 | RX_RSSI | R | signed dBm, **as heard at this node** |
| 0x3C | RX_FROM | R | sending csim node id, -1 unknown |
| 0x40 | RX_CHANNEL | R | sender's channel |
| 0x44/0x48 | RX_TIME_LO/HI | R | the frame's start **on the air**, ns |
| 0x4C | RX_DROPPED | R | frames lost to a full queue |
| 0x50/0x54 | SIM_TIME_LO/HI | R | csim's clock, ns |
| 0x58 | NODE_COUNT | R | nodes in the csim simulation |
| 0x5C | SELF_ID | R | this device's csim node id |
| 0x60 | UART_NODE | RW | which node's console UART_DATA bridges |
| 0x64 | UART_DATA | RW | read a byte (`0xFFFFFFFF` if empty) / write to inject |
| 0x68 | UART_COUNT | R | console bytes available |
| 0x6C | IRQ_STATUS | R | b0 RX, b1 console (unmasked sources) |

Interrupt 0 is **level-triggered**: high while an unmasked source has data.
Only a change is sent, evaluated at each quantum boundary and after any bus
access that can change it.

Because it is a level, an unmasked source that the guest never drains holds
the line high forever and no further transitions are reported — the same as
real hardware, and worth knowing before enabling `IRQ_UART_EN`: csim's nodes
print continuously, so a guest that enables the console interrupt without
reading `UART_DATA` sees exactly one edge and then a permanently asserted
line. The demo firmware enables `IRQ_RX_EN` only.

**RSSI is per-receiver, and so is the on-air time.** Two nodes at different
distances hear the same transmission at different strengths, so RSSI is asked
of the medium rather than carried on the frame. `RX_TIME` is the frame's
first byte **on the air**, not the moment csim handed it over — the
distinction that `external-nodes-plan.md` §13 shows was worth 160 µs and
silently destroyed every acknowledgement.

**csim's clock has its own origin.** Nodes boot at a staggered start offset,
so `SIM_TIME` does not begin at zero. What tracks Renode's tick clock exactly
is *elapsed* time. A master that wants to check the coupling should read
`SIM_TIME` once before ticking and compare the difference.

## 6. Running it

Renode starts csim itself: setting `simulationFilePath` is what opens the two
sockets, launches the peer with them on its command line, and shakes hands.
`tools/csim-renode-launch.sh` maps Renode's positional
`<mainPort> <asyncPort> <address>` onto csim's single `--renode` argument,
and moves to the csim checkout first — Renode spawns the peer with *its own*
working directory, so a config's relative firmware paths would not otherwise
resolve.

```sh
export CSIM_CONFIG=$PWD/configs/test-renode-rpl-sky.yaml
export CSIM_RENODE_FREQ_HZ=1000000
renode examples/renode/csim-rpl.resc     # then `start` in the monitor
```

```
# examples/renode/csim-host.repl
csim: CoSimulated.CoSimulatedPeripheral @ sysbus <0x40100000, +0x100>
    frequency: 1000000
    limitBuffer: 1000            // 1 ms quanta
    timeout: 240000
    address: "127.0.0.1"
    cosimToRenodeSignalRange: <0, +1>
    0 -> nvic@10
```

Three of those lines are not guessable and were found by running it:
`cosimToRenodeSignalRange` declares how many interrupt lines the peer may
drive — without it the `0 -> nvic@10` wiring has nothing to connect; the
`timeout` is how long Renode waits for a tick reply, and it must comfortably
exceed the wall time csim needs for one quantum; and `frequency` must match
csim's `--renode-freq`, because the `tickClock` message carries only a tick
count. Renode's own scheduling quantum is 100 µs by default, so a csim
quantum finer than that buys no accuracy and costs one socket round trip
each time.

Configuration can also come from the environment (`CSIM_RENODE`,
`CSIM_RENODE_FREQ_HZ`, `CSIM_RENODE_TIMEOUT_MS`,
`CSIM_RENODE_CONNECT_TIMEOUT_MS`, `CSIM_RENODE_LOG_LEVEL`,
`CSIM_RENODE_TRACE`), which is what `plugins: ["renode"]` in a config uses —
a built-in service receives no arguments through that path.
`CSIM_RENODE_TRACE=1` prints one line per protocol step with wall-clock
timestamps; it is what a stall diagnosis needs, because the symptom of a
protocol mistake is both sides waiting and neither saying why.

The device node itself is an ordinary config entry whose firmware extension
is `.renode`. The path is never opened; the extension only selects the mote
kind, exactly as `.py` does for external nodes.

## 7. Why esp32sim keeps the NDJSON protocol

Renode's protocol is a *device-on-a-bus* protocol; csim's external-node
protocol is a *node-in-a-network* protocol. esp32sim is a whole node with its
own CPU and radio, so it does not sit on anyone's bus, and two properties
decide it:

- **Time model.** Renode's is a fixed quantum: the master says "advance N
  ticks" on a fixed grid, and the peer cannot say "wake me at 37 µs". Events
  the peer raises are seen at the next grid point. csim's node protocol steps
  a peer to the exact time of its next event, and every transmission carries
  its own nanosecond stamp inside the slice. On a 100 µs grid, every
  acknowledgement and every TSCH slot boundary would be misplaced; a 1 µs
  grid fixes that at a million socket round trips per simulated second.
- **Frame semantics.** Renode's protocol has nowhere to put "when did the
  first preamble byte hit the air". csim's `rx` carries it, along with the
  channel and the per-receiver RSSI, in one message.

What Renode's protocol does better, and is worth borrowing for the node
protocol later: an asynchronous back-channel (our peer can only speak in its
reply), a reset message, a peripheral index so one connection can carry
several models, a TCP transport, and an in-process shared-library variant.

## 8. Where this sits against the co-simulation standards

An addition to `external-nodes-plan.md` §14, which compares FMI, DCP, HLA and
DIS but predates this work.

Renode's protocol is **not** a standard; it is one tool's integration
interface, in the same family as Verilator's DPI bridge. Placed against §14's
table: its `tickClock`/`tickClock` exchange is the same shape as FMI's
`fmi3DoStep` and HLA's `timeAdvanceRequest`/`timeAdvanceGrant`, with the
master fixed and the step size fixed. Where DCP would wrap that in a
standardised PDU state machine, Renode ships 24 raw bytes, which is why a
peer can be written in an afternoon in any language.

The reason to implement it anyway is the same reason §14.3 gives for not
adopting DCP: the value is in the connection people actually want, not in the
envelope. Renode is a widely used emulator with a large board library and no
wireless network model; csim is a wireless network simulator. Speaking
Renode's own protocol makes the pair work with zero code on the Renode side,
which no standard would have achieved.

§14.4's lesson applies here too, and this implementation acts on it: the
register map documents `RX_TIME` as the frame's start **on the air**, in
normative language, rather than leaving "the timestamp" to the reader.

## 8.5 What this took from the open co-simulation PR (#1)

PR #1 ("Add co-simulation support", branch `pr-1-cosim`) solved the same
half of this problem two years earlier, for a generic JSON coordinator
instead of Renode: length-prefixed JSON over TCP, with `time_advance`,
`step_to`, `run_until`/`continue`, `rx` injection, and `tx`/`console`/
`node_idle` coming back. It predates the Phase 1–10 kernel and does not
apply cleanly, but it is the closest prior art in this tree and this design
is better for it.

**The loop shape is the same, independently.** PR #1's `cosim_wait_command()`
reads messages until a time command arrives, servicing non-time messages
(`rx` injections) along the way, and returns -1 on connection loss. That is
exactly `renode_cosim_next_horizon()`, which reads messages until a
`tickClock` arrives, services bus accesses along the way, and returns -1 on
disconnect. Two designs converging on one shape is the argument that
`sim_clock_source_t` is the right abstraction rather than a Renode-shaped
one.

**Its four commands all fit the one-function hook.** `time_advance(target)`
and `step_to(target)` are just a horizon. `run_until(target)` is a horizon
plus a service-side observer that calls `sim_runtime_request_stop()` on TX;
because `next_horizon` is handed `cur_ns`, the service can see the pump
stopped short and report the pause point (PR #1's `tx_done`) before waiting
for `continue`. Checking the hook against PR #1's vocabulary — not just
Renode's — is why it is a single function returning a time, and why no
`sim_scheduler_ops_t` vtable was needed.

**Where to put the side effects, learned from its cost.** PR #1 did its RX
injection and TX bridging *in the runner*: 656 lines in
`test_mixed_multinode.c`. Here the equivalent work happens inside the hook,
in the service, and the runner sees only a horizon — about a dozen lines.
Same protocol shape, a fiftieth of the runner footprint. That is the single
most useful thing PR #1 taught this design.

**Its DES lookahead is still worth having.** PR #1's `node_idle` reports each
node's next scheduled timer so a discrete-event coordinator can compute the
global next event instead of imposing a quantum — the LBTS idea from HLA.
Renode's fixed-quantum protocol has nowhere to put it, but the hook does not
preclude it: csim's next event time is one `sim_eq_peek_time()` call away,
and a service is free to report it. Worth building when a DES master exists.

**Shared vocabulary.** `external-nodes-plan.md` §11 asks that the two
protocols spell the same things the same way. The register window follows
it: a received frame carries the sender's node id, its channel, the
per-receiver RSSI and the frame's on-air start time — the same quantities as
PR #1's `rx`/`tx` and this tree's external-node `rx`, under the same names.

## 9. Limitations

- **Fixed quantum.** Interrupt latency is up to one quantum: csim raises the
  line at the boundary, so a frame arriving early in a 1 ms quantum is seen
  by the guest up to 1 ms later. Shrink `limitBuffer` for tighter coupling,
  at one round trip per quantum.
- **Bus accesses take zero simulation time** and are serviced at the boundary
  of the last completed quantum.
- **No bridge to Renode's own wireless medium.** csim's medium is the only
  medium. Connecting a Renode-emulated radio chip to csim's air would need a
  Renode-side C# component and is a separate project.
- **One device per csim process.** Two would make a bus access ambiguous;
  attach fails naming the count it found.
- **`resetPeripheral` resets the device, not csim's motes.** A machine reset
  in Renode clears the FIFOs and control state; it does not reboot the
  emulated network.
- **Determinism holds per master script.** csim's own run is deterministic
  for a given sequence of ticks and accesses, and the CI check asserts it.
  A master that varies its script varies the run.
- **Do not combine `--ui` pacing with Renode pacing**: both want to decide
  the horizon.
- The peer never runs Renode's co-simulated-CPU actions, so csim cannot be
  used as an HDL CPU core inside Renode.

## 10. Verification

```sh
./build/test_runner renode-cosim -v          # codec, device window, protocol loop
./build/test_runner test configs/test-renode-sky.yaml -q   # passive device, no master
tools/check-renode-cosim.sh                  # end-to-end against the Python mock master
tools/check-determinism.sh test configs/chain-4node-sky.yaml   # the off path is untouched
```

`test/test_renode_cosim.c` covers three layers: the wire codec against a
hand-written byte vector (so a layout change cannot pass by agreeing with
itself), the register window and its FIFOs, and the real service driven by a
scripted mock master over a `socketpair` — handshake, interleaved bus
accesses, tick arithmetic at a frequency that does not divide a second,
the async plane, reset, unsupported actions, disconnect and peer loss.

`tools/renode-mock-master.py` is the same thing at process level, and is what
`tools/check-renode-cosim.sh` gates in CI. **CI tests the protocol, not
Renode**: Renode is not a build dependency, and the mock speaks its wire
format. The script runs coarse quanta (100 ms) because the protocol exchange
is identical at any quantum size while the cost is socket round trips, not
simulation — at 1 ms quanta the same coverage took about eight minutes
instead of half a second. `LIMIT`/`TICKS` override it for a slower, finer
run. It asserts that elapsed simulation
time matches the ticks exactly, that frames cross in both directions (an
injected frame must raise the emulated Sky nodes' CRC-valid reception count),
that console lines arrive as log messages, that two identical scripts produce
identical output, and that a master that dies mid-run ends the csim run
cleanly with a reason.

Running against real Renode is documented (`examples/renode/`) but not
gated: Renode is not a build dependency.

### What the mocks could not catch

Worth stating plainly, because it shaped the test suite. Three separate mock
masters — the unit test's socketpair master, `tools/renode-mock-master.py`,
and the inline one in `check-renode-cosim.sh` — all agreed that the
`tickClock` confirmation goes back on the *main* socket. All three passed.
Renode 1.17 then deadlocked on the first tick: csim replied in 3 ms and
Renode waited out its entire tick timeout, because the confirmation belongs
on the **async** socket (`renode_bus.cpp`, `case tickClock` → `sendSender`).

The mocks were not testing the protocol. They were testing that the
implementation agreed with my reading of the protocol, and they were written
from the same reading, so they could only ever confirm it. A mock derived
from the implementation cannot find a misunderstanding shared by both.

Two things follow, and both are now done. The mocks encode the real
asymmetry — bus replies on main, everything else on async — and the unit test
asserts the negative as well (`"nothing but bus replies goes on the main
socket"`), so an implementation that drifts back fails. And
`CSIM_RENODE_TRACE=1` exists because the symptom of a protocol mistake is
both sides waiting and neither able to say why; the trace showing csim's
reply going out 3 ms after the tick, against Renode's 20-second timeout, is
what turned a deadlock into a one-line fix.

## 11. Sharing the air: Renode's radio on csim's medium

§7 says csim's medium is the only medium and bridging Renode's own wireless
model is out of scope. It is now prototyped, because it is the integration
people actually want: firmware Renode can run, on a network csim models.

`examples/renode/bridge/CsimBridge.cs` is a Renode radio that owns no
hardware. It sits on Renode's `IEEE802_15_4Medium`, forwards every frame it
hears into csim, and re-emits every frame csim delivers back — carried over
the same co-simulation window that keeps the clocks in step, so there is no
second protocol and no csim-side change. Renode compiles it at run time
(`include @…/CsimBridge.cs`), so there is nothing to build.

```
Renode CC2538 ── wireless medium ── CsimBridge ── window ── csim UDGM medium
   (Contiki-NG)                                                (Sky, …)
```

### What is proven

Running `examples/renode/cc2538-csim-rpl.resc` against
`configs/test-renode-bridge-sky.yaml`, with unmodified Contiki-NG on both
sides (CC2538 in Renode, Tmote Sky in csim — different CPU architectures, in
different simulators):

- **Frames cross both ways.** csim's Sky transmits RPL DIOs; the bridge
  delivers them to Renode's CC2538. The CC2538 transmits; csim's CC2420
  reports `crc_ok`, so the frames arrive intact through csim's medium.
- **The RPL DAG forms across the boundary.** The CC2538 emits unicast frames
  addressed to `0101010001741200` — the Sky's link-layer address
  `0012.7401.0001.0101`. It can only have learned that from the Sky's DIO, so
  it parsed the DIO, ran the objective function, and selected a node in the
  other simulator as its RPL parent.

### Unicast works — once the acknowledgement respects the turnaround

`configs/test-renode-root-rpl.yaml` runs the rpl-udp **server in Renode**
(the CC2538 is the DAG root) and the **client in csim** (a Tmote Sky). The
client hears the root's DIOs, selects it as parent, registers with a DAO, and
completes UDP request/response round trips through it — every one of which
crosses the simulator boundary:

```
[Renode root]  Received request 'hello 0' from fd00::212:7401:1:101
[Renode root]  Sending response.
[csim Sky]     Received response 'hello 0' from fd00::200:0:0:2
```

`configs/test-renode-root-rpl-2clients.yaml` adds an nRF52840 client
alongside the Sky: the root serves both, from their own IPv6 addresses, in
one run — two csim ISAs joining a DAG rooted in a third simulator.

### The bug, found by reading the root's MAC log rather than csim's counters

Rebuilding the root with `LOG_CONF_LEVEL_MAC=4` showed the whole failure in
three lines:

```
[INFO: CSMA] received packet from 0012.7401.0001.0101, seqno 198, len 79
[WARN: CSMA] drop duplicate link layer packet from 0012.7401.0001.0101, seqno 198
[WARN: CSMA] drop duplicate link layer packet from 0012.7401.0001.0101, seqno 198
```

The root receives csim's DAO, acknowledges it, and processes it — then the
client retransmits the *same* sequence number, and every retransmission is
dropped as a duplicate. The client is retransmitting because it never
accepted the acknowledgement: its CC2420 reported `ack_rx=0`, and
`dropped=22` bytes — exactly two 11-byte ACK frames — arriving while the radio
was not in receive.

The acknowledgements were correct (`seq=0xc6` = 198, matching the DAO). They
were **early**. Renode's radio has no air time, so its ACK is ready the same
instant the frame is, and the bridge put it on csim's air immediately — while
the Sky was still in its TX→RX turnaround and could not hear it. 802.15.4
gives that turnaround 192 µs, and csim's own auto-ACK path waits exactly that
long for exactly this reason (`sim_radio_bus.c`: "otherwise perpetual CSMA
retransmits"). The bridge now holds a Renode-originated ACK in its own slot
until `frame_end + 192 µs` of csim time, where `frame_end` is the on-air end
of the frame it acknowledges (`RX_TIME + (len + 8) × 32 µs`). With that one
change the Sky's `ack_rx` went from 0 to 12 and the DAG completed.

This is the second time the frame-level/PHY-level mismatch has been the
cause, and the two halves are symmetric: §11's air-time fix made csim able
to receive Renode's frames; this makes csim able to accept Renode's
acknowledgements. In both, the bridge's job is to supply the time Renode's
model does not have.

### The reverse direction, and why it is not pursued

The **reverse** topology — Renode's CC2538 as a *client*, csim's node as
root — does not complete RPL (`configs/test-renode-bridge-sky.yaml`). There
csim generates the acknowledgement with correct turnaround and the bridge
re-emits it into Renode's medium; whether Renode's `CC2538RF` model matches it
to its pending transmission has not been instrumented.

It is left alone by design. Renode is the natural **server** side of this
pairing: its value is emulating the gateway — a Linux host such as a
Raspberry Pi running a native border router (NBR) — at the head of a
constrained network that csim models at chip level. In that arrangement
Renode is always the RPL root and csim's nodes always join it, which is the
direction that works. Renode as a leaf of a csim-rooted network has no
corresponding use, so its ACK path is not worth the Renode-side
instrumentation.

### Ruled out on the way, each by measurement

- Renode's medium is not lossy (`SimpleMediumFunction`, no range function).
- The bridge does not write partial frames (whole 3/25/83/100-byte frames;
  the device drops a short write rather than transmitting it).
- The co-simulation quantum is not the limit: 1 ms, 100 µs and 50 µs quanta
  behave identically, against a `CSMA_ACK_WAIT_TIME` of 400 µs.
- Carrier sense was not the cause. The window exposes csim's own CCA
  (`RENODE_REG_CCA`, 0x70) and the bridge defers on it, and it never asserts:
  `tx_busy_until_ns` is written at frame_complete with the frame's air *end*,
  so it is always already past when a peer polls. The register stays — it is
  correct and any frame-level peer will want it — but it is not what was
  breaking RPL.
- csim's radio models are not the cause: Sky (CC2420), CC2538 (RF Core) and
  nRF52840 all failed identically as the root before the fix, so the common
  factor was the bridge.
- csim's own acknowledgement path works (`auto_ack=8` in the same runs, and
  `auto_ack=1` for a cleanly injected unicast with no Renode present).

### Also found while building this

- The 802.15.4 FCS is **CRC-16/KERMIT** (reflected 0x8408, init 0), not the
  MSB-first CCITT variant. The bridge got this wrong at first and every
  injected frame was silently dropped; it now recomputes the checksum of a
  frame Renode transmitted and logs a warning if the two disagree, which
  turns an invisible failure into one line.
- An earlier revision also reported that csim's **nRF52840** receiver never
  sees injected frames, "because not even the radio-level counters move".
  Also wrong: the nRF has no per-chip RX counters to move — a known-good
  two-node nRF RPL-UDP run prints the same zeroes. The nRF interoperates
  happily with the Sky and the CC2538 in
  `configs/test-mixed-platform-rpl.yaml`.
- Both retracted claims came from reading a counter that was never wired up
  for the platform in question. Absence of a counter is not evidence of
  absence of delivery; the reliable question is whether the *application*
  received something, which is what `configs/test-mixed-platform-rpl.yaml`
  asserts.
- And a third: csim's auto-ACK was reported as not arming for frames from a
  frame-level sender. It does — `auto_ack=1` for a correctly addressed
  injected unicast, no Renode involved. Every one of these retractions came
  from reading a counter in a cross-simulator run and blaming the side that
  was easier to instrument. The discipline that actually worked: reproduce it
  with the mock master and one csim process before believing anything about
  which simulator is at fault.

## 12. Three simulators at once: csim as both clock slave and clock master

The two co-simulation protocols in this tree run in opposite directions —
Renode owns the clock in §2, csim owns it in `external-nodes-plan.md`. They
compose. Running both at once makes csim a clock **slave** and a clock
**master** simultaneously, while it owns the radio medium for everyone:

```
Renode ──tickClock──▶ csim ──step/done──▶ esp32sim
 CC2538              Sky                  ESP32-C6
 ARM Cortex-M3       MSP430, 16-bit       RISC-V, 32-bit
 (binary, bus-shaped) (owns the medium)   (NDJSON, node-shaped)
```

Three independent emulators, three instruction sets, three separate OS
processes, one 802.15.4 medium, one timeline. Every node runs unmodified
vendor firmware: Contiki-NG on the Sky and the CC2538, and Contiki-NG on
ESP-IDF (mask ROM, second-stage bootloader, FreeRTOS) on the C6.

**The network is the standard stack.** IPv6 over 6LoWPAN, RPL Lite routing,
UDP — IETF protocols, which is what makes this an interoperation claim rather
than a frame-delivery one. The configuration is
`configs/test-threeway-cosim.yaml`: the Sky is the RPL root and UDP server,
the ESP32-C6 a client that must join the DAG and complete request/response
round trips through it.

### Measured, against real Renode 1.17

**Standard-stack interoperation between two emulators.** csim's MSP430 and
esp32sim's RISC-V complete UDP request/response round trips over 6LoWPAN and
RPL, in separate OS processes, from different vendor SDKs:

```
24.247 [Node 2/EXT]    Sending request 0 to fd00::212:7401:1:101
                       6LoWPAN output: sending IPv6 packet with len 63
24.266 [Node 1/MSP430] Received request 'hello 0' from fd00::ff:fe00:2
24.317 [Node 2/EXT]    Received response 'hello 0' from fd00::212:7401:1:101
```

This needs everything a real deployment needs: DAG formation, 6LoWPAN header
compression and decompression across two independent implementations, unicast
with link-layer acknowledgements, and CSMA. Ten round trips in a 120 s run
without a clock master; five in 70 s under Renode.

**The clocks stay locked.** 70 000 quanta of 1 ms drove exactly 70 s of
simulated time in every simulator, with zero Renode errors.

**Exact-time stepping survives the outer quantum.** This is the property that
makes the composition worth having rather than merely possible. csim relays
Renode's quantum to esp32sim as *event-driven* stepping, not as a grid:
esp32sim's transmissions land at the times its own guest firmware chose,
inside the quantum. A naive relay that forwarded the master's grid would
round them and destroy exactly the acknowledgement timing that RPL's unicast
depends on — which is why the two protocols keep their own time models and
csim mediates between them, the asymmetry §7 gives for keeping them separate.

**It is deterministic end to end.** Two full three-simulator runs under real
Renode produce an identical sequence of application and radio events, and two
runs under the mock master reproduce every frame timestamp to the nanosecond.
Determinism is a gated guarantee in this tree, and composing three simulators
across three processes does not cost it.

### Renode as the root of the whole thing

`configs/test-threeway-root-rpl.yaml` moves the RPL root *into Renode*: the
CC2538 runs the rpl-udp server, and both other simulators' nodes — csim's
MSP430 Sky and esp32sim's RISC-V ESP32-C6 — are clients that must join its
DAG. Measured against Renode 1.17, in one 90 s run:

- the root served **8** request/response round trips from the Sky
  (`fd00::212:7401:1:101`) and **6** from the ESP32-C6 (`fd00::ff:fe00:2`);
- 900 000 quanta of 100 µs drove exactly 90 s in every simulator, 0 errors.

Every DIO, DAO, DAO-ACK, and UDP datagram in that network crosses at least
one simulator boundary, and the root crossing is over Renode's own radio
model. That makes Renode a **routed** participant, not merely a link-layer
one — the gap §11 closed, by supplying the ACK turnaround its frame-level
radio does not model.

### Running it

esp32sim is an out-of-tree binary (`ESP32SIM_C6`, default
`~/work/esp32sim/target/release/esp32sim-c6`), so this is a documented
harness, not a CI gate — same status as the real-Renode runs. The
csim+esp32sim half needs no Renode and is a plain config run.

```sh
# two simulators, 6LoWPAN/RPL/UDP: csim drives esp32sim
./build/test_runner test configs/test-threeway-cosim.yaml -v

# all three: Renode drives csim, csim drives esp32sim
CSIM_CONFIG=$PWD/configs/test-threeway-cosim.yaml \
    renode examples/renode/cc2538-csim-rpl.resc
```

### What this demonstrates

csim is not merely *connectable* to another simulator; it composes as a
co-simulation **hub**. It accepts an external time authority through
`sim_clock_source_t` (§3) without the runner loop naming the protocol, relays
that authority to its own subordinate peers in a different protocol with a
different time model, and remains the single owner of the shared physical
medium that all of them transmit into — while the guests on top of it speak
standard IPv6/6LoWPAN/RPL to each other. Adding a fourth participant on
either side is a config line, not a code change.
