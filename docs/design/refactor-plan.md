# Cooja-NG Refactor Plan

Audience: Codex, Claude Code, and humans doing incremental architecture work.

Purpose: describe where the simulator is today, define the target model for
CPUs/platforms/motes/plugins, and give a safe phase-by-phase plan that preserves
current behavior while reducing coupling.

This is a refactor plan, not a rewrite plan. The CPU emulators, radio chip
drivers, firmware tests, and Cooja timing compatibility are valuable working
assets. The goal is to move them behind stable contracts.

## 1. Current State

### 1.1 What the project already has

The codebase is no longer just Sky + CC2538. It currently contains:

- MSP430/MSP430X CPU, MCU configs including MSP430FR5969, GPIO, USART/eUSCI,
  timers, clock, ELF loader, and CC2420.
- ARM M-profile CPU interpreter and platform support for CC2538, nRF52840, and
  nRF54L15-oriented SoC files.
- CC2538 RF core, Nordic RADIO models, and CC1200 sub-GHz off-SoC radio.
- Native Cooja motes loaded through `dlopen`.
- JavaScript app motes through QuickJS.
- JSON simulation configs, Cooja-style JS test scripts, WebSocket UI, timeline,
  packet analyzer, PCAP writer, GDB stub, serial socket bridge.
- Device-oriented docs and specs under `devices/`.
- A CPU-agnostic host vtable for off-SoC chips: `include/common/sim_host.h`.
- A unified per-CPU event type for chip drivers: `include/common/cpu_event.h`.
- A multi-radio radio medium with spectra/channels/RX-enable state:
  `include/common/radio_medium.h`.

Important existing docs:

- `docs/architecture.md`
- `docs/porting-a-device.md`
- `docs/radio-medium.md`
- `devices/SPEC-template.md`

### 1.2 Main architecture debt

`test/test_mixed_multinode.c` is still the real simulation kernel. At the time
of this plan it is about 5400 lines and owns too many responsibilities:

- command-line mode handling below `test_main.c`
- node type detection by firmware filename extension
- platform selection and firmware boot patching
- global simulation state
- mote lifecycle
- architecture-specific stepping
- event-driven scheduler loop
- radio dispatch and ACK/collision timing
- per-radio channel bridging
- native/JS/emulated RF bridging
- serial socket bridge
- test actions
- JS test engine integration
- UI server integration
- timeline and packet logging
- PCAP capture
- GDB attachment
- debug/stat dumps

This makes every new platform or radio feature require edits in the runner. The
runner should become a frontend around a reusable simulation library.

### 1.3 Good partial refactors already in place

Build on these. Do not duplicate them.

#### CPU-agnostic chip host

`sim_host_t` lets off-SoC chips call back into the owning CPU/GPIO without
including MSP430 or ARM types:

- `now_ns`
- `schedule_ns`
- `cancel`
- `set_input_pin`
- `force_irq_edge`
- `radio_set_channel`

This is the right direction. Future external chips should take `const
sim_host_t *host`, not `msp430_cpu_t *`, `arm_cpu_t *`, or GPIO-specific types.

#### ARM SoC ops

`include/arm/arm_platform.h` already moved ARM toward:

- `arm_platform_t`: CPU + NVIC + SysTick + opaque SoC pointer
- `arm_soc_ops_t`: SoC lifecycle vtable
- `arm_platform_config_t`: board/platform descriptor

This is a good model. The refactor should generalize the idea, not undo it.

#### Multi-radio medium

`radio_medium_t` is now a policy oracle with per-node radio slots, spectrum
identity, channel state, RX-enable state, and profile-aware frame tracking. It
does not own chip pointers or delivery callbacks. Keep that separation.

#### Porting workflow

`docs/porting-a-device.md` and `devices/SPEC-template.md` define a useful
bottom-up process. The refactor should make this process easier by moving port
logic out of the runner.

## Chosen Direction

The first architecture target is an internal simulation kernel, not a public ABI.
The kernel should be extracted far enough that `test/test_mixed_multinode.c`
becomes a frontend, but public API/ABI stability is explicitly deferred.

Plugin architecture starts with static registration of built-in mote types,
platforms, SoCs, radio media, and services. Dynamic `dlopen` plugins are a later
phase after the static registry and kernel APIs prove stable.

The first milestone is runner shrinkage: move global time, the event queue,
radio-medium ownership, mote slots, and observer dispatch into `sim_runtime_t`
without changing simulation behavior.

## 2. Target Architecture

### 2.1 Core object model

Use these terms consistently:

```text
Simulation runtime
  Owns global time, event queue, motes, registry, services, RNG, stats.

CPU architecture
  Instruction set and CPU execution engine.
  Examples: msp430, msp430x, armv7m, future armv8m/riscv/avr.

MCU / SoC
  Memory map, interrupt controller, built-in peripherals, clock tree.
  Examples: MSP430F1611, MSP430F5437, CC2538, nRF52840, nRF54L15.

Board / platform
  Concrete wiring around an MCU/SoC: console, LEDs, buttons, external chips.
  Examples: sky, z1, cc2538dk, zoul-firefly, nrf52840-dongle.

Chip / device
  A peripheral block or external chip.
  Examples: CC2420, CC1200, CC2538 RFCore, Nordic RADIO, M25P16 flash.

Mote type
  The simulation-level executable model.
  Examples: emulated ELF mote, native Cooja mote, JS app mote.

Mote instance
  One node in a simulation: mote type + firmware + id + position + runtime state.

Service
  Optional behavior around the simulation: UI, PCAP, GDB, serial socket,
  packet analyzer, timeline, test engine, custom radio medium. All built-in
  components are services. The kernel calls them through `sim_service_ops_t`.

Plugin
  A *service* (or other registry entry) loaded dynamically via `dlopen`. This
  is the Phase 9+ extension model. Until then, "plugin" appears only in
  forward-looking design notes; everything built-in is a "service".
```

The important distinction: a board is not a CPU, and a mote type is not a
board. A `sky` mote happens to be an emulated ELF mote using the MSP430F1611 MCU
on the Sky board with a CC2420. Those parts should be separately registered.

### 2.2 Desired module layout

The exact directory names can change, but new work should converge toward this
shape:

```text
include/sim/
  sim_runtime.h
  sim_event.h
  sim_mote.h
  sim_registry.h
  sim_service.h
  sim_radio_bus.h
  sim_config_v2.h

src/sim/
  sim_runtime.c
  sim_event.c
  sim_mote.c
  sim_registry.c
  sim_radio_bus.c
  sim_services.c

src/motes/
  emulated_elf_mote.c
  native_cooja_mote.c
  js_app_mote.c

src/platforms/
  sky_platform.c
  z1_platform.c
  cc2538dk_platform.c
  zoul_firefly_platform.c
  nrf52840_dongle_platform.c
  nrf52840_dk_platform.c
  nrf54l15_dk_platform.c

src/services/
  timeline_service.c
  packet_analyzer_service.c
  pcap_service.c
  websocket_ui_service.c
  serial_socket_service.c
  gdb_service.c
  js_test_service.c
```

Keep existing `src/msp430`, `src/arm`, `src/common`, `src/native`, and `src/ui`
working while extracting. Do not move many files in the first phases.

## 3. Main Simulation Kernel Design

This section is the concrete target for the core "main" kernel. The kernel is
the deterministic engine underneath multi-node simulation, UI mode, tests,
future plugins, and batch/CI execution.

Kernel contract:

- Owns simulation time and the global event queue.
- Dispatches events in `(time_ns, seq)` order.
- Owns mote slots and generation counters.
- Calls motes only through `sim_mote_ops_t`.
- Routes RF only through `sim_radio_bus`.
- Exposes observer events to services.
- Does not know CPU, SoC, chip, UI, GDB, PCAP, or JS-test internals.

Non-goal for v1: expose a stable embeddable library ABI. The first API is
internal and may change while the runner is being reduced.

### 3.1 Kernel responsibilities

The kernel owns:

- global simulation time (`now_ns`)
- the single global event queue
- mote slots and lifecycle state
- node identity, activity, and generation counters
- radio bus and radio medium
- service/plugin list
- observer event fan-out
- deterministic RNG streams
- run/stop/pause state
- common stats

The kernel does not own:

- CPU instruction execution details
- platform boot patching details
- chip state-machine internals
- UI serialization details
- JS test semantics
- GDB protocol details
- PCAP file format details

Those live behind mote ops, radio bus ops, or service ops.

### 3.2 Source-of-truth scheduler

The source of truth must be the event-driven Cooja-style scheduler:

```text
global queue ordered by (time_ns, seq)
  -> pop one event
  -> set sim->now_ns = event.time_ns
  -> dispatch event
  -> event/mote/service may schedule more future events
```

Same-time FIFO ordering is mandatory. This is how Cooja orders mote execution,
radio byte deliveries, immediate wakeups, serial input, and test actions.

The current `sim_event_queue_t` already implements the core idea for two event
kinds:

- `SIM_EV_NODE_WAKEUP`
- `SIM_EV_RX_BYTE`

The kernel should generalize that without losing the important current
semantics:

- at most one pending mote wakeup per mote, reschedule replaces the old one
- many RF byte events may target the same mote
- all events share one `(time_ns, seq)` ordering
- event callbacks observe the exact event time, not a coarse outer tick time

### 3.3 Kernel event model

The eventual queue should support more than node wakeup and RF byte events, but
the implementation can evolve from the current struct.

Target event kinds:

```c
typedef enum sim_event_kind {
    SIM_EVENT_MOTE_EXECUTE = 1,
    SIM_EVENT_RADIO_BYTE,
    SIM_EVENT_RADIO_FRAME_END,
    SIM_EVENT_SERIAL_INPUT,
    SIM_EVENT_SERVICE_TIMER,
    SIM_EVENT_TEST_ACTION,
    SIM_EVENT_USER,
} sim_event_kind_t;
```

Target event shape:

```c
typedef struct sim_event {
    sim_event_kind_t kind;
    int64_t time_ns;
    uint64_t seq;

    /* Target identity. generation lets the kernel drop stale events after
     * mote removal/reboot without searching every payload. */
    int target_mote;
    uint32_t target_generation;

    union {
        struct {
            int sender_mote;
            int sender_radio;
            int receiver_radio;
            uint8_t byte;
            int8_t rssi;
        } radio_byte;

        struct {
            const uint8_t *data;
            int len;
        } serial_input;

        struct {
            int service_id;
            uint32_t timer_id;
        } service_timer;

        struct {
            int action_id;
        } test_action;
    } u;
} sim_event_t;
```

Implementation note: do not introduce heap allocation for every RF byte if it
can be avoided. The current by-value event payload is good for hot radio paths.
Use handles or fixed buffers for large payloads.

### 3.4 Mote slot model

The runtime should keep a stable mote slot table:

```c
typedef enum sim_mote_state {
    SIM_MOTE_EMPTY = 0,
    SIM_MOTE_ACTIVE,
    SIM_MOTE_STOPPED,
    SIM_MOTE_REMOVED,
} sim_mote_state_t;

typedef struct sim_mote_slot {
    sim_mote_t mote;
    sim_mote_state_t state;
    uint32_t generation;
    int64_t start_time_ns;
    int64_t last_execute_ns;
    double clock_deviation;
} sim_mote_slot_t;
```

Why generation matters:

- test scripts can remove/re-add nodes
- UI can restart simulation
- reboot can destroy and recreate platform state
- RF byte events may already be queued for a node when it is removed

Every queued event targeting a mote should carry the generation observed when
it was scheduled. If the slot generation changed, dispatch drops the event.

### 3.5 Mote host callbacks

Motes need a kernel-facing host API. This is separate from `sim_host_t`, which
is for off-SoC chips talking to their owning CPU/GPIO.

```c
typedef struct sim_mote_host {
    sim_runtime_t *sim;
    int mote_index;

    int64_t (*now_ns)(sim_runtime_t *sim);

    void (*schedule_wakeup)(sim_runtime_t *sim, int mote_index,
                            int64_t time_ns);
    void (*schedule_wakeup_if_earlier)(sim_runtime_t *sim, int mote_index,
                                       int64_t time_ns);

    void (*radio_tx_byte)(sim_runtime_t *sim, int mote_index, int radio_idx,
                          uint8_t byte, int64_t tx_time_ns);
    void (*radio_tx_frame)(sim_runtime_t *sim, int mote_index, int radio_idx,
                           const uint8_t *frame, int len, int64_t tx_time_ns);
    void (*radio_set_channel)(sim_runtime_t *sim, int mote_index,
                              int radio_idx, int channel);
    void (*radio_set_rx_enabled)(sim_runtime_t *sim, int mote_index,
                                 int radio_idx, bool enabled);

    void (*emit_log)(sim_runtime_t *sim, int mote_index, const char *line);
    void (*emit_led)(sim_runtime_t *sim, int mote_index, int led, bool on);
    void (*emit_debug)(sim_runtime_t *sim, int mote_index, const char *msg);
} sim_mote_host_t;
```

Emulated mote adapters use this to bridge platform callbacks into the kernel.
Native and JS motes use it directly for log/RF output.

### 3.6 Mote execution contract

The kernel calls a mote only at a specific global time. A mote must not advance
global time.

```c
typedef struct sim_mote_exec_result {
    int64_t next_wakeup_ns;   /* INT64_MAX = no known wakeup */
    bool requested_stop;
} sim_mote_exec_result_t;

int sim_mote_execute(sim_mote_t *mote, int64_t now_ns,
                     sim_mote_exec_result_t *out);
```

For emulated MSP430/ARM motes, `execute(now_ns)` wraps the current
`tick_one_msp430` / `tick_one_arm` behavior:

1. convert global `now_ns` to the mote's local jump interval
2. apply clock deviation
3. pin the CPU/peripheral `sim_time_ns` to `now_ns`
4. run the CPU for the Cooja-style execution slice
5. return a next wakeup hint based on CPU events/interrupts

**Timing hazard: cpu->cycles ≠ sim_time.** Step 3 pins `sim_time_ns` to the
event's `now_ns` even when that goes backward relative to the mote's last
execution slice. The mote's cycle counter never goes backward, so a receiver
whose CPU was WFI-fast-forwarded ahead of the byte stream's sim_time will
process dozens of consecutive byte events at the same `cpu->cycles` value
before the local accumulator (`last_micros_delta`) catches up. Any chip-level
timer scheduled in cpu-cycle coordinates — stall watchdog, ACK turnaround,
TX-end defer — that needs to track "wall-clock since last RF byte" must
therefore be anchored in sim-time (`SIM_EVENT_*` in the global queue) and
not in the chip's cpu-event-queue. The historical `nrf54l_radio_rx_stall_event`
was the first place this bit us; section 6.3 makes it an invariant going
forward.

For native Cooja motes, `execute(now_ns)` wraps the current `tick_one_native`
behavior.

For JS motes, `execute(now_ns)` dispatches queued frames and scheduled JS
callbacks up to `now_ns`.

### 3.7 Kernel run loop

The deterministic single-threaded kernel loop should look like this:

```c
int sim_runtime_run_until(sim_runtime_t *sim, int64_t end_ns) {
    sim_notify_start(sim);

    while (!sim->stop_requested) {
        int64_t next = sim_event_queue_peek_time(&sim->queue);

        if (next == INT64_MAX) {
            if (!sim_services_keepalive(sim))
                break;
            sim_services_poll_idle(sim);
            continue;
        }

        if (next > end_ns) {
            sim->now_ns = end_ns;
            break;
        }

        sim_event_t ev = sim_event_queue_pop(&sim->queue);
        sim->now_ns = ev.time_ns;

        if (!sim_event_target_is_current(sim, &ev))
            continue;

        sim_dispatch_event(sim, &ev);

        sim_services_poll_after_event(sim);
    }

    sim_notify_stop(sim);
    return sim->exit_code;
}
```

`sim_dispatch_event`:

```c
static void sim_dispatch_event(sim_runtime_t *sim, const sim_event_t *ev) {
    switch (ev->kind) {
    case SIM_EVENT_MOTE_EXECUTE:
        sim_dispatch_mote_execute(sim, ev);
        break;
    case SIM_EVENT_RADIO_BYTE:
        sim_radio_bus_deliver_byte(sim, ev);
        break;
    case SIM_EVENT_RADIO_FRAME_END:
        sim_radio_bus_deliver_frame_end(sim, ev);
        break;
    case SIM_EVENT_SERIAL_INPUT:
        sim_dispatch_serial_input(sim, ev);
        break;
    case SIM_EVENT_SERVICE_TIMER:
        sim_dispatch_service_timer(sim, ev);
        break;
    case SIM_EVENT_TEST_ACTION:
        sim_dispatch_test_action(sim, ev);
        break;
    default:
        sim_log_warn(sim, "unknown event kind");
        break;
    }
}
```

Mote execute dispatch:

```c
static void sim_dispatch_mote_execute(sim_runtime_t *sim,
                                      const sim_event_t *ev) {
    sim_mote_slot_t *slot = &sim->motes[ev->target_mote];
    sim_mote_exec_result_t r = { .next_wakeup_ns = INT64_MAX };

    slot->mote.ops->execute(&slot->mote, sim->now_ns, &r);

    if (r.next_wakeup_ns < INT64_MAX)
        sim_schedule_mote_wakeup_if_earlier(sim, ev->target_mote,
                                            r.next_wakeup_ns);
}
```

The exact implementation will differ, but these ownership boundaries should
hold.

### 3.8 Event scheduling API

The kernel needs a small public scheduling API:

```c
void sim_schedule_mote_wakeup(sim_runtime_t *sim, int mote_index,
                              int64_t time_ns);
void sim_schedule_mote_wakeup_if_earlier(sim_runtime_t *sim, int mote_index,
                                         int64_t time_ns);

void sim_schedule_radio_byte(sim_runtime_t *sim,
                             int receiver_mote, int receiver_radio,
                             int sender_mote, int sender_radio,
                             uint8_t byte, int8_t rssi,
                             int64_t time_ns);

void sim_schedule_serial_input(sim_runtime_t *sim, int mote_index,
                               const uint8_t *data, int len,
                               int64_t time_ns);

void sim_schedule_service_timer(sim_runtime_t *sim, int service_id,
                                uint32_t timer_id, int64_t time_ns);

void sim_cancel_mote_events(sim_runtime_t *sim, int mote_index);
```

Coalescing rules:

- mote wakeup: one pending event per mote; reschedule replaces the old one
- radio byte: never coalesced
- serial input: usually not coalesced, but may point to a stable buffer
- service timer: one pending event per `(service_id, timer_id)` if the service
  asks for coalescing
- test action: never coalesced unless action IDs are explicitly unique

### 3.9 Radio bus relationship to the kernel

The kernel owns the radio bus. The radio bus owns dispatch policy for on-air
bytes and frames. The radio medium remains a policy oracle.

TX path:

```text
chip/native/js emits TX byte
  -> mote host radio_tx_byte()
  -> sim_radio_bus_tx_byte(sim, sender, radio_idx, byte, tx_time_ns)
  -> radio_medium_filter_byte_radio(...) for every candidate receiver/radio
  -> sim_schedule_radio_byte(...) for each accepted byte
```

RX path:

```text
SIM_EVENT_RADIO_BYTE
  -> sim_radio_bus_deliver_byte(...)
  -> receiver mote ops radio_receive_byte(...)
  -> receiver chip may raise IRQ through its CPU-local event queue
  -> kernel schedules receiver mote wakeup if needed
```

Critical rule: the radio bus must not call `step_node_until()` or directly run
a CPU. It schedules kernel events and lets the main loop execute motes. This
prevents re-entrant CPU execution from becoming the simulator's hidden control
flow.

### 3.10 Services relationship to the kernel

Services observe and request work through public APIs.

Examples:

- timeline service observes radio/LED/log events
- UI service observes state and polls sockets
- PCAP service observes completed TX frames
- JS test service observes log lines and schedules test actions
- serial socket service polls TCP and schedules serial input events
- GDB service polls debug sockets and may pause a mote

Services should not reach into `msp430_cpu_t`, `arm_cpu_t`, or chip internals
from the kernel layer. If a service needs architecture-specific access, it
should request an interface from the mote:

```c
void *iface = NULL;
if (mote->ops->get_interface(mote, SIM_IFACE_GDB_ARM, &iface) == 0) {
    /* attach GDB service to ARM-specific debug interface */
}
```

### 3.11 Observer events

The kernel should provide one observer stream for services:

```c
typedef enum sim_observer_event_kind {
    SIM_OBS_MOTE_ADDED,
    SIM_OBS_MOTE_REMOVED,
    SIM_OBS_MOTE_UART_BYTE,    /* raw byte off the console UART       */
    SIM_OBS_MOTE_LOG_LINE,     /* line-assembled "\n"-terminated text */
    SIM_OBS_LED_CHANGED,
    SIM_OBS_RADIO_TX_START,
    SIM_OBS_RADIO_TX_END,
    SIM_OBS_RADIO_RX_START,
    SIM_OBS_RADIO_RX_END,
    SIM_OBS_RADIO_INTERFERENCE,
    SIM_OBS_PACKET_FRAME,
    SIM_OBS_SIM_STOP,
} sim_observer_event_kind_t;
```

UART-byte vs log-line distinction matters: TUN/serial-socket consumers need the
raw byte stream, JS test scripts and `COOJA.testlog` need line-assembled
output, and PCAP wants neither. Keeping them separate avoids a service having
to undo line-assembly to get bytes.

Services subscribe to observer events. They do not become hard-coded calls in
the main loop.

### 3.12 Stop, pause, and wall-clock pacing

The deterministic kernel should only know simulation time. Wall-clock pacing is
a service concern.

State:

```c
typedef enum sim_run_state {
    SIM_RUN_STOPPED = 0,
    SIM_RUN_RUNNING,
    SIM_RUN_PAUSED,
    SIM_RUN_STOP_REQUESTED,
} sim_run_state_t;
```

Rules:

- `pause` means services keep polling, but simulation events are not dispatched.
- `stop` exits the run loop.
- UI speed throttling sleeps in the UI/pacing service, not in mote execution.
- Serial socket keepalive is a service policy, not a kernel condition.

### 3.13 Threading model

The event-driven single-threaded kernel is the reference semantics.

The optional `--threads N` batch stepping path **was retired in Phase 5 M29**
(commit follows §3.18). It never had the kernel's clean event semantics, and
keeping two divergent schedulers alive through the radio-bus extraction would
have doubled the surface every milestone had to keep byte-identical. The
original decision was "port to a `sim_scheduler_ops::batch` entry or retire by
Phase 6"; with zero usage signals (no config, CI, tool, or doc referenced it)
the resolution was **retire**. A future batch scheduler, if wanted, is now
greenfield work on the post-Phase-5 bus APIs (`sim_radio_bus_*`,
`sim_mote_ops_t`, `sim_runtime_run_until`) rather than a port of the deleted
type-switched path. The original staged plan is preserved below for context.

> _Historical (superseded by M29):_
> - Through Phase 5 (radio bus extraction): keep `--threads N` working as a
>   separate code path; do not let it define the core API.
> - After Phase 5: re-implement `--threads N` behind a second
>   `sim_scheduler_ops_t` entry (`batch`) sitting on top of the new
>   mote/radio/service APIs. If that proves more invasive than expected, retire
>   the threaded path instead of letting two divergent schedulers persist.
>
> The choice was "port or retire by Phase 6" — not "keep both forever."

```c
typedef struct sim_scheduler_ops {
    const char *name;
    int (*run_until)(sim_runtime_t *sim, int64_t end_ns);
} sim_scheduler_ops_t;
```

Built-in scheduler policies:

- `event`: deterministic Cooja-style single event queue; reference scheduler.
- `batch`: parallel/batched scheduler for large topologies; uses the same
  mote/radio/service APIs; may trade some fidelity for throughput only when
  explicitly selected.

### 3.14 Kernel invariants

These are non-negotiable:

- `sim->now_ns` is monotonic.
- Dispatch sets `sim->now_ns` to the exact event time.
- Same-time events fire by insertion sequence.
- Motes cannot directly advance global time.
- Motes can request wakeups but cannot replace other motes' wakeups except via
  kernel APIs.
- Radio byte events are not deduplicated.
- Removed/rebooted mote events are dropped by generation check.
- Services observe through observer events and mutate through public APIs.
- The reference scheduler is deterministic for a fixed config and seed.
- CPU-local cycle queues remain inside mote/platform implementations.

### 3.14.1 Performance budget

The kernel introduces an extra layer between chip TX callback and chip RX
callback. Budget (per-phase regression vs. the immediately-preceding tag on the
2-node Sky `udp-server.sky` + `udp-client.sky` 60 s run, release build):

- Phases 1-2 (runtime struct + mote vtable around existing helpers): ≤5% wall.
- Phases 3-5 (platform/boot/radio-bus extraction): ≤10% wall combined.
- Phases 6-10 (services + plugins): ≤5% additional wall.

Total cumulative slowdown over the whole refactor: ≤20% on the Sky baseline.
If a phase exceeds its budget, profile and inline the hot path (typically:
`sim_radio_bus_deliver_byte` and `sim_dispatch_mote_execute`) before merging.

### 3.14.2 Error and panic policy

The kernel commits to:

- `mote->ops->execute()` returning non-zero: log + drop that mote's wakeup +
  continue. No automatic sim abort.
- Event queue empty AND no service `keepalive()` requesting more time: clean
  exit with `exit_code = 0`.
- Service callback returning failure during init: refuse to start sim. During
  run: log + disable that service, continue.
- Out-of-memory in kernel paths: log + `abort()`. The kernel does not attempt
  partial-state recovery.
- Mote API violations (motes calling `sim_runtime_set_now_ns` etc.): assert
  in debug builds, log + ignore in release.

Services that need different policies (e.g. JS test engine wanting
"first failed assertion stops sim") must request that explicitly through
`sim_runtime_request_stop()` rather than `_exit()`-ing inline.

### 3.15 Kernel extraction milestones (canonical Phase 1 task list)

> **Status: all 10 milestones landed (Phase 1 complete).** M9 landed in
> five slices (struct moves `91db750`, ops vtable `a36aa43`+`363e636`,
> TX path `c2eca11`, RX-stall sim-time timer `580086d`); M10 is
> `adbea53` (`sim_runtime_run_until` pump; runner dispatch seam is
> `mixed_dispatch_event`). Next: Phase 2 (mote vtable).

These 10 milestones are the canonical Phase 1 task list — §9 Phase 1 points
back here. Land them in order, one PR per milestone where practical:

1. Create `sim_runtime_t` with `now_ns`, `end_ns`, `run_state`, `event_queue`,
   and `radio_medium`.
2. Move `current_sim_ns` behind `sim_runtime_now_ns(sim)`.
3. Move event queue scheduling wrappers behind `sim_schedule_*` functions.
4. Add mote slot generation counters and use them for node removal/reboot.
5. Replace direct `sim_eq_*` calls in the runner with kernel wrappers.
6. Introduce observer event dispatch with no subscribers.
7. Convert timeline logging to the first observer subscriber.
8. Convert serial socket/JS test actions into scheduled kernel events.
9. Move radio byte scheduling to `sim_radio_bus`.
10. Collapse the runner loop into `sim_runtime_run_until`.

Only after these are done should plugin loading or config v2 become a priority.

### 3.16 Mote vtable milestones (canonical Phase 2 task list)

> **Status: PHASE 2 COMPLETE.** M11 `0bf3d12`, M12 `f88b214`,
> M13 `20a535e`, M14 `718faa6`, M15 `bc23459`, M16 `3444ab4`, M17
> (slim) in the closing commit.  M17 was descoped after survey: the
> receive_frame op landed (5 duplicated native/JS frame-RX sites →
> 1 op), but the deep frame_complete de-typing turned out to be
> chip-delivery policy entangled with CPU stepping, not vtable work —
> converting its `type == NODE_NATIVE` tests to mode queries while the
> bodies still poke plat.native gains nothing structural.  That work
> moves to the Phase 5 remnants (frame delivery restructuring), along
> with the threaded-mode `distribute_rf_outgoing` duplication.
> Remaining type switches in the runner: registration/boot/init
> (Phase 3/4), frame-delivery policy (Phase 5), end-of-run stats (by
> design).  Numbering continues from Phase 1. Stop condition
> (from §9 Phase 2): dispatch abstraction only — no platform-init/boot
> logic moves (that is Phase 4).

Design decisions locked for this phase:

- `include/sim/sim_mote.h` defines `sim_mote_ops_t` and
  `sim_mote_t { int id; const sim_mote_ops_t *ops; void *impl; }`.
- The four adapter implementations (MSP430 / ARM / native / JS) **stay in the
  runner file** for all of Phase 2 — they reference runner globals
  (`ticking_node_idx`, `gdb_stubs`, `emu_rx_queue_drain`, `native_had_tx`)
  by design. *(Amended by §3.17: Phase 4 moves the JS and native adapter
  sets plus all boot policy; the MSP430/ARM execute/serial adapters follow
  their dependencies — radio bus in Phase 5, GDB service in Phase 6.)*
- Motes register into a `sim_runtime_t` slot table
  (`sim_runtime_register_mote()`), called from `init_node()` next to
  `register_node_radio_ops()` so reboots and JS-ADD dynamic motes stay
  covered. The kernel does not *call* mote ops yet — kernel-side
  `MOTE_EXECUTE` dispatch is a non-goal here (Phase 10).
- `execute()` contract per §3.6: `int64_t execute(sim_mote_t *m, int64_t
  now_ns)` returns the next wakeup time (`INT64_MAX` = none); the *caller*
  does the single `sim_schedule_mote_wakeup_if_earlier()`.
- Adapter bodies are **character-identical moves** of existing code. No
  clock-semantics changes — this phase must be byte-for-byte behavior
  preserving in the Cooja suite.

Milestones (one commit each, full validation gate before each):

11. Scaffold + read-only accessor ops (`kind`, `sim_time_ns`, `cycles`,
    `freq_hz`, `instructions`); `node_*` accessors become ops dispatches.
12. Execution ops: `execute`, `step_until`, `next_wakeup_ns`.
    `dispatch_mote_wakeup` collapses to guards → snapshot → `ops->execute`
    → schedule returned wakeup → post-tick RF distribution (stays
    runner-side: cross-node policy). Highest-risk milestone.
13. Time-sync op `sync_to_time(mote, now_ns)`; `deliver_msp430_rx_byte` +
    `deliver_arm_rx_byte` merge into one type-blind `deliver_rx_byte`.
14. Serial-input op `serial_input(mote, buf, len)`; one `inject_serial()`
    helper replaces the serial-bridge callback and the four duplicated
    TEST_ACTION_SEND/SEND_ALL blocks. Unify on the append + immediate-wakeup
    semantics (Cooja-accurate); GENERATE_MSG tests are the detector.
15. Lifecycle ops: `destroy`, `reset_time` (dedupes the two post-reboot
    time-reseed blocks).
16. Introspection: `get_radio_state`, `get_leds` (type-blind
    `update_radio_state`/`update_led_state`); `get_interface(mote, iface)`
    for `SIM_IFACE_GDB_ARM` and `SIM_IFACE_CC2420` (trace_tsch_ack goes
    ops-clean). End-of-run JIT-stats reporting stays type-specific.
17. (Stretch, deferrable) RF-path de-typing: replace `type == NODE_NATIVE/JS`
    checks in frame delivery with radio-bus delivery-mode queries (SYNC ⇔
    native) plus a `radio_receive_frame` op for native + JS.

Validation gate per milestone: `make clean && make`; `correctness` /
`arm-correctness` / `cc1200-mock-host` / `radio-medium`; sky + firefly
2-node RPL-UDP 60 s; chain configs (6/7 baseline — firefly-subghz 4-node is
the pre-existing cc1200 TX-end failure); js-hello + js-rpl-udp; Cooja suite
81/81 (never rebuild while it runs). M12 additionally: TSCH + clock-drift
configs. M14 additionally: the GENERATE_MSG-heavy 14-rpl-lite tests.

### 3.17 Boot-policy extraction milestones (canonical Phase 4 task list)

> **Status: PHASE 4 COMPLETE (2026-06-12).** M18 `39d6d66`
> (scaffolding: mote_impl.h + env bundle + rf_ctx fold-in), M19
> `ce37365` (js_app_mote: boot + full ops; receive_frame returns
> queue-full status so call sites own the stats), M20 `84c8a99`
> (native_cooja_mote: boot + full ops + tick helpers;
> native_had_tx/exec_had_tx become node fields), M21 `a31b1ba`
> (msp430_elf_mote: boot + exported tick + radio ops; perf flat),
> M22 `6966a12` (arm_elf_mote: same shape, all four SoC branches),
> M23 in the closing commit (sim_mote_kind_t registry; init_node is
> data-driven; detect_node_type/node_type_for_board ladders gone).
> Runner shrank ~900 lines net.  Remaining runner-side per the locked
> descope: MSP430/ARM execute/serial adapter tables (runner-injected
> via sim_mote_kind_set_ops) — they follow emu_rx_queue_drain +
> ticking_node_idx to the Phase 5 bus and gdb_stubs to the Phase 6
> service.  Numbering continues from Phase 2 (M18–M23).
> Goal (from §9 Phase 4): firmware loading, run-to-main, node-id/
> linkaddr patching, and board-specific quirks move out of
> `test/test_mixed_multinode.c` into per-kind modules under
> `src/motes/`, so adding a board stops requiring runner edits.
> Stop condition: firmware-specific patches stay local to the board
> boot policy with a comment naming the firmware symbol they depend on.

Design decisions locked for this phase:

- **Scope: boot policy, radio-endpoint ops, pure tick helpers, and the
  full JS + native adapter sets.** The emulated (MSP430/ARM)
  execute/serial adapters stay in the runner and follow their
  dependencies — `emu_rx_queue_drain` (frame-delivery policy, Phase 5),
  `ticking_node_idx` (shared with runner RF handlers, Phase 5),
  `gdb_stubs` (GDB service, Phase 6). Moving them now would add ~4 env
  seams that Phases 5/6 delete again. This supersedes the §3.16 note
  that all adapters move in Phase 4 (same precedent as the M17 descope).
- **Shared private header `src/motes/mote_impl.h`** — *not*
  `include/sim/` — holds `node_type_t`, `rf_listener_ctx_t`,
  `mixed_node_t` (verbatim move plus new fields: `slot`, `env`,
  `rf_ctx[2]`; M20 adds `native_had_tx`, `exec_had_tx`), and
  `sim_mote_env_t`. It embeds all four platform structs, so it stays
  private to `src/motes/*` and the runner frontend. Names stay
  `mixed_node_t`/`node_type_t` (character-identical-move discipline;
  renames are Phase 10 cosmetics).
- **`sim_mote_env_t` is the runner-owned glue bundle injected into
  boot** — `src/motes` never links against runner symbols. Members:
  `sim_runtime_t *sim`, `sim_radio_bus_t *radio_bus`, `const int
  *verbose`, `const int *num_threads`, `const int64_t *node_start_ns`,
  plus fn ptrs `uart_byte`, `chip_tx_byte`, `radio_set_channel`,
  `rfcore_state_change`, `rfcore_channel_change`, `cc1200_channel_busy`,
  `rf_tx_byte`, `rf_frame`, `native_yield`, `js_rf_frame`,
  `native_channel_sync`. Everything in the env is documented Phase 5/6
  debt; members retire as those phases land.
- **Per-kind module API**: `int <kind>_mote_boot(mixed_node_t *n, int
  slot, const char *path, int node_id, const sim_mote_env_t *env)` plus
  a *separate* `void <kind>_mote_register_radio(mixed_node_t *n, int
  slot, sim_radio_bus_t *bus)`. Radio registration stays a separate
  post-boot call: `js_node_start()` can TX during script `init()`,
  which today happens *before* radio-ops registration — merging the two
  would change behavior.
- `tick_one_msp430` exports as `msp430_elf_mote_tick()` — it has a
  second runner caller (the frame-delivery pre-sync), which keeps
  calling the exported symbol until Phase 5 moves that path into the
  bus. `tick_one_arm` exports likewise as `arm_elf_mote_tick()`.
- `native_yield_callback` **stays in the runner** (cross-node RF
  policy: walks `nodes[]`, calls the deliver path) and is injected via
  the env. Most tempting wrong move in this phase.
- `nodes[].type` survives Phase 4 — frame-delivery policy (Phase 5) and
  end-of-run stats still key on it by design.

Milestones (one commit each, full validation gate before each):

18. Scaffolding: `src/motes/` Makefile rules + `-Isrc/motes`;
    `mote_impl.h` with `mixed_node_t` + `sim_mote_env_t`; runner
    `mixed_mote_env` instance; the `rf_ctx_slot0/1[]` listener-context
    arrays fold into `node->rf_ctx[2]`.
19. `js_app_mote.c`: boot + full ops table + BATCH radio registration
    (smallest kind — proves the env pattern). Prep in the same commit:
    `stat_rx_frames_queued`/`queue_full` hoist from the JS/native
    `receive_frame` adapters to their call sites (behavior-identical).
20. `native_cooja_mote.c`: boot + full ops + `tick_one_native` /
    `native_next_wakeup_after_tick` / `native_has_pending_work`.
    `native_had_tx[]`/`native_exec_had_tx` become node fields; the
    TSCH channel push goes through `env->native_channel_sync`.
21. `msp430_elf_mote.c`: boot policy verbatim (ds2411_init/xmem RET
    patches, ds2411_id/infomem-0x1980/node_id/Z1-linkaddr patches,
    run-to-main, run-to-ready), exported tick, `msp_radio_ops` +
    PER_BYTE registration. Ops table + execute/serial adapters stay
    runner-side. Record the §3.14.1 perf baseline here (the tick is
    the hottest moved function).
22. `arm_elf_mote.c`: boot policy verbatim (cc2538/firefly/nrf52840/
    nrf54l15 wiring branches, FICR/RFRND seeding, run-to-main,
    step-past-first-event), exported tick, `arm_radio_ops` /
    `arm54l_radio_ops` + the PER_BYTE-vs-BATCH delivery decision.
    Ops table stays runner-side.
23. Kind registry `mote_kinds.c`: `sim_mote_kind_t { name, node_type,
    boot, register_radio, ops }`, `sim_mote_kind_for(board_kind)`,
    `sim_mote_kind_set_ops()` (runner injects the MSP430/ARM ops
    tables at startup until Phase 5/6 moves them). `init_node`
    dispatch becomes data-driven (kind lookup → boot → register_radio,
    ordering preserved); `detect_node_type`/`node_type_for_board`
    ladders deleted; `sim_board.h`/`sim_mote.h` header notes updated.

Validation gate per milestone: same as §3.16 (make clean && make;
`correctness` / `arm-correctness` / `cc1200-mock-host` /
`radio-medium`; sky + firefly 2-node RPL-UDP 60 s; chain configs (6/7
baseline); js-hello + js-rpl-udp; Cooja suite 81/81 — never rebuild
while it runs). Per-milestone fail-fast detectors: M19 JS tests +
dynamic-ADD; M20 Cooja suite + GENERATE_MSG-heavy 14-rpl-lite
(serial-input moved); M21 TSCH + clock-drift configs; M22 firefly
dual-radio + nrf52840-dk/nrf54l15 multinode + GDB smoke; M23 reboot +
JS-ADD paths + a mixed-platform config (all four kinds in one sim).

### 3.18 Radio-bus extraction milestones (canonical Phase 5 task list)

> **Status: complete (M24–M30).** M24 `355e95e` (guardrails: radio-bus unit
> suite, 83 assertions + tools/check-determinism.sh + perf baseline —
> sky 2-node 60 s ≈ 134 ms, firefly-subghz-fixed ≈ 8.4 s); M25
> `074430e` (de-typing: rx_byte_sync/rx_pre_sync ops + SIM_RADIO_CAP_*
> register caps; emu_deliver_bytes/frame_complete/drain type checks →
> mode/caps/ops-presence queries); M26 `9cfd8f4` (emu RX core into the
> bus: deliver_bytes/queue_frame/drain_rx + executing_node + on_rx hook
> + bus->stats; deliver_rx_byte + schedule_emulated_wakeup stay runner-
> side, not M27 prerequisites; cross-build diff IDENTICAL; radio-bus
> suite → 104); M27 `de1eedc` (frame-complete delivery policy into the
> bus — the 432-line move; node_tx_busy_until_ns + suppress_state +
> schedule_emulated_wakeup also moved; fat frame_complete hook deleted,
> replaced by frame_observed/on_rx_frame/on_ack; dual 192 µs ACK
> windows digit-for-digit; cross-build diff IDENTICAL on sky/firefly/
> cc2538 + 4 chain configs) + `f69c3e6` (M27 follow-up: frame_complete
> policy unit tests — collision/backpressure/ACK-window with a scripted
> auto-ACK mock; radio-bus suite → 121).  M28 split in two: part 1
> `b74f0e8` (channel consolidation — sim_radio_bus_push_channel +
> current_channel pull op; sync_channel host hook deleted) + part 2
> `b0d168f` (native/JS frame path — mixed_rf_frame_handler →
> sim_radio_bus_tx_frame; native direct-buffer fast path into the
> receive_frame op; mark_collisions op; native stats → bus->stats;
> Cooja 81/81 is the native gate).  Two documented unified-path
> changes: NONE-medium native delivery now uses the direct-if-empty
> fast path; full-queue native frame_queue_full now counted on UDGM
> too (stat-only).  M29 `c7cbbfa` (retire `--threads`: deleted the
> threaded callbacks/distribute/flush path, the fixed-1 ms loop arm,
> `--threads` parsing + all `num_threads` guards, the `advance_to_time`
> op + its 4 impls, `rf_outgoing`/`defer_wakeups` from the bus,
> `env->num_threads`, and the orphaned `sim_threads.{c,h}` thread pool;
> the kernel's sequential event pump is now the only scheduler — the
> guard collapse is behavior-preserving since `num_threads == 0` was the
> default path, proven by IDENTICAL cross-build diffs on
> sky/cc2538/firefly-subghz 2-node + 5 chain configs + JS broadcast,
> Cooja 81/81).  M30 (this commit — docs close-out: §6.3 RF-timer audit
> note, §10 determinism wording switched to the stdout-diff script with
> the phantom `--timeline-out` dropped, Decisions Log finalized,
> CLAUDE.md).  **Phase 5 complete; the RF-delivery policy now lives in
> `src/sim/sim_radio_bus.c`.  The MSP430/ARM execute/serial adapters
> stay runner-side until Phase 6 (GDB service).**
> Numbering continues from Phase 4 (M24–M30).
> Goal (from §9 Phase 5): the remaining RF delivery *policy* moves out
> of `test/test_mixed_multinode.c` into `src/sim/sim_radio_bus.c`.
> The M9.4/M9.5 slices already moved the TX byte path (byte clock,
> frame assembler, capture, medium-filtered dispatch, delivery modes,
> RX-stall sim-time timer); what remains runner-side is the policy
> behind the `sim_radio_bus_host_t` hooks — above all the ~413-line
> `frame_complete` body (air-time windows, snapshots, collision
> windows, RXFIFO backpressure, MSP430 pre-sync, sync-vs-queue
> delivery, dual 192 µs auto-ACK windows, interference marking) plus
> `emu_deliver_bytes`, the emu RX queue, the native/JS frame path,
> and channel/CCA plumbing.
> Stop condition (from §9 Phase 5, non-negotiable): never simplify
> byte timing into frame-only delivery; the dual ACK-window
> arithmetic — receivers `accurate_tx_end + 192000`, sender
> `now + 192000`, sender's own ACK always queued, others
> sync-if-space — is copied digit-for-digit.

Design decisions locked for this phase:

- **`--threads N` is retired (done, M29)** (resolves §3.13's
  "port or retire by Phase 6": zero usage signals — no config, CI
  script, tool, or documented workflow exercised it).  A future batch
  scheduler builds on the bus APIs behind `sim_scheduler_ops_t`.
  The path stayed *working* through M26–M28 (its
  `distribute_rf_outgoing` called the new bus APIs); M29 then deleted
  it, the orphaned `sim_threads.{c,h}` pool, and the `advance_to_time`
  op it was the last caller of.
- **Ops placement rule**: anything that advances a mote's CPU/local
  clock is a `sim_mote_ops_t` member; anything that talks to the
  chip's radio endpoint is a `mote_radio_ops_t` member.  New optional
  `sim_mote_ops_t` members (M25): `rx_byte_sync(m, byte_time_ns)`
  (per-byte pre-delivery clock sync; MSP430 = existing sync_to_time
  path, ARM = the verbatim ex-inline body — unclamped, sim_time
  pinned before AND after the step; semantics differ, do not unify)
  and `rx_pre_sync(m, time_ns)` (MSP430-only full execute-slice
  pre-sync, ex the frame_complete `msp430_elf_mote_tick` call).
  New optional `mote_radio_ops_t` members (M28):
  `current_channel(mote)` (native pull-model channel — replaces the
  `sync_channel` host hook) and `mark_collisions(mote, start, end)`
  (native rx-queue collision marking; the bus marks its own emu
  queues directly).
- **Registration caps**: `sim_radio_bus_register` gains a caps
  bitmask encoding platform delivery quirks currently expressed as
  `nodes[].type` checks (e.g. `SIM_RADIO_CAP_WAKE_SENDER_POST_TX`,
  `SIM_RADIO_CAP_DRAIN_MINI_STEP` — MSP430-only behaviors).  Each
  src/motes module sets its own caps.
- **Host hooks v2**: `node_active`, `on_tx_byte`, `on_byte_accepted`
  stay; `sync_channel` dies in M28; the fat `frame_complete` policy
  hook is replaced by slim *notification* hooks the runner consumes
  for observability until Phase 6 turns them into services:
  `frame_observed(frame_info)` (PCAP / packet analyzer / TX timeline
  / UI / traces), `on_rx(receiver, outcome, data, len, start, end,
  subghz)`, `on_ack(acker, start, dur)`.
- **State ownership after the move**: `node_tx_busy_until_ns` → bus,
  queried via `sim_radio_bus_channel_busy()` (the CC1200 CCA env
  callback becomes a delegation); `ticking_node_idx` → bus-owned
  `executing_node` + setter; counters incremented by moved code →
  `bus->stats` (end-of-run print reads them);
  `suppress_state_callback` → `sim_radio_bus_in_delivery()`.
- **`native_yield_callback` stays runner-side** (cross-node ACK
  turnaround policy that ticks receiver motes; revisit when the bus
  can express "tick receiver to generate ACK", likely never — it is
  ContikiMote semantics, not bus routing).
- **MSP430/ARM execute/serial adapters move in Phase 6** with the GDB
  service (re-affirms the §3.17 descope; after Phase 5 their only
  remaining runner dependency is the GDB stubs).
- **Fail-fast: cross-build empty-diff.** M25–M27 claim byte-identical
  behavior; each runs sky 2-node, firefly-subghz 2-node, and cc2538
  configs on the previous milestone's binary and the new one, and
  diffs stdout+stderr (minus wall-clock/rate lines).  Non-empty diff
  = moved-code bug → revert per §11.5.  Phase 4's byte-identical
  chain history across six commits shows the bar is realistic.

Milestones (one commit each, full validation gate before each):

24. Guardrails (no production-code changes): new `radio-bus` unit
    suite (`test/test_radio_bus.c`, mock `mote_radio_ops_t` receivers
    + scripted chip TX byte streams through `sim_radio_bus_tx_byte`,
    pumped via `sim_runtime_run_until`): 802.15.4 + 802.15.4g frame
    completion, capture + `first_byte_ns` arming + 32/160 µs byte-
    clock spacing of PER_BYTE events, delivery-mode routing,
    re-entrant depth staging, RX-stall arm/extend/stale-rearm/expiry,
    `pick_receiver_radio`, `frame_fifo_bytes`.  New
    `tools/check-determinism.sh` (run config twice, diff output)
    joins the per-milestone gate.  Record the §3.14.1 perf baseline
    (sky + firefly-subghz-fixed 2-node 60 s).  If a unit test exposes
    a bug: document, don't fix here.
25. De-typing prep (runner-side, behavior-preserving): add the
    `rx_byte_sync`/`rx_pre_sync` ops + registration caps; the RF
    path's `nodes[].type` checks become mode/caps/ops-presence
    queries; `emu_deliver_bytes`' ARM branch goes through the op
    verbatim.  Cross-build diff must be empty.  Detectors: TSCH +
    clock-drift, firefly-subghz chain, nrf52840-dk (BATCH), nrf54l15
    (rx_stall).
26. Emulated RX core into the bus (character-identical):
    `emu_deliver_bytes` → `sim_radio_bus_deliver_bytes`,
    `emu_rx_queue_push/drain` → bus APIs, `deliver_rx_byte` →
    `sim_radio_bus_deliver_rx_byte`, `schedule_emulated_wakeup` →
    `sim_radio_bus_wake_mote`; `ticking_node_idx` →
    `bus->executing_node` + `sim_radio_bus_set_executing`; RX
    timeline/prints via the `on_rx` hook; emu counters →
    `bus->stats`.  Bus code compiles with no chip/platform headers —
    a moved line needing `plat.msp/arm` was mis-scoped.  Perf ≤3%
    cumulative.
27. Frame-complete policy into the bus — **highest-risk milestone**:
    air-time math, sub-GHz `first_byte_ns` fixups,
    `node_tx_busy_until_ns` + `sim_radio_bus_channel_busy`,
    snapshots, collision windows, RXFIFO backpressure mini-step via
    ops, `rx_pre_sync`, sync-vs-queue, the dual ACK windows
    digit-for-digit, interference marking, sender post-TX wake via
    caps.  Fat hook deleted; runner implements `frame_observed` +
    `on_ack`.  Unit suite gains collision/backpressure/ACK-window
    arithmetic tests with a scripted auto-ACK mock.  If any timing
    constant (192000, 5000 cycles, 1 ms) "needs adjusting" to go
    green — that is a moved-code bug, STOP.  Perf ≤8% cumulative.
    Detectors: pcap byte-diff between builds, nrf52840 4-node
    convergence, firefly-subghz 4-node (stays at the known red, must
    not worsen), TSCH-ACK traces.
28. Native/JS frame path + channel consolidation:
    `mixed_rf_frame_handler`'s delivery loop → `sim_radio_bus_tx_frame`
    (native direct-`simInDataBuffer` fast path + `simLastPacketTimestamp`
    becomes part of `native_cooja_mote`'s `receive_frame` op; NONE-medium
    branch; interference unified via `mark_collisions`);
    `mixed_node_radio_set_channel` → `sim_radio_bus_push_channel`;
    `sync_native_node_channel`/`native_channel_sync`/host `sync_channel`
    collapse onto the `current_channel` op.  If op-ifying the native
    fast path changes any Cooja result, fall back to a bus-side
    fast-path op and note it.  Perf ≤10% phase total (channel pull
    enters the per-byte loop — measure).
29. Retire `--threads`: delete the threaded callbacks,
    `distribute_rf_outgoing`, `flush_pending_output`, `thread_state`,
    `frame_outgoing`, the fixed-1 ms loop arm, `--threads` parsing +
    `num_threads` guards (mechanical removal only), `rf_outgoing`
    from `sim_radio_bus_t`, `env->num_threads`, the now-unused
    `advance_to_time` op + its four impls, and the runner's
    `sim_thread_pool` usage.
30. Close-out (docs only): §6.3 RF-timer audit note (cc2538/nrf52840
    ACK turnarounds verified ns-based; nrf54l15 done in M9.5), §3.18
    status, §10 determinism wording (stdout diff script; drop the
    phantom `--timeline-out`), Decisions Log finalized, CLAUDE.md.

Validation gate per milestone: §3.17's gate plus the new `radio-bus`
suite and the determinism run-twice diff; M25–M27 add the cross-build
empty-diff.  Perf checkpoints at M24 (baseline), M26 (≤3%), M27
(≤8%), M28 (≤10% phase total) per §3.14.1; if exceeded, cache ops
pointers in the delivery loops before merging.

### 3.19 Service-extraction milestones (canonical Phase 6 task list)

> **Status: complete (M31–M40).** Phase 6 (§9) extracts the
> runner's embedded optional/observation features into
> `src/services/*_service.c` behind a `sim_service_ops_t` vtable host, then
> — once the GDB stub is a service — finally moves the MSP430/ARM
> execute/serial adapter tables out of the runner (the §3.17/§3.18 deferred
> debt).  Numbering continues from Phase 5 (M31–M40).  Two services were
> already extracted as the template (`sim_serial_bridge.c`,
> `sim_external_command.c`); the observer stream (`sim_observer.h`) and the
> bus host hooks (`sim_radio_bus_host_t`) already exist.
> Landed: M31 `101cb9d` (service host scaffolding — sim_service.{h,c} vtable
> + fan-out observer + ordered poll/teardown + error policy; serial bridge
> and external command adopted as the first clients); M32 `783bf85`
> (timeline service — the on_event consumer + owned timeline_t, node-id via
> a runner resolver; node_states[] stays runner-side for the UI/M39); M33
> `688f5fb` (PCAP service — pcap_writer_t + --pcap CLI + open/write/close;
> the analyzer is NOT a service — stateless decoder, chip-coupled verbose,
> moves with the UI emit at M39); M34 `7fb1c04` (progress-report service —
> cadence + per-node summary via a describe callback, explicit
> position-pinned tick); M35 `3f448a2` (JSON-test service — step/validator/
> fail_on checker on_event + per-step timeout + "--- Test Results ---"
> report; the timed-action executor stays runner-side as shared node
> scripting); M36 `f756397` (JS-test service — the JS line feed onto the
> fan-out as on_event, deleting the runner's test_engine_observer; engine
> lifecycle/drain/results stay runner-side; re-entrancy depth assert added
> to the fan-out); M37 `a5e31c8` (GDB service — gdb_stub storage + --gdb CLI
> + bind/attach into the service; arm_mote_execute polls cpu->gdb_stub, not
> the runner globals, which unblocks M38; full RSP session validated
> end-to-end); M38 `d3b94af` (the §3.17/§3.18 debt paid — the MSP430/ARM
> execute/serial adapter tables + 30 functions move into
> src/motes/{msp430,arm}_elf_mote.c as msp430_elf_mote_ops/arm_elf_mote_ops,
> runner-globals rewired through node->env, sim_mote_kind_set_ops deleted;
> runner −493 lines).  All byte-identical (cross-build empty-diff on
> sky/cc2538/firefly-subghz 2-node + 6 chains incl. z1 MSP430X + JS
> broadcast + step-based configs + test-js-hello/test-js-rpl-udp + a --gdb
> bind smoke; M33 adds a pcap byte-diff; M38 re-verifies the full GDB RSP
> session); M39 `3be54dd` (WebSocket-UI service — ws_server + console +
> flags + message handler + serialization into the service; the runner
> keeps loop control + pacing + node_states; NOT byte-identical on the UI
> path (wall-clock pacing), but the headless path stays IDENTICAL and the UI
> path is validated by a WS client receiving the full-state JSON + a CBOR
> delta + handling a speed command); M40 (close-out — the end-of-run stats
> stay runner-side as documented type-specific diagnostics; they read chip
> memory + firmware symbols and the cycle-totals loop is fused with
> destroy_node, so service-ifying them would violate the no-chip-internals
> rule for no gain).  **Phase 6 COMPLETE.**  The runner is now a frontend
> over the kernel + the service host: CLI/config parsing, node lifecycle,
> the loop control, the node-scripting timed actions, and the type-specific
> terminal diagnostics.  The remaining MSP430/ARM execute/serial adapters
> moved out in M38; the only emulated-CPU code left in the runner is the
> firmware-symbol stats dump.

Design decisions locked for this phase:

- **The service host is a thin `sim_service_ops_t` vtable host built now**
  (M31), not the ad-hoc per-service pattern and not the full Phase-8
  registry.  `sim_service_ops_t {name, init, destroy, on_event, poll}`; a
  `{ops,state,enabled}` array lives in `sim_runtime_t` next to
  `observers[]`; the host subscribes **one** fan-out observer that walks
  enabled services → `on_event` (services don't each consume an observer
  slot) and a `sim_service_poll_all()` that walks them → `poll`.
  Attach order = fan-out order (cheap/pure-observer first, control-coupled
  last); **teardown is strict reverse**.  Error policy (per §Decisions
  Log) enforced in the host: `init` non-zero → runner refuses to start; a
  service reporting a runtime failure is marked `enabled=false` and
  skipped (run continues); `destroy` runs for all attached services.
  Phase 8 adds register-by-name (`"builtin:pcap"`) on top.
- **PCAP and the packet analyzer stay on the bus host hooks**
  (`frame_observed`/`on_rx_frame`), not the observer stream:
  `sim_radio_frame_info_t` already carries `capture`/`capture_len` (raw
  on-air bytes) + `tx_start_ns`, whereas `SIM_OBS_PACKET_FRAME` carries
  only `is_tx` + `summary`.  Enriching the observer payload with raw
  bytes is Phase 8+ debt; in Phase 6 these services register a
  `sim_radio_bus_host_t` and still *emit* `SIM_OBS_PACKET_FRAME` for the
  timeline to consume the summary.
- **The GDB stub decouples via `cpu->gdb_stub`.**  The ARM cpu already
  holds the back pointer; the inline poll in `arm_mote_execute` becomes a
  `cpu->gdb_stub` check inside the ARM module, erasing the runner globals
  `gdb_node[]`/`gdb_stubs[]` and unblocking the adapter move.  The GDB
  service (M37) lands **before** the adapter move (M38).
- **The MSP430/ARM execute/serial adapters move in M38** (re-affirms the
  §3.17/§3.18 deferral — after the GDB service their only runner
  dependency is gone); `sim_mote_kind_set_ops` injection is deleted,
  finishing the M23 TODO.
- **End-of-run stats (M40) may stay type-specific** (reads chip internals
  + firmware symbols); per the M16/M23 precedent it can keep a
  runner-provided accessor or defer to Phase 10 rather than go
  observer-clean.  If un-clean, Phase 6 closes at M39.

Milestones (one commit each, full validation gate before each):

31. Service host scaffolding: `sim_service.{h,c}` (the vtable + host),
    service array + `sim_service_attach/poll_all/dispatch_event/keepalive`
    in `sim_runtime`; wrap `serial_bridge` + `external_command` as the
    first two host clients (validates the host against working code).
32. Timeline service (`src/services/timeline_service.c`): move
    `timeline_observer_cb` + `emit_*_obs` helpers + `tl_flush`/CBOR;
    the timeline owns `node_states[]`, the UI reads via an accessor.
33. PCAP service: move `pcap_writer_t` + the `--pcap` CLI + open/write/
    close lifecycle into `src/services/pcap_service.c`; the runner's single
    radio-bus `frame_observed` hook feeds it the MAC bytes via
    `pcap_service_write()` (the bus has one host, not per-service hosts).
    The open/close prints stay at their original call sites for
    byte-identity.  **The packet analyzer is NOT extracted here**: it is a
    stateless decoder (`pkt_analyze`) whose verbose output is interleaved
    with the UI frame-summary emit and reaches MSP430 firmware symbols
    (`[UIP]`) — no clean service home — so it moves with the UI
    frame-summary path in M39.
34. Progress-printing service: per-tick progress block into a `poll()`.
35. JSON-test service: the step/validator/fail_on checker (`on_event` on
    SIM_OBS_MOTE_LOG_LINE) + per-run state + per-step timeout + the
    end-of-run "--- Test Results ---" report; the loop reads `finished`
    via a query.  The timed-action executor (`config.test.actions[]` →
    MOVE/SEND/SEND_ALL/REMOVE/ADD) **stays runner-side**: it is config-driven
    node scripting (mutates nodes[]/radio_medium, dynamic add), structurally
    shared with the JS engine's action path (M36) and not test-engine state.
36. JS-test engine service: move the JS console-line feed onto the host
    fan-out as `on_event` and delete the runner's dedicated
    test_engine_observer (both engines consume lines as services now).  The
    re-entrancy contract was already satisfied by construction —
    js_test_feed_line only matches/queues, the action drain+execute runs in
    the loop (the deferred-resume point), not the callback — so the engine
    lifecycle/drain/results stay runner-side (shared node scripting +
    config-coupled setup), like M35's timed-action split.  A re-entrancy
    depth assert was added to `sim_service_dispatch_event`.  Came out
    byte-identical (cross-build empty-diff incl. test-js-hello /
    test-js-rpl-udp).
37. GDB service: stub storage + `--gdb` CLI + attach into the service
    (sets `cpu->gdb_stub`); ARM module polls `cpu->gdb_stub`; service
    `poll()` keepalive while any stub is attached.
38. Move the MSP430/ARM execute/serial adapter tables into
    `src/motes/{msp430,arm}_elf_mote.c`; delete `sim_mote_kind_set_ops`.
39. WebSocket-UI service: sockets/serialization/console/pacing/message
    handling into the service; the loop keeps 4 query funcs
    (`paused`/`restart_requested`/`ui_cap_ns`/`pace`).
40. End-of-run stats: **resolved to stay runner-side** (the sanctioned
    fallback).  The Performance / Phase-Timing / detailed-stats block reads
    `plat.msp.cpu.memory` + firmware symbols (TSCH state, neighbor/SR
    tables, CC2420 stats) directly, and its cycle/instruction totals loop is
    fused with `destroy_node()` — service-ifying it would violate the
    no-chip-internals rule for no real gain.  Per the M16/M23 precedent it is
    documented type-specific terminal diagnostics; revisit in Phase 10 if the
    runner is reduced to a pure frontend.  **Phase 6 closes at M39 + this
    note.**

Validation gate per milestone: §3.18's gate (make clean && make;
correctness / arm-correctness / cc1200-mock-host / radio-medium /
radio-bus; sky + firefly-subghz-fixed 2-node RPL-UDP 60 s; chains 6/7;
js-hello + js-rpl-udp; Cooja 81/81; determinism run-twice) plus the
Phase-6 additions where relevant (`timeline`, `test
configs/test-js-hello.json`, `mixed-multinode configs/ui-demo-cc2538dk.json
-t 5000`, a `--gdb` smoke).  M31–M35/M38/M40 are byte-identical →
cross-build empty-diff (M33 adds a pcap byte-diff); M36 (deferred resume)
gates on js output equality + determinism; M39 (wall-clock pacing) gates
on the ui-demo smoke, not a diff.  Perf ≤5% additional wall for Phases
6–10 combined (§3.14.1).

### 3.20 Config-v2 milestones (canonical Phase 7 task list)

> **Status: in progress.** Phase 7 (§9 / §8.3) adds a **config v2** format
> alongside the legacy v1 JSON so configs can name mote types / platforms /
> plugins (which Phase 8's static registry resolves by name).  Numbering
> continues from Phase 6 (M41–M43).
> Key finding: `sim_config_t` is **already** the normalized config struct
> §8.3 calls for — the runner consumes the parsed struct (40 field reads),
> not the JSON (the cJSON tree is freed in `sim_config_load` before it
> returns).  So §8.3 step 5 ("runtime consumes normalized config") is
> already satisfied; **Phase 7 does not touch the runtime**.  The work is a
> versioned parser dispatch + a v2 parser that populates the *same* struct
> + struct extensions that *capture* v2 metadata (named mote-types,
> plugins) for Phase 8 to consume.
> Stop condition (§9 Phase 7, non-negotiable): do not break existing
> configs — all v1 JSON configs + the CLI multinode mode (firmware args +
> `-n N`, extension→board resolution) keep working byte-identically;
> `tools/csc2json.py` keeps emitting v1 (§8.3 step 6).

Design decisions locked for this phase:

- **`sim_config_t` is renamed `sim_normalized_config_t`** (fidelity to the
  §8.3 commitment) — cheap because the literal type name appears in only 3
  files (header typedef + 2 signatures, the parser, one runner line); the
  40 `config.X` reads are member accesses, unaffected.  Sub-structs keep
  their names (`sim_test_config_t`, `sim_node_config_t`, … — renaming them
  would touch json_test_service / js_test_engine for nothing).
- **The config module relocates** `include/native/`+`src/native/` →
  `include/sim/`+`src/sim/` (config is general, not native-mote-specific);
  a zero-risk `git mv` + Makefile move (both `-I` paths already in CFLAGS).
- **The legacy `mote_type_firmware[8][256]` index table is retained** — it
  is read by integer index at runtime (`TEST_ACTION_ADD`); the v2 parser
  writes both the rich `mote_types[]` table and the legacy index array, so
  the runtime is unchanged.  Migrating the runtime to the rich table is
  Phase 8 debt.
- **v2 resolves through the firmware extension, as today** — v2's
  `mote_type.firmware` carries the `.sky`/`.cc2538dk` extension, so the v2
  node parser writes the resolved firmware into `nodes[i].firmware` and the
  existing `sim_board_for_path` resolution is unchanged.  The captured
  `cpu`/`soc`/`board`/`plugins` are inert in Phase 7 (Phase 8 consumes them).

Milestones (one commit each, full validation gate before each):

41. Versioned-dispatch refactor + relocate + rename (byte-identical): split
    `sim_config_load` into file-read + top-level `version` detection →
    `parse_v1` (verbatim move of the current body) + shared `parse_test` /
    `parse_medium_object` leaf helpers; `git mv` the config module to the
    sim namespace; rename the struct; add a `version` field (=1, unused).
42. Config v2: add `sim_mote_type_t` (name/kind/cpu/soc/board/firmware) +
    `mote_types[]` alongside the retained index table + `plugins[]` +
    node `type_name`; `parse_v2` (reuses the shared helpers; resolves
    `node.type` → named mote-type → firmware, writing the legacy index too);
    version-guarded print; v2 example configs (twins of existing v1) + a
    v1↔v2 equivalence diff (byte-identical output).
43. Close-out (docs only): §3.20 status, §8.3 amendment (normalized struct =
    `sim_normalized_config_t`, step 5 already satisfied), legacy index table
    noted as Phase-8 debt, `csc2json.py` stays v1, Decisions Log, CLAUDE.md.

Validation gate per milestone: §3.19's gate plus the two Phase-7 config
gates (`test configs/test-rpl-udp-sky.json`, `test
configs/rpl-udp-cc2538dk.json`).  M41 is byte-identical → cross-build
empty-diff (the v1-extraction backstop); M42 adds the v2↔v1 equivalence
diff (each v2 twin produces byte-identical sim output to its v1 original)
plus a v1-only cross-build empty-diff.

## 4. Core API Sketches

These are design sketches for the contracts the refactor should converge on.
They are not mandatory exact code.

### 4.1 Simulation runtime

```c
typedef struct sim_runtime sim_runtime_t;
typedef struct sim_mote sim_mote_t;
typedef struct sim_registry sim_registry_t;

typedef struct sim_runtime_config {
    int64_t end_time_ns;
    uint32_t seed;
    int max_nodes;
    bool deterministic;
} sim_runtime_config_t;

int  sim_runtime_init(sim_runtime_t *sim, const sim_runtime_config_t *cfg);
void sim_runtime_destroy(sim_runtime_t *sim);

int  sim_runtime_add_mote(sim_runtime_t *sim, sim_mote_t *mote);
int  sim_runtime_remove_mote(sim_runtime_t *sim, int mote_index);
int  sim_runtime_run_until(sim_runtime_t *sim, int64_t end_ns);
int  sim_runtime_step(sim_runtime_t *sim);

int64_t sim_runtime_now_ns(const sim_runtime_t *sim);
sim_registry_t *sim_runtime_registry(sim_runtime_t *sim);
```

### 4.2 Global simulation event

Do not confuse this with `cpu_event_t`. `cpu_event_t` is per-CPU/per-chip. The
simulation event is global and should eventually replace the ad hoc combination
of `sim_event_queue_t`, MSP byte heap, serial action timing, and service timers.

```c
typedef enum sim_event_type {
    SIM_EVENT_MOTE_EXECUTE = 1,
    SIM_EVENT_RADIO_BYTE,
    SIM_EVENT_RADIO_FRAME_END,
    SIM_EVENT_SERIAL_INPUT,
    SIM_EVENT_SERVICE_TIMER,
    SIM_EVENT_TEST_ACTION,
} sim_event_type_t;

typedef struct sim_event {
    int64_t time_ns;
    uint64_t seq;
    sim_event_type_t type;
    int mote_index;
    void (*callback)(sim_runtime_t *sim, struct sim_event *ev, void *data);
    void *data;
} sim_event_t;
```

The global queue must preserve Cooja's same-time FIFO semantics.

### 4.3 Mote ops

```c
typedef struct sim_mote_ops {
    const char *kind;

    int  (*init)(sim_mote_t *mote, sim_runtime_t *sim);
    void (*destroy)(sim_mote_t *mote);
    int  (*reset)(sim_mote_t *mote, int64_t at_ns);

    int  (*execute)(sim_mote_t *mote, int64_t time_ns);
    int64_t (*next_wakeup)(sim_mote_t *mote);

    int  (*serial_input)(sim_mote_t *mote, const uint8_t *data, int len,
                         int64_t time_ns);
    int  (*radio_receive_byte)(sim_mote_t *mote, int radio_idx, uint8_t byte,
                               int8_t rssi, int64_t time_ns);
    int  (*radio_receive_frame)(sim_mote_t *mote, int radio_idx,
                                const uint8_t *frame, int len,
                                int8_t rssi, int64_t time_ns);

    int  (*get_info)(sim_mote_t *mote, sim_mote_info_t *out);
    int  (*get_interface)(sim_mote_t *mote, sim_iface_id_t iface, void **out);
} sim_mote_ops_t;
```

The runner should call these methods instead of switching on `NODE_MSP430`,
`NODE_ARM`, `NODE_NATIVE`, and `NODE_JS`.

### 4.4 Registry

```c
typedef struct sim_registry_ops {
    int (*register_mote_type)(sim_registry_t *, const sim_mote_type_t *);
    int (*register_platform)(sim_registry_t *, const sim_platform_desc_t *);
    int (*register_radio_medium)(sim_registry_t *, const sim_medium_type_t *);
    int (*register_service)(sim_registry_t *, const sim_service_type_t *);
} sim_registry_ops_t;
```

Start with a static registry populated by built-in C functions. Add `dlopen`
plugins only after the static API is stable.

## 5. Smart CPU / Platform Model

**Why this section exists.** MSP430 and ARM today carry parallel but
non-unified platform models. MSP430 has `msp430_platform_t` / `msp430_config_t`
flat-bound to MCU variants; ARM has `arm_platform_t` / `arm_soc_ops_t` /
`arm_platform_config_t` already split into SoC-ops + board-config. Adding a new
board today requires touching the runner *and* the per-arch platform table —
two unrelated places. The goal of §5 is to converge both architectures on the
ARM-style split (SoC descriptor + board descriptor + mote-type) so each new
board is *data registered into the registry*, not new platform-init code.

**Multi-radio per node.** Each mote owns 0..N radio endpoints, identified by
`(mote_index, radio_idx)`. Single-radio motes use radio_idx=0; Firefly today is
the only 2-radio platform (slot 0 = CC2538, slot 1 = CC1200), and future dual-
band ports follow the same pattern. The mote ops, radio bus, and observer
events all key on `(mote_index, radio_idx)`; this is an invariant, not a
per-platform choice.

### 5.1 CPU architecture

The CPU architecture layer owns instruction execution and register/memory
access. It should not know board names.

Examples:

- `msp430_cpu_t`: current MSP430/MSP430X CPU state and interpreter/JIT.
- `arm_cpu_t`: current ARM M-profile interpreter.

Target metadata:

```c
typedef struct sim_cpu_arch {
    const char *name;       /* "msp430", "msp430x", "armv7m", "armv8m" */
    const char *endian;     /* "little" for all current targets */
    uint32_t default_freq_hz;
    const sim_cpu_ops_t *ops;
} sim_cpu_arch_t;
```

Do not add new board-specific code to CPU files unless the instruction set or
core exception model truly requires it.

**JIT placement.** The MSPSim JIT (`src/msp430/msp430_jit.c`) is behavior-
affecting performance state with a global compiled cache and per-block
execution counters. It lives at the CPU-arch layer — owned by the MSP430 CPU,
shared across all MSP430 mote instances on the same SoC family. Threshold/
debug tuning stays on `MSPSIM_JIT_*` env vars; nothing in the kernel, mote,
or platform layer should know the JIT exists.

### 5.2 MCU / SoC

The MCU/SoC layer owns memory map, built-in peripherals, interrupt routing, and
clock tree.

Current examples:

- MSP430F1611/F149/F2617/F5437/CC430F5137/MSP430FR5969 via
  `msp430_config_t` and `msp430_platform_t`.
- CC2538, nRF52840, nRF54L15 through ARM platform configs and SoC ops.

Target:

```c
typedef struct sim_soc_desc {
    const char *name;
    const char *cpu_arch;
    uint32_t flash_base;
    uint32_t flash_size;
    uint32_t ram_base;
    uint32_t ram_size;
    uint32_t reset_vtor;
    const void *soc_config;
    const void *soc_ops;
} sim_soc_desc_t;
```

ARM already approximates this. MSP430 can be migrated later, once the runtime
and mote vtable exist.

### 5.3 Board / platform

The board layer owns concrete wiring:

- console UART/USART
- LED pins
- button pins
- external flash
- external radios
- reset/power pins
- default firmware extension/platform string

Target:

```c
typedef struct sim_board_desc {
    const char *name;          /* "sky", "zoul-firefly" */
    const char *soc_name;      /* "msp430f1611", "cc2538" */
    const char *platform_ext;  /* filename extension compatibility */
    const sim_pin_desc_t *leds;
    int led_count;
    const sim_chip_wiring_t *chips;
    int chip_count;
} sim_board_desc_t;
```

Board descriptors should be data-heavy and logic-light. Per-board firmware
quirks should live in mote adapter boot policies, not in generic CPU/SoC code.

### 5.4 Mote type

Mote type is the execution strategy:

- `emulated-elf`: CPU/SoC/board + ELF firmware.
- `native-cooja`: host shared library with Cooja variables.
- `js-app`: QuickJS application mote.
- future: external process mote, record/replay mote, hardware-in-loop mote.

Do not model native or JS motes as fake CPUs. They are mote types with their own
ops.

## 6. Radio Architecture Plan

### 6.1 Keep the current medium as policy oracle

`radio_medium_t` should remain responsible for:

- position and neighbor lists
- spectrum/channel/RX-enabled matching
- per-frame loss decisions
- frame profile tracking
- RSSI estimation

It should not:

- include chip headers
- call chip receive functions
- own per-node chip pointers
- schedule CPU execution directly

### 6.2 Extract a radio bus/dispatcher

Move the harness-side radio dispatch out of `test/test_mixed_multinode.c` into a
new module, tentatively `src/sim/sim_radio_bus.c`.

Responsibilities:

- register each mote radio endpoint `(mote_index, radio_idx, spectrum, profile)`
- receive bytes from chip TX callbacks
- feed `radio_medium_filter_byte_radio`
- schedule/deliver RX bytes to matching mote radio endpoints
- preserve byte-accurate timing and same-time FIFO order
- handle frame assembly metadata needed for timeline/PCAP/packet analyzer
- expose channel busy queries for radio chips such as CC1200

The bus should be the only module that knows how to dispatch:

- CC2420 bytes to `cc2420_receive_byte`
- CC2538 RFCore bytes to `cc2538_rfcore_receive_byte`
- CC1200 bytes to `cc1200_receive_byte`
- Nordic RADIO bytes to the Nordic receive functions
- native Cooja frames/bytes
- JS frame receive callbacks

### 6.3 Required radio invariants

Preserve these during every extraction:

- Radio timing is nanosecond-based.
- Byte delivery order is deterministic.
- Same-time events are FIFO by sequence number.
- ACK generation remains able to happen inside the sender's ACK wait window.
- Channel changes are pushed at the moment the chip changes channel, not by
  periodic sweeping.
- Multi-radio nodes must not leak cross-band traffic.
- Receiver RX-enabled gating must remain enforced.
- The medium never owns chip pointers.
- **RF-derived timers anchor in sim-time, not cpu-cycles.** Any timer that
  measures "elapsed wall-clock since the last RF event" — frame-stall
  watchdogs, ACK-wait timeouts, TX air-time defers, byte-period gates —
  must be scheduled as a `SIM_EVENT_*` in the global queue. Scheduling
  them in a chip's cpu-event-queue (e.g. `arm_schedule_event(cpu, …,
  cpu->cycles + N)`) silently breaks under the bulk-delivery pattern from
  §3.6: many byte events fire at the same `cpu->cycles`, the watchdog's
  fire_cycle never moves with the byte stream, and it trips mid-frame.
  The radio bus owns RF-derived scheduling for exactly this reason.

> **Phase 5 RF-timer audit (M30).** With the frame-delivery policy now in
> `sim_radio_bus.c`, the RF-derived timers were re-audited against the
> invariant above and are all sim-time anchored: the bus's RX-stall
> watchdog and the dual 192 µs auto-ACK windows schedule on `now_ns`
> (`accurate_tx_end + 192000` receivers, `now + 192000` sender); the
> cc2538 and nrf52840 RX→TX ACK turnarounds were confirmed ns-based; the
> nrf54l15 GRTC RX-stall fix landed in M9.5. No RF timer remains on a
> chip cpu-event queue.

## 7. Service & Plugin Model

The extension architecture starts as a static in-process registry of
**services** (built-in, statically linked). **Plugins** — dynamically loaded
services via `dlopen` — come later (§7.2) once the static API has proven stable.

The §2.1 glossary fixes the terminology: every built-in observer/contributor
is a "service"; "plugin" is reserved for the Phase 9+ `dlopen` case.

### 7.1 Service v1: static registry

Built-in components register at process startup:

```c
void csim_register_builtin_mote_types(sim_registry_t *r);
void csim_register_builtin_platforms(sim_registry_t *r);
void csim_register_builtin_socs(sim_registry_t *r);
void csim_register_builtin_media(sim_registry_t *r);
void csim_register_builtin_services(sim_registry_t *r);
```

V1 rules:

- built-in registration only
- no `dlopen`
- no ABI promises
- register mote types, platforms, SoCs, radio media, and services
- used by runtime creation and config normalization

Static registry categories (all registered as services or descriptors):

- mote types
- platforms/boards
- SoCs/MCUs
- radio media
- services (observers/contributors)
- packet analyzers
- test action providers
- UI state providers

### 7.2 Plugins (dynamic loading)

Plugin loading is optional and later:

- only after the static registry is stable
- likely starts with observer/service plugins
- dynamic mote/platform/radio plugins require a stricter ABI and come later
- plugin ABI must expose handles and functions, not internal structs
- the simulator must keep working with only built-in services

Sketch for the later ABI:

```c
#define CSIM_PLUGIN_API_VERSION 1

typedef struct csim_api {
    uint32_t version;
    const sim_registry_ops_t *registry;
    const sim_log_ops_t *log;
    const sim_alloc_ops_t *alloc;
} csim_api_t;

int csim_plugin_init(const csim_api_t *api);
```

## 8. Configuration Plan

### 8.1 Keep old configs working

Existing JSON configs and filename-extension detection must continue to work
during the refactor.

Current compatibility behavior:

- `.sky` -> MSP430 Sky
- `.z1` -> MSP430 Z1
- `.cc2538dk` -> ARM CC2538DK
- `.zoul-firefly` -> ARM CC2538 + CC1200 board
- `.nrf52840-dongle`, `.nrf52840-dk`, `.nrf54l15-dk` -> ARM/Nordic boards
- `.cooja` -> native Cooja mote
- `.js` -> JS mote

### 8.2 Add explicit config v2

Add explicit mote type and board modeling while accepting legacy shorthand.

Example:

```json
{
  "version": 2,
  "title": "RPL UDP on mixed platforms",
  "timeout_ms": 60000,
  "plugins": [
    "builtin:timeline",
    "builtin:pcap",
    "builtin:js-test"
  ],
  "mote_types": [
    {
      "name": "sky-client",
      "kind": "emulated-elf",
      "cpu": "msp430",
      "soc": "msp430f1611",
      "board": "sky",
      "firmware": "firmware/sky/udp-client.sky"
    },
    {
      "name": "firefly-subghz-server",
      "kind": "emulated-elf",
      "cpu": "armv7m",
      "soc": "cc2538",
      "board": "zoul-firefly",
      "firmware": "firmware/zoul-firefly/udp-server-subghz.zoul-firefly"
    }
  ],
  "nodes": [
    { "id": 1, "type": "firefly-subghz-server", "x": 0, "y": 0 },
    { "id": 2, "type": "sky-client", "x": 10, "y": 0 }
  ],
  "medium": {
    "type": "udgm",
    "tx_range": 50,
    "interference_range": 100,
    "success_ratio_tx": 1.0,
    "success_ratio_rx": 1.0
  }
}
```

### 8.3 Config parser migration

Strategy (committed): introduce one normalized internal config struct
(`sim_normalized_config_t`) that both v1 and v2 parsers populate. Runtime
creation consumes only the normalized struct, never the raw JSON layout.
Adding a v3 later means writing a v3→normalized adapter, not editing
`sim_runtime_init`.

Steps:

1. Add `version` field detection in the existing `sim_config_load`.
2. Define `sim_normalized_config_t` (mote types, nodes, medium, services,
   plugins, test actions, etc.) decoupled from JSON shape.
3. Convert the existing v1 path to produce `sim_normalized_config_t` instead of
   today's `sim_config_t`.
4. Add a v2 parser path producing the same `sim_normalized_config_t`.
5. Update `sim_runtime_init` to consume normalized config.
6. Keep `tools/csc2json.py` emitting v1 until v2 is stable.

## 9. Refactor Phases

Each phase should be reviewable and testable on its own. Avoid large moves with
behavior changes mixed in.

### Phase 0 - Baseline and guardrails

Goal: record current behavior and make future regressions obvious.

Tasks:

- Run and record baseline tests before structural changes.
- Add missing smoke-test commands to this plan if any current command fails or
  has become too expensive for every phase.
- Confirm no unrelated dirty files need to be touched.

Suggested baseline commands:

```sh
make
./build/test_runner correctness
./build/test_runner arm-correctness
./build/test_runner mock-host
./build/test_runner cc1200-mock-host
./build/test_runner radio-medium
./build/test_runner timeline
./build/test_runner firmware
./build/test_runner arm-firmware
```

Representative integration commands:

```sh
./build/test_runner multinode -t 20000 -q
./build/test_runner arm-multinode firmware/cc2538dk/nullnet-broadcast.cc2538dk -t 20000 -q
./build/test_runner zoul-firefly-multinode firmware/zoul-firefly/nullnet-broadcast-subghz.zoul-firefly -t 20000 -q
./build/test_runner nrf52840-dongle-multinode firmware/nrf52840-dongle/udp-client.nrf52840-dongle -t 20000 -q
```

Stop condition:

- If a baseline test fails before refactoring, document it and do not mix its
  fix with architecture extraction.

### Phase 1 - Introduce simulation runtime context

Goal: move global simulation state into a struct while keeping the runner as the
entry point.

The canonical task list is §3.15 above (10 numbered milestones). Land them in
order, one PR per milestone where practical. The earlier looser version of this
phase has been removed in favor of §3.15 — they were the same idea stated
twice.

Candidate files:

- Add `include/sim/sim_runtime.h`
- Add `src/sim/sim_runtime.c`
- Update `Makefile`
- Edit `test/test_mixed_multinode.c` incrementally

Validation:

```sh
make
./build/test_runner radio-medium
./build/test_runner timeline
./build/test_runner multinode -t 5000 -q
```

Stop condition:

- Do not change scheduling semantics in this phase.

### Phase 2 - Wrap nodes with a mote vtable

Goal: replace type-switching helper functions with `sim_mote_ops_t`.

Start by wrapping existing behavior:

- `node_sim_time_ns`
- `node_cycles`
- `node_freq`
- `node_instructions`
- `node_type_str`
- `step_node_until`
- `node_next_wakeup_ns`
- serial input injection
- radio RX delivery

Initial implementation can keep `mixed_node_t` internally:

```c
typedef struct sim_mote {
    int id;
    const sim_mote_ops_t *ops;
    void *impl; /* points at legacy mixed_node_t or an adapter-owned struct */
} sim_mote_t;
```

Then define adapters:

- MSP430 emulated mote ops
- ARM emulated mote ops
- native Cooja mote ops
- JS app mote ops

Validation:

```sh
make
./build/test_runner correctness
./build/test_runner arm-correctness
./build/test_runner mock-host
./build/test_runner multinode -t 5000 -q
./build/test_runner arm-multinode firmware/cc2538dk/nullnet-broadcast.cc2538dk -t 5000 -q
```

Stop condition:

- No platform init logic moves yet. This phase is dispatch abstraction only.

### Phase 3 - Extract platform and board selection

> **Status: COMPLETE.**  `sim_board` registry (include/sim/sim_board.h +
> src/sim/sim_board.c): one static row per board with extension, arch
> platform-lookup name, kind, and banner label.  The runner's four
> extension ladders (detect_node_type, MSP430/ARM platform-name
> derivations, init banner) read the registry row stashed on the node.
> `sim_board_find()` (by name) is in place for config/CLI platform
> overrides.  Kind stays an enum until Phase 4 binds rows to
> sim_mote_ops_t directly.

Goal: remove filename-extension platform knowledge from the scheduler.

Tasks:

- Create a platform registry.
- Register existing board descriptors.
- Move `detect_node_type` and platform selection into registry lookup.
- Keep filename extension as compatibility input.
- Add explicit platform override support in config/CLI later.

Current platform knowledge to move:

- `.sky`, `.z1`, `.esb`, `.wismote`, `.exp5438`, `.msp430fr5969`
- `.cc2538dk`, `.zoul-firefly`
- `.nrf52840-dongle`, `.nrf52840-dk`, `.nrf54l15-dk`
- `.cooja`, `.js`

Validation:

```sh
make
./build/test_runner firmware
./build/test_runner arm-firmware
./build/test_runner multinode -t 5000 -q
./build/test_runner zoul-firefly-multinode firmware/zoul-firefly/nullnet-broadcast-subghz.zoul-firefly -t 5000 -q
```

Stop condition:

- Do not redesign boot patching in this phase. Only move where the decision is
  made.

### Phase 4 - Move platform boot policies into mote adapters

Goal: move firmware loading, run-to-main, node id/linkaddr patching, and
board-specific quirks out of `test/test_mixed_multinode.c`.

The canonical task list is §3.17 above (milestones M18–M23). Land them in
order, one commit per milestone. The lists below summarize the scope.

Create boot policy modules:

- `src/motes/msp430_elf_mote.c`
- `src/motes/arm_elf_mote.c`
- `src/motes/native_cooja_mote.c`
- `src/motes/js_app_mote.c`

Move these responsibilities:

- ELF load
- reset
- run to `main`
- `ds2411_init` patch
- `ds2411_id` patch
- infomem/node_id/linkaddr patch
- ARM `node_id` and `linkaddr_node_addr` patch
- Nordic FICR/device ID seeding
- platform-specific radio TX listener wiring
- console callback wiring

Validation:

```sh
make
./build/test_runner firmware -v
./build/test_runner arm-firmware -v
./build/test_runner multinode -t 20000 -q
./build/test_runner nrf52840-dk-multinode firmware/nrf52840-dk/udp-client.nrf52840-dk -t 20000 -q
```

Stop condition:

- If a patch is firmware-specific and undocumented, keep it local to the board
  boot policy and add a comment that names the firmware symbol it depends on.

### Phase 5 - Extract radio bus from the runner

Goal: make RF routing reusable and remove the biggest correctness risk from the
runner.

The canonical task list is §3.18 above (milestones M24–M30). Land them in
order, one commit per milestone. The lists below summarize the scope; where
they disagree with §3.18 (written later, against the post-Phase-4 code),
§3.18 wins.

Move these concepts into `sim_radio_bus`:

- `rf_listener_ctx_t`
- per-sender frame assembly
- per-receiver RX queues
- MSP byte event queue or its replacement in the global event queue
- TX capture for packet analyzer/PCAP/timeline
- radio channel callback bridge
- channel busy query
- RF dispatch to chip receive functions through mote ops

Keep `radio_medium_t` as policy only.

Recommended order:

1. Extract pure helper structs and functions with no behavior change.
2. Pass `sim_runtime_t *` instead of using globals.
3. Replace direct chip calls with mote radio endpoint ops.
4. Move scheduling of radio byte events into the global simulation queue.
5. Migrate existing chip-internal RF-derived timers off `cpu_event_queue` and
   onto sim-time events the bus owns. Current known cases:
   - `nrf54l15` RX stall watchdog (`nrf54l_radio_rx_stall_event`) — today
     hacked around with a 50 ms cpu-cycle delay; the principled fix is
     "fire 50 µs of *sim-time* after the last RF byte for this receiver,"
     which only the bus can express.
   - `nrf52840` / `cc2538` ACK turnaround windows — currently survive on
     ns-based event scheduling, but the same hazard applies if any future
     chip adds a "no byte received in N µs → abort" timer.

Validation:

```sh
make
./build/test_runner radio-medium
./build/test_runner cc1200-mock-host
./build/test_runner multinode -t 20000 -q
./build/test_runner arm-multinode firmware/cc2538dk/nullnet-broadcast.cc2538dk -t 20000 -q
./build/test_runner zoul-firefly-multinode firmware/zoul-firefly/nullnet-broadcast-subghz.zoul-firefly -t 20000 -q
```

Stop condition:

- Do not "simplify" byte timing into frame-only delivery. The current RF model
  relies on byte-accurate scheduling for ACK timing.

### Phase 6 - Extract services

Goal: move optional features out of the scheduler.

The canonical task list is **§3.19 above (milestones M31–M40)**; land them
in order, one commit per milestone.  The notes below summarize the scope;
where they disagree with §3.19 (written later, against the post-Phase-5
code), §3.19 wins.

Services to extract:

- timeline
- packet analyzer
- PCAP writer
- WebSocket UI
- serial socket bridge *(special case — see notes)*
- GDB stub
- JS test engine *(special case — see notes)*
- JSON test actions/validators
- progress/stat printing

**Serial-socket caveat.** Today's `serial_socket` block interleaves TCP
listen/accept, child-process management (`test-border-router.sh`, etc.),
per-mote UART callback rewriting, `COOJA.testlog` tee-writing, and signal
handling on shutdown. Treat it as **two services** to avoid reproducing the
tangle:

- `serial_bridge_service`: pure TCP↔UART byte plumbing for the bridged mote.
  Subscribes to `SIM_OBS_MOTE_UART_BYTE` for the bridged node, schedules
  `SIM_EVENT_SERIAL_INPUT` from socket reads.
- `external_command_service`: forks/manages the bash test driver and the
  `COOJA.testlog` file. Subscribes to `SIM_OBS_MOTE_LOG_LINE` for tee-writing.

**JS test engine caveat.** The engine mutates sim state inside dispatched
events (`log.testFailed` → stop sim; `WAIT_UNTIL` → resume from line callback).
The new service implementation must route those through
`sim_runtime_request_stop()` and a deferred-resume queue, not by calling
kernel APIs that re-enter dispatch. Observer callbacks must be re-entrancy-
safe by construction.

Service ops sketch:

```c
typedef struct sim_service_ops {
    const char *name;
    int  (*init)(sim_runtime_t *sim, const void *cfg, void **state);
    void (*destroy)(sim_runtime_t *sim, void *state);
    void (*on_event)(sim_runtime_t *sim, void *state, const sim_observer_event_t *ev);
    void (*poll)(sim_runtime_t *sim, void *state);
} sim_service_ops_t;
```

Use observer events rather than direct calls:

- mote log line
- radio TX/RX/INTF
- LED change
- packet frame
- simulation start/stop
- node add/remove/reset

Validation:

```sh
make
./build/test_runner timeline
./build/test_runner test configs/test-js-hello.json -q
./build/test_runner mixed-multinode configs/ui-demo-cc2538dk.json -t 5000 -q
```

Stop condition:

- Observation services must not change simulation outcomes.

### Phase 7 - Config v2 and normalized runtime creation

Goal: create a clean config-to-runtime path.

The canonical task list is **§3.20 above (milestones M41–M43)**; land them in
order, one commit per milestone.  The notes below summarize the scope; where
they disagree with §3.20 (written later, against the post-Phase-6 code),
§3.20 wins.

Tasks:

- Add normalized config structs independent of legacy JSON layout.
- Convert legacy JSON into normalized config.
- Parse config v2 into the same normalized config.
- Update `csc2json.py` only after runtime supports v2.

Validation:

```sh
make
./build/test_runner test configs/test-rpl-udp-sky.json -q
./build/test_runner test configs/rpl-udp-cc2538dk.json -q
```

Stop condition:

- Do not break existing configs.

### Phase 8 - Static plugin registry

Goal: all built-in platforms, mote types, media, and services register through
one registry.

Tasks:

- Add `sim_registry_t`.
- Register built-ins at startup.
- Replace direct `*_find` calls in top-level runtime creation with registry
  lookups.
- Keep low-level `msp430_platform_find` and `arm_platform_find` for now as
  implementation details.

Validation:

```sh
make
./build/test_runner all
./build/test_runner multinode -t 20000 -q
./build/test_runner zoul-firefly-multinode firmware/zoul-firefly/nullnet-broadcast-subghz.zoul-firefly -t 20000 -q
```

Stop condition:

- Do not implement `dlopen` plugins yet.

### Phase 9 - Dynamic plugin ABI

Goal: optional external plugin loading.

Prerequisites:

- Static registry is stable.
- Config v2 can name plugins and mote/platform/media types.
- Runtime does not expose internal structs as required plugin API.

Tasks:

- Define ABI version.
- Define `csim_plugin_init`.
- Add plugin load path and error reporting.
- Add one tiny example plugin, likely a toy radio medium or packet sink.

Validation:

```sh
make
./build/test_runner all
```

Stop condition:

- If the plugin API needs access to many internal fields, stop and add explicit
  runtime functions instead of exporting structs.

### Phase 10 - Shrink the runner

Goal: `test/test_mixed_multinode.c` becomes a frontend.

Target responsibilities:

- parse CLI
- load config
- instantiate runtime
- register built-ins/plugins
- attach requested services
- run
- report exit code

Everything else should live under `src/sim`, `src/motes`, `src/platforms`, or
`src/services`.

Success metric:

- The runner no longer includes chip headers directly.
- The runner no longer switches on `NODE_MSP430`/`NODE_ARM` for normal
  simulation.
- New platforms can be added by registering descriptors/adapters, not editing
  scheduler code.

## 10. Testing Matrix

Use the smallest test that covers the change.

| Change area | Minimum tests |
|---|---|
| CPU dispatch only | `correctness`, `arm-correctness` |
| Off-SoC chip host/event work | `mock-host`, relevant chip mock test |
| CC1200 | `cc1200-mock-host`, `radio-medium`, Firefly multinode |
| Radio medium policy | `radio-medium`, one 2.4 GHz multinode, one sub-GHz multinode |
| Timeline/UI serialization | `timeline`, one `--ui` smoke if practical |
| Config parser | representative `test configs/*.json` |
| Mote vtable | MSP430, ARM, native, JS smoke tests |
| Scheduler/event queue | all unit tests plus at least Sky and CC2538 multinode |
| Platform registry | one test per platform family touched |

### Broad gate (mandatory before merging a phase)

```sh
make
./build/test_runner all
./build/test_runner multinode -t 20000 -q
./build/test_runner arm-multinode firmware/cc2538dk/nullnet-broadcast.cc2538dk -t 20000 -q
./build/test_runner zoul-firefly-multinode firmware/zoul-firefly/nullnet-broadcast-subghz.zoul-firefly -t 20000 -q

# Cooja regression — non-TUN tests must stay 81/81 green at every phase boundary.
# Phases that legitimately need to break this temporarily (e.g. mid-Phase 5
# radio-bus extraction) must call that out explicitly in the PR description.
make cooja-tests

# Determinism reproducibility — run the same config twice with the same seed
# and diff stdout+stderr (minus host-timing noise lines).  Any per-phase
# divergence is a bug, not a "minor refactor side effect".
tools/check-determinism.sh mixed-multinode configs/test-4node-chain.json -t 10000 -q
```

(`tools/check-determinism.sh` (added in M24) runs the given config twice and
diffs the captured output with wall-clock/throughput lines filtered out.
Stdout is strictly richer than the once-planned `--timeline-out` JSON — every
delivered byte, RX outcome, and ACK is printed — so that flag was never built;
the script supersedes it.  The same stdout-diff harness, run across two
*builds* instead of two runs, is the M25–M28 cross-build empty-diff gate.)

### Performance regression check

After each phase, on a quiet machine, repeat:

```sh
./build/test_runner mixed-multinode \
    firmware/sky/udp-server.sky firmware/sky/udp-client.sky -t 60000 -q
```

Record the `Wall-clock time` and compare to the immediately preceding tag.
Budget per §3.14.1. If exceeded, profile (`perf`/`Instruments`) on
`sim_radio_bus_deliver_byte` and `sim_dispatch_mote_execute` before merging.

## 11. Agent Instructions

Follow these rules when using this plan with Codex or Claude Code.

### 11.1 Before editing

1. Run `git status --short`.
2. Read this file and the relevant docs:
   - `docs/architecture.md`
   - `docs/porting-a-device.md`
   - `docs/radio-medium.md` for radio changes
3. Identify the exact phase and scope.
4. Run the smallest relevant baseline test.

### 11.2 During editing

- Make one architectural move per patch.
- Keep behavior-preserving extraction separate from behavior fixes.
- Do not edit CPU emulators while extracting runtime structure unless the phase
  explicitly requires it.
- Do not add new direct references from chip drivers to CPU/GPIO concrete types.
- Keep `sim_host_t` as the off-SoC chip boundary.
- Keep `radio_medium_t` free of chip pointers.
- Keep old configs and filename extension detection working until config v2 is
  fully adopted.
- Update `Makefile` whenever adding `.c` files. It does not auto-discover.
- Add tests close to the layer being changed.

### 11.3 After editing

1. Run the validation commands for the phase.
2. Report any commands not run.
3. Summarize files changed and behavior preserved.
4. If a design decision changed, update this file in the same patch or a
   follow-up doc-only patch.

### 11.4 Stop conditions

Stop and ask for review when:

- A phase requires changing radio timing semantics.
- A platform boot quirk is not understood.
- A dynamic plugin needs direct access to internal structs.
- Tests fail in a way unrelated to the current extraction.
- A patch exceeds roughly 1500-2000 lines without a green test.
- A new platform requires CPU emulator changes not described by its SPEC.
- A phase's perf budget (§3.14.1) is exceeded by more than 2×.
- The Cooja non-TUN suite (`make cooja-tests`) drops below 81/81 and you are
  not in a phase that explicitly authorizes it.

### 11.5 Rolling back

If a merged phase introduces a regression that escapes the broad gate (e.g. a
subtle ACK-timing bug that only shows up in `tools/run-cooja-tests.sh --with-tun`):

1. **Revert by default.** `git revert <merge>` the offending phase as a single
   commit. Don't try to fix forward unless the fix is one-line obvious.
2. **Open a follow-up branch** with the same phase number plus a suffix
   (`phase-5-radio-bus-v2`). Address the regression in the branch with a new
   test that catches it.
3. **Re-merge** only after the new test is in the gate.

This is cheaper than living with a bisect-hostile head while debugging.

## 12. Immediate Next Steps

The first implementation branch should be small and behavior-preserving.
Its purpose is runner shrinkage, not a full plugin system.

1. Add `sim_runtime_t` with `now_ns`, `event_queue`, `radio_medium`, run state, and stats.
2. Add runtime scheduling wrappers around current `sim_eq_*` calls.
3. Move `current_sim_ns` access behind `sim_runtime_now_ns(sim)`.
4. Add mote slot metadata and generation counters.
5. Add empty observer event dispatch.
6. Convert timeline as the first observer subscriber.
7. Add `sim_mote_ops_t` around read-only helpers:
   - time
   - cycles
   - frequency
   - type string
8. Only after that, begin extracting platform boot and radio bus logic.

This sequence gives structure early while keeping the hardest correctness areas
-- radio byte timing and platform boot patching -- intact until the runtime and
mote boundaries are ready.

## Decisions Log

Architectural choices made in this doc that should not be silently reversed in
implementation patches. If a phase needs to revisit one, update this list in
the same patch.

- **Phase 1 task list is §3.15 (10 numbered milestones).** §9 Phase 1 points at
  it; do not re-invent a parallel list.
- **Phase 4 task list is §3.17 (M18–M23).** It moves boot policy,
  radio-endpoint ops, tick helpers, and the full JS/native adapter sets to
  `src/motes/`; the emulated execute/serial adapters follow their
  dependencies (radio bus Phase 5, GDB service Phase 6) — superseding the
  §3.16 "adapters move in Phase 4" note (M17-descope precedent).
- **Phase 5 task list is §3.18 (M24–M30).** Frame-delivery policy moves into
  the bus behind slim notification hooks; the dual 192 µs ACK-window
  arithmetic is copied digit-for-digit; M25–M27 are gated by a cross-build
  empty-diff.
- **`--threads N` is retired (done, Phase 5 M29)** — resolved the §3.13
  port-or-retire decision toward *retire* (zero usage signals).  The threaded
  callbacks, `distribute_rf_outgoing`/`flush_pending_output`, the fixed-1 ms
  loop arm, `num_threads` + its guards, `rf_outgoing`/`defer_wakeups`, the
  `advance_to_time` op + 4 impls, and the `sim_threads.{c,h}` pool are deleted;
  the sequential kernel pump is the sole scheduler.  A future batch scheduler,
  if wanted, is greenfield work on the bus APIs behind `sim_scheduler_ops_t`,
  not a port of the deleted path.
- **Ops placement rule (Phase 5)**: CPU/local-clock motion lives on
  `sim_mote_ops_t`; chip radio-endpoint behavior lives on
  `mote_radio_ops_t`.  Platform delivery quirks become registration caps,
  not type checks.
- **MSP430/ARM execute/serial adapters move in Phase 6 M38** with the GDB
  service (after Phase 5, GDB is their only runner dependency; the GDB
  service M37 clears it via `cpu->gdb_stub`).
- **Phase 6 is complete (§3.19, M31–M40).**  Optional/observation features
  extracted into `src/services/*_service.c` behind a `sim_service_ops_t`
  vtable host built in M31 (one fan-out observer + ordered poll + the
  §Error-policy enforcement; the full register-by-name registry is
  Phase 8).  Seven services landed: timeline, pcap, progress, json-test,
  js-test, gdb, websocket-ui.  PCAP rides the bus host hook (raw bytes +
  `tx_start_ns` live there, not on `SIM_OBS_PACKET_FRAME`); observer-payload
  enrichment is Phase 8+ debt.  The packet analyzer is a stateless decoder
  with no service home (its verbose path reaches chip internals), so it
  stays runner-side.  The JS-test service routes stop through
  `sim_runtime_request_stop()` + a deferred-resume queue, never re-entering
  dispatch.  The MSP430/ARM execute/serial adapters moved to `src/motes/`
  (M38, the §3.17/§3.18 debt).  **End-of-run stats stay runner-side (M40)**:
  they read chip memory + firmware symbols and the cycle-totals loop is
  fused with `destroy_node`, so service-ifying them would violate the
  no-chip-internals rule for no gain — documented type-specific terminal
  diagnostics, revisit in Phase 10.
- **`--threads N` was "port to `sim_scheduler_ops::batch` or retire by Phase 6"**
  (§3.13) — resolved to *retire* in M29 (see entry above). Two divergent
  schedulers persisting indefinitely was not an option.
- **Performance budget**: ≤5% Phases 1–2, ≤10% Phases 3–5 combined, ≤5% Phases
  6–10, ≤20% cumulative on the 2-node Sky baseline (§3.14.1).
- **Error policy**: mote execute failure → drop wakeup, continue. Service init
  failure → refuse to start. Service runtime failure → disable + continue.
  OOM in kernel → `abort()` (§3.14.2).
- **Config v2 migration uses a single normalized internal struct that both v1
  and v2 parsers populate** (§8.3). Not "v2 parser OR extend v1 parser."
  Phase 7 task list is **§3.20 (M41–M43)**.  That normalized struct is the
  existing `sim_config_t`, **renamed `sim_normalized_config_t`** (the runner
  already consumes the parsed struct, not the JSON — §8.3 step 5 was already
  satisfied, so Phase 7 leaves the runtime untouched).  The legacy
  index-keyed `mote_type_firmware[8][256]` table is retained (read by index
  in `TEST_ACTION_ADD`); the v2 parser writes it alongside the richer
  `mote_types[]` table.  v2's `cpu`/`soc`/`board`/`plugins` metadata is
  captured but inert in Phase 7 — node→board resolution still goes through
  the firmware extension; the registry-by-name lookup is Phase 8.
- **Terminology**: "service" for built-in components via `sim_service_ops_t`,
  "plugin" for dynamic-loaded services only (§2.1).
- **JIT lives at the CPU-arch layer** (§5), not at runtime/mote/platform.
- **Multi-radio per node is `(mote_index, radio_idx)` everywhere** (§5).
- **Observer events distinguish UART bytes vs log lines** (§3.11).
- **Serial-socket extracts as two services** (`serial_bridge_service` +
  `external_command_service`), not one (§9 Phase 6).
- **JS test engine observer callbacks route stop-requests through
  `sim_runtime_request_stop()`**, never re-enter dispatch (§9 Phase 6).
- **Cooja non-TUN suite (`make cooja-tests`) is a per-phase gate**; drops below
  81/81 require explicit authorization (§10, §11.4).
- **Determinism reproducibility check is in the broad gate** (§10).
- **Rolling back a regressed phase is revert + follow-up branch**, not fix-
  forward by default (§11.5).

## Doc Status

This plan intentionally documents architecture direction before implementation.
The first implementation branch should be small and behavior-preserving:
introduce `sim_runtime_t` and wrappers, then run existing tests before moving
radio or platform logic. Decisions in the §Decisions Log are binding for
implementation patches unless updated in the same patch.
