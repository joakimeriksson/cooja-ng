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

Service / plugin
  Optional behavior around the simulation: UI, PCAP, GDB, serial socket,
  packet analyzer, timeline, test engine, custom radio medium.
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
    SIM_OBS_MOTE_LOG,
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

The current optional `--threads N` path is a batch stepping optimization and
does not have the same clean event semantics. Keep it working during migration,
but do not let it define the core API.

Target approach:

```c
typedef struct sim_scheduler_ops {
    const char *name;
    int (*run_until)(sim_runtime_t *sim, int64_t end_ns);
} sim_scheduler_ops_t;
```

Built-in scheduler policies:

- `event`: deterministic Cooja-style single event queue; reference scheduler
- `batch`: future parallel/batched scheduler for large topologies

The batch scheduler must use the same mote/radio/service APIs. It may trade
some fidelity for throughput only when explicitly selected.

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

### 3.15 Kernel extraction milestones

Use these smaller milestones before the broader phases below:

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

## 7. Plugin Model

The plugin architecture starts as a static in-process registry. That gives
Cooja-NG plugin-shaped extension points without committing to an external ABI
while the kernel is still being extracted.

### 7.1 Plugin v1: static registry

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

Static plugin categories:

- mote types
- platforms/boards
- SoCs/MCUs
- radio media
- services
- packet analyzers
- test action providers
- UI state providers

### 7.2 Plugin v2: dynamic loading

Dynamic plugin loading is optional and later:

- only after the static registry is stable
- likely starts with observer/service plugins
- dynamic mote/platform/radio plugins require a stricter ABI and come later
- plugin ABI must expose handles and functions, not internal structs
- the simulator must keep working with only built-ins

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

Steps:

1. Add `version` field detection.
2. Keep `sim_config_load` as the legacy parser.
3. Add `sim_config_v2_load` or extend `sim_config_load` with a normalized
   output structure.
4. Normalize both v1 and v2 into the same runtime creation API.
5. Keep `tools/csc2json.py` emitting legacy v1 until v2 is stable.

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

Candidate files:

- Add `include/sim/sim_runtime.h`
- Add `src/sim/sim_runtime.c`
- Update `Makefile`
- Edit `test/test_mixed_multinode.c` incrementally

State to move first:

- `num_nodes`
- `nodes`
- `radio_medium`
- `current_sim_ns`
- `sim_eq`
- RF pending buffers
- per-node start times
- stats counters
- timeline/node state arrays

Do this in small slices. The first slice can define:

```c
typedef struct sim_runtime {
    int node_count;
    int64_t current_time_ns;
    sim_event_queue_t event_queue;
    radio_medium_t radio_medium;
} sim_runtime_t;
```

Then move fields gradually.

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

Services to extract:

- timeline
- packet analyzer
- PCAP writer
- WebSocket UI
- serial socket bridge
- GDB stub
- JS test engine
- JSON test actions/validators
- progress/stat printing

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

Recommended broad gate before merging a large phase:

```sh
make
./build/test_runner all
./build/test_runner multinode -t 20000 -q
./build/test_runner arm-multinode firmware/cc2538dk/nullnet-broadcast.cc2538dk -t 20000 -q
./build/test_runner zoul-firefly-multinode firmware/zoul-firefly/nullnet-broadcast-subghz.zoul-firefly -t 20000 -q
```

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

## Doc Status

This plan intentionally documents architecture direction before implementation.
The first implementation branch should be small and behavior-preserving:
introduce `sim_runtime_t` and wrappers, then run existing tests before moving
radio or platform logic.
