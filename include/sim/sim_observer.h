/*
 * Observer events — Phase 1 milestone 6 of the refactor.
 *
 * The kernel emits observer events at well-defined points (mote add/remove,
 * UART byte/line, LED change, radio TX/RX boundary, packet frame, sim
 * start/stop).  Services subscribe via `sim_runtime_subscribe()` and
 * receive a `const sim_observer_event_t *` on every emission.
 *
 * Services must not:
 *   - call back into kernel APIs that re-enter dispatch (use
 *     sim_runtime_request_stop() for stop, schedule a future event for
 *     deferred work).
 *   - block.  Observer callbacks run on the dispatch thread.
 *   - retain the event pointer past the callback — payloads are
 *     stack-allocated by the emitter.
 *
 * See docs/design/refactor-plan.md §3.11.
 */
#ifndef SIM_OBSERVER_H
#define SIM_OBSERVER_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum sim_observer_kind {
    SIM_OBS_MOTE_ADDED         = 1,
    SIM_OBS_MOTE_REMOVED,
    SIM_OBS_MOTE_UART_BYTE,    /* raw byte off the console UART          */
    SIM_OBS_MOTE_LOG_LINE,     /* line-assembled "\n"-terminated text    */
    SIM_OBS_LED_CHANGED,
    SIM_OBS_RADIO_TX_START,
    SIM_OBS_RADIO_TX_END,
    SIM_OBS_RADIO_RX_START,
    SIM_OBS_RADIO_RX_END,
    SIM_OBS_RADIO_INTERFERENCE,
    SIM_OBS_PACKET_FRAME,
    SIM_OBS_SIM_START,
    SIM_OBS_SIM_STOP,
    /* A per-node radio-state transition (OFF/ON/TX/RX/INTF).  Carries the new
     * state in u.radio_state.state (values match ui/sim_state.h's
     * sim_radio_state_t: 0=OFF 1=ON 2=TX 3=RX 4=INTF — kept as a plain int so
     * this kernel header stays independent of the UI header).  Emitted only
     * while radio-state tracking is on (the live UI, or any loaded plugin —
     * see sim_runtime_set_radio_state_tracking()); a duty-cycle / energy
     * service accumulates time-in-state from this stream. */
    SIM_OBS_RADIO_STATE,
    /* A per-mote CPU power-time snapshot: cumulative active and low-power-mode
     * (LPM/WFI) nanoseconds in u.cpu.  Emitted at end-of-run (one per mote) for
     * an energy service; carries totals, not a transition, so the last snapshot
     * is exact.  Only when radio-state tracking is on. */
    SIM_OBS_CPU_STATE,
} sim_observer_kind_t;

typedef struct sim_observer_event {
    sim_observer_kind_t kind;
    int64_t  time_ns;
    int      mote_index;     /* -1 for sim-wide events                    */
    int      radio_idx;      /* -1 for non-radio events                   */
    union {
        struct { uint8_t  byte; } uart;
        struct {
            const char *line;
            int len;
            int node_id;   /* Cooja node ID (≠ mote_index) — testlog/JS
                            * consumers key on this, matching Cooja's
                            * log.log(time + " " + id + " " + msg) */
        } log_line;
        struct { int led_index; bool on; } led;
        struct {
            const uint8_t *data;
            int len;
            int channel;
            int8_t rssi;
        } radio;
        struct {
            bool is_tx;             /* true = sender side, false = receiver side */
            const char *summary;    /* packet-analyzer summary text; NULL if none */
        } frame;
        struct { int state; } radio_state;  /* SIM_OBS_RADIO_STATE: new state */
        struct { int64_t active_ns, lpm_ns; } cpu; /* SIM_OBS_CPU_STATE totals */
    } u;
} sim_observer_event_t;

typedef void (*sim_observer_callback_t)(void *user,
                                        const sim_observer_event_t *ev);

#ifdef __cplusplus
}
#endif

#endif /* SIM_OBSERVER_H */
