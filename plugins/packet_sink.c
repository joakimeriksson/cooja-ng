/*
 * packet_sink — the Phase 9 example plugin (docs/design/refactor-plan.md
 * §3.23 / §7.2).
 *
 * A minimal external plugin: it registers an observer SERVICE that taps the
 * kernel event stream and prints a one-line tally once, at teardown.  It
 * reaches the host only through the csim_api vtable handed to
 * csim_plugin_init — it never names a kernel internal struct.
 *
 * It tallies three event kinds: radio frames (PACKET_FRAME — emitted only when
 * the live UI is active), firmware console lines (MOTE_LOG_LINE), and console
 * bytes (MOTE_UART_BYTE) — the latter two always flow, so the tally is
 * non-zero in a normal headless run.
 *
 * Build (see the Makefile `plugins` target):
 *   cc -shared -fPIC -O2 -std=c11 -I include/sim plugins/packet_sink.c \
 *      -o build/plugins/packet_sink.so
 * Load:  ./build/test_runner test <cfg> --plugin build/plugins/packet_sink.so
 *   or via a config v2 "plugins": ["build/plugins/packet_sink.so"].
 */
#include "csim_plugin.h"

#include <stdio.h>
#include <stdlib.h>

typedef struct {
    long frames;      /* SIM_OBS_PACKET_FRAME (UI-only)   */
    long tx;          /*   ... of which sender-side       */
    long rx;          /*   ... of which receiver-side     */
    long log_lines;   /* SIM_OBS_MOTE_LOG_LINE            */
    long uart_bytes;  /* SIM_OBS_MOTE_UART_BYTE           */
} sink_state_t;

/* The api STRUCT is call-scoped (the host passes a stack local); its vtable
 * pointers are process-stable.  So copy the struct by value at init and use
 * the copy from the callbacks. */
static csim_api_t g_api;
static int        g_have_api;

static int sink_init(sim_runtime_t *sim, const void *cfg, void **state) {
    (void)sim; (void)cfg;
    sink_state_t *st = calloc(1, sizeof *st);
    if (!st) return -1;
    *state = st;
    return 0;
}

static void sink_on_event(sim_runtime_t *sim, void *state,
                          const sim_observer_event_t *ev) {
    (void)sim;
    sink_state_t *st = (sink_state_t *)state;
    switch (ev->kind) {
    case SIM_OBS_PACKET_FRAME:
        st->frames++;
        if (ev->u.frame.is_tx) st->tx++;
        else                   st->rx++;
        break;
    case SIM_OBS_MOTE_LOG_LINE:  st->log_lines++;  break;
    case SIM_OBS_MOTE_UART_BYTE: st->uart_bytes++; break;
    default: break;
    }
}

/* Print exactly once, at teardown — a single deterministic flush (the kernel
 * does not emit SIM_OBS_SIM_STOP). */
static void sink_destroy(sim_runtime_t *sim, void *state) {
    (void)sim;
    sink_state_t *st = (sink_state_t *)state;
    if (!st) return;
    if (g_have_api && g_api.log && g_api.log->printf)
        g_api.log->printf("packet-sink: %ld frames (%ld tx, %ld rx), "
                          "%ld log lines, %ld uart bytes\n",
                          st->frames, st->tx, st->rx,
                          st->log_lines, st->uart_bytes);
    else
        printf("packet-sink: %ld frames (%ld tx, %ld rx), "
               "%ld log lines, %ld uart bytes\n",
               st->frames, st->tx, st->rx, st->log_lines, st->uart_bytes);
    free(st);
}

static const sim_service_ops_t sink_ops = {
    .name     = "packet-sink",
    .init     = sink_init,
    .destroy  = sink_destroy,
    .on_event = sink_on_event,
};

int csim_plugin_init(const csim_api_t *api) {
    /* Additive-ABI contract: accept any host at or above the version this
     * plugin needs (v1 — it only uses register_service). */
    if (!api || api->version < 1u)
        return -1;
    if (!api->registry || !api->registry->register_service)
        return -1;
    g_api = *api;       /* copy by value — the struct itself is call-scoped */
    g_have_api = 1;
    return api->registry->register_service(api->reg, &sink_ops) < 0 ? -1 : 0;
}
