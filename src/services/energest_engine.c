/*
 * energest_engine — see energest_engine.h.  Host-agnostic energy accounting:
 * accumulate-on-transition per radio state + a cumulative CPU/LPM snapshot,
 * a throttled live UI panel, and an end-of-run summary — all output via the
 * caller-supplied sink (no host symbol named here).
 */
#include "energest_engine.h"

#include <stdio.h>
#include <stdlib.h>

/* Radio-state indices match ui/sim_state.h's sim_radio_state_t, carried in
 * SIM_OBS_RADIO_STATE's u.radio_state.state (kept numeric — no UI header). */
enum { ST_OFF = 0, ST_ON = 1, ST_TX = 2, ST_RX = 3, ST_INTF = 4, ST_COUNT = 5 };

/* Nominal CC2420 figures (datasheet, @ 3.0 V). OFF ~ 0; INTF is the radio
 * receiving (energy ~ RX).  Edit these to model another transceiver. */
#define SUPPLY_V       3.0
static const double STATE_CURRENT_MA[ST_COUNT] = {
    [ST_OFF]  = 0.0,
    [ST_ON]   = 18.8,   /* listen / idle-RX */
    [ST_TX]   = 17.4,   /* transmit @ 0 dBm */
    [ST_RX]   = 18.8,   /* receiving        */
    [ST_INTF] = 18.8,   /* receiving (collision) */
};
/* MSP430/Sky-class CPU figures (Contiki Energest @ 3.0 V): active ~1.8 mA,
 * LPM3 ~0.0545 mA. */
#define CPU_ACTIVE_MA  1.8
#define CPU_LPM_MA     0.0545

#define MAX_MOTES 64

typedef struct {
    int     seen;                  /* this slot has had at least one event */
    int     cur_state;             /* current radio state                  */
    int64_t entered_ns;            /* time the current state was entered    */
    int64_t time_in[ST_COUNT];     /* accumulated ns per state              */
    int64_t cpu_active_ns;         /* CPU active time (SIM_OBS_CPU_STATE)   */
    int64_t cpu_lpm_ns;            /* CPU low-power-mode time                */
    int     have_cpu;              /* a CPU snapshot was received            */
} mote_energy_t;

struct energest_engine {
    energest_sink_t sink;
    mote_energy_t   m[MAX_MOTES];
    int64_t         last_ns;       /* latest event time, for final flush     */
    int64_t         last_panel_ns; /* last UI-panel publish time (throttle)  */
};

energest_engine_t *energest_engine_create(energest_sink_t sink) {
    energest_engine_t *e = calloc(1, sizeof *e);
    if (e) e->sink = sink;
    return e;
}

void energest_engine_free(energest_engine_t *e) {
    free(e);
}

static void emit_line(energest_engine_t *e, const char *line) {
    if (e->sink.log) e->sink.log(line);
    else             fputs(line, stdout);
}

/* Close out the current dwell of mote `m` at time `now`, crediting its state. */
static void accumulate(mote_energy_t *m, int64_t now) {
    int64_t dt = now - m->entered_ns;
    if (dt > 0 && m->cur_state >= 0 && m->cur_state < ST_COUNT)
        m->time_in[m->cur_state] += dt;
    m->entered_ns = now;
}

/* Build the live UI panel JSON ({"title","rows":[[label,value],...]}) from the
 * current accumulators — read-only (folds the open dwell into locals). */
static void publish_panel(energest_engine_t *e, sim_runtime_t *sim, int64_t now) {
    char buf[2048];
    int off = snprintf(buf, sizeof buf,
                       "{\"title\":\"Energy (CC2420 @3V)\",\"rows\":[");
    int rows = 0;
    for (int i = 0; i < MAX_MOTES && off < (int)sizeof buf; i++) {
        mote_energy_t *m = &e->m[i];
        if (!m->seen) continue;
        int64_t on_ns = m->time_in[ST_ON] + m->time_in[ST_TX] +
                        m->time_in[ST_RX] + m->time_in[ST_INTF];
        int64_t tot_ns = on_ns + m->time_in[ST_OFF];
        int64_t open = now - m->entered_ns;
        if (open > 0) { tot_ns += open; if (m->cur_state != ST_OFF) on_ns += open; }

        double radio_mj = 0.0;
        for (int s = 0; s < ST_COUNT; s++)
            radio_mj += (m->time_in[s] / 1e9) * STATE_CURRENT_MA[s] * SUPPLY_V;
        double cpu_mj = (m->cpu_active_ns / 1e9) * CPU_ACTIVE_MA * SUPPLY_V +
                        (m->cpu_lpm_ns    / 1e9) * CPU_LPM_MA    * SUPPLY_V;
        double duty = tot_ns > 0 ? (100.0 * (double)on_ns / (double)tot_ns) : 0.0;

        off += snprintf(buf + off, sizeof buf - off,
                        "%s[\"mote %d\",\"duty %.1f%%  ~%.1f mJ\"]",
                        rows ? "," : "", i, duty, radio_mj + cpu_mj);
        rows++;
    }
    if (off < (int)sizeof buf)
        snprintf(buf + off, sizeof buf - off, "]}");
    e->sink.publish(sim, "energest", buf);
}

void energest_engine_on_event(energest_engine_t *e, sim_runtime_t *sim,
                              const sim_observer_event_t *ev) {
    int idx = ev->mote_index;
    if (idx < 0 || idx >= MAX_MOTES) return;
    mote_energy_t *m = &e->m[idx];

    if (ev->kind == SIM_OBS_CPU_STATE) {
        /* Cumulative snapshot — store the latest (the end-of-run one is exact). */
        m->cpu_active_ns = ev->u.cpu.active_ns;
        m->cpu_lpm_ns    = ev->u.cpu.lpm_ns;
        m->have_cpu      = 1;
        return;
    }
    if (ev->kind != SIM_OBS_RADIO_STATE) return;
    int new_state = ev->u.radio_state.state;
    if (new_state < 0 || new_state >= ST_COUNT) return;

    e->last_ns = ev->time_ns;
    if (!m->seen) {
        /* First sighting: start integrating from here in the reported state. */
        m->seen = 1;
        m->cur_state = new_state;
        m->entered_ns = ev->time_ns;
        return;
    }
    accumulate(m, ev->time_ns);
    m->cur_state = new_state;

    /* Refresh the live UI panel from the event stream (the service host only
     * polls when a serial/extcmd child is active, so poll() can't be relied
     * on).  Throttled to ~250 ms of sim time; skipped when no UI sink. */
    if (e->sink.publish && e->last_ns - e->last_panel_ns >= 250000000LL) {
        e->last_panel_ns = e->last_ns;
        publish_panel(e, sim, e->last_ns);
    }
}

void energest_engine_report(energest_engine_t *e) {
    char buf[256];
    emit_line(e, "energest: per-mote radio energy estimate "
                 "(CC2420 @ 3.0V; on=listen/rx, off uncharged)\n");

    double net_mj = 0.0;
    for (int i = 0; i < MAX_MOTES; i++) {
        mote_energy_t *m = &e->m[i];
        if (!m->seen) continue;
        accumulate(m, e->last_ns);  /* flush the final open dwell */

        int64_t on_ns = m->time_in[ST_ON] + m->time_in[ST_TX] +
                        m->time_in[ST_RX] + m->time_in[ST_INTF];
        int64_t tot_ns = on_ns + m->time_in[ST_OFF];

        double radio_mj = 0.0;
        for (int s = 0; s < ST_COUNT; s++)
            radio_mj += (m->time_in[s] / 1e9) * STATE_CURRENT_MA[s] * SUPPLY_V;
        double cpu_mj = (m->cpu_active_ns / 1e9) * CPU_ACTIVE_MA * SUPPLY_V +
                        (m->cpu_lpm_ns    / 1e9) * CPU_LPM_MA    * SUPPLY_V;
        double mj = radio_mj + cpu_mj;
        net_mj += mj;

        double duty = tot_ns > 0 ? (100.0 * (double)on_ns / (double)tot_ns) : 0.0;
        int n = snprintf(buf, sizeof buf,
                 "energest:  mote %-2d  radio-on %.3fs (tx %.3f rx %.3f listen %.3f)  "
                 "duty %.2f%%",
                 i, on_ns / 1e9, m->time_in[ST_TX] / 1e9,
                 (m->time_in[ST_RX] + m->time_in[ST_INTF]) / 1e9,
                 m->time_in[ST_ON] / 1e9, duty);
        if (m->have_cpu && n > 0 && n < (int)sizeof buf)
            snprintf(buf + n, sizeof buf - n, "  cpu %.3fs lpm %.3fs",
                     m->cpu_active_ns / 1e9, m->cpu_lpm_ns / 1e9);
        emit_line(e, buf);
        snprintf(buf, sizeof buf, "  ~%.3f mJ (radio %.3f + cpu %.3f)\n",
                 mj, radio_mj, cpu_mj);
        emit_line(e, buf);
    }
    snprintf(buf, sizeof buf, "energest: network total ~%.3f mJ\n", net_mj);
    emit_line(e, buf);
}
