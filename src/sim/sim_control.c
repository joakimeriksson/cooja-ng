/*
 * sim_control — see include/sim/sim_control.h.
 *
 * The mutation bodies are the JSON/JS action executors' former inline
 * code, lifted verbatim (same order of operations, same medium calls) so
 * the existing configs stay byte-identical; the flags encode the two
 * historical differences between those executors (wake-after-send,
 * reboot-after-add).
 */
#include "sim_control.h"

#include <string.h>
#include <limits.h>

#define SC_MS_TO_NS 1000000LL

void sim_control_init(sim_control_t *c, sim_runtime_t *sim,
                      const sim_control_ops_t *ops) {
    memset(c, 0, sizeof(*c));
    c->sim = sim;
    if (ops) c->ops = *ops;
    c->pause_at_ns = INT64_MAX;
}

/* --- node lookup ------------------------------------------------------- */

int sim_control_node_count(const sim_control_t *c) {
    return (c && c->ops.node_count) ? c->ops.node_count(c->ops.user) : 0;
}

bool sim_control_describe(const sim_control_t *c, int idx,
                          sim_control_node_info_t *out) {
    if (!c || !c->ops.describe || !out) return false;
    memset(out, 0, sizeof(*out));
    return c->ops.describe(c->ops.user, idx, out);
}

int sim_control_index_of_id(const sim_control_t *c, int node_id) {
    int n = sim_control_node_count(c);
    for (int i = 0; i < n; i++) {
        sim_control_node_info_t info;
        if (sim_control_describe(c, i, &info) && info.id == node_id)
            return i;
    }
    return -1;
}

bool sim_control_node_active(const sim_control_t *c, int idx) {
    sim_control_node_info_t info;
    return sim_control_describe(c, idx, &info) && info.active;
}

/* --- mutations --------------------------------------------------------- */

int sim_control_move(sim_control_t *c, int node_id, double x, double y) {
    int idx = sim_control_index_of_id(c, node_id);
    if (idx < 0 || !c->ops.set_position) return -1;
    c->ops.set_position(c->ops.user, idx, x, y);
    return 0;
}

int sim_control_remove(sim_control_t *c, int node_id) {
    int idx = sim_control_index_of_id(c, node_id);
    if (idx < 0 || !c->ops.remove) return -1;
    c->ops.remove(c->ops.user, idx);
    c->pending[idx].len = 0;
    return 0;
}

static void wake_slot(sim_control_t *c, int idx, int64_t at_ns) {
    sim_schedule_mote_wakeup_if_earlier(c->sim, idx, at_ns);
}

int sim_control_reboot(sim_control_t *c, int node_id, unsigned flags) {
    int idx = sim_control_index_of_id(c, node_id);
    if (idx < 0 || !c->ops.reboot || !c->ops.start) return -1;
    c->pending[idx].len = 0;
    if (c->ops.reboot(c->ops.user, idx) != 0) return -1;
    c->ops.start(c->ops.user, idx);
    if (flags & SIM_CONTROL_WAKE)
        wake_slot(c, idx, sim_runtime_now_ns(c->sim));
    return 0;
}

int sim_control_add(sim_control_t *c, const char *fw_or_type,
                    const char *secure_fw, int node_id,
                    double x, double y, unsigned flags) {
    if (!c || !fw_or_type || !c->ops.add) return -1;
    if (sim_control_index_of_id(c, node_id) >= 0) return -1;  /* id in use */
    const char *fw = NULL, *sfw = NULL;
    if (c->ops.firmware_for_type)
        fw = c->ops.firmware_for_type(c->ops.user, fw_or_type, &sfw);
    if (!fw) { fw = fw_or_type; sfw = secure_fw; }
    int idx = c->ops.add(c->ops.user, fw, sfw, node_id);
    if (idx < 0) return -1;
    if ((x != 0.0 || y != 0.0) && c->ops.set_position)
        c->ops.set_position(c->ops.user, idx, x, y);
    c->pending[idx].len = 0;
    if (flags & SIM_CONTROL_START) {
        if (c->ops.start) c->ops.start(c->ops.user, idx);
        if (flags & SIM_CONTROL_WAKE)
            wake_slot(c, idx, sim_runtime_now_ns(c->sim));
    }
    return idx;
}

static int send_to_slot(sim_control_t *c, int idx, const uint8_t *buf,
                        int len, unsigned flags) {
    if (len <= 0 || !c->ops.inject_serial) return 0;
    int consumed;
    if ((flags & SIM_CONTROL_RETRY) && c->pending[idx].len > 0) {
        /* Preserve ordering: queue behind what is already waiting. */
        consumed = 0;
    } else {
        consumed = c->ops.inject_serial(c->ops.user, idx, buf, len);
        if (consumed < 0) consumed = 0;
    }
    if ((flags & SIM_CONTROL_RETRY) && consumed < len) {
        int rest = len - consumed;
        int room = SIM_CONTROL_PENDING_MAX - c->pending[idx].len;
        if (rest > room) rest = room;
        memcpy(c->pending[idx].buf + c->pending[idx].len, buf + consumed, (size_t)rest);
        c->pending[idx].len += rest;
        consumed += rest;
    }
    if (flags & SIM_CONTROL_WAKE)
        wake_slot(c, idx, sim_runtime_now_ns(c->sim) + SC_MS_TO_NS);
    return consumed;
}

int sim_control_send(sim_control_t *c, int node_id, const uint8_t *buf,
                     int len, unsigned flags) {
    int idx = sim_control_index_of_id(c, node_id);
    if (idx < 0) return -1;
    return send_to_slot(c, idx, buf, len, flags);
}

void sim_control_send_all(sim_control_t *c, const uint8_t *buf, int len,
                          unsigned flags) {
    int n = sim_control_node_count(c);
    for (int i = 0; i < n; i++) {
        if (!sim_control_node_active(c, i)) continue;  /* not started / removed */
        send_to_slot(c, i, buf, len, flags);
    }
}

void sim_control_send_flush_pending(sim_control_t *c) {
    int n = sim_control_node_count(c);
    for (int i = 0; i < n && i < SIM_EQ_MAX_NODES; i++) {
        if (c->pending[i].len <= 0) continue;
        if (!sim_control_node_active(c, i)) continue;
        int consumed = c->ops.inject_serial(c->ops.user, i,
                                            c->pending[i].buf, c->pending[i].len);
        if (consumed <= 0) continue;
        if (consumed >= c->pending[i].len) {
            c->pending[i].len = 0;
        } else {
            memmove(c->pending[i].buf, c->pending[i].buf + consumed,
                    (size_t)(c->pending[i].len - consumed));
            c->pending[i].len -= consumed;
        }
        wake_slot(c, i, sim_runtime_now_ns(c->sim) + SC_MS_TO_NS);
    }
}

int sim_control_pending_len(const sim_control_t *c, int idx) {
    if (!c || idx < 0 || idx >= SIM_EQ_MAX_NODES) return 0;
    return c->pending[idx].len;
}

/* --- run state --------------------------------------------------------- */

void sim_control_pause(sim_control_t *c) {
    if (!c || !c->sim) return;
    if (c->sim->run_state != SIM_RUN_STOP_REQUESTED)
        c->sim->run_state = SIM_RUN_PAUSED;
    c->pause_at_ns = INT64_MAX;
    c->step_armed = false;
    c->step_events_left = 0;
}

void sim_control_resume(sim_control_t *c) {
    if (!c || !c->sim) return;
    if (c->sim->run_state != SIM_RUN_STOP_REQUESTED)
        c->sim->run_state = SIM_RUN_RUNNING;
    c->pause_at_ns = INT64_MAX;
    c->step_armed = false;
    c->step_events_left = 0;
}

void sim_control_run_for(sim_control_t *c, int64_t dur_ns) {
    sim_control_resume(c);
    if (dur_ns < 0) dur_ns = 0;
    c->pause_at_ns = sim_runtime_now_ns(c->sim) + dur_ns;
}

void sim_control_step_events(sim_control_t *c, int n) {
    sim_control_resume(c);
    if (n < 1) n = 1;
    c->step_events_left = n;
    c->step_armed = true;
}

bool sim_control_paused(const sim_control_t *c) {
    return c && c->sim && c->sim->run_state == SIM_RUN_PAUSED;
}

void sim_control_request_exit(sim_control_t *c) {
    if (c && c->sim) sim_runtime_request_stop(c->sim);
}

void sim_control_set_speed(sim_control_t *c, double ratio) {
    if (!c) return;
    c->speed_ratio = ratio > 0.0 ? ratio : 0.0;
}

/* --- loop hooks -------------------------------------------------------- */

int64_t sim_control_slice_cap(const sim_control_t *c, int64_t cap_ns) {
    if (c && c->pause_at_ns < cap_ns) return c->pause_at_ns;
    return cap_ns;
}

void sim_control_note_event(sim_control_t *c) {
    if (!c || !c->step_armed) return;
    if (c->step_events_left > 0) c->step_events_left--;
    /* Budget spent: flip to PAUSED now.  The dispatcher calls this before
     * dispatching the event, and the pump checks PAUSED before popping the
     * next one, so exactly N events run. */
    if (c->step_events_left <= 0 && c->sim &&
        c->sim->run_state != SIM_RUN_STOP_REQUESTED)
        c->sim->run_state = SIM_RUN_PAUSED;
}

bool sim_control_after_pump(sim_control_t *c) {
    if (!c || !c->sim) return false;
    if (c->sim->run_state == SIM_RUN_STOP_REQUESTED) return false;
    bool pause = false;
    if (c->pause_at_ns != INT64_MAX &&
        sim_runtime_now_ns(c->sim) >= c->pause_at_ns)
        pause = true;
    if (c->step_armed && c->step_events_left <= 0)
        pause = true;
    if (pause) sim_control_pause(c);
    return pause;
}
