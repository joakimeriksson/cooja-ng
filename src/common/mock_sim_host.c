/*
 * mock_sim_host_t implementation. See include/common/mock_sim_host.h
 * for usage.
 */
#include "mock_sim_host.h"

/* The "cpu" handle in the mock is the mock_sim_host_t itself, which lets
 * the shims reach back to the queue and counters. Same for "gpio". */

static int64_t mock_now_ns(void *cpu) {
    return ((mock_sim_host_t *)cpu)->now_ns;
}

/* Internal queue-removal that does NOT bump the cancel counter. Used by
 * the public cancel and by schedule_ns's cancel-and-reinsert path. */
static void mock_remove(mock_sim_host_t *m, cpu_event_t *ev) {
    for (int i = 0; i < m->queue_len; i++) {
        if (m->queue[i] == ev) {
            for (int j = i; j < m->queue_len - 1; j++)
                m->queue[j] = m->queue[j + 1];
            m->queue_len--;
            return;
        }
    }
}

static void mock_cancel(void *cpu, cpu_event_t *ev) {
    mock_sim_host_t *m = (mock_sim_host_t *)cpu;
    m->cancel_calls++;
    mock_remove(m, ev);
}

static void mock_schedule_ns(void *cpu, cpu_event_t *ev, int64_t fire_ns) {
    mock_sim_host_t *m = (mock_sim_host_t *)cpu;
    /* If already queued, remove first (cancel-and-reinsert semantics).
     * Use the internal remove so this doesn't pollute the cancel counter. */
    mock_remove(m, ev);
    m->schedule_calls++;

    ev->fire_ns = fire_ns;
    /* Insert sorted by fire_ns ascending; oldest tiebreaker first. */
    int pos = 0;
    while (pos < m->queue_len && m->queue[pos]->fire_ns <= fire_ns)
        pos++;
    if (m->queue_len >= MOCK_SIM_HOST_MAX_EVENTS) return;  /* drop on overflow */
    for (int j = m->queue_len; j > pos; j--)
        m->queue[j] = m->queue[j - 1];
    m->queue[pos] = ev;
    m->queue_len++;
}

static void mock_set_input_pin(void *gpio, int port, int pin, bool value) {
    mock_sim_host_t *m = (mock_sim_host_t *)gpio;
    m->set_input_pin_calls++;
    m->last_set_pin.port  = port;
    m->last_set_pin.pin   = pin;
    m->last_set_pin.value = value;
}

static void mock_force_irq_edge(void *gpio, int port, int pin, bool rising) {
    mock_sim_host_t *m = (mock_sim_host_t *)gpio;
    m->force_irq_edge_calls++;
    m->last_force_irq.port   = port;
    m->last_force_irq.pin    = pin;
    m->last_force_irq.rising = rising;
}

static void mock_radio_set_channel(void *user_data, int radio_idx, int channel) {
    mock_sim_host_t *m = (mock_sim_host_t *)user_data;
    m->radio_set_channel_calls++;
    m->last_radio_channel.radio_idx = radio_idx;
    m->last_radio_channel.channel   = channel;
}

static void mock_radio_set_power(void *user_data, int radio_idx,
                                 int indicator, int max) {
    mock_sim_host_t *m = (mock_sim_host_t *)user_data;
    m->radio_set_power_calls++;
    (void)radio_idx; (void)indicator; (void)max;
}

void mock_sim_host_init(mock_sim_host_t *mock) {
    memset(mock, 0, sizeof(*mock));
    /* The mock is its own cpu, gpio, and radio_user_data — every shim
     * family reaches the same struct and inspects/mutates the same state. */
    mock->host.cpu              = mock;
    mock->host.gpio             = mock;
    mock->host.radio_user_data  = mock;
    mock->host.now_ns           = mock_now_ns;
    mock->host.schedule_ns      = mock_schedule_ns;
    mock->host.cancel           = mock_cancel;
    mock->host.set_input_pin    = mock_set_input_pin;
    mock->host.force_irq_edge   = mock_force_irq_edge;
    mock->host.radio_set_channel = mock_radio_set_channel;
    mock->host.radio_set_power   = mock_radio_set_power;
}

void mock_sim_host_advance_ns(mock_sim_host_t *mock, int64_t delta_ns) {
    int64_t target = mock->now_ns + delta_ns;
    while (mock->queue_len > 0 && mock->queue[0]->fire_ns <= target) {
        cpu_event_t *ev = mock->queue[0];
        /* Remove from queue head before firing — the callback may re-schedule. */
        for (int j = 0; j < mock->queue_len - 1; j++)
            mock->queue[j] = mock->queue[j + 1];
        mock->queue_len--;
        mock->now_ns = ev->fire_ns;
        if (ev->callback) ev->callback(ev->user_data, ev);
    }
    mock->now_ns = target;
}
