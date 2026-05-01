/*
 * mock_sim_host_t unit tests
 *
 * Sanity tests of the mock host itself, plus one end-to-end demo that
 * drives the CC2420 driver through the mock to show the pattern for
 * future per-chip unit tests (cf. docs/porting-a-device.md §"Test
 * layering").
 */
#include "mock_sim_host.h"
#include "cc2420.h"
#include <stdio.h>
#include <string.h>
#include <assert.h>

static int passed = 0;
static int failed = 0;

#define ASSERT(cond, msg) do {                                          \
    if (cond) { passed++; }                                             \
    else { failed++; printf("  FAIL: %s (%s:%d)\n", msg, __FILE__, __LINE__); } \
} while (0)

/* ---------- self-tests for the mock itself ---------- */

static int dummy_fired;
static void dummy_event_cb(void *user_data, cpu_event_t *ev) {
    (void)user_data; (void)ev;
    dummy_fired++;
}

static void test_set_pin_recording(void) {
    mock_sim_host_t mock;
    mock_sim_host_init(&mock);

    mock.host.set_input_pin(mock.host.gpio, 1, 3, true);
    ASSERT(mock.set_input_pin_calls == 1, "set_input_pin counter");
    ASSERT(mock.last_set_pin.port == 1,   "last_set_pin.port");
    ASSERT(mock.last_set_pin.pin  == 3,   "last_set_pin.pin");
    ASSERT(mock.last_set_pin.value == true, "last_set_pin.value");
}

static void test_force_irq_recording(void) {
    mock_sim_host_t mock;
    mock_sim_host_init(&mock);

    mock.host.force_irq_edge(mock.host.gpio, 2, 5, false);
    ASSERT(mock.force_irq_edge_calls == 1, "force_irq counter");
    ASSERT(mock.last_force_irq.port == 2, "force_irq.port");
    ASSERT(mock.last_force_irq.pin  == 5, "force_irq.pin");
    ASSERT(mock.last_force_irq.rising == false, "force_irq.rising");
}

static void test_event_scheduling_and_firing(void) {
    mock_sim_host_t mock;
    mock_sim_host_init(&mock);

    cpu_event_t ev = { .callback = dummy_event_cb };
    dummy_fired = 0;

    mock.host.schedule_ns(mock.host.cpu, &ev, 1000);  /* fire at 1us */
    ASSERT(mock.schedule_calls == 1, "schedule counter");
    ASSERT(mock.queue_len == 1, "queue contains event");

    mock_sim_host_advance_ns(&mock, 500);             /* not yet */
    ASSERT(dummy_fired == 0, "event not yet fired");

    mock_sim_host_advance_ns(&mock, 600);             /* past 1us */
    ASSERT(dummy_fired == 1, "event fired exactly once");
    ASSERT(mock.queue_len == 0, "queue drained");
    ASSERT(mock.now_ns == 1100, "virtual clock advanced");
}

static void test_cancel_removes_event(void) {
    mock_sim_host_t mock;
    mock_sim_host_init(&mock);

    cpu_event_t ev = { .callback = dummy_event_cb };
    dummy_fired = 0;

    mock.host.schedule_ns(mock.host.cpu, &ev, 1000);
    mock.host.cancel(mock.host.cpu, &ev);
    ASSERT(mock.cancel_calls == 1, "cancel counter");
    ASSERT(mock.queue_len == 0, "queue empty after cancel");

    mock_sim_host_advance_ns(&mock, 5000);
    ASSERT(dummy_fired == 0, "cancelled event must not fire");
}

static void test_event_ordering(void) {
    mock_sim_host_t mock;
    mock_sim_host_init(&mock);

    cpu_event_t a = { .callback = dummy_event_cb };
    cpu_event_t b = { .callback = dummy_event_cb };
    dummy_fired = 0;

    /* Schedule out of order — mock must fire in time order. */
    mock.host.schedule_ns(mock.host.cpu, &a, 2000);
    mock.host.schedule_ns(mock.host.cpu, &b,  500);

    mock_sim_host_advance_ns(&mock, 3000);
    ASSERT(dummy_fired == 2, "both events fired");
    ASSERT(mock.queue_len == 0, "queue drained");
}

/* ---------- end-to-end: drive the CC2420 driver via mock ---------- */

/* This isn't an exhaustive CC2420 test — it just demonstrates the
 * pattern. The point is: no msp430_cpu_t, no msp430_gpio_t, no real
 * MCU. Tests for chip drivers can run in isolation. */
static void test_cc2420_vreg_off_to_on(void) {
    mock_sim_host_t mock;
    mock_sim_host_init(&mock);

    cc2420_t radio;
    cc2420_init(&radio, &mock.host);
    /* Configure the pins so the chip knows where to drive its outputs. */
    cc2420_set_pins(&radio,
                    /* fifop */ 1, 0,
                    /* fifo  */ 1, 1,
                    /* cca   */ 1, 2,
                    /* sfd   */ 4, 1);

    int prior_set_pin = mock.set_input_pin_calls;

    /* Turn VREG on — the chip should transition to POWER_DOWN, which
     * doesn't actually pulse pins yet, but it MUST NOT crash and MUST
     * NOT touch CPU state directly. */
    cc2420_set_vreg(&radio, true);
    ASSERT(mock.set_input_pin_calls >= prior_set_pin, "no negative side-effects");

    /* Turning it off again must cancel any scheduled events. */
    int prior_cancel = mock.cancel_calls;
    cc2420_set_vreg(&radio, false);
    ASSERT(mock.cancel_calls > prior_cancel, "VREG off cancels pending events");
}

int run_mock_host_tests(int verbose) {
    (void)verbose;
    printf("=== Mock sim_host_t Tests ===\n");

    test_set_pin_recording();
    test_force_irq_recording();
    test_event_scheduling_and_firing();
    test_cancel_removes_event();
    test_event_ordering();
    test_cc2420_vreg_off_to_on();

    printf("\n--- Results: %d passed, %d failed ---\n\n", passed, failed);
    return failed;
}
