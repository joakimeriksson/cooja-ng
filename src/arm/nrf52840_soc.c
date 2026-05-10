/*
 * nRF52840 SoC peripheral bundle — skeleton implementation.
 *
 * The first goal is L0–L1: load an nrf52840 ELF, reset the CPU, and step
 * until either main() is reached or the firmware accesses a peripheral
 * we haven't modelled yet. Each unmodelled access lands on the unmapped-IO
 * path in `arm_cpu.c` and surfaces in stderr — that's our discovery
 * mechanism.
 *
 * Set-console writes to stderr until a UARTE0 model is added.
 */
#include "nrf52840_soc.h"
#include "arm_cpu.h"
#include <stdio.h>
#include <stdlib.h>

/* sim_host shims — the host vtable used by off-SoC chip drivers.
 * nRF52840 has no off-SoC chips on the dongle, but we wire the basics
 * anyway so anything that consults plat->host doesn't NULL-deref. */

static int64_t nrf_host_now_ns(void *cpu) {
    return ((arm_cpu_t *)cpu)->sim_time_ns;
}

static void nrf_host_schedule_ns(void *cpu, cpu_event_t *ev, int64_t fire_ns) {
    arm_schedule_event_ns((arm_cpu_t *)cpu, ev, fire_ns);
}

static void nrf_host_cancel(void *cpu, cpu_event_t *ev) {
    arm_cancel_event((arm_cpu_t *)cpu, ev);
}

static void nrf_console_stub(void *user_data, uint8_t byte) {
    (void)user_data;
    /* Until UARTE0 is modelled, just drop bytes. The default console
     * callback is replaced by the test runner with one that captures
     * to stdout / a buffer. */
    (void)byte;
}

static void nrf52840_soc_init(arm_platform_t *plat) {
    nrf52840_soc_t *soc = (nrf52840_soc_t *)calloc(1, sizeof(*soc));
    plat->soc = soc;

    /* Minimum host vtable — enough that NULL derefs don't happen.
     * GPIO and the input-pin / IRQ shims stay NULL until we model
     * GPIO/GPIOTE. */
    plat->host.cpu         = &plat->cpu;
    plat->host.gpio        = NULL;
    plat->host.now_ns      = nrf_host_now_ns;
    plat->host.schedule_ns = nrf_host_schedule_ns;
    plat->host.cancel      = nrf_host_cancel;
}

static void nrf52840_soc_destroy(arm_platform_t *plat) {
    free(plat->soc);
    plat->soc = NULL;
}

static void nrf52840_soc_set_console(arm_platform_t *plat,
                                      arm_uart_tx_callback cb, void *user_data) {
    /* No UARTE0 yet — record the callback on the platform so a future
     * UARTE0 model can pick it up. For now, swallow bytes. */
    (void)plat;
    (void)cb;
    (void)user_data;
    /* Intentionally a no-op until uarte0 lands. */
    nrf_console_stub(NULL, 0);
}

const arm_soc_ops_t nrf52840_soc_ops = {
    .name        = "nrf52840",
    .init        = nrf52840_soc_init,
    .destroy     = nrf52840_soc_destroy,
    .set_console = nrf52840_soc_set_console,
};
