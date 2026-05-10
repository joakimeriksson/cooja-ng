/*
 * nRF52840 SoC — minimum-viable peripheral set.
 *
 * Models just enough to run a real Contiki-NG nrf52840 firmware build
 * (UART console variant) up to the point where it emits its first
 * printf:
 *
 *   - CLOCK (0x40000000): HFCLK/LFCLK start-task ↔ event handshake.
 *     Without this, boot spins forever in the HFCLKSTART loop.
 *
 *   - UART0 legacy window (0x40002000): TXD register + EVENTS_TXDRDY.
 *     `uart0_writeb` writes a byte and spins on TXDRDY; we set it
 *     immediately after the byte arrives.
 *
 * Everything else (RTC, TIMER, GPIO, GPIOTE, RADIO, PPI, NVMC, RNG)
 * is intentionally NOT modelled yet — the L0–L4 ladder will add them
 * as firmware traps on each missing peripheral.
 */
#include "nrf52840_soc.h"
#include "arm_cpu.h"
#include <stdio.h>
#include <stdlib.h>

/* sim_host shims — the host vtable used by off-SoC chip drivers.
 * The dongle has none, but anything that consults plat->host should
 * find a usable now_ns / schedule_ns / cancel. */
static int64_t nrf_host_now_ns(void *cpu) {
    return ((arm_cpu_t *)cpu)->sim_time_ns;
}
static void nrf_host_schedule_ns(void *cpu, cpu_event_t *ev, int64_t fire_ns) {
    arm_schedule_event_ns((arm_cpu_t *)cpu, ev, fire_ns);
}
static void nrf_host_cancel(void *cpu, cpu_event_t *ev) {
    arm_cancel_event((arm_cpu_t *)cpu, ev);
}

/* ============================================================
 * CLOCK (0x40000000)
 *
 * Task offsets:
 *   0x000 TASKS_HFCLKSTART  — write 1 → set EVENTS_HFCLKSTARTED
 *   0x004 TASKS_HFCLKSTOP   — accept (no event)
 *   0x008 TASKS_LFCLKSTART  — write 1 → set EVENTS_LFCLKSTARTED
 *   0x00C TASKS_LFCLKSTOP
 *
 * Event offsets:
 *   0x100 EVENTS_HFCLKSTARTED
 *   0x104 EVENTS_LFCLKSTARTED
 *
 * Real hardware takes a few hundred microseconds to start a crystal;
 * csim flips the event immediately. Firmware that polls is happy;
 * firmware that uses an interrupt would need NVIC wiring (not yet).
 * ============================================================ */
#define NRF_CLOCK_BASE            0x40000000u
#define NRF_CLOCK_SIZE            0x1000u
#define NRF_CLOCK_TASKS_HFCLKSTART 0x000
#define NRF_CLOCK_TASKS_LFCLKSTART 0x008
#define NRF_CLOCK_EVENTS_HFCLKSTARTED 0x100
#define NRF_CLOCK_EVENTS_LFCLKSTARTED 0x104

static int nrf_clock_read(void *user_data, uint32_t addr) {
    nrf_clock_state_t *clock = (nrf_clock_state_t *)user_data;
    uint32_t off = addr - NRF_CLOCK_BASE;
    switch (off) {
        case NRF_CLOCK_EVENTS_HFCLKSTARTED: return (int)clock->hfclkstarted;
        case NRF_CLOCK_EVENTS_LFCLKSTARTED: return (int)clock->lfclkstarted;
        default: return 0;
    }
}

static void nrf_clock_write(void *user_data, uint32_t addr, uint32_t value) {
    nrf_clock_state_t *clock = (nrf_clock_state_t *)user_data;
    uint32_t off = addr - NRF_CLOCK_BASE;
    switch (off) {
        case NRF_CLOCK_TASKS_HFCLKSTART:
            if (value == 1) clock->hfclkstarted = 1;
            break;
        case NRF_CLOCK_TASKS_LFCLKSTART:
            if (value == 1) clock->lfclkstarted = 1;
            break;
        case NRF_CLOCK_EVENTS_HFCLKSTARTED:
            clock->hfclkstarted = value & 1;
            break;
        case NRF_CLOCK_EVENTS_LFCLKSTARTED:
            clock->lfclkstarted = value & 1;
            break;
        default: break;
    }
}

/* ============================================================
 * UART0 legacy register window (0x40002000)
 *
 *   0x11C EVENTS_TXDRDY  — TX byte transmitted and ready for next
 *   0x500 ENABLE         — write 4 to enable in legacy UART mode
 *   0x51C TXD            — write byte to transmit
 *
 * (PSEL.TXD/RXD/CTS/RTS, BAUDRATE, CONFIG: writes accepted, no
 * behaviour. Reads return 0.)
 *
 * UARTE EasyDMA path (TXD.PTR / TXD.MAXCNT / TASKS_STARTTX) is NOT
 * modelled — Contiki uses the legacy API with NRF52840_NATIVE_USB=0.
 * ============================================================ */
#define NRF_UART0_BASE             0x40002000u
#define NRF_UART0_SIZE             0x1000u
#define NRF_UART_EVENTS_TXDRDY     0x11C
#define NRF_UART_ENABLE            0x500
#define NRF_UART_TXD               0x51C

static int nrf_uart_read(void *user_data, uint32_t addr) {
    nrf_uart_state_t *uart = (nrf_uart_state_t *)user_data;
    uint32_t off = addr - NRF_UART0_BASE;
    switch (off) {
        case NRF_UART_EVENTS_TXDRDY: return (int)uart->txdrdy;
        case NRF_UART_ENABLE:        return (int)uart->enable;
        default: return 0;
    }
}

static void nrf_uart_write(void *user_data, uint32_t addr, uint32_t value) {
    nrf_uart_state_t *uart = (nrf_uart_state_t *)user_data;
    uint32_t off = addr - NRF_UART0_BASE;
    switch (off) {
        case NRF_UART_TXD:
            /* Forward to console callback if installed. Set TXDRDY so
             * the firmware's spin-loop exits on the next read. */
            if (uart->tx_cb)
                uart->tx_cb(uart->tx_user, (uint8_t)(value & 0xFF));
            uart->txdrdy = 1;
            break;
        case NRF_UART_EVENTS_TXDRDY:
            /* Firmware writes 0 to ack the event after consuming it. */
            uart->txdrdy = value & 1;
            break;
        case NRF_UART_ENABLE:
            uart->enable = value;
            break;
        default: break;
    }
}

/* ============================================================
 * SoC lifecycle
 * ============================================================ */

static void nrf52840_soc_init(arm_platform_t *plat) {
    nrf52840_soc_t *soc = (nrf52840_soc_t *)calloc(1, sizeof(*soc));
    plat->soc = soc;

    /* Minimum host vtable. */
    plat->host.cpu         = &plat->cpu;
    plat->host.gpio        = NULL;
    plat->host.now_ns      = nrf_host_now_ns;
    plat->host.schedule_ns = nrf_host_schedule_ns;
    plat->host.cancel      = nrf_host_cancel;

    /* CLOCK and UART register windows. */
    arm_register_io(&plat->cpu, NRF_CLOCK_BASE, NRF_CLOCK_SIZE,
                    nrf_clock_read, nrf_clock_write, &soc->clock);
    arm_register_io(&plat->cpu, NRF_UART0_BASE, NRF_UART0_SIZE,
                    nrf_uart_read, nrf_uart_write, &soc->uart0);
}

static void nrf52840_soc_destroy(arm_platform_t *plat) {
    free(plat->soc);
    plat->soc = NULL;
}

static void nrf52840_soc_set_console(arm_platform_t *plat,
                                      arm_uart_tx_callback cb, void *user_data) {
    nrf52840_soc_t *soc = (nrf52840_soc_t *)plat->soc;
    soc->uart0.tx_cb   = cb;
    soc->uart0.tx_user = user_data;
}

const arm_soc_ops_t nrf52840_soc_ops = {
    .name        = "nrf52840",
    .init        = nrf52840_soc_init,
    .destroy     = nrf52840_soc_destroy,
    .set_console = nrf52840_soc_set_console,
};
