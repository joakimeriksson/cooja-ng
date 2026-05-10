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
#include "arm_nvic.h"
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
 * RTC0 (0x4000B000)
 *
 * 24-bit counter clocked from LFCLK (32 768 Hz) via 12-bit PRESCALER.
 * Real tick period = (PRESCALER + 1) / 32 768 seconds.
 *
 * Contiki's clock_init enables TICK interrupts to drive the etimer
 * subsystem. Our model schedules a recurring TICK event on the ARM
 * event queue: when it fires we latch EVENTS_TICK and (if INTENSET.TICK
 * is set) raise the RTC0 NVIC IRQ. The CPU's existing WFI handling
 * advances simulated time to the next event automatically, so
 * `wfi`-based idle loops resume on the tick.
 * ============================================================ */
#define NRF_RTC0_BASE          0x4000B000u
#define NRF_RTC0_SIZE          0x1000u
#define NRF_RTC0_IRQ           11
#define NRF_LFCLK_HZ           32768u

#define RTC_TASKS_START        0x000
#define RTC_TASKS_STOP         0x004
#define RTC_TASKS_CLEAR        0x008
#define RTC_TASKS_TRIGOVRFLW   0x00C
#define RTC_EVENTS_TICK        0x100
#define RTC_EVENTS_OVRFLW      0x104
#define RTC_EVENTS_COMPARE_0   0x140
#define RTC_INTENSET           0x304
#define RTC_INTENCLR           0x308
#define RTC_EVTENSET           0x340
#define RTC_EVTENCLR           0x344
#define RTC_COUNTER            0x504
#define RTC_PRESCALER          0x508
#define RTC_CC_0               0x540

#define RTC_INT_TICK           (1u << 0)
#define RTC_INT_OVRFLW         (1u << 1)
#define RTC_INT_COMPARE_0      (1u << 16)

static uint32_t rtc_compute_counter(const nrf_rtc_state_t *rtc, int64_t now_cycles) {
    if (!rtc->running || rtc->tick_period_cycles == 0)
        return rtc->counter_at_anchor;
    int64_t elapsed = now_cycles - rtc->counter_anchor_cycles;
    if (elapsed < 0) elapsed = 0;
    uint64_t ticks = (uint64_t)elapsed / rtc->tick_period_cycles;
    return (rtc->counter_at_anchor + (uint32_t)ticks) & 0xFFFFFFu;
}

static uint32_t rtc_period_for_prescaler(uint32_t prescaler, uint32_t cpu_freq_hz) {
    /* tick_rate = LFCLK / (prescaler+1).  cycles_per_tick = cpu_freq / tick_rate. */
    uint64_t denom = (uint64_t)NRF_LFCLK_HZ;
    uint64_t num   = (uint64_t)cpu_freq_hz * (uint64_t)(prescaler + 1);
    if (denom == 0) return 1;
    uint64_t period = num / denom;
    if (period == 0) period = 1;
    return (uint32_t)period;
}

static void rtc_tick_event_cb(void *user_data, cpu_event_t *event) {
    nrf52840_soc_t *soc = (nrf52840_soc_t *)user_data;
    nrf_rtc_state_t *rtc = &soc->rtc0;
    arm_platform_t *plat = soc->plat;
    if (!plat || !rtc->running) return;

    /* Counter advances by one tick each event firing. Re-anchor so reads
     * stay consistent without divisions for callers that read COUNTER
     * between ticks. */
    rtc->counter_at_anchor = (rtc->counter_at_anchor + 1) & 0xFFFFFFu;
    rtc->counter_anchor_cycles = event->fire_cycle;

    rtc->evt_tick = 1;
    if (rtc->counter_at_anchor == 0) rtc->evt_ovrflw = 1;

    /* Compare events */
    for (int i = 0; i < 4; i++) {
        if (rtc->cc[i] == rtc->counter_at_anchor)
            rtc->evt_compare[i] = 1;
    }

    /* Raise IRQ if any enabled event is now latched. */
    bool fire = false;
    if ((rtc->intenset & RTC_INT_TICK)   && rtc->evt_tick)   fire = true;
    if ((rtc->intenset & RTC_INT_OVRFLW) && rtc->evt_ovrflw) fire = true;
    for (int i = 0; i < 4; i++)
        if ((rtc->intenset & (RTC_INT_COMPARE_0 << i)) && rtc->evt_compare[i])
            fire = true;
    if (fire && plat->cpu.nvic)
        arm_nvic_set_pending((arm_nvic_t *)plat->cpu.nvic, rtc->irq_num);

    /* Reschedule next tick. */
    int64_t next = event->fire_cycle + (int64_t)rtc->tick_period_cycles;
    arm_schedule_event(&plat->cpu, (arm_event_t *)rtc->tick_event, next);
}

static void rtc_start(nrf_rtc_state_t *rtc, arm_platform_t *plat) {
    if (rtc->running) return;
    rtc->running = true;
    rtc->tick_period_cycles =
        rtc_period_for_prescaler(rtc->prescaler, plat->cpu.cpu_freq_hz);
    rtc->counter_anchor_cycles = (int64_t)plat->cpu.cycles;
    arm_schedule_event(&plat->cpu, (arm_event_t *)rtc->tick_event,
                       (int64_t)plat->cpu.cycles + (int64_t)rtc->tick_period_cycles);
}

static void rtc_stop(nrf_rtc_state_t *rtc, arm_platform_t *plat) {
    if (!rtc->running) return;
    /* Snapshot current counter so further reads return the right value. */
    rtc->counter_at_anchor = rtc_compute_counter(rtc, (int64_t)plat->cpu.cycles);
    rtc->counter_anchor_cycles = (int64_t)plat->cpu.cycles;
    rtc->running = false;
    arm_cancel_event(&plat->cpu, (arm_event_t *)rtc->tick_event);
}

static int nrf_rtc_read(void *user_data, uint32_t addr) {
    nrf52840_soc_t *soc = (nrf52840_soc_t *)user_data;
    nrf_rtc_state_t *rtc = &soc->rtc0;
    arm_platform_t *plat = soc->plat;
    uint32_t off = addr - NRF_RTC0_BASE;
    switch (off) {
        case RTC_EVENTS_TICK:    return (int)rtc->evt_tick;
        case RTC_EVENTS_OVRFLW:  return (int)rtc->evt_ovrflw;
        case RTC_EVENTS_COMPARE_0:
        case RTC_EVENTS_COMPARE_0 + 4:
        case RTC_EVENTS_COMPARE_0 + 8:
        case RTC_EVENTS_COMPARE_0 + 12:
            return (int)rtc->evt_compare[(off - RTC_EVENTS_COMPARE_0) / 4];
        case RTC_INTENSET: case RTC_INTENCLR: return (int)rtc->intenset;
        case RTC_EVTENSET: case RTC_EVTENCLR: return (int)rtc->evtenset;
        case RTC_COUNTER:
            return (int)rtc_compute_counter(rtc, (int64_t)plat->cpu.cycles);
        case RTC_PRESCALER: return (int)rtc->prescaler;
        case RTC_CC_0:      case RTC_CC_0 + 4:
        case RTC_CC_0 + 8:  case RTC_CC_0 + 12:
            return (int)rtc->cc[(off - RTC_CC_0) / 4];
        default: return 0;
    }
}

static void nrf_rtc_write(void *user_data, uint32_t addr, uint32_t value) {
    nrf52840_soc_t *soc = (nrf52840_soc_t *)user_data;
    nrf_rtc_state_t *rtc = &soc->rtc0;
    arm_platform_t *plat = soc->plat;
    uint32_t off = addr - NRF_RTC0_BASE;
    switch (off) {
        case RTC_TASKS_START: if (value == 1) rtc_start(rtc, plat); break;
        case RTC_TASKS_STOP:  if (value == 1) rtc_stop(rtc, plat);  break;
        case RTC_TASKS_CLEAR:
            if (value == 1) {
                rtc->counter_at_anchor = 0;
                rtc->counter_anchor_cycles = (int64_t)plat->cpu.cycles;
            }
            break;
        case RTC_TASKS_TRIGOVRFLW:
            if (value == 1) {
                rtc->counter_at_anchor = 0xFFFFF0;
                rtc->counter_anchor_cycles = (int64_t)plat->cpu.cycles;
            }
            break;
        case RTC_EVENTS_TICK:    rtc->evt_tick = value & 1; break;
        case RTC_EVENTS_OVRFLW:  rtc->evt_ovrflw = value & 1; break;
        case RTC_EVENTS_COMPARE_0:
        case RTC_EVENTS_COMPARE_0 + 4:
        case RTC_EVENTS_COMPARE_0 + 8:
        case RTC_EVENTS_COMPARE_0 + 12:
            rtc->evt_compare[(off - RTC_EVENTS_COMPARE_0) / 4] = value & 1;
            break;
        case RTC_INTENSET: rtc->intenset |= value; break;
        case RTC_INTENCLR: rtc->intenset &= ~value; break;
        case RTC_EVTENSET: rtc->evtenset |= value; break;
        case RTC_EVTENCLR: rtc->evtenset &= ~value; break;
        case RTC_PRESCALER:
            rtc->prescaler = value & 0xFFF;
            if (rtc->running)
                rtc->tick_period_cycles =
                    rtc_period_for_prescaler(rtc->prescaler, plat->cpu.cpu_freq_hz);
            break;
        case RTC_CC_0:      case RTC_CC_0 + 4:
        case RTC_CC_0 + 8:  case RTC_CC_0 + 12:
            rtc->cc[(off - RTC_CC_0) / 4] = value & 0xFFFFFFu;
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
    soc->plat = plat;

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

    /* RTC0 — Contiki's clock source.  Allocate the recurring TICK
     * event and pre-fill its callback / user_data; rtc_start arms it. */
    soc->rtc0.irq_num = NRF_RTC0_IRQ;
    cpu_event_t *tev = (cpu_event_t *)calloc(1, sizeof(*tev));
    tev->callback  = rtc_tick_event_cb;
    tev->user_data = soc;
    soc->rtc0.tick_event = tev;
    soc->rtc0.soc = soc;
    arm_register_io(&plat->cpu, NRF_RTC0_BASE, NRF_RTC0_SIZE,
                    nrf_rtc_read, nrf_rtc_write, soc);
}

static void nrf52840_soc_destroy(arm_platform_t *plat) {
    nrf52840_soc_t *soc = (nrf52840_soc_t *)plat->soc;
    if (soc) {
        if (soc->rtc0.tick_event) {
            arm_cancel_event(&plat->cpu, (arm_event_t *)soc->rtc0.tick_event);
            free(soc->rtc0.tick_event);
        }
        free(soc);
    }
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
