/*
 * nRF52840 SoC peripheral model.
 *
 * Runs real Contiki-NG nrf52840 firmware AND boots Zephyr OS (the
 * nrf52840dk hello_world prints over UART).  Modelled peripherals:
 *
 *   - CLOCK (0x40000000): HFCLK/LFCLK start-task ↔ event handshake, plus
 *     the HFCLKSTAT/LFCLKSTAT/LFCLKSRC status registers Zephyr's
 *     clock_control_nrf spin-waits on (lfclk_spinwait).
 *   - UART0 legacy window (0x40002000): TXD + EVENTS_TXDRDY (the non-DMA
 *     driver; Contiki uses it natively, Zephyr via a `nordic,nrf-uart`
 *     devicetree overlay — the UARTE EasyDMA path is not modelled).
 *   - RTC0 (0x4000B000, Contiki clock) + RTC1 (0x40011000, Zephyr's
 *     nrf_rtc_timer system clock): counter + COMPARE + TICK/OVRFLW + IRQ.
 *   - RADIO (0x40001000): 802.15.4 TX/RX state machine + EasyDMA frames.
 *   - TIMER0..4, RNG, FICR.
 *
 * Still unmodelled (firmware that needs them will trap; ARM_MMIO_TRACE=1
 * surfaces the first access to each missing peripheral page): GPIO/GPIOTE
 * (writes ignored), NVMC, PPI, SPI/TWI.
 */
#include "nrf52840_soc.h"
#include "arm_cpu.h"
#include "arm_nvic.h"
#include "ieee_802154.h"
#include "nrf_radio_common.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

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
/* Status / source registers — Zephyr's clock_control_nrf spin-waits on these
 * (lfclk_spinwait polls LFCLKSTAT.STATE); Contiki only used the events. */
#define NRF_CLOCK_HFCLKRUN         0x408
#define NRF_CLOCK_HFCLKSTAT        0x40C
#define NRF_CLOCK_LFCLKRUN         0x414
#define NRF_CLOCK_LFCLKSTAT        0x418
#define NRF_CLOCK_LFCLKSRC         0x518
#define NRF_CLOCK_STAT_STATE       (1u << 16)   /* "clock is running" */

static int nrf_clock_read(void *user_data, uint32_t addr) {
    nrf_clock_state_t *clock = (nrf_clock_state_t *)user_data;
    uint32_t off = addr - NRF_CLOCK_BASE;
    switch (off) {
        case NRF_CLOCK_EVENTS_HFCLKSTARTED: return (int)clock->hfclkstarted;
        case NRF_CLOCK_EVENTS_LFCLKSTARTED: return (int)clock->lfclkstarted;
        case NRF_CLOCK_HFCLKRUN:  return clock->hfclkstarted ? 1 : 0;
        case NRF_CLOCK_HFCLKSTAT:
            /* SRC=XTAL(1) + STATE=running once started. */
            return clock->hfclkstarted ? (int)(NRF_CLOCK_STAT_STATE | 1u) : 0;
        case NRF_CLOCK_LFCLKRUN:  return clock->lfclkstarted ? 1 : 0;
        case NRF_CLOCK_LFCLKSTAT:
            /* Report the configured source as running once started — what
             * Zephyr's lfclk_spinwait waits for (STATE bit + SRC match). */
            return (int)((clock->lfclksrc & 3u) |
                         (clock->lfclkstarted ? NRF_CLOCK_STAT_STATE : 0u));
        case NRF_CLOCK_LFCLKSRC:  return (int)clock->lfclksrc;
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
        case NRF_CLOCK_LFCLKSRC:
            clock->lfclksrc = value;
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
/* UARTE EasyDMA TX (shares the 0x40002000 page; stock Zephyr console path). */
#define NRF_UARTE_IRQ              2
#define NRF_UARTE_TASKS_STARTTX    0x008
#define NRF_UARTE_TASKS_STOPTX     0x00C
#define NRF_UARTE_EVENTS_ENDTX     0x120
#define NRF_UARTE_EVENTS_TXSTARTED 0x150
#define NRF_UARTE_EVENTS_TXSTOPPED 0x158
#define NRF_UARTE_INTENSET         0x304
#define NRF_UARTE_INTENCLR         0x308
#define NRF_UARTE_TXD_PTR          0x544
#define NRF_UARTE_TXD_MAXCNT       0x548
#define NRF_UARTE_TXD_AMOUNT       0x54C
#define NRF_UARTE_INT_ENDTX        (1u << 8)

static int nrf_uart_read(void *user_data, uint32_t addr) {
    nrf_uart_state_t *uart = (nrf_uart_state_t *)user_data;
    uint32_t off = addr - NRF_UART0_BASE;
    if (getenv("NRF_UART_TRACE")) fprintf(stderr, "[uartR] 0x%03x\n", off);
    switch (off) {
        case NRF_UART_EVENTS_TXDRDY:     return (int)uart->txdrdy;
        case NRF_UART_ENABLE:            return (int)uart->enable;
        /* UARTE EasyDMA TX read-backs. */
        case NRF_UARTE_EVENTS_ENDTX:     return (int)uart->endtx;
        case NRF_UARTE_EVENTS_TXSTARTED: return (int)uart->txstarted;
        case NRF_UARTE_EVENTS_TXSTOPPED: return (int)uart->txstopped;
        case NRF_UARTE_TXD_PTR:          return (int)uart->txd_ptr;
        case NRF_UARTE_TXD_MAXCNT:       return (int)uart->txd_maxcnt;
        case NRF_UARTE_TXD_AMOUNT:       return (int)uart->txd_amount;
        case NRF_UARTE_INTENSET:
        case NRF_UARTE_INTENCLR:         return (int)uart->intenset;
        default: return 0;
    }
}

/* UARTE TASKS_STARTTX: DMA `maxcnt` bytes from RAM at `txd_ptr` straight out
 * the console, then latch ENDTX/TXSTARTED (and raise the IRQ if enabled).
 * Instant transfer — no per-byte baud timing, same lenience as the legacy
 * path. */
static void uarte_start_tx(nrf_uart_state_t *uart) {
    nrf52840_soc_t *soc = (nrf52840_soc_t *)uart->soc;
    arm_cpu_t *cpu = &soc->plat->cpu;
    uint32_t n = uart->txd_maxcnt;
    if (n > 4096) n = 4096;                 /* sanity bound */
    for (uint32_t i = 0; i < n; i++) {
        uint8_t b = arm_read8(cpu, uart->txd_ptr + i);
        if (uart->tx_cb) uart->tx_cb(uart->tx_user, b);
    }
    uart->txd_amount = n;
    uart->txstarted  = 1;
    uart->endtx      = 1;                    /* poll_out spins on this */
    /* csim's transfer is instantaneous, so TX is inactive the moment STARTTX
     * returns — what the driver's ENDTX→STOPTX PPI link would yield (csim
     * doesn't model PPI).  The init/poll path then exits its TXSTOPPED spin. */
    uart->txstopped  = 1;
    if ((uart->intenset & NRF_UARTE_INT_ENDTX) && soc->plat->cpu.nvic)
        arm_nvic_set_pending((arm_nvic_t *)soc->plat->cpu.nvic, uart->irq_num);
}

static void nrf_uart_write(void *user_data, uint32_t addr, uint32_t value) {
    nrf_uart_state_t *uart = (nrf_uart_state_t *)user_data;
    uint32_t off = addr - NRF_UART0_BASE;
    if (getenv("NRF_UART_TRACE")) fprintf(stderr, "[uartW] 0x%03x = 0x%08x\n", off, value);
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
        /* UARTE EasyDMA TX path. */
        case NRF_UARTE_TASKS_STARTTX:  if (value == 1) uarte_start_tx(uart); break;
        case NRF_UARTE_TASKS_STOPTX:   if (value == 1) uart->txstopped = 1; break;
        case NRF_UARTE_TXD_PTR:        uart->txd_ptr    = value; break;
        case NRF_UARTE_TXD_MAXCNT:     uart->txd_maxcnt = value; break;
        case NRF_UARTE_EVENTS_ENDTX:   uart->endtx      = value & 1; break;
        case NRF_UARTE_EVENTS_TXSTARTED: uart->txstarted = value & 1; break;
        case NRF_UARTE_EVENTS_TXSTOPPED: uart->txstopped = value & 1; break;
        case NRF_UARTE_INTENSET:       uart->intenset |= value; break;
        case NRF_UARTE_INTENCLR:       uart->intenset &= ~value; break;
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
/* RTC1 — same peripheral, used by Zephyr's nrf_rtc_timer as the system clock. */
#define NRF_RTC1_BASE          0x40011000u
#define NRF_RTC1_SIZE          0x1000u
#define NRF_RTC1_IRQ           17
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

/* Tickless scheduling: instead of one event per LFCLK tick (32768/s — which
 * makes idle-heavy firmware like Zephyr crawl), schedule a SINGLE event at the
 * next moment something enabled is due — the nearest of TICK (only if its
 * interrupt is on), OVRFLW, or an enabled COMPARE.  COUNTER is always derived
 * from elapsed cycles (rtc_compute_counter), so reads stay exact between
 * wake-ups.  Re-armed whenever CC / INTEN / the counter base changes. */
static void rtc_reschedule(nrf_rtc_state_t *rtc, arm_platform_t *plat) {
    if (!rtc->running || rtc->tick_period_cycles == 0) {
        arm_cancel_event(&plat->cpu, (arm_event_t *)rtc->tick_event);
        return;
    }
    int64_t now_cycles = (int64_t)plat->cpu.cycles;
    uint32_t now = rtc_compute_counter(rtc, now_cycles);
    uint64_t best = UINT64_MAX;                 /* ticks from `now` */
    if (rtc->intenset & RTC_INT_TICK) best = 1;
    if (rtc->intenset & RTC_INT_OVRFLW) {
        uint64_t d = 0x1000000u - now;          /* ticks to wrap (now < 2^24) */
        if (d < best) best = d;
    }
    for (int i = 0; i < 4; i++) {
        if (rtc->intenset & (RTC_INT_COMPARE_0 << i)) {
            uint32_t d = (rtc->cc[i] - now) & 0xFFFFFFu;
            if (d == 0) d = 0x1000000u;         /* equal now → next match a wrap away */
            if (d < best) best = d;
        }
    }
    if (best == UINT64_MAX) {                    /* nothing enabled to wake for */
        arm_cancel_event(&plat->cpu, (arm_event_t *)rtc->tick_event);
        return;
    }
    int64_t period = (int64_t)rtc->tick_period_cycles;
    int64_t into_tick = (now_cycles - rtc->counter_anchor_cycles) % period;
    int64_t fire = now_cycles - into_tick + (int64_t)best * period;
    arm_schedule_event(&plat->cpu, (arm_event_t *)rtc->tick_event, fire);
}

static void rtc_event_cb(void *user_data, cpu_event_t *event) {
    nrf_rtc_state_t *rtc = (nrf_rtc_state_t *)user_data;
    nrf52840_soc_t *soc = (nrf52840_soc_t *)rtc->soc;
    arm_platform_t *plat = soc->plat;
    if (!plat || !rtc->running) return;

    uint32_t cnt = rtc_compute_counter(rtc, event->fire_cycle);
    bool fire = false;
    if (rtc->intenset & RTC_INT_TICK)              { rtc->evt_tick = 1;   fire = true; }
    if ((rtc->intenset & RTC_INT_OVRFLW) && cnt==0) { rtc->evt_ovrflw = 1; fire = true; }
    for (int i = 0; i < 4; i++)
        if ((rtc->intenset & (RTC_INT_COMPARE_0 << i)) && rtc->cc[i] == cnt) {
            rtc->evt_compare[i] = 1; fire = true;
        }
    if (fire && plat->cpu.nvic)
        arm_nvic_set_pending((arm_nvic_t *)plat->cpu.nvic, rtc->irq_num);

    rtc_reschedule(rtc, plat);
}

static void rtc_start(nrf_rtc_state_t *rtc, arm_platform_t *plat) {
    if (rtc->running) return;
    rtc->running = true;
    rtc->tick_period_cycles =
        rtc_period_for_prescaler(rtc->prescaler, plat->cpu.cpu_freq_hz);
    rtc->counter_anchor_cycles = (int64_t)plat->cpu.cycles;
    rtc_reschedule(rtc, plat);
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
    nrf_rtc_state_t *rtc = (nrf_rtc_state_t *)user_data;
    arm_platform_t *plat = ((nrf52840_soc_t *)rtc->soc)->plat;
    uint32_t off = addr - rtc->base;
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
    nrf_rtc_state_t *rtc = (nrf_rtc_state_t *)user_data;
    arm_platform_t *plat = ((nrf52840_soc_t *)rtc->soc)->plat;
    uint32_t off = addr - rtc->base;
    /* Writes that change WHEN the next event is due must re-arm the tickless
     * schedule (CC / INTEN / counter base / prescaler). */
    bool resched = false;
    switch (off) {
        case RTC_TASKS_START: if (value == 1) rtc_start(rtc, plat); break;
        case RTC_TASKS_STOP:  if (value == 1) rtc_stop(rtc, plat);  break;
        case RTC_TASKS_CLEAR:
            if (value == 1) {
                rtc->counter_at_anchor = 0;
                rtc->counter_anchor_cycles = (int64_t)plat->cpu.cycles;
                resched = true;
            }
            break;
        case RTC_TASKS_TRIGOVRFLW:
            if (value == 1) {
                rtc->counter_at_anchor = 0xFFFFF0;
                rtc->counter_anchor_cycles = (int64_t)plat->cpu.cycles;
                resched = true;
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
        case RTC_INTENSET: rtc->intenset |= value; resched = true; break;
        case RTC_INTENCLR: rtc->intenset &= ~value; resched = true; break;
        case RTC_EVTENSET: rtc->evtenset |= value; break;
        case RTC_EVTENCLR: rtc->evtenset &= ~value; break;
        case RTC_PRESCALER:
            rtc->prescaler = value & 0xFFF;
            if (rtc->running)
                rtc->tick_period_cycles =
                    rtc_period_for_prescaler(rtc->prescaler, plat->cpu.cpu_freq_hz);
            resched = true;
            break;
        case RTC_CC_0:      case RTC_CC_0 + 4:
        case RTC_CC_0 + 8:  case RTC_CC_0 + 12:
            rtc->cc[(off - RTC_CC_0) / 4] = value & 0xFFFFFFu;
            resched = true;
            break;
        default: break;
    }
    if (resched && rtc->running) rtc_reschedule(rtc, plat);
}

/* ============================================================
 * RADIO (0x40001000) — first cut
 *
 * Models the state machine + tasks/events/SHORTS that the Contiki
 * nrf52840-ieee driver busy-waits on, but does NOT yet move frame
 * data through PACKETPTR. TX bytes are dropped; RX never fires END
 * from external data. That keeps the firmware's TX/RX paths
 * progressing without spin-locking; multinode delivery comes when we
 * wire RX into radio_medium.
 * ============================================================ */
#define NRF_RADIO_BASE   0x40001000u
#define NRF_RADIO_SIZE   0x1000u
#define NRF_RADIO_IRQ    1

/* Tasks */
#define RADIO_TASKS_TXEN          0x000
#define RADIO_TASKS_RXEN          0x004
#define RADIO_TASKS_START         0x008
#define RADIO_TASKS_STOP          0x00C
#define RADIO_TASKS_DISABLE       0x010
#define RADIO_TASKS_RSSISTART     0x014
#define RADIO_TASKS_RSSISTOP      0x018
#define RADIO_TASKS_BCSTART       0x01C
#define RADIO_TASKS_BCSTOP        0x020
#define RADIO_TASKS_EDSTART       0x024
#define RADIO_TASKS_EDSTOP        0x028
#define RADIO_TASKS_CCASTART      0x02C
#define RADIO_TASKS_CCASTOP       0x030

/* Event offsets */
#define RADIO_EVENTS_READY        0x100
#define RADIO_EVENTS_ADDRESS      0x104
#define RADIO_EVENTS_PAYLOAD      0x108
#define RADIO_EVENTS_END          0x10C
#define RADIO_EVENTS_DISABLED     0x110
#define RADIO_EVENTS_DEVMATCH     0x114
#define RADIO_EVENTS_DEVMISS      0x118
#define RADIO_EVENTS_RSSIEND      0x11C
#define RADIO_EVENTS_BCMATCH      0x128
#define RADIO_EVENTS_CRCOK        0x130
#define RADIO_EVENTS_CRCERROR     0x134
#define RADIO_EVENTS_FRAMESTART   0x138
#define RADIO_EVENTS_EDEND        0x13C
#define RADIO_EVENTS_EDSTOPPED    0x140
#define RADIO_EVENTS_CCAIDLE      0x144
#define RADIO_EVENTS_CCABUSY      0x148
#define RADIO_EVENTS_CCASTOPPED   0x14C
#define RADIO_EVENTS_RATEBOOST    0x150
#define RADIO_EVENTS_TXREADY      0x154
#define RADIO_EVENTS_RXREADY      0x158
#define RADIO_EVENTS_MHRMATCH     0x15C
#define RADIO_EVENTS_PHYEND       0x16C

/* Other registers */
#define RADIO_SHORTS              0x200
#define RADIO_INTENSET            0x304
#define RADIO_INTENCLR            0x308
#define RADIO_CRCSTATUS           0x400
#define RADIO_RXMATCH             0x408
#define RADIO_RXCRC               0x40C
#define RADIO_DAI                 0x410
#define RADIO_PDUSTAT             0x414
#define RADIO_PACKETPTR           0x504
#define RADIO_FREQUENCY           0x508
#define RADIO_TXPOWER             0x50C
#define RADIO_MODE                0x510
#define RADIO_PCNF0               0x514
#define RADIO_PCNF1               0x518
#define RADIO_BASE0               0x51C
#define RADIO_BASE1               0x520
#define RADIO_PREFIX0             0x524
#define RADIO_PREFIX1             0x528
#define RADIO_TXADDRESS           0x52C
#define RADIO_RXADDRESSES         0x530
#define RADIO_CRCCNF              0x534
#define RADIO_CRCPOLY             0x538
#define RADIO_CRCINIT             0x53C
#define RADIO_STATE               0x550
#define RADIO_DATAWHITEIV         0x554
#define RADIO_BCC                 0x560
#define RADIO_DAB_0               0x600
#define RADIO_DAP_0               0x620
#define RADIO_DACNF               0x640
#define RADIO_MHRMATCHCONF        0x644
#define RADIO_MHRMATCHMAS         0x648
#define RADIO_MODECNF0            0x650
#define RADIO_SFD                 0x660
#define RADIO_EDCNT               0x664
#define RADIO_EDSAMPLE            0x668
#define RADIO_CCACTRL             0x66C
#define RADIO_POWER               0xFFC

/* SHORTS bit positions (subset relevant to 802.15.4 driver) */
#define SHORT_READY_START         (1u << 0)
#define SHORT_END_DISABLE         (1u << 1)
#define SHORT_DISABLED_TXEN       (1u << 2)
#define SHORT_DISABLED_RXEN       (1u << 3)
#define SHORT_ADDRESS_RSSISTART   (1u << 4)
#define SHORT_END_START           (1u << 5)
#define SHORT_ADDRESS_BCSTART     (1u << 6)
#define SHORT_DISABLED_RSSISTOP   (1u << 8)
#define SHORT_RXREADY_CCASTART    (1u << 11)
#define SHORT_CCAIDLE_TXEN        (1u << 12)
#define SHORT_CCABUSY_DISABLE     (1u << 13)
#define SHORT_FRAMESTART_BCSTART  (1u << 14)
#define SHORT_READY_EDSTART       (1u << 15)
#define SHORT_EDEND_DISABLE       (1u << 16)
#define SHORT_CCAIDLE_STOP        (1u << 17)
#define SHORT_TXREADY_START       (1u << 18)
#define SHORT_RXREADY_START       (1u << 19)
#define SHORT_PHYEND_DISABLE      (1u << 20)
#define SHORT_PHYEND_START        (1u << 21)

/* INTENSET bit positions (one per event) — per nRF52840 PS v1.7 §6.20.15.21. */
#define RADIO_INT_READY        (1u << 0)
#define RADIO_INT_ADDRESS      (1u << 1)
#define RADIO_INT_PAYLOAD      (1u << 2)
#define RADIO_INT_END          (1u << 3)
#define RADIO_INT_DISABLED     (1u << 4)
#define RADIO_INT_BCMATCH      (1u << 10)
#define RADIO_INT_CRCOK        (1u << 12)
#define RADIO_INT_CRCERROR     (1u << 13)
#define RADIO_INT_FRAMESTART   (1u << 14)
#define RADIO_INT_TXREADY      (1u << 21)
#define RADIO_INT_RXREADY      (1u << 22)
#define RADIO_INT_MHRMATCH     (1u << 23)
#define RADIO_INT_PHYEND       (1u << 27)

/* IEEE 802.15.4 on-air framing — same convention used by cc2420 /
 * cc2538_rfcore in this tree, so radio_medium can route bytes between
 * nrf52840 and cc2538dk nodes if anyone tries it.
 * Constants (PREAMBLE_BYTE/LEN/SFD) and the CCITT-16 step now live in
 * include/common/ieee_802154.h. */

/* RX byte parser phase aliases (kept so existing case labels don't
 * have to churn; underlying values come from the shared enum). */
#define NRF_RX_WAIT_PREAMBLE IEEE802154_RX_WAIT_PREAMBLE
#define NRF_RX_WAIT_SFD      IEEE802154_RX_WAIT_SFD
#define NRF_RX_READ_PHR      IEEE802154_RX_READ_PHR
#define NRF_RX_READ_PAYLOAD  IEEE802154_RX_READ_PAYLOAD

/* Forward decls */
static void radio_trigger_task(nrf52840_soc_t *soc, uint32_t off);
static void radio_set_state(nrf52840_soc_t *soc, uint32_t new_state);
static void radio_event(nrf52840_soc_t *soc, uint32_t *evt_field, uint32_t int_mask);
static void radio_emit_tx(nrf52840_soc_t *soc);

#define nrf_crc_add(crc, data) ieee802154_crc_add((crc), (data))

/* Raise IRQ if any of the latched events have INTENSET bit set. */
static void radio_event(nrf52840_soc_t *soc, uint32_t *evt_field, uint32_t int_mask) {
    nrf_radio_state_t *r = &soc->radio;
    *evt_field = 1;
    if ((r->intenset & int_mask) && soc->plat && soc->plat->cpu.nvic)
        arm_nvic_set_pending((arm_nvic_t *)soc->plat->cpu.nvic, r->irq_num);
}

/* Apply SHORTS that fire on entering a state. The driver enables
 * READY_START / TXREADY_START / RXREADY_START so the radio rolls
 * straight from RU to RX/TX without firmware intervention. */
static void radio_apply_shorts_after_event(nrf52840_soc_t *soc, uint32_t event_mask) {
    nrf_radio_state_t *r = &soc->radio;
    if (r->shorts & event_mask) {
        switch (event_mask) {
            case SHORT_READY_START:
            case SHORT_TXREADY_START:
            case SHORT_RXREADY_START:
            case SHORT_PHYEND_START:
            case SHORT_END_START:
                radio_trigger_task(soc, RADIO_TASKS_START);
                break;
            case SHORT_END_DISABLE:
            case SHORT_PHYEND_DISABLE:
            case SHORT_CCABUSY_DISABLE:
            case SHORT_EDEND_DISABLE:
                radio_trigger_task(soc, RADIO_TASKS_DISABLE);
                break;
            case SHORT_DISABLED_TXEN:
                radio_trigger_task(soc, RADIO_TASKS_TXEN);
                break;
            case SHORT_DISABLED_RXEN:
                radio_trigger_task(soc, RADIO_TASKS_RXEN);
                break;
            case SHORT_CCAIDLE_TXEN:
                radio_trigger_task(soc, RADIO_TASKS_TXEN);
                break;
        }
    }
}

static void radio_set_state(nrf52840_soc_t *soc, uint32_t new_state) {
    nrf_radio_state_t *r = &soc->radio;
    r->state = new_state;
    /* Fire entry events. We collapse RU → IDLE in one step (no rampup
     * delay). After firing each event, check SHORTS for chained tasks. */
    switch (new_state) {
        case NRF_RADIO_STATE_RXIDLE:
            radio_event(soc, &r->evt_ready,   RADIO_INT_READY);
            radio_event(soc, &r->evt_rxready, RADIO_INT_RXREADY);
            radio_apply_shorts_after_event(soc, SHORT_READY_START);
            radio_apply_shorts_after_event(soc, SHORT_RXREADY_START);
            break;
        case NRF_RADIO_STATE_TXIDLE:
            radio_event(soc, &r->evt_ready,   RADIO_INT_READY);
            radio_event(soc, &r->evt_txready, RADIO_INT_TXREADY);
            radio_apply_shorts_after_event(soc, SHORT_READY_START);
            radio_apply_shorts_after_event(soc, SHORT_TXREADY_START);
            break;
        case NRF_RADIO_STATE_DISABLED:
            radio_event(soc, &r->evt_disabled, RADIO_INT_DISABLED);
            radio_apply_shorts_after_event(soc, SHORT_DISABLED_TXEN);
            radio_apply_shorts_after_event(soc, SHORT_DISABLED_RXEN);
            break;
        default:
            break;
    }
}

static void radio_trigger_task(nrf52840_soc_t *soc, uint32_t off) {
    nrf_radio_state_t *r = &soc->radio;
    switch (off) {
        case RADIO_TASKS_TXEN:
            /* Per nRF52840 PS, TXEN is only legal from DISABLED, but
             * Contiki's nrf52840-ieee driver fires TXEN directly from
             * RXIDLE (after STOP) and depends on hardware to handle the
             * implicit transition. Mirror that lenience. */
            if (r->state != NRF_RADIO_STATE_TX &&
                r->state != NRF_RADIO_STATE_TXRU)
                radio_set_state(soc, NRF_RADIO_STATE_TXIDLE);
            break;
        case RADIO_TASKS_RXEN:
            if (r->state != NRF_RADIO_STATE_RX &&
                r->state != NRF_RADIO_STATE_RXRU)
                radio_set_state(soc, NRF_RADIO_STATE_RXIDLE);
            break;
        case RADIO_TASKS_START:
            /* TXIDLE → TX → (instantly transmit) → TXIDLE.
             * RXIDLE → RX (stay there until a frame arrives). */
            if (r->state == NRF_RADIO_STATE_TXIDLE) {
                /* Walk PACKETPTR, build the on-air frame, push bytes
                 * out via the TX listener (multinode harness installs
                 * one). Then fire post-TX events and return to TXIDLE. */
                /* Drop any buffered RX bytes — they pre-date our own TX
                 * and are stale. */
                r->rx_incoming_len = 0;
                r->state = NRF_RADIO_STATE_TX;
                radio_emit_tx(soc);
                radio_event(soc, &r->evt_address,    RADIO_INT_ADDRESS);
                radio_event(soc, &r->evt_framestart, RADIO_INT_FRAMESTART);
                radio_event(soc, &r->evt_payload,    RADIO_INT_PAYLOAD);
                radio_event(soc, &r->evt_end,        RADIO_INT_END);
                radio_event(soc, &r->evt_phyend,     0);
                r->state = NRF_RADIO_STATE_TXIDLE;
                radio_apply_shorts_after_event(soc, SHORT_END_DISABLE);
                radio_apply_shorts_after_event(soc, SHORT_PHYEND_DISABLE);
                radio_apply_shorts_after_event(soc, SHORT_END_START);
                radio_apply_shorts_after_event(soc, SHORT_PHYEND_START);
            } else if (r->state == NRF_RADIO_STATE_RXIDLE) {
                /* Sit in RX waiting for external bytes (delivered via
                 * nrf_radio_receive_byte). Reset the byte parser. */
                r->state    = NRF_RADIO_STATE_RX;
                r->rx_phase = NRF_RX_WAIT_PREAMBLE;
                /* Drain bytes that arrived while we were not in RX
                 * (auto-ACK landing after our own TX, before driver
                 * finished its TXIDLE → DISABLED → RX state walk). */
                if (r->rx_incoming_len > 0) {
                    int len = r->rx_incoming_len;
                    uint8_t buf[256];
                    memcpy(buf, r->rx_incoming, (size_t)len);
                    r->rx_incoming_len = 0;
                    for (int i = 0; i < len; i++)
                        nrf_radio_receive_byte(soc, buf[i]);
                }
            }
            break;
        case RADIO_TASKS_STOP:
            if (r->state == NRF_RADIO_STATE_RX)  r->state = NRF_RADIO_STATE_RXIDLE;
            if (r->state == NRF_RADIO_STATE_TX)  r->state = NRF_RADIO_STATE_TXIDLE;
            break;
        case RADIO_TASKS_DISABLE:
            radio_set_state(soc, NRF_RADIO_STATE_DISABLED);
            break;
        case RADIO_TASKS_CCASTART:
            /* No interference model — always idle. */
            radio_event(soc, &r->evt_ccaidle, 0);
            radio_apply_shorts_after_event(soc, SHORT_CCAIDLE_TXEN);
            radio_apply_shorts_after_event(soc, SHORT_CCAIDLE_STOP);
            break;
        case RADIO_TASKS_RSSISTART:
            radio_event(soc, &r->evt_rssiend, 0);
            break;
        case RADIO_TASKS_EDSTART:
            radio_event(soc, &r->evt_edend,   0);
            break;
        default:
            break;
    }
}

static int nrf_radio_read(void *user_data, uint32_t addr) {
    nrf52840_soc_t *soc = (nrf52840_soc_t *)user_data;
    nrf_radio_state_t *r = &soc->radio;
    uint32_t off = addr - NRF_RADIO_BASE;
    if (getenv("NRF_RADIO_TRACE")) fprintf(stderr, "[radioR] 0x%03x\n", off);
    switch (off) {
        case RADIO_STATE:        return (int)r->state;
        case RADIO_EVENTS_READY:        return (int)r->evt_ready;
        case RADIO_EVENTS_ADDRESS:      return (int)r->evt_address;
        case RADIO_EVENTS_PAYLOAD:      return (int)r->evt_payload;
        case RADIO_EVENTS_END:          return (int)r->evt_end;
        case RADIO_EVENTS_DISABLED:     return (int)r->evt_disabled;
        case RADIO_EVENTS_DEVMATCH:     return (int)r->evt_devmatch;
        case RADIO_EVENTS_DEVMISS:      return (int)r->evt_devmiss;
        case RADIO_EVENTS_RSSIEND:      return (int)r->evt_rssiend;
        case RADIO_EVENTS_BCMATCH:      return (int)r->evt_bcmatch;
        case RADIO_EVENTS_CRCOK:        return (int)r->evt_crcok;
        case RADIO_EVENTS_CRCERROR:     return (int)r->evt_crcerror;
        case RADIO_EVENTS_FRAMESTART:   return (int)r->evt_framestart;
        case RADIO_EVENTS_EDEND:        return (int)r->evt_edend;
        case RADIO_EVENTS_CCAIDLE:      return (int)r->evt_ccaidle;
        case RADIO_EVENTS_CCABUSY:      return (int)r->evt_ccabusy;
        case RADIO_EVENTS_TXREADY:      return (int)r->evt_txready;
        case RADIO_EVENTS_RXREADY:      return (int)r->evt_rxready;
        case RADIO_EVENTS_MHRMATCH:     return (int)r->evt_mhrmatch;
        case RADIO_EVENTS_PHYEND:       return (int)r->evt_phyend;
        case RADIO_SHORTS:       return (int)r->shorts;
        case RADIO_INTENSET:
        case RADIO_INTENCLR:     return (int)r->intenset;
        case RADIO_CRCSTATUS:    return 1;          /* always OK */
        case RADIO_PACKETPTR:    return (int)r->packetptr;
        case RADIO_FREQUENCY:    return (int)r->frequency;
        case RADIO_TXPOWER:      return (int)r->txpower;
        case RADIO_MODE:         return (int)r->mode;
        case RADIO_PCNF0:        return (int)r->pcnf0;
        case RADIO_PCNF1:        return (int)r->pcnf1;
        case RADIO_BASE0:        return (int)r->base0;
        case RADIO_BASE1:        return (int)r->base1;
        case RADIO_PREFIX0:      return (int)r->prefix0;
        case RADIO_PREFIX1:      return (int)r->prefix1;
        case RADIO_TXADDRESS:    return (int)r->txaddress;
        case RADIO_RXADDRESSES:  return (int)r->rxaddresses;
        case RADIO_CRCCNF:       return (int)r->crccnf;
        case RADIO_CRCPOLY:      return (int)r->crcpoly;
        case RADIO_CRCINIT:      return (int)r->crcinit;
        case RADIO_MODECNF0:     return (int)r->modecnf0;
        case RADIO_CCACTRL:      return (int)r->ccactrl;
        case RADIO_POWER:        return (int)r->power;
        default: return 0;
    }
}

static void nrf_radio_write(void *user_data, uint32_t addr, uint32_t value) {
    nrf52840_soc_t *soc = (nrf52840_soc_t *)user_data;
    nrf_radio_state_t *r = &soc->radio;
    uint32_t off = addr - NRF_RADIO_BASE;
    if (getenv("NRF_RADIO_TRACE")) fprintf(stderr, "[radioW] 0x%03x = 0x%08x\n", off, value);

    /* Tasks: trigger on write of 1 */
    if (off <= RADIO_TASKS_CCASTOP) {
        if (value == 1) radio_trigger_task(soc, off);
        return;
    }
    /* Events: firmware writes 0 to ack. */
    if (off >= RADIO_EVENTS_READY && off <= RADIO_EVENTS_PHYEND) {
        switch (off) {
            case RADIO_EVENTS_READY:        r->evt_ready      = value & 1; break;
            case RADIO_EVENTS_ADDRESS:      r->evt_address    = value & 1; break;
            case RADIO_EVENTS_PAYLOAD:      r->evt_payload    = value & 1; break;
            case RADIO_EVENTS_END:          r->evt_end        = value & 1; break;
            case RADIO_EVENTS_DISABLED:     r->evt_disabled   = value & 1; break;
            case RADIO_EVENTS_DEVMATCH:     r->evt_devmatch   = value & 1; break;
            case RADIO_EVENTS_DEVMISS:      r->evt_devmiss    = value & 1; break;
            case RADIO_EVENTS_RSSIEND:      r->evt_rssiend    = value & 1; break;
            case RADIO_EVENTS_BCMATCH:      r->evt_bcmatch    = value & 1; break;
            case RADIO_EVENTS_CRCOK:        r->evt_crcok      = value & 1; break;
            case RADIO_EVENTS_CRCERROR:     r->evt_crcerror   = value & 1; break;
            case RADIO_EVENTS_FRAMESTART:   r->evt_framestart = value & 1; break;
            case RADIO_EVENTS_EDEND:        r->evt_edend      = value & 1; break;
            case RADIO_EVENTS_CCAIDLE:      r->evt_ccaidle    = value & 1; break;
            case RADIO_EVENTS_CCABUSY:      r->evt_ccabusy    = value & 1; break;
            case RADIO_EVENTS_TXREADY:      r->evt_txready    = value & 1; break;
            case RADIO_EVENTS_RXREADY:      r->evt_rxready    = value & 1; break;
            case RADIO_EVENTS_MHRMATCH:     r->evt_mhrmatch   = value & 1; break;
            case RADIO_EVENTS_PHYEND:       r->evt_phyend     = value & 1; break;
        }
        return;
    }
    switch (off) {
        case RADIO_SHORTS:       r->shorts    = value; break;
        case RADIO_INTENSET:     r->intenset |= value; break;
        case RADIO_INTENCLR:     r->intenset &= ~value; break;
        case RADIO_PACKETPTR:    r->packetptr = value; break;
        case RADIO_FREQUENCY:    r->frequency = value; break;
        case RADIO_TXPOWER:      r->txpower   = value; break;
        case RADIO_MODE:         r->mode      = value; break;
        case RADIO_PCNF0:        r->pcnf0     = value; break;
        case RADIO_PCNF1:        r->pcnf1     = value; break;
        case RADIO_BASE0:        r->base0     = value; break;
        case RADIO_BASE1:        r->base1     = value; break;
        case RADIO_PREFIX0:      r->prefix0   = value; break;
        case RADIO_PREFIX1:      r->prefix1   = value; break;
        case RADIO_TXADDRESS:    r->txaddress = value; break;
        case RADIO_RXADDRESSES:  r->rxaddresses = value; break;
        case RADIO_CRCCNF:       r->crccnf    = value; break;
        case RADIO_CRCPOLY:      r->crcpoly   = value; break;
        case RADIO_CRCINIT:      r->crcinit   = value; break;
        case RADIO_MODECNF0:     r->modecnf0  = value; break;
        case RADIO_CCACTRL:      r->ccactrl   = value; break;
        case RADIO_POWER:        r->power     = value; break;
        default: break;
    }
}

/* Emit the on-air byte sequence (4×preamble + SFD + PHR + payload + CRC)
 * by reading the buffer at PACKETPTR.  Called when the radio enters TX
 * state via TASKS_START.  Body lives in nrf_radio_common — shared with
 * nrf54l15 RADIO. */
static void radio_emit_tx(nrf52840_soc_t *soc) {
    nrf_radio_emit_ieee802154_frame(&soc->plat->cpu, soc->radio.packetptr,
                                     soc->radio_tx_cb, soc->radio_tx_user);
}

void nrf_radio_set_tx_listener(nrf52840_soc_t *soc, nrf_radio_tx_listener_t cb, void *user_data) {
    soc->radio_tx_cb = cb;
    soc->radio_tx_user = user_data;
}

/* Push a single on-air byte into the receiver. Frames bytes through the
 * preamble/SFD/PHR/payload state machine, writes accepted bytes into
 * the PACKETPTR-pointed RAM buffer, and fires FRAMESTART/CRCOK/END
 * events on completion. Called by the multinode harness when the medium
 * delivers a byte to this node. */
void nrf_radio_receive_byte(nrf52840_soc_t *soc, uint8_t byte) {
    nrf_radio_state_t *r = &soc->radio;
    /* Outside RX: buffer the byte for replay on next RX entry.  This
     * catches auto-ACK bytes that arrive 192 µs after our own TX,
     * before the driver has finished the TXIDLE → DISABLED → RXEN →
     * RXIDLE → RX state walk.  Buffer is cleared on TX entry so we
     * never carry stale bytes across a TX cycle. */
    if (r->state != NRF_RADIO_STATE_RX) {
        if (r->rx_incoming_len < (int)sizeof(r->rx_incoming))
            r->rx_incoming[r->rx_incoming_len++] = byte;
        return;
    }
    arm_cpu_t *cpu = &soc->plat->cpu;
    /* Bounds-check PACKETPTR — must point into SRAM. */
    if (r->packetptr < cpu->sram_base || r->packetptr + 128 > cpu->sram_end)
        return;

    switch (r->rx_phase) {
        case NRF_RX_WAIT_PREAMBLE:
            if (byte == IEEE802154_PREAMBLE_BYTE)
                r->rx_phase = NRF_RX_WAIT_SFD;
            break;
        case NRF_RX_WAIT_SFD:
            if (byte == IEEE802154_SFD) {
                r->rx_phase = NRF_RX_READ_PHR;
                radio_event(soc, &r->evt_address,    RADIO_INT_ADDRESS);
                radio_event(soc, &r->evt_framestart, RADIO_INT_FRAMESTART);
            } else if (byte != IEEE802154_PREAMBLE_BYTE) {
                /* Stray byte — restart preamble search. */
                r->rx_phase = NRF_RX_WAIT_PREAMBLE;
            }
            break;
        case NRF_RX_READ_PHR:
            if (byte < 2 || byte > 127) {
                r->rx_phase = NRF_RX_WAIT_PREAMBLE;
                break;
            }
            arm_write8(cpu, r->packetptr, byte);
            r->rx_remaining = byte;          /* PHR includes 2 FCS bytes */
            r->rx_offset    = 1;
            r->rx_phase     = NRF_RX_READ_PAYLOAD;
            break;
        case NRF_RX_READ_PAYLOAD:
            arm_write8(cpu, r->packetptr + (uint32_t)r->rx_offset, byte);
            r->rx_offset++;
            r->rx_remaining--;
            if (r->rx_remaining == 0) {
                /* Frame complete. Always treat as CRC-OK for now —
                 * generators sign their own CRCs and the medium drops
                 * collided bytes already. */
                radio_event(soc, &r->evt_payload,  RADIO_INT_PAYLOAD);
                radio_event(soc, &r->evt_end,      RADIO_INT_END);
                radio_event(soc, &r->evt_phyend,   0);
                radio_event(soc, &r->evt_crcok,    RADIO_INT_CRCOK);

                /* Hardware-style auto-ACK for unicast Data frames that
                 * have the ACK_REQUEST bit set. Real nRF52840 silicon
                 * achieves sub-200 µs ACK turnaround via PPI + TIMER +
                 * SHORTS choreography (set up by Nordic's 802.15.4
                 * driver). The Contiki nrf driver leaves that path
                 * unconfigured and relies on CSMA's software ACK
                 * (`CSMA_SEND_SOFT_ACK`), but csim's tick granularity
                 * makes the software path arrive after the sender's
                 * CSMA_ACK_WAIT_TIME — driving a retx storm that's
                 * pure simulator artefact.  Emit the ACK synchronously
                 * inside the chip model, same convention as cc2538
                 * (cc2538_rfcore.c). The firmware's later software
                 * ACK arrives as a duplicate and is harmlessly dropped
                 * at the sender (already acked, seqno already cleared).
                 *
                 * Layout in PACKETPTR: [PHR][FCF0][FCF1][DSN][...].
                 * Per IEEE 802.15.4 FCF: bit 0:2 = frame type, bit 5 =
                 * ACK_REQUEST. Broadcast frames never set ACK_REQUEST. */
                if (soc->radio_tx_cb && r->rx_offset > 4) {
                    uint8_t fcf0       = arm_read8(cpu, r->packetptr + 1);
                    uint8_t dsn        = arm_read8(cpu, r->packetptr + 3);
                    int     frame_type = fcf0 & 0x07;
                    int     ack_req    = (fcf0 >> 5) & 1;
                    if (frame_type == 0x1 /* Data */ && ack_req) {
                        uint8_t ack_fcf0 = 0x02; /* frame type = ACK */
                        uint8_t ack_fcf1 = 0x00;
                        uint16_t crc = 0;
                        crc = nrf_crc_add(crc, ack_fcf0);
                        crc = nrf_crc_add(crc, ack_fcf1);
                        crc = nrf_crc_add(crc, dsn);
                        nrf_radio_tx_listener_t cb = soc->radio_tx_cb;
                        void *ud                   = soc->radio_tx_user;
                        for (int i = 0; i < IEEE802154_PREAMBLE_LEN; i++)
                            cb(ud, IEEE802154_PREAMBLE_BYTE);
                        cb(ud, IEEE802154_SFD);
                        cb(ud, 5);              /* PHR: FCF(2)+DSN(1)+FCS(2) */
                        cb(ud, ack_fcf0);
                        cb(ud, ack_fcf1);
                        cb(ud, dsn);
                        cb(ud, (uint8_t)(crc & 0xFF));
                        cb(ud, (uint8_t)((crc >> 8) & 0xFF));
                    }
                }

                if (getenv("NRF_RX_TRACE")) {
                    fprintf(stderr, "[nrf-rx] frame off=%d phr=", r->rx_offset);
                    int dump = r->rx_offset > 24 ? 24 : r->rx_offset;
                    for (int i = 0; i < dump; i++)
                        fprintf(stderr, "%02x ", arm_read8(cpu, r->packetptr + (uint32_t)i));
                    fprintf(stderr, "\n");
                }
                /* Driver typically transitions back to RXIDLE via END
                 * processing; mirror that to drain the parser. */
                r->state    = NRF_RADIO_STATE_RXIDLE;
                r->rx_phase = NRF_RX_WAIT_PREAMBLE;
                radio_apply_shorts_after_event(soc, SHORT_END_DISABLE);
            }
            break;
    }
}

/* ============================================================
 * RNG (0x4000D000)
 *
 * Tiny PRNG-backed model. TASKS_START sets EVENTS_VALRDY immediately;
 * VALUE returns the next byte from a per-node xorshift32 stream.
 * Contiki uses this for CSMA backoff and RPL DAG-ID seeding.
 * ============================================================ */
#define NRF_RNG_BASE              0x4000D000u
#define NRF_RNG_TASKS_START       0x000
#define NRF_RNG_TASKS_STOP        0x004
#define NRF_RNG_EVENTS_VALRDY     0x100
#define NRF_RNG_SHORTS            0x200
#define NRF_RNG_INTENSET          0x304
#define NRF_RNG_INTENCLR          0x308
#define NRF_RNG_CONFIG            0x504
#define NRF_RNG_VALUE             0x508

static uint8_t rng_next(nrf_rng_state_t *rng) {
    uint32_t s = rng->prng_state ? rng->prng_state : 0x12345678u;
    s ^= s << 13; s ^= s >> 17; s ^= s << 5;
    rng->prng_state = s;
    return (uint8_t)(s & 0xFF);
}

static int nrf_rng_read(void *user_data, uint32_t addr) {
    nrf_rng_state_t *rng = (nrf_rng_state_t *)user_data;
    uint32_t off = addr - NRF_RNG_BASE;
    switch (off) {
        case NRF_RNG_EVENTS_VALRDY: return (int)rng->evt_valrdy;
        case NRF_RNG_INTENSET:
        case NRF_RNG_INTENCLR:      return (int)rng->intenset;
        case NRF_RNG_SHORTS:        return (int)rng->shorts;
        case NRF_RNG_VALUE:         return (int)rng_next(rng);
        default: return 0;
    }
}

static void nrf_rng_write(void *user_data, uint32_t addr, uint32_t value) {
    nrf_rng_state_t *rng = (nrf_rng_state_t *)user_data;
    uint32_t off = addr - NRF_RNG_BASE;
    switch (off) {
        case NRF_RNG_TASKS_START:
            if (value == 1) { rng->running = true; rng->evt_valrdy = 1; }
            break;
        case NRF_RNG_TASKS_STOP:
            if (value == 1) rng->running = false;
            break;
        case NRF_RNG_EVENTS_VALRDY:
            /* Firmware acks VALRDY by writing 0.  If the RNG is still running,
             * immediately re-set it — our PRNG delivers bytes instantly. */
            rng->evt_valrdy = (value & 1) ? 1 : (rng->running ? 1 : 0);
            break;
        case NRF_RNG_SHORTS:        rng->shorts     = value;     break;
        case NRF_RNG_INTENSET:      rng->intenset  |= value;     break;
        case NRF_RNG_INTENCLR:      rng->intenset  &= ~value;    break;
        default: break;
    }
}

/* ============================================================
 * FICR at 0x10000000 — only DEVICEADDR pair is useful here.
 *
 * Per nRF52840 PS, FICR is in the same 0x10000000 page as UICR;
 * we expose a 4 KiB window. DEVICEADDR[0] = 0xA4, DEVICEADDR[1] = 0xA8.
 * ============================================================ */
#define NRF_FICR_BASE        0x10000000u
#define NRF_FICR_DEVICEADDR0 0xA4
#define NRF_FICR_DEVICEADDR1 0xA8
/* Other commonly-read FICR fields we return useful constants for. */
#define NRF_FICR_CODEPAGESIZE 0x10
#define NRF_FICR_CODESIZE     0x14
#define NRF_FICR_DEVICEID0    0x60
#define NRF_FICR_DEVICEID1    0x64
#define NRF_FICR_DEVICEADDRTYPE 0xA0

static int nrf_ficr_read(void *user_data, uint32_t addr) {
    nrf_ficr_state_t *ficr = (nrf_ficr_state_t *)user_data;
    uint32_t off = addr - NRF_FICR_BASE;
    switch (off) {
        case NRF_FICR_CODEPAGESIZE:    return 4096;            /* 4 KiB pages */
        case NRF_FICR_CODESIZE:        return 256;             /* 256 pages → 1 MiB */
        case NRF_FICR_DEVICEADDRTYPE:  return 1;               /* random-static */
        case NRF_FICR_DEVICEADDR0:     return (int)ficr->deviceaddr0;
        case NRF_FICR_DEVICEADDR1:     return (int)ficr->deviceaddr1;
        case NRF_FICR_DEVICEID0:       return (int)ficr->deviceaddr0;
        case NRF_FICR_DEVICEID1:       return (int)ficr->deviceaddr1;
        default: return 0xFFFFFFFF;     /* erased flash */
    }
}

static void nrf_ficr_write(void *user_data, uint32_t addr, uint32_t value) {
    /* FICR is read-only on real hardware. Allow writes from the
     * harness via direct struct access; the firmware never writes here. */
    (void)user_data; (void)addr; (void)value;
}

/* ============================================================
 * TIMER0..4 — minimum for rtimer_arch_now()
 *
 * 16 MHz base clock, scaled by 2^PRESCALER. Counter increments
 * lazily on read; TASKS_CAPTURE[i] latches into CC[i].
 * ============================================================ */
#define TIMER_TASKS_START      0x000
#define TIMER_TASKS_STOP       0x004
#define TIMER_TASKS_COUNT      0x008
#define TIMER_TASKS_CLEAR      0x00C
#define TIMER_TASKS_SHUTDOWN   0x010
#define TIMER_TASKS_CAPTURE_0  0x040  /* +4 each, up to CAPTURE_5 */
#define TIMER_EVENTS_COMPARE_0 0x140  /* +4 each */
#define TIMER_SHORTS           0x200
#define TIMER_INTENSET         0x304
#define TIMER_INTENCLR         0x308
#define TIMER_MODE             0x504
#define TIMER_BITMODE          0x508
#define TIMER_PRESCALER        0x510
#define TIMER_CC_0             0x540  /* +4 each, up to CC_5 */

#define TIMER_BASE_HZ          16000000u

static uint32_t timer_bit_mask(uint32_t bitmode) {
    switch (bitmode) {
        case 0: return 0xFFFFu;        /* 16-bit */
        case 1: return 0xFFu;          /* 8-bit */
        case 2: return 0xFFFFFFu;      /* 24-bit */
        case 3: default: return 0xFFFFFFFFu;  /* 32-bit */
    }
}

static uint32_t timer_compute_counter(const nrf_timer_state_t *t, int64_t now_cycles, uint32_t cpu_freq_hz) {
    if (!t->running) return t->counter_at_start;
    int64_t elapsed = now_cycles - t->start_cycles;
    if (elapsed < 0) elapsed = 0;
    /* tick_rate_hz = TIMER_BASE_HZ >> prescaler */
    uint64_t tick_hz = (uint64_t)TIMER_BASE_HZ >> t->prescaler;
    if (tick_hz == 0) tick_hz = 1;
    /* ticks = elapsed * tick_hz / cpu_freq_hz */
    uint64_t ticks = ((uint64_t)elapsed * tick_hz) / (uint64_t)cpu_freq_hz;
    return (t->counter_at_start + (uint32_t)ticks) & timer_bit_mask(t->bitmode);
}

static int nrf_timer_read(void *user_data, uint32_t addr) {
    nrf_timer_state_t *t = (nrf_timer_state_t *)user_data;
    uint32_t off = addr - t->base;
    if (off >= TIMER_EVENTS_COMPARE_0 && off < TIMER_EVENTS_COMPARE_0 + 6 * 4)
        return (int)t->evt_compare[(off - TIMER_EVENTS_COMPARE_0) / 4];
    if (off >= TIMER_CC_0 && off < TIMER_CC_0 + 6 * 4)
        return (int)t->cc[(off - TIMER_CC_0) / 4];
    switch (off) {
        case TIMER_SHORTS:    return (int)t->shorts;
        case TIMER_INTENSET:
        case TIMER_INTENCLR:  return (int)t->intenset;
        case TIMER_MODE:      return (int)t->mode;
        case TIMER_BITMODE:   return (int)t->bitmode;
        case TIMER_PRESCALER: return (int)t->prescaler;
        default: return 0;
    }
}

static void nrf_timer_write(void *user_data, uint32_t addr, uint32_t value) {
    nrf_timer_state_t *t = (nrf_timer_state_t *)user_data;
    arm_cpu_t *cpu = &t->soc->plat->cpu;
    uint32_t off = addr - t->base;

    if (off >= TIMER_TASKS_CAPTURE_0 && off < TIMER_TASKS_CAPTURE_0 + 6 * 4) {
        if (value == 1) {
            int i = (off - TIMER_TASKS_CAPTURE_0) / 4;
            t->cc[i] = timer_compute_counter(t, (int64_t)cpu->cycles, cpu->cpu_freq_hz);
        }
        return;
    }
    if (off >= TIMER_EVENTS_COMPARE_0 && off < TIMER_EVENTS_COMPARE_0 + 6 * 4) {
        t->evt_compare[(off - TIMER_EVENTS_COMPARE_0) / 4] = value & 1;
        return;
    }
    if (off >= TIMER_CC_0 && off < TIMER_CC_0 + 6 * 4) {
        t->cc[(off - TIMER_CC_0) / 4] = value;
        return;
    }
    switch (off) {
        case TIMER_TASKS_START:
            if (value == 1 && !t->running) {
                t->counter_at_start = timer_compute_counter(t, (int64_t)cpu->cycles, cpu->cpu_freq_hz);
                t->start_cycles = (int64_t)cpu->cycles;
                t->running = true;
            }
            break;
        case TIMER_TASKS_STOP:
        case TIMER_TASKS_SHUTDOWN:
            if (value == 1 && t->running) {
                t->counter_at_start = timer_compute_counter(t, (int64_t)cpu->cycles, cpu->cpu_freq_hz);
                t->start_cycles = (int64_t)cpu->cycles;
                t->running = false;
            }
            break;
        case TIMER_TASKS_CLEAR:
            if (value == 1) {
                t->counter_at_start = 0;
                t->start_cycles = (int64_t)cpu->cycles;
            }
            break;
        case TIMER_SHORTS:    t->shorts    = value; break;
        case TIMER_INTENSET:  t->intenset |= value; break;
        case TIMER_INTENCLR:  t->intenset &= ~value; break;
        case TIMER_MODE:      t->mode      = value & 3; break;
        case TIMER_BITMODE:   t->bitmode   = value & 3; break;
        case TIMER_PRESCALER: t->prescaler = value & 0xF; break;
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

    /* Honour the platform's VTOR override (board-specific: DK = 0x0,
     * Dongle = 0x1000 due to the Open Bootloader). Falls back to the
     * SoC config's vtor_default if the platform leaves it 0. */
    if (plat->config && plat->config->vtor_override)
        plat->cpu.vtor_default = plat->config->vtor_override;

    /* Minimum host vtable. */
    plat->host.cpu         = &plat->cpu;
    plat->host.gpio        = NULL;
    plat->host.now_ns      = nrf_host_now_ns;
    plat->host.schedule_ns = nrf_host_schedule_ns;
    plat->host.cancel      = nrf_host_cancel;

    /* CLOCK and UART register windows.  The UART page serves both the legacy
     * UART (Contiki) and UARTE EasyDMA (stock Zephyr) register maps. */
    arm_register_io(&plat->cpu, NRF_CLOCK_BASE, NRF_CLOCK_SIZE,
                    nrf_clock_read, nrf_clock_write, &soc->clock);
    soc->uart0.soc     = soc;
    soc->uart0.irq_num = NRF_UARTE_IRQ;
    arm_register_io(&plat->cpu, NRF_UART0_BASE, NRF_UART0_SIZE,
                    nrf_uart_read, nrf_uart_write, &soc->uart0);

    /* RTC0 (Contiki's clock source) + RTC1 (Zephyr's nrf_rtc_timer system
     * clock).  Same model, two instances; each owns its recurring TICK event,
     * keyed by its own base + IRQ.  rtc_start arms the event. */
    soc->rtc0.irq_num = NRF_RTC0_IRQ;
    soc->rtc0.base    = NRF_RTC0_BASE;
    soc->rtc0.soc     = soc;
    cpu_event_t *tev0 = (cpu_event_t *)calloc(1, sizeof(*tev0));
    tev0->callback  = rtc_event_cb;
    tev0->user_data = &soc->rtc0;
    soc->rtc0.tick_event = tev0;
    arm_register_io(&plat->cpu, NRF_RTC0_BASE, NRF_RTC0_SIZE,
                    nrf_rtc_read, nrf_rtc_write, &soc->rtc0);

    soc->rtc1.irq_num = NRF_RTC1_IRQ;
    soc->rtc1.base    = NRF_RTC1_BASE;
    soc->rtc1.soc     = soc;
    cpu_event_t *tev1 = (cpu_event_t *)calloc(1, sizeof(*tev1));
    tev1->callback  = rtc_event_cb;
    tev1->user_data = &soc->rtc1;
    soc->rtc1.tick_event = tev1;
    arm_register_io(&plat->cpu, NRF_RTC1_BASE, NRF_RTC1_SIZE,
                    nrf_rtc_read, nrf_rtc_write, &soc->rtc1);

    /* RADIO — on-chip 2.4 GHz multi-protocol radio. */
    soc->radio.irq_num = NRF_RADIO_IRQ;
    soc->radio.state   = NRF_RADIO_STATE_DISABLED;
    soc->radio.power   = 1;       /* powered up by default */
    arm_register_io(&plat->cpu, NRF_RADIO_BASE, NRF_RADIO_SIZE,
                    nrf_radio_read, nrf_radio_write, soc);

    /* TIMER0..4. Bases per nRF52840 PS table 19. */
    static const uint32_t timer_bases[5] = {
        0x40008000u,  /* TIMER0 */
        0x40009000u,  /* TIMER1 */
        0x4000A000u,  /* TIMER2 */
        0x4001A000u,  /* TIMER3 */
        0x4001B000u   /* TIMER4 */
    };
    for (int i = 0; i < 5; i++) {
        soc->timer[i].base = timer_bases[i];
        soc->timer[i].soc  = soc;
        soc->timer[i].bitmode = 0;          /* 16-bit default */
        arm_register_io(&plat->cpu, timer_bases[i], 0x1000,
                        nrf_timer_read, nrf_timer_write, &soc->timer[i]);
    }

    /* RNG */
    soc->rng.prng_state = 0xDEADBEEF;
    arm_register_io(&plat->cpu, NRF_RNG_BASE, 0x1000,
                    nrf_rng_read, nrf_rng_write, &soc->rng);

    /* FICR — default values; harness patches DEVICEADDR0/1 per node. */
    soc->ficr.deviceaddr0 = 0x00000000;
    soc->ficr.deviceaddr1 = 0x0000F4CE;     /* upper bytes used as MAC OUI fragment */
    arm_register_io(&plat->cpu, NRF_FICR_BASE, 0x1000,
                    nrf_ficr_read, nrf_ficr_write, &soc->ficr);
}

static void nrf52840_soc_destroy(arm_platform_t *plat) {
    nrf52840_soc_t *soc = (nrf52840_soc_t *)plat->soc;
    if (soc) {
        if (soc->rtc0.tick_event) {
            arm_cancel_event(&plat->cpu, (arm_event_t *)soc->rtc0.tick_event);
            free(soc->rtc0.tick_event);
        }
        if (soc->rtc1.tick_event) {
            arm_cancel_event(&plat->cpu, (arm_event_t *)soc->rtc1.tick_event);
            free(soc->rtc1.tick_event);
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
