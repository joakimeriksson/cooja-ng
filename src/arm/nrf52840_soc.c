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
/* Debug-trace env flags, latched once per process.  These used to call
 * getenv() on EVERY MMIO access; firmware busy-polls RADIO/TIMER registers
 * (TSCH slot waits, driver PHYEND polls), and getenv's lock made those
 * probes ~47%% of total simulation wall time (measured with sample(1)). */
static int nrf_trace_flag(int *cache, const char *name) {
    if (__builtin_expect(*cache < 0, 0)) *cache = getenv(name) != NULL;
    return *cache;
}
static int trc_clock = -1, trc_uart = -1, trc_radio = -1,
           trc_rxbyte = -1, trc_rx = -1, trc_ppi = -1;

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
/* CLOCK interrupt (POWER + CLOCK share IRQ 0).  nrf_802154's RSCH waits on the
 * HFCLKSTARTED interrupt to approve its radio precondition; the nrfx clock
 * driver / on-off manager waits on LFCLKSTARTED.  Contiki polls the events
 * (INTENSET stays 0) → no IRQ → byte-identical. */
#define NRF_CLOCK_INTENSET         0x304
#define NRF_CLOCK_INTENCLR         0x308
#define NRF_CLOCK_INT_HFCLKSTARTED (1u << 0)
#define NRF_CLOCK_INT_LFCLKSTARTED (1u << 1)
#define NRF_POWER_CLOCK_IRQ        0

static int nrf_clock_read(void *user_data, uint32_t addr) {
    nrf_clock_state_t *clock = (nrf_clock_state_t *)user_data;
    uint32_t off = addr - NRF_CLOCK_BASE;
    switch (off) {
        /* EVENTS = the latched event (ISR clears), separate from run-state. */
        case NRF_CLOCK_EVENTS_HFCLKSTARTED: return (int)clock->evt_hfclkstarted;
        case NRF_CLOCK_EVENTS_LFCLKSTARTED: return (int)clock->evt_lfclkstarted;
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
        case NRF_CLOCK_INTENSET:
        case NRF_CLOCK_INTENCLR:  return (int)clock->intenset;
        default: return 0;
    }
}

/* Raise the POWER_CLOCK IRQ (0) for any enabled + latched CLOCK event. */
static void nrf_clock_irq(nrf_clock_state_t *clock) {
    nrf52840_soc_t *soc = (nrf52840_soc_t *)clock->soc;
    if (!soc || !soc->plat || !soc->plat->cpu.nvic) return;
    if (((clock->intenset & NRF_CLOCK_INT_HFCLKSTARTED) && clock->evt_hfclkstarted) ||
        ((clock->intenset & NRF_CLOCK_INT_LFCLKSTARTED) && clock->evt_lfclkstarted))
        arm_nvic_set_pending((arm_nvic_t *)soc->plat->cpu.nvic, NRF_POWER_CLOCK_IRQ);
}

static void nrf_clock_write(void *user_data, uint32_t addr, uint32_t value) {
    nrf_clock_state_t *clock = (nrf_clock_state_t *)user_data;
    uint32_t off = addr - NRF_CLOCK_BASE;
    if (nrf_trace_flag(&trc_clock, "NRF_CLOCK_TRACE")) fprintf(stderr, "[clkW] 0x%03x = 0x%08x\n", off, value);
    switch (off) {
        case NRF_CLOCK_TASKS_HFCLKSTART:    /* run-state + latched event + IRQ */
            if (value == 1) { clock->hfclkstarted = 1;
                              clock->evt_hfclkstarted = 1; nrf_clock_irq(clock); }
            break;
        case NRF_CLOCK_TASKS_LFCLKSTART:
            if (value == 1) { clock->lfclkstarted = 1;
                              clock->evt_lfclkstarted = 1; nrf_clock_irq(clock); }
            break;
        case NRF_CLOCK_EVENTS_HFCLKSTARTED:   /* ISR clears the event only */
            clock->evt_hfclkstarted = value & 1;
            break;
        case NRF_CLOCK_EVENTS_LFCLKSTARTED:
            clock->evt_lfclkstarted = value & 1;
            break;
        case NRF_CLOCK_INTENSET:
            clock->intenset |= value;
            nrf_clock_irq(clock);   /* enable-while-latched → fire now */
            break;
        case NRF_CLOCK_INTENCLR:
            clock->intenset &= ~value;
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
#define NRF_UARTE_INT_TXSTOPPED    (1u << 22)  /* int-driven shell/console TX "ready" */
/* UARTE EasyDMA RX (console shell input). */
#define NRF_UARTE_TASKS_STARTRX    0x000
#define NRF_UARTE_TASKS_STOPRX     0x004
#define NRF_UARTE_EVENTS_RXDRDY    0x108
#define NRF_UARTE_EVENTS_ENDRX     0x110
#define NRF_UARTE_EVENTS_RXSTARTED 0x14C
#define NRF_UARTE_SHORTS           0x200
#define NRF_UARTE_RXD_PTR          0x534
#define NRF_UARTE_RXD_MAXCNT       0x538
#define NRF_UARTE_RXD_AMOUNT       0x53C
#define NRF_UARTE_INT_ENDRX        (1u << 4)
#define NRF_UARTE_SHORT_ENDRX_STARTRX (1u << 5)
#define NRF_UARTE_SHORT_ENDRX_STOPRX  (1u << 6)

/* Deliver one buffered console byte via EasyDMA: write it to the RX buffer in
 * RAM, latch ENDRX/RXDRDY, raise the IRQ, and apply the ENDRX_STARTRX/STOPRX
 * short.  Called when the firmware arms RX (STARTRX) or acks the previous byte
 * (clears ENDRX) — so input self-paces to the driver's consumption rate. */
static void uarte_deliver_rx(nrf_uart_state_t *uart) {
    nrf52840_soc_t *soc = (nrf52840_soc_t *)uart->soc;
    if (nrf_trace_flag(&trc_uart, "NRF_UART_TRACE"))
        fprintf(stderr, "[uartRX] deliver armed=%d endrx=%d ring=%d->%d\n",
                uart->rx_armed, uart->endrx, uart->rx_tail, uart->rx_head);
    if (!uart->rx_armed || uart->endrx) return;          /* not ready */
    if (uart->rx_head == uart->rx_tail) return;          /* ring empty */
    uint8_t b = uart->rx_ring[uart->rx_tail];
    uart->rx_tail = (uart->rx_tail + 1) % (int)sizeof(uart->rx_ring);
    if (uart->rxd_ptr) arm_write8(&soc->plat->cpu, uart->rxd_ptr, b);
    uart->rxd_amount = 1;
    uart->rxdrdy     = 1;
    uart->rxstarted  = 1;
    uart->endrx      = 1;
    /* ENDRX_STARTRX re-arms RX in hardware; ENDRX_STOPRX (or no short) stops. */
    if (uart->shorts & NRF_UARTE_SHORT_ENDRX_STOPRX)        uart->rx_armed = 0;
    else if (!(uart->shorts & NRF_UARTE_SHORT_ENDRX_STARTRX)) uart->rx_armed = 0;
    if ((uart->intenset & NRF_UARTE_INT_ENDRX) && soc->plat->cpu.nvic)
        arm_nvic_set_pending((arm_nvic_t *)soc->plat->cpu.nvic, uart->irq_num);
}

/* Enqueue host console input; deliver immediately if the firmware is waiting. */
void nrf_uart_feed_rx(nrf_uart_state_t *uart, const uint8_t *buf, int len) {
    for (int i = 0; i < len; i++) {
        int next = (uart->rx_head + 1) % (int)sizeof(uart->rx_ring);
        if (next == uart->rx_tail) break;                /* ring full */
        uart->rx_ring[uart->rx_head] = buf[i];
        uart->rx_head = next;
    }
    uarte_deliver_rx(uart);
}

static int nrf_uart_read(void *user_data, uint32_t addr) {
    nrf_uart_state_t *uart = (nrf_uart_state_t *)user_data;
    uint32_t off = addr - NRF_UART0_BASE;
    if (nrf_trace_flag(&trc_uart, "NRF_UART_TRACE")) fprintf(stderr, "[uartR] 0x%03x\n", off);
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
        /* UARTE EasyDMA RX read-backs. */
        case NRF_UARTE_EVENTS_ENDRX:     return (int)uart->endrx;
        case NRF_UARTE_EVENTS_RXDRDY:    return (int)uart->rxdrdy;
        case NRF_UARTE_EVENTS_RXSTARTED: return (int)uart->rxstarted;
        case NRF_UARTE_RXD_PTR:          return (int)uart->rxd_ptr;
        case NRF_UARTE_RXD_MAXCNT:       return (int)uart->rxd_maxcnt;
        case NRF_UARTE_RXD_AMOUNT:       return (int)uart->rxd_amount;
        case NRF_UARTE_SHORTS:           return (int)uart->shorts;
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
    /* The interrupt-driven console/shell driver waits on TXSTOPPED (not ENDTX)
     * as its "TX ready" signal; raise the IRQ for either so it feeds the next
     * chunk until its buffer drains. */
    if ((uart->intenset & (NRF_UARTE_INT_ENDTX | NRF_UARTE_INT_TXSTOPPED)) &&
        soc->plat->cpu.nvic)
        arm_nvic_set_pending((arm_nvic_t *)soc->plat->cpu.nvic, uart->irq_num);
}

static void nrf_uart_write(void *user_data, uint32_t addr, uint32_t value) {
    nrf_uart_state_t *uart = (nrf_uart_state_t *)user_data;
    uint32_t off = addr - NRF_UART0_BASE;
    if (nrf_trace_flag(&trc_uart, "NRF_UART_TRACE")) fprintf(stderr, "[uartW] 0x%03x = 0x%08x\n", off, value);
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
        case NRF_UARTE_INTENSET:
            uart->intenset |= value;
            /* int-driven TX bootstrap: enabling TXSTOPPED while the event is
             * already latched fires the IRQ now, kicking the first TX. */
            if ((value & NRF_UARTE_INT_TXSTOPPED) && uart->txstopped &&
                ((nrf52840_soc_t *)uart->soc)->plat->cpu.nvic)
                arm_nvic_set_pending(
                    (arm_nvic_t *)((nrf52840_soc_t *)uart->soc)->plat->cpu.nvic,
                    uart->irq_num);
            break;
        case NRF_UARTE_INTENCLR:       uart->intenset &= ~value; break;
        /* UARTE EasyDMA RX path (console shell input). */
        case NRF_UARTE_TASKS_STARTRX:  if (value == 1) { uart->rx_armed = 1; uarte_deliver_rx(uart); } break;
        case NRF_UARTE_TASKS_STOPRX:   if (value == 1) uart->rx_armed = 0; break;
        case NRF_UARTE_EVENTS_ENDRX:
            uart->endrx = value & 1;
            if (!uart->endrx) uarte_deliver_rx(uart);   /* firmware acked → next byte */
            break;
        case NRF_UARTE_EVENTS_RXDRDY:    uart->rxdrdy    = value & 1; break;
        case NRF_UARTE_EVENTS_RXSTARTED: uart->rxstarted = value & 1; break;
        case NRF_UARTE_RXD_PTR:          uart->rxd_ptr    = value; break;
        case NRF_UARTE_RXD_MAXCNT:       uart->rxd_maxcnt = value; break;
        case NRF_UARTE_SHORTS:           uart->shorts     = value; break;
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

/* Compute the exact CPU cycle at which COUNTER will next equal `now+ticks`,
 * aligned to a tick boundary so rtc_compute_counter(fire) lands precisely on
 * that counter value. */
static int64_t rtc_fire_cycle(const nrf_rtc_state_t *rtc, int64_t now_cycles,
                              uint64_t ticks) {
    int64_t period = (int64_t)rtc->tick_period_cycles;
    int64_t into_tick = (now_cycles - rtc->counter_anchor_cycles) % period;
    return now_cycles - into_tick + (int64_t)ticks * period;
}

/* Tickless scheduling: COUNTER is always derived from elapsed cycles
 * (rtc_compute_counter), so reads stay exact between wake-ups.  The recurring
 * TICK and the OVRFLW wrap share one event (rtc_arm_periodic); each of the four
 * COMPARE channels gets its OWN event scheduled at the exact cycle COUNTER will
 * equal CC[i] (rtc_arm_compare).  Per-channel events remove the old
 * single-shared-event contention and the fragile cnt==CC[i] guard: a channel
 * fires exactly once on its edge, then re-arms a full wrap later (matching the
 * hardware's latch-then-refire-on-COUNTER==CC behaviour). */
static void rtc_arm_periodic(nrf_rtc_state_t *rtc, arm_platform_t *plat) {
    arm_event_t *ev = (arm_event_t *)rtc->tick_event;
    if (!rtc->running || rtc->tick_period_cycles == 0) {
        arm_cancel_event(&plat->cpu, ev);
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
    if (best == UINT64_MAX) {                    /* nothing periodic enabled */
        arm_cancel_event(&plat->cpu, ev);
        return;
    }
    arm_schedule_event(&plat->cpu, ev, rtc_fire_cycle(rtc, now_cycles, best));
}

/* Schedule COMPARE channel i's dedicated event at the exact cycle COUNTER next
 * equals CC[i].  Gated on the channel's interrupt being enabled, preserving the
 * prior behaviour of only waking for INTEN-armed compares. */
static void rtc_arm_compare(nrf_rtc_state_t *rtc, arm_platform_t *plat, int i) {
    arm_event_t *ev = (arm_event_t *)rtc->compare_event[i];
    if (!ev) return;
    if (!rtc->running || rtc->tick_period_cycles == 0 ||
        !(rtc->intenset & (RTC_INT_COMPARE_0 << i))) {
        arm_cancel_event(&plat->cpu, ev);
        return;
    }
    int64_t now_cycles = (int64_t)plat->cpu.cycles;
    uint32_t now = rtc_compute_counter(rtc, now_cycles);
    uint32_t d = (rtc->cc[i] - now) & 0xFFFFFFu;
    if (d == 0) d = 0x1000000u;                 /* equal now → next match a wrap away */
    arm_schedule_event(&plat->cpu, ev, rtc_fire_cycle(rtc, now_cycles, d));
}

/* Re-arm every source — used on register writes and start. */
static void rtc_reschedule(nrf_rtc_state_t *rtc, arm_platform_t *plat) {
    rtc_arm_periodic(rtc, plat);
    for (int i = 0; i < 4; i++) rtc_arm_compare(rtc, plat, i);
}

/* Shared TICK / OVRFLW callback. */
static void rtc_event_cb(void *user_data, cpu_event_t *event) {
    nrf_rtc_state_t *rtc = (nrf_rtc_state_t *)user_data;
    nrf52840_soc_t *soc = (nrf52840_soc_t *)rtc->soc;
    arm_platform_t *plat = soc->plat;
    if (!plat || !rtc->running) return;

    uint32_t cnt = rtc_compute_counter(rtc, event->fire_cycle);
    bool fire = false;
    if (rtc->intenset & RTC_INT_TICK)              { rtc->evt_tick = 1;   fire = true; }
    if ((rtc->intenset & RTC_INT_OVRFLW) && cnt==0) { rtc->evt_ovrflw = 1; fire = true; }
    if (fire && plat->cpu.nvic)
        arm_nvic_set_pending((arm_nvic_t *)plat->cpu.nvic, rtc->irq_num);

    rtc_arm_periodic(rtc, plat);
}

/* Per-channel COMPARE callback: fires the one channel this event belongs to,
 * then re-arms it (next COUNTER==CC[i] is a full wrap away — the latched edge). */
static void rtc_compare_cb(void *user_data, cpu_event_t *event) {
    nrf_rtc_state_t *rtc = (nrf_rtc_state_t *)user_data;
    nrf52840_soc_t *soc = (nrf52840_soc_t *)rtc->soc;
    arm_platform_t *plat = soc->plat;
    if (!plat || !rtc->running) return;
    for (int i = 0; i < 4; i++) {
        if (event != (cpu_event_t *)rtc->compare_event[i]) continue;
        if (rtc->intenset & (RTC_INT_COMPARE_0 << i)) {
            rtc->evt_compare[i] = 1;
            if (plat->cpu.nvic)
                arm_nvic_set_pending((arm_nvic_t *)plat->cpu.nvic, rtc->irq_num);
        }
        rtc_arm_compare(rtc, plat, i);
        return;
    }
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
    for (int i = 0; i < 4; i++)
        arm_cancel_event(&plat->cpu, (arm_event_t *)rtc->compare_event[i]);
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
#define RADIO_INT_EDEND        (1u << 16)
#define RADIO_INT_CCAIDLE      (1u << 17)
#define RADIO_INT_CCABUSY      (1u << 18)
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


void nrf_ppi_event_notify(nrf52840_soc_t *soc, uint32_t event_addr);

/* Map an event's INTENSET bit to its EVENTS_ register offset (for PPI routing). */
static uint32_t radio_event_offset(uint32_t int_mask) {
    switch (int_mask) {
        case RADIO_INT_READY:      return RADIO_EVENTS_READY;
        case RADIO_INT_ADDRESS:    return RADIO_EVENTS_ADDRESS;
        case RADIO_INT_PAYLOAD:    return RADIO_EVENTS_PAYLOAD;
        case RADIO_INT_END:        return RADIO_EVENTS_END;
        case RADIO_INT_DISABLED:   return RADIO_EVENTS_DISABLED;
        case RADIO_INT_BCMATCH:    return RADIO_EVENTS_BCMATCH;
        case RADIO_INT_CRCOK:      return RADIO_EVENTS_CRCOK;
        case RADIO_INT_CRCERROR:   return RADIO_EVENTS_CRCERROR;
        case RADIO_INT_FRAMESTART: return RADIO_EVENTS_FRAMESTART;
        case RADIO_INT_EDEND:      return RADIO_EVENTS_EDEND;
        case RADIO_INT_CCAIDLE:    return RADIO_EVENTS_CCAIDLE;
        case RADIO_INT_CCABUSY:    return RADIO_EVENTS_CCABUSY;
        case RADIO_INT_TXREADY:    return RADIO_EVENTS_TXREADY;
        case RADIO_INT_RXREADY:    return RADIO_EVENTS_RXREADY;
        case RADIO_INT_PHYEND:     return RADIO_EVENTS_PHYEND;
        default: return 0;
    }
}

/* Raise IRQ if any of the latched events have INTENSET bit set, and route the
 * event through PPI (nrf_802154 chains RADIO events → tasks in hardware). */
static void radio_event(nrf52840_soc_t *soc, uint32_t *evt_field, uint32_t int_mask) {
    nrf_radio_state_t *r = &soc->radio;
    *evt_field = 1;
    if ((r->intenset & int_mask) && soc->plat && soc->plat->cpu.nvic)
        arm_nvic_set_pending((arm_nvic_t *)soc->plat->cpu.nvic, r->irq_num);
    uint32_t off = radio_event_offset(int_mask);
    if (off) nrf_ppi_event_notify(soc, NRF_RADIO_BASE + off);
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
            case SHORT_RXREADY_CCASTART:
                radio_trigger_task(soc, RADIO_TASKS_CCASTART);
                break;
            case SHORT_CCAIDLE_STOP:
                radio_trigger_task(soc, RADIO_TASKS_STOP);
                break;
        }
    }
}

/* Ramp-down delay for TASKS_DISABLE.  Real nRF52840 takes ~6 µs to
 * transition from any active state to DISABLED.  The firmware clears
 * EVENTS_DISABLED then spins until it reads back 1 — if we set it
 * synchronously in the same instruction, the firmware clears it before
 * the poll starts and spins forever. */
/* Delay just long enough that the firmware's "clear EVENTS_DISABLED →
 * read EVENTS_DISABLED" busy-wait resolves (2 instructions ≈ 4 cycles),
 * but short enough that the DISABLED → PPI → TXEN chain fires BEFORE
 * the driver clears the PPI channels (which happens in the CRCOK ISR
 * epilogue, ~20-50 instructions after the SHORTS triggered DISABLE). */
#define NRF_RADIO_DISABLE_CYCLES  10

void nrf_radio_disable_cb(void *user_data, cpu_event_t *event) {
    (void)event;
    nrf52840_soc_t *soc = (nrf52840_soc_t *)user_data;
    nrf_radio_state_t *r = &soc->radio;
    /* Only complete the transition if we're still in the DISABLE
     * intermediate state.  If the firmware issued another task (e.g.
     * TASKS_RXEN immediately after TASKS_DISABLE), the state has
     * already moved on and we must not stomp it back to DISABLED. */
    if (r->state != NRF_RADIO_STATE_TXDISABLE &&
        r->state != NRF_RADIO_STATE_RXDISABLE)
        return;
    r->state = NRF_RADIO_STATE_DISABLED;
    radio_event(soc, &r->evt_disabled, RADIO_INT_DISABLED);
    radio_apply_shorts_after_event(soc, SHORT_DISABLED_TXEN);
    radio_apply_shorts_after_event(soc, SHORT_DISABLED_RXEN);
}

/* 802.15.4 air-time: 250 kbps = 32 µs/byte = 2048 cycles @ 64 MHz, plus a
 * 6-byte SHR+PHR preamble overhead.  Modelling real TX air-time (instead of an
 * instant transmit) is what keeps nrf_802154's calibrated busy-wait/poll in
 * sync — its PHYEND IRQ then lands DURING the wait, not before the firmware has
 * even cleared the event. */
#define RADIO_TX_BYTE_CYCLES   2048
#define RADIO_TX_SHR_PHR_BYTES 6

/* Deferred TX completion: fires the post-transmit events (PAYLOAD/END/PHYEND),
 * pushes the frame onto the medium, and applies the post-TX shorts — one
 * air-time after TASKS_START, while the radio sat in TX state. */
static void nrf_radio_tx_done_cb(void *user_data, cpu_event_t *event) {
    (void)event;
    nrf52840_soc_t *soc = (nrf52840_soc_t *)user_data;
    nrf_radio_state_t *r = &soc->radio;
    /* If the firmware aborted (DISABLE/STOP) mid-air, the state has moved on. */
    if (r->state != NRF_RADIO_STATE_TX)
        return;
    /* Frame bytes already went to the medium at TASKS_START (start of
     * air) — this callback only completes the events/shorts. */
    radio_event(soc, &r->evt_payload,  RADIO_INT_PAYLOAD);
    radio_event(soc, &r->evt_end,      RADIO_INT_END);
    radio_event(soc, &r->evt_phyend,   RADIO_INT_PHYEND);
    r->state = NRF_RADIO_STATE_TXIDLE;
    radio_apply_shorts_after_event(soc, SHORT_END_DISABLE);
    radio_apply_shorts_after_event(soc, SHORT_PHYEND_DISABLE);
    radio_apply_shorts_after_event(soc, SHORT_END_START);
    radio_apply_shorts_after_event(soc, SHORT_PHYEND_START);
}

/* Deferred chip auto-ACK: real 802.15.4 acknowledges 12 symbols (192 µs)
 * after the frame ends — the sender needs that turnaround to get its own
 * radio from TX back into RX.  Emitting the ACK synchronously at
 * rx-complete (the old behaviour) put the ACK's preamble on the air while
 * the peer was still walking TX→DISABLED→RXEN; those bytes were dropped
 * (only RXRU/RXIDLE buffer) and the ACK never frame-synced. */
static void nrf_radio_ack_cb(void *user_data, cpu_event_t *event) {
    (void)event;
    nrf52840_soc_t *soc = (nrf52840_soc_t *)user_data;
    nrf_radio_state_t *r = &soc->radio;
    if (r->ack_pending_len <= 0 || soc->radio_tx_cb == NULL) return;
    for (int i = 0; i < r->ack_pending_len; i++)
        soc->radio_tx_cb(soc->radio_tx_user, r->ack_pending[i]);
    r->ack_pending_len = 0;
}

/* RXEN/TXEN ramp-up: the RADIO does not reach RXIDLE/TXIDLE instantly — it ramps
 * for RX_RAMP_UP_TIME/TX_RAMP_UP_TIME = 40 µs (nrf_802154_procedures_duration.h),
 * sitting in RXRU/TXRU.  nrf_802154 sequences off this (it polls STATE==TXRU and
 * paces its TIMER from the ramp), so collapsing it to zero desyncs the driver.
 * 40 µs @ 64 MHz = 2560 cycles. */
#define RADIO_RAMP_UP_CYCLES   2560

/* Ramp-up completion: RXRU→RXIDLE / TXRU→TXIDLE (fires READY + RX/TXREADY and
 * applies the *READY_START / RXREADY_CCASTART shorts via radio_set_state). */
static void nrf_radio_ramp_cb(void *user_data, cpu_event_t *event) {
    (void)event;
    nrf52840_soc_t *soc = (nrf52840_soc_t *)user_data;
    nrf_radio_state_t *r = &soc->radio;
    if (nrf_trace_flag(&trc_radio, "NRF_RADIO_TRACE"))
        fprintf(stderr, "[ramp] cb fired state=%d @cyc=%lld\n", r->state,
                (long long)soc->plat->cpu.cycles);
    if (r->state == NRF_RADIO_STATE_RXRU)
        radio_set_state(soc, NRF_RADIO_STATE_RXIDLE);
    else if (r->state == NRF_RADIO_STATE_TXRU)
        radio_set_state(soc, NRF_RADIO_STATE_TXIDLE);
}

/* Enter the RXRU/TXRU ramp state and schedule its 40 µs completion. */
static void radio_start_rampup(nrf52840_soc_t *soc, uint32_t ru_state) {
    soc->radio.state = ru_state;
    if (nrf_trace_flag(&trc_radio, "NRF_RADIO_TRACE"))
        fprintf(stderr, "[ramp] start state=%d ev=%p @cyc=%lld\n", ru_state,
                (void*)soc->radio.ramp_event, (long long)soc->plat->cpu.cycles);
    if (soc->radio.ramp_event && soc->plat)
        arm_schedule_event(&soc->plat->cpu, (arm_event_t *)soc->radio.ramp_event,
                           (int64_t)soc->plat->cpu.cycles + RADIO_RAMP_UP_CYCLES);
}

/* Force-complete a deferred TX still in flight.  nrf_802154 arms the next
 * operation (RXEN to await the ACK) the moment it has finished issuing the
 * transmit, before our modelled air-time elapses — so without this the pending
 * tx_event would later find state != TX and drop the frame on the floor.  When
 * the firmware moves the radio on, finish the transmit synchronously instead. */
static void radio_complete_pending_tx(nrf52840_soc_t *soc) {
    if (soc->radio.state != NRF_RADIO_STATE_TX) return;
    if (soc->radio.tx_event && soc->plat)
        arm_cancel_event(&soc->plat->cpu, (arm_event_t *)soc->radio.tx_event);
    nrf_radio_tx_done_cb(soc, NULL);
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
            radio_apply_shorts_after_event(soc, SHORT_RXREADY_CCASTART);
            break;
        case NRF_RADIO_STATE_TXIDLE:
            radio_event(soc, &r->evt_ready,   RADIO_INT_READY);
            radio_event(soc, &r->evt_txready, RADIO_INT_TXREADY);
            radio_apply_shorts_after_event(soc, SHORT_READY_START);
            radio_apply_shorts_after_event(soc, SHORT_TXREADY_START);
            break;
        case NRF_RADIO_STATE_DISABLED:
            /* DISABLED events are now deferred via nrf_radio_disable_cb
             * (see RADIO_TASKS_DISABLE).  If we get here from a non-DISABLE
             * path (e.g. direct state write), fire synchronously as
             * fallback. */
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
             * implicit transition. Mirror that lenience.
             * If we're in a DISABLE intermediate state (deferred event
             * pending), cancel it and fire DISABLED synchronously first
             * so the driver's DISABLED poll resolves, then proceed. */
            if (r->state == NRF_RADIO_STATE_TXDISABLE ||
                r->state == NRF_RADIO_STATE_RXDISABLE) {
                if (r->disable_event && soc->plat)
                    arm_cancel_event(&soc->plat->cpu, (arm_event_t *)r->disable_event);
                r->state = NRF_RADIO_STATE_DISABLED;
                radio_event(soc, &r->evt_disabled, RADIO_INT_DISABLED);
            }
            if (r->state == NRF_RADIO_STATE_DISABLED)
                radio_start_rampup(soc, NRF_RADIO_STATE_TXRU);
            else if (r->state != NRF_RADIO_STATE_TX &&
                     r->state != NRF_RADIO_STATE_TXRU &&
                     r->state != NRF_RADIO_STATE_TXIDLE)
                radio_set_state(soc, NRF_RADIO_STATE_TXIDLE);
            break;
        case RADIO_TASKS_RXEN:
            /* If a deferred TX is still in flight, finish it before switching
             * to RX (the driver arms RXEN to receive the ACK right after the
             * transmit) so the frame is actually sent. */
            radio_complete_pending_tx(soc);
            if (r->state == NRF_RADIO_STATE_TXDISABLE ||
                r->state == NRF_RADIO_STATE_RXDISABLE) {
                if (r->disable_event && soc->plat)
                    arm_cancel_event(&soc->plat->cpu, (arm_event_t *)r->disable_event);
                r->state = NRF_RADIO_STATE_DISABLED;
                radio_event(soc, &r->evt_disabled, RADIO_INT_DISABLED);
            }
            if (r->state == NRF_RADIO_STATE_DISABLED) {
                /* If a just-received frame's CRCOK is still awaiting its ISR
                 * (rxframe_finish busy-waits for STATE==DISABLED), the firmware
                 * re-arming RX here would ramp the radio out of DISABLED before
                 * the ISR runs — and it would spin out.  Defer the ramp-up until
                 * the ISR clears CRCOK (handled in the EVENTS_CRCOK write). */
                if (r->evt_crcok)
                    r->rxen_pending = 1;
                else
                    radio_start_rampup(soc, NRF_RADIO_STATE_RXRU);
            } else if (r->state != NRF_RADIO_STATE_RX &&
                     r->state != NRF_RADIO_STATE_RXRU &&
                     r->state != NRF_RADIO_STATE_RXIDLE)
                radio_set_state(soc, NRF_RADIO_STATE_RXIDLE);
            break;
        case RADIO_TASKS_START:
            /* TXIDLE → TX → (instantly transmit) → TXIDLE.
             * RXIDLE → RX (stay there until a frame arrives). */
            if (r->state == NRF_RADIO_STATE_TXIDLE) {
                /* Begin transmitting: enter TX state, fire the start-of-frame
                 * events now (ADDRESS/FRAMESTART), and DEFER the completion
                 * (PAYLOAD/END/PHYEND + the frame emission + shorts) by one
                 * air-time so the driver's busy-wait/poll and PHYEND IRQ land
                 * correctly.  Firing the whole transmit instantly desynced
                 * nrf_802154's calibrated delays → a re-transmit storm. */
                /* Drop any buffered RX bytes — they pre-date our own TX
                 * and are stale. */
                r->rx_incoming_len = 0;
                r->state = NRF_RADIO_STATE_TX;
                radio_event(soc, &r->evt_address,    RADIO_INT_ADDRESS);
                radio_event(soc, &r->evt_framestart, RADIO_INT_FRAMESTART);
                /* Emit the frame to the medium NOW, at start-of-air; the
                 * per-byte bus spreads the bytes over their true air time
                 * for each receiver.  Emission used to happen in
                 * nrf_radio_tx_done_cb, one air-time later, which put
                 * every frame on the air one frame-duration LATE: a TSCH
                 * node syncing to this radio's EBs inherited that lateness
                 * into its whole slot grid, and its unicast replies then
                 * missed the coordinator's on-grid RX window (every
                 * keepalive NOACK).  The completion events (PAYLOAD/END/
                 * PHYEND + post-TX shorts) stay deferred by one air-time
                 * below, so nrf_802154's calibrated busy-waits still see
                 * PHYEND at the right time. */
                /* Serialize with a still-staged deferred auto-ACK: the
                 * driver may start its own TX inside the 192 us ACK
                 * turnaround (e.g. RPL replying to the very frame being
                 * acked).  Flush the ACK onto the bus first — the bus's
                 * per-sender byte clock then air-schedules the ACK and
                 * this frame back-to-back, instead of interleaving two
                 * byte streams into garbage. */
                /* Firmware is transmitting (typically its OWN ack for
                 * the frame we just received): cancel any staged chip
                 * auto-ACK so acknowledgements are never duplicated. */
                if (r->ack_pending_len > 0) {
                    if (r->ack_event && soc->plat)
                        arm_cancel_event(&soc->plat->cpu,
                                         (arm_event_t *)r->ack_event);
                    r->ack_pending_len = 0;
                }
                radio_emit_tx(soc);
                if (r->tx_event && soc->plat) {
                    uint32_t phr = arm_read8(&soc->plat->cpu, r->packetptr);
                    if (phr < 2 || phr > 127) phr = 127;
                    int64_t air = (int64_t)(RADIO_TX_SHR_PHR_BYTES + phr) *
                                  RADIO_TX_BYTE_CYCLES;
                    arm_schedule_event(&soc->plat->cpu,
                        (arm_event_t *)r->tx_event,
                        (int64_t)soc->plat->cpu.cycles + air);
                }
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
            if (r->state == NRF_RADIO_STATE_TX) {
                r->state = NRF_RADIO_STATE_TXIDLE;
                if (r->tx_event && soc->plat)
                    arm_cancel_event(&soc->plat->cpu, (arm_event_t *)r->tx_event);
            }
            break;
        case RADIO_TASKS_DISABLE:
            /* Defer the DISABLED event so the firmware's
             * "clear EVENTS_DISABLED → poll" pattern works.  Set an
             * intermediate state immediately so SHORTS don't re-trigger,
             * and schedule the real DISABLED event after ramp-down. */
            if (r->state == NRF_RADIO_STATE_DISABLED) {
                /* Already disabled — just fire the event synchronously
                 * (no ramp-down needed). */
                radio_event(soc, &r->evt_disabled, RADIO_INT_DISABLED);
            } else {
                /* Abort any in-flight ramp-up or deferred transmit. */
                if (r->ramp_event && soc->plat)
                    arm_cancel_event(&soc->plat->cpu, (arm_event_t *)r->ramp_event);
                if (r->tx_event && soc->plat)
                    arm_cancel_event(&soc->plat->cpu, (arm_event_t *)r->tx_event);
                if (r->state == NRF_RADIO_STATE_TX ||
                    r->state == NRF_RADIO_STATE_TXIDLE ||
                    r->state == NRF_RADIO_STATE_TXRU)
                    r->state = NRF_RADIO_STATE_TXDISABLE;
                else
                    r->state = NRF_RADIO_STATE_RXDISABLE; /* RX/RXIDLE/RXRU/catch-all */
                /* Schedule the DISABLED event after ramp-down delay */
                if (r->disable_event && soc->plat) {
                    arm_schedule_event(&soc->plat->cpu,
                        (arm_event_t *)r->disable_event,
                        (int64_t)soc->plat->cpu.cycles + NRF_RADIO_DISABLE_CYCLES);
                }
            }
            break;
        case RADIO_TASKS_CCASTART:
            /* No interference model — always idle. */
            radio_event(soc, &r->evt_ccaidle, RADIO_INT_CCAIDLE);
            radio_apply_shorts_after_event(soc, SHORT_CCAIDLE_TXEN);
            radio_apply_shorts_after_event(soc, SHORT_CCAIDLE_STOP);
            break;
        case RADIO_TASKS_RSSISTART:
            radio_event(soc, &r->evt_rssiend, 0);
            break;
        case RADIO_TASKS_EDSTART:
            radio_event(soc, &r->evt_edend,   RADIO_INT_EDEND);
            break;
        default:
            break;
    }
}

static int nrf_radio_read(void *user_data, uint32_t addr) {
    nrf52840_soc_t *soc = (nrf52840_soc_t *)user_data;
    nrf_radio_state_t *r = &soc->radio;
    uint32_t off = addr - NRF_RADIO_BASE;
    if (nrf_trace_flag(&trc_radio, "NRF_RADIO_TRACE")) {
        if (off == RADIO_STATE) fprintf(stderr, "[radioR] STATE = %d\n", r->state);
        else fprintf(stderr, "[radioR] 0x%03x\n", off);
    }
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
        case RADIO_CRCSTATUS:    return (int)r->crcstatus;
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
    if (nrf_trace_flag(&trc_radio, "NRF_RADIO_TRACE")) fprintf(stderr, "[radioW] 0x%03x = 0x%08x\n", off, value);

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
            case RADIO_EVENTS_END:
                r->evt_end = value & 1;
                /* RIOT's nrf802154 driver consumes a received frame via
                 * EVENTS_END + CRCSTATUS and never clears EVENTS_CRCOK.  So when
                 * the frame is consumed (END cleared), drop the stale latched
                 * CRCOK — otherwise the driver's *subsequent* RX re-arm (RXEN)
                 * sees CRCOK still set, gets deferred (rxen_pending), and the
                 * radio parks DISABLED forever (the node goes deaf after its
                 * first frame).  Zephyr's nrf_802154 clears CRCOK itself, before
                 * its re-arm, so the latch is already gone here and this is a
                 * no-op for it. */
                if (!(value & 1))
                    r->evt_crcok = 0;
                break;
            case RADIO_EVENTS_DISABLED:     r->evt_disabled   = value & 1; break;
            case RADIO_EVENTS_DEVMATCH:     r->evt_devmatch   = value & 1; break;
            case RADIO_EVENTS_DEVMISS:      r->evt_devmiss    = value & 1; break;
            case RADIO_EVENTS_RSSIEND:      r->evt_rssiend    = value & 1; break;
            case RADIO_EVENTS_BCMATCH:      r->evt_bcmatch    = value & 1; break;
            case RADIO_EVENTS_CRCOK:
                r->evt_crcok = value & 1;
                /* The ISR dispatcher clears CRCOK BEFORE running rxframe_finish
                 * (which busy-waits STATE==DISABLED), so just drop the deferred
                 * re-arm here and keep the radio DISABLED.  The driver's own
                 * receive_frame_received (after rxframe_finish) re-arms RX, and
                 * by then evt_crcok is clear so that RXEN ramps normally. */
                if (!(value & 1))
                    r->rxen_pending = 0;
                break;
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
    if (nrf_trace_flag(&trc_rxbyte, "NRF_RXBYTE_TRACE"))
        fprintf(stderr, "[rxbyte] n=%u state=%d phase=%d byte=%02x @ %.6f\n",
                soc->ficr.deviceaddr1, r->state, r->rx_phase, byte,
                (double)arm_cycles_to_ns(soc->plat->cpu.cycles, soc->plat->cpu.cpu_freq_hz)/1e9);
    /* Outside RX: buffer the byte for replay on next RX entry.  This
     * catches auto-ACK bytes that arrive 192 µs after our own TX,
     * before the driver has finished the TXIDLE → DISABLED → RXEN →
     * RXIDLE → RX state walk.  Buffer is cleared on TX entry so we
     * never carry stale bytes across a TX cycle. */
    if (r->state != NRF_RADIO_STATE_RX) {
        /* Only buffer bytes that arrive while the radio is actually arming for
         * RX (RXRU ramp / RXIDLE awaiting START) — e.g. an auto-ACK 192 µs after
         * our own TX, before the RXEN→RX walk finishes; those replay when RX is
         * reached.  A DISABLED radio (or one busy in a TX state) is NOT
         * listening — drop the bytes, exactly as hardware does.  Buffering them
         * let an idle node whose radio sits DISABLED between sends accumulate
         * many frames of stale bytes in rx_incoming, which then replayed as
         * garbage and corrupted the next real RX (RIOT RPL: node never saw a
         * clean DIO). */
        if (r->state == NRF_RADIO_STATE_RXRU || r->state == NRF_RADIO_STATE_RXIDLE) {
            if (r->rx_incoming_len < (int)sizeof(r->rx_incoming))
                r->rx_incoming[r->rx_incoming_len++] = byte;
        }
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
            r->rx_crc       = 0;
            r->rx_phase     = NRF_RX_READ_PAYLOAD;
            break;
        case NRF_RX_READ_PAYLOAD:
            arm_write8(cpu, r->packetptr + (uint32_t)r->rx_offset, byte);
            if (r->rx_remaining > 2)
                r->rx_crc = ieee802154_crc_add_bitrev(r->rx_crc, byte);
            else
                r->rx_fcs[2 - r->rx_remaining] = byte;
            r->rx_offset++;
            r->rx_remaining--;
            if (r->rx_remaining == 0) {
                /* Frame complete — verify the FCS over the bytes that
                 * actually arrived.  Under collisions the per-byte medium
                 * interleaves overlapping frames; on silicon those garbled
                 * assemblies fail the hardware CRC and the driver never
                 * sees them, so report CRCSTATUS honestly here too. */
                int crc_ok =
                    r->rx_fcs[0] == ieee802154_bitrev((uint8_t)((r->rx_crc >> 8) & 0xFF)) &&
                    r->rx_fcs[1] == ieee802154_bitrev((uint8_t)(r->rx_crc & 0xFF));
                r->crcstatus = crc_ok ? 1 : 0;
                radio_event(soc, &r->evt_payload,  RADIO_INT_PAYLOAD);
                radio_event(soc, &r->evt_end,      RADIO_INT_END);
                /* PHYEND has no RX interrupt enabled (nrf_802154 keys RX off
                 * CRCOK), so latch it but it raises no ISR here. */
                radio_event(soc, &r->evt_phyend,   RADIO_INT_PHYEND);

                /* Apply the END/PHYEND_DISABLE short BEFORE raising CRCOK.
                 * nrf_802154's CRCOK ISR (irq_handler_crcok) runs rxframe_finish(),
                 * which busy-waits for STATE==DISABLED — on silicon the END_DISABLE
                 * short completes a few cycles after CRCOK, during that wait.  csim
                 * takes the ISR synchronously the moment CRCOK is raised, so the
                 * radio must ALREADY read DISABLED or rxframe_finish spins out its
                 * whole MAX_RAMPDOWN budget (3200 cycles) and the receive stalls.
                 * Latch DISABLED but don't fire PPI / DISABLED_* shorts here — the
                 * driver's CRCOK ISR reconfigures PPI and re-enables explicitly. */
                if (r->shorts & (SHORT_END_DISABLE | SHORT_PHYEND_DISABLE)) {
                    r->state = NRF_RADIO_STATE_DISABLED;
                    r->evt_disabled = 1;
                }
                if (crc_ok)
                    radio_event(soc, &r->evt_crcok,     RADIO_INT_CRCOK);
                else
                    radio_event(soc, &r->evt_crcerror,  RADIO_INT_CRCERROR);
                radio_apply_shorts_after_event(soc, SHORT_END_START);
                radio_apply_shorts_after_event(soc, SHORT_PHYEND_START);

                /* Deferred chip auto-ACK, 96 us out — a stand-in for
                 * firmware stacks whose own ACK path isn't modelled
                 * (Zephyr's nrf_802154 hardware ACK choreography).  Two
                 * gates keep it from DUPLICATING acknowledgements:
                 *  - frame version 2 (802.15.4e/TSCH) never gets one:
                 *    those need MAC-built ENHANCED ACKs, and a fabricated
                 *    imm-ACK burned the sender's ACK slot as NOACK;
                 *  - if the firmware transmits ANYTHING before the 96 us
                 *    elapse (Contiki's stack sends its own ACK ~60 us
                 *    after rx-complete), the staged fiction is cancelled
                 *    at TASKS_START — otherwise the two identical ACKs
                 *    interleaved on air and corrupted each other. */
                if (soc->radio_tx_cb && crc_ok && r->rx_offset > 4) {
                    uint8_t fcf0       = arm_read8(cpu, r->packetptr + 1);
                    uint8_t fcf1       = arm_read8(cpu, r->packetptr + 2);
                    uint8_t dsn        = arm_read8(cpu, r->packetptr + 3);
                    int     frame_type = fcf0 & 0x07;
                    int     ack_req    = (fcf0 >> 5) & 1;
                    int     frame_ver  = (fcf1 >> 4) & 0x3;
                    if (frame_type == 0x1 /* Data */ && ack_req && frame_ver < 2) {
                        uint8_t ack_fcf0 = 0x02; /* frame type = ACK */
                        uint8_t ack_fcf1 = 0x00;
                        uint16_t crc = 0;
                        crc = ieee802154_crc_add_bitrev(crc, ack_fcf0);
                        crc = ieee802154_crc_add_bitrev(crc, ack_fcf1);
                        crc = ieee802154_crc_add_bitrev(crc, dsn);
                        int n = 0;
                        for (int i = 0; i < IEEE802154_PREAMBLE_LEN; i++)
                            r->ack_pending[n++] = IEEE802154_PREAMBLE_BYTE;
                        r->ack_pending[n++] = IEEE802154_SFD;
                        r->ack_pending[n++] = 5; /* PHR: FCF(2)+DSN(1)+FCS(2) */
                        r->ack_pending[n++] = ack_fcf0;
                        r->ack_pending[n++] = ack_fcf1;
                        r->ack_pending[n++] = dsn;
                        r->ack_pending[n++] = ieee802154_bitrev((uint8_t)((crc >> 8) & 0xFF));
                        r->ack_pending[n++] = ieee802154_bitrev((uint8_t)(crc & 0xFF));
                        r->ack_pending_len = n;
                        if (r->ack_event && soc->plat)
                            arm_schedule_event(&soc->plat->cpu,
                                (arm_event_t *)r->ack_event,
                                (int64_t)soc->plat->cpu.cycles + 96LL * 64);
                    }
                }

                if (nrf_trace_flag(&trc_rx, "NRF_RX_TRACE")) {
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
#define NRF_RNG_INT_VALRDY        (1u << 0)
#define NRF_RNG_IRQ               13      /* RNG_IRQn on nRF52840 */
/* RNG VALRDY pacing.  The initial delay is large enough that the first
 * byte fires well AFTER Zephyr's prepare_multithreading() has run —
 * otherwise any ISR during pre-kernel triggers z_arm_exc_exit's PendSV
 * check while ready_q.cache is still NULL, crashing the boot.  Once the
 * first byte is delivered (POST_KERNEL), subsequent bytes use fast
 * pacing so the entropy pool fills quickly for nrf_802154_random_init. */
#define NRF_RNG_VALRDY_CYCLES_INITIAL  500000  /* ~7.8 ms @ 64 MHz */
#define NRF_RNG_VALRDY_CYCLES_FAST     1000    /* ~15.6 µs @ 64 MHz */

/* Paced VALRDY generator.  Real HW produces one random byte roughly every
 * ~167 µs and raises VALRDY each time; the entropy_nrf5 driver takes one byte
 * per interrupt until its pool is full, then disables the interrupt.  Delivering
 * bytes *instantly* (back to back with no time between) storms the NVIC and
 * corrupts the exception frame, so we pace them through a scheduled event.
 * Contiki polls VALRDY (INTENSET stays 0) so it never arms this → byte-identical.*/
static void nrf_rng_arm(nrf_rng_state_t *rng) {
    nrf52840_soc_t *soc = (nrf52840_soc_t *)rng->soc;
    if (!soc || !soc->plat || !rng->valrdy_event) return;
    if (!rng->running || !(rng->intenset & NRF_RNG_INT_VALRDY)) return;
    int64_t delay = rng->first_byte_delivered
                  ? NRF_RNG_VALRDY_CYCLES_FAST
                  : NRF_RNG_VALRDY_CYCLES_INITIAL;
    arm_schedule_event(&soc->plat->cpu, (arm_event_t *)rng->valrdy_event,
                       (int64_t)soc->plat->cpu.cycles + delay);
}

static void nrf_rng_event_cb(void *user_data, cpu_event_t *event) {
    (void)event;
    nrf_rng_state_t *rng = (nrf_rng_state_t *)user_data;
    nrf52840_soc_t *soc = (nrf52840_soc_t *)rng->soc;
    if (!soc || !soc->plat || !rng->running) return;
    rng->evt_valrdy = 1;
    rng->first_byte_delivered = true;
    if ((rng->intenset & NRF_RNG_INT_VALRDY) && soc->plat->cpu.nvic)
        arm_nvic_set_pending((arm_nvic_t *)soc->plat->cpu.nvic, rng->irq_num);
    nrf_rng_arm(rng);   /* keep producing while running + enabled */
}

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
            if (value == 1) {
                rng->running = true;
                rng->first_byte_delivered = false;
                /* First byte: instant for pollers (Contiki reads VALRDY then
                 * VALUE); the paced event drives interrupt-driven consumers.
                 * The initial VALRDY event is intentionally delayed (see
                 * NRF_RNG_VALRDY_CYCLES_INITIAL) so it cannot fire during
                 * Zephyr's pre-kernel phase. */
                rng->evt_valrdy = 1;
                nrf_rng_arm(rng);
            }
            break;
        case NRF_RNG_TASKS_STOP:
            if (value == 1) {
                rng->running = false;
                if (rng->valrdy_event)
                    arm_cancel_event(&((nrf52840_soc_t *)rng->soc)->plat->cpu,
                                     (arm_event_t *)rng->valrdy_event);
            }
            break;
        case NRF_RNG_EVENTS_VALRDY:
            /* Firmware acks VALRDY by writing 0.  Pollers (Contiki) see it
             * re-set instantly (PRNG is always ready); interrupt-driven
             * consumers get the next byte from the paced event, not here. */
            rng->evt_valrdy = (value & 1) ? 1 : (rng->running ? 1 : 0);
            break;
        case NRF_RNG_SHORTS:        rng->shorts     = value;     break;
        case NRF_RNG_INTENSET:      rng->intenset  |= value;
                                    nrf_rng_arm(rng); break;
        case NRF_RNG_INTENCLR:      rng->intenset  &= ~value;    break;
        default: break;
    }
}

/* ============================================================
 * PPI at 0x4001F000 — Programmable Peripheral Interconnect.
 * Routes EVENT → TASK in hardware.  nrf_802154 uses it to ramp the RADIO up
 * (TASKS_RXEN/TXEN) off a TIMER/event, so the radio never enables without it.
 * ============================================================ */
#define NRF_PPI_BASE       0x4001F000u
#define NRF_PPI_CHEN       0x500
#define NRF_PPI_CHENSET    0x504
#define NRF_PPI_CHENCLR    0x508
#define NRF_PPI_CH_EEP0    0x510   /* CH[n].EEP = 0x510 + n*8, TEP = +4 */

static int nrf_ppi_read(void *user_data, uint32_t addr) {
    nrf_ppi_state_t *ppi = (nrf_ppi_state_t *)user_data;
    uint32_t off = addr - NRF_PPI_BASE;
    if (off == NRF_PPI_CHEN || off == NRF_PPI_CHENSET || off == NRF_PPI_CHENCLR)
        return (int)ppi->chen;
    if (off >= NRF_PPI_CH_EEP0 && off < NRF_PPI_CH_EEP0 + 20 * 8) {
        uint32_t i = (off - NRF_PPI_CH_EEP0) / 8;
        return (off & 4) ? (int)ppi->tep[i] : (int)ppi->eep[i];
    }
    return 0;
}

/* Check if a PPI event address (absolute) points at a currently-latched
 * RADIO or EGU event.  Returns true if the event register reads as 1.
 * Used for level-sensitive PPI: if you enable a channel while its EEP is
 * already high, the task fires immediately — matching real PPI hardware. */
static bool ppi_event_is_latched(nrf52840_soc_t *soc, uint32_t event_addr) {
    /* RADIO events at NRF_RADIO_BASE + 0x100..0x16C */
    if (event_addr >= NRF_RADIO_BASE + 0x100 &&
        event_addr < NRF_RADIO_BASE + 0x200) {
        return arm_read32(&soc->plat->cpu, event_addr) != 0;
    }
    /* EGU0 events at 0x40014100..0x4001413C */
    if (event_addr >= 0x40014100u && event_addr < 0x40014140u) {
        int n = (int)(event_addr - 0x40014100u) / 4;
        return (soc->egu0.events >> n) & 1;
    }
    return false;
}

static void nrf_ppi_write(void *user_data, uint32_t addr, uint32_t value) {
    nrf_ppi_state_t *ppi = (nrf_ppi_state_t *)user_data;
    nrf52840_soc_t *soc = (nrf52840_soc_t *)ppi->soc;
    uint32_t off = addr - NRF_PPI_BASE;
    if (nrf_trace_flag(&trc_ppi, "NRF_PPI_TRACE")) fprintf(stderr, "[ppiW] 0x%03x = 0x%08x\n", off, value);
    if (off == NRF_PPI_CHEN || off == NRF_PPI_CHENSET) {
        uint32_t newly_enabled = value & ~ppi->chen;
        ppi->chen |= value;
        /* Level-sensitive: if a newly-enabled channel's EEP event is
         * already latched, fire its TEP immediately.  This is how real
         * PPI works and is critical for the nrf_802154 ACK path: the
         * ISR reconfigures CH7 (EGU→TXEN), re-enables the channel,
         * and expects the already-latched EGU TRIGGERED event to fire
         * TXEN immediately. */
        if (soc) {
            for (int i = 0; i < 20; i++) {
                if ((newly_enabled & (1u << i)) && ppi->eep[i] && ppi->tep[i] &&
                    ppi_event_is_latched(soc, ppi->eep[i]))
                    arm_write32(&soc->plat->cpu, ppi->tep[i], 1);
            }
        }
        return;
    }
    if (off == NRF_PPI_CHENCLR) { ppi->chen &= ~value; return; }
    if (off >= NRF_PPI_CH_EEP0 && off < NRF_PPI_CH_EEP0 + 20 * 8) {
        uint32_t i = (off - NRF_PPI_CH_EEP0) / 8;
        if (off & 4) ppi->tep[i] = value; else ppi->eep[i] = value;
        return;
    }
}

/* Called whenever csim raises a peripheral EVENT (passes the absolute event
 * register address).  Fires the TASK of every enabled channel whose EEP matches
 * — i.e. the hardware event→task routing nrf_802154 relies on. */
void nrf_ppi_event_notify(nrf52840_soc_t *soc, uint32_t event_addr) {
    nrf_ppi_state_t *ppi = &soc->ppi;
    if (!ppi->chen) return;
    for (int i = 0; i < 20; i++) {
        if ((ppi->chen & (1u << i)) && ppi->eep[i] == event_addr && ppi->tep[i])
            arm_write32(&soc->plat->cpu, ppi->tep[i], 1);
    }
}

/* ============================================================
 * EGU0/SWI0 at 0x40014000 — Event Generator Unit.
 * TASKS_TRIGGER[n] (0x000+n*4) latches EVENTS_TRIGGERED[n] (0x100+n*4), which
 * routes onward through PPI (and raises the SWI IRQ if enabled).
 * ============================================================ */
#define NRF_EGU0_BASE      0x40014000u
#define NRF_EGU_IRQ        20      /* SWI0_EGU0 */

static void nrf_egu_fire(nrf_egu_state_t *egu, int n) {
    nrf52840_soc_t *soc = (nrf52840_soc_t *)egu->soc;
    egu->events |= (1u << n);
    nrf_ppi_event_notify(soc, NRF_EGU0_BASE + 0x100 + n * 4);
    if ((egu->intenset & (1u << n)) && soc->plat && soc->plat->cpu.nvic)
        arm_nvic_set_pending((arm_nvic_t *)soc->plat->cpu.nvic, egu->irq_num);
}

static int nrf_egu_read(void *user_data, uint32_t addr) {
    nrf_egu_state_t *egu = (nrf_egu_state_t *)user_data;
    uint32_t off = addr - NRF_EGU0_BASE;
    if (off >= 0x100 && off < 0x140) return (egu->events >> ((off - 0x100) / 4)) & 1;
    if (off == 0x304 || off == 0x308) return (int)egu->intenset;
    return 0;
}

static void nrf_egu_write(void *user_data, uint32_t addr, uint32_t value) {
    nrf_egu_state_t *egu = (nrf_egu_state_t *)user_data;
    uint32_t off = addr - NRF_EGU0_BASE;
    if (off < 0x40) {                       /* TASKS_TRIGGER[0..15] */
        if (value == 1) nrf_egu_fire(egu, off / 4);
        return;
    }
    if (off >= 0x100 && off < 0x140) {      /* EVENTS_TRIGGERED ack */
        if (!(value & 1)) egu->events &= ~(1u << ((off - 0x100) / 4));
        return;
    }
    if (off == 0x304) { egu->intenset |= value; return; }
    if (off == 0x308) { egu->intenset &= ~value; return; }
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
 * TEMP at 0x4000C000 — on-die temperature sensor.
 * nrf_802154 measures die temperature periodically for radio calibration; the
 * driver writes TASKS_START and blocks on a semaphore until DATARDY.  Complete
 * it instantly so the read returns and the workq (and DAD) proceeds.
 * ============================================================ */
#define NRF_TEMP_BASE            0x4000C000u
#define NRF_TEMP_TASKS_START     0x000
#define NRF_TEMP_TASKS_STOP      0x004
#define NRF_TEMP_EVENTS_DATARDY  0x100
#define NRF_TEMP_INTENSET        0x304
#define NRF_TEMP_INTENCLR        0x308
#define NRF_TEMP_TEMP            0x508
#define NRF_TEMP_INT_DATARDY     (1u << 0)

static int nrf_temp_read(void *user_data, uint32_t addr) {
    nrf_temp_state_t *temp = (nrf_temp_state_t *)user_data;
    switch (addr - NRF_TEMP_BASE) {
        case NRF_TEMP_EVENTS_DATARDY: return (int)temp->evt_datardy;
        case NRF_TEMP_INTENSET:       return (int)temp->intenset;
        case NRF_TEMP_TEMP:           return 100;  /* 25.0 C in 0.25 C units */
        default:                      return 0;
    }
}

static void nrf_temp_write(void *user_data, uint32_t addr, uint32_t value) {
    nrf_temp_state_t *temp = (nrf_temp_state_t *)user_data;
    nrf52840_soc_t *soc = (nrf52840_soc_t *)temp->soc;
    switch (addr - NRF_TEMP_BASE) {
        case NRF_TEMP_TASKS_START:
            /* Measurement completes immediately: latch DATARDY + raise IRQ. */
            temp->evt_datardy = 1;
            if ((temp->intenset & NRF_TEMP_INT_DATARDY) && soc && soc->plat &&
                soc->plat->cpu.nvic)
                arm_nvic_set_pending((arm_nvic_t *)soc->plat->cpu.nvic, temp->irq_num);
            break;
        case NRF_TEMP_TASKS_STOP:     break;
        case NRF_TEMP_EVENTS_DATARDY: temp->evt_datardy = value & 1; break;  /* ISR clears */
        case NRF_TEMP_INTENSET:       temp->intenset |= value; break;
        case NRF_TEMP_INTENCLR:       temp->intenset &= ~value; break;
        default:                      break;
    }
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

/* Forward declarations for timer compare event scheduling. */
static void timer_schedule_next_compare(nrf_timer_state_t *t);
static void timer_cancel_compare(nrf_timer_state_t *t);

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
        if (t->running) timer_schedule_next_compare(t);
        return;
    }
    switch (off) {
        case TIMER_TASKS_START:
            if (value == 1 && !t->running) {
                t->counter_at_start = timer_compute_counter(t, (int64_t)cpu->cycles, cpu->cpu_freq_hz);
                t->start_cycles = (int64_t)cpu->cycles;
                t->running = true;
                timer_schedule_next_compare(t);
            }
            break;
        case TIMER_TASKS_STOP:
        case TIMER_TASKS_SHUTDOWN:
            if (value == 1 && t->running) {
                t->counter_at_start = timer_compute_counter(t, (int64_t)cpu->cycles, cpu->cpu_freq_hz);
                t->start_cycles = (int64_t)cpu->cycles;
                t->running = false;
                timer_cancel_compare(t);
            }
            break;
        case TIMER_TASKS_CLEAR:
            if (value == 1) {
                t->counter_at_start = 0;
                t->start_cycles = (int64_t)cpu->cycles;
                if (t->running) timer_schedule_next_compare(t);
            }
            break;
        case TIMER_SHORTS:    t->shorts    = value; break;
        case TIMER_INTENSET:
            /* Enabling a COMPARE interrupt must (re)schedule its event: drivers
             * commonly write CC[chan] first and INTENSET second (e.g. RIOT's
             * nrf5x timer_set / ztimer), so the CC-write reschedule saw no
             * listener yet.  Without this the compare never fires. */
            t->intenset |= value;
            if (t->running) timer_schedule_next_compare(t);
            break;
        case TIMER_INTENCLR:  t->intenset &= ~value; break;
        case TIMER_MODE:      t->mode      = value & 3; break;
        case TIMER_BITMODE:   t->bitmode   = value & 3; break;
        case TIMER_PRESCALER: t->prescaler = value & 0xF; break;
        default: break;
    }
}

/* --- TIMER COMPARE event scheduling ---
 * The nrf_802154 ACK path uses TIMER0 CC1 + PPI to trigger RADIO TXEN
 * at the exact turnaround time. Without COMPARE events firing, the PPI
 * chain never executes and ACKs never transmit. */

static void timer_compare_fire(nrf_timer_state_t *t, int i) {
    nrf52840_soc_t *soc = t->soc;
    if (!soc || !soc->plat) return;
    t->evt_compare[i] = 1;
    uint32_t int_bit = (1u << (16 + i));
    if ((t->intenset & int_bit) && soc->plat->cpu.nvic)
        arm_nvic_set_pending((arm_nvic_t *)soc->plat->cpu.nvic, t->irq_num);
    nrf_ppi_event_notify(soc, t->base + TIMER_EVENTS_COMPARE_0 + (uint32_t)i * 4);
    if (t->shorts & (1u << i)) {          /* COMPARE[i]_CLEAR */
        t->counter_at_start = 0;
        t->start_cycles = (int64_t)soc->plat->cpu.cycles;
    }
    if (t->shorts & (1u << (8 + i))) {    /* COMPARE[i]_STOP */
        t->counter_at_start = timer_compute_counter(t, (int64_t)soc->plat->cpu.cycles, soc->plat->cpu.cpu_freq_hz);
        t->running = false;
    }
}

static void timer_compare_event_cb(void *user_data, cpu_event_t *event) {
    (void)event;
    nrf_timer_state_t *t = (nrf_timer_state_t *)user_data;
    if (!t->soc || !t->soc->plat || !t->running) return;
    int i = t->next_cc;
    if (i < 0 || i >= 6) return;
    t->next_cc = -1;
    timer_compare_fire(t, i);
    timer_schedule_next_compare(t);
}

static void timer_schedule_next_compare(nrf_timer_state_t *t) {
    nrf52840_soc_t *soc = t->soc;
    if (!soc || !soc->plat || !t->running || !t->compare_event) return;
    arm_cpu_t *cpu = &soc->plat->cpu;
    uint32_t mask = timer_bit_mask(t->bitmode);
    uint32_t now = timer_compute_counter(t, (int64_t)cpu->cycles, cpu->cpu_freq_hz);
    uint64_t tick_hz = (uint64_t)TIMER_BASE_HZ >> t->prescaler;
    if (tick_hz == 0) tick_hz = 1;

    int best_i = -1;
    int64_t best_delta = INT64_MAX;
    for (int i = 0; i < 6; i++) {
        /* Only schedule if something listens: INTENSET bit or PPI EEP. */
        uint32_t int_bit = (1u << (16 + i));
        bool has_listener = (t->intenset & int_bit) != 0;
        if (!has_listener) {
            uint32_t evt_addr = t->base + TIMER_EVENTS_COMPARE_0 + (uint32_t)i * 4;
            for (int p = 0; p < 20 && !has_listener; p++)
                if ((soc->ppi.chen & (1u << p)) && soc->ppi.eep[p] == evt_addr)
                    has_listener = true;
        }
        if (!has_listener) continue;
        /* Distance to the compare value on the wrapping counter.  Do the
         * "equal means a full wrap" case in 64-bit: with BITMODE=32 the
         * mask is 0xFFFFFFFF and a uint32 mask+1 overflowed to 0, which
         * the dc<1 clamp turned into a compare event every single CPU
         * cycle.  Contiki's rtimer-arch starts TIMER0 (32-bit, CC0=0,
         * COMPARE0 int enabled) with counter==CC0, hitting exactly this:
         * a boot-time interrupt storm that pinned the CPU in the TIMER0
         * ISR so TSCH slot operation never ran. */
        uint64_t ticks = (uint64_t)((t->cc[i] - now) & mask);
        if (ticks == 0) ticks = (uint64_t)mask + 1;
        int64_t dc = (int64_t)(ticks * (uint64_t)cpu->cpu_freq_hz / tick_hz);
        if (dc < 1) dc = 1;  /* avoid 0-delay infinite reschedule */
        if (dc < best_delta) { best_delta = dc; best_i = i; }
    }
    if (best_i >= 0) {
        t->next_cc = best_i;
        arm_schedule_event(cpu, (arm_event_t *)t->compare_event,
                           (int64_t)cpu->cycles + best_delta);
    }
}

static void timer_cancel_compare(nrf_timer_state_t *t) {
    if (t->compare_event && t->soc && t->soc->plat) {
        arm_cancel_event(&t->soc->plat->cpu, (arm_event_t *)t->compare_event);
        t->next_cc = -1;
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
    soc->clock.soc = soc;
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
    for (int i = 0; i < 4; i++) {
        cpu_event_t *cev = (cpu_event_t *)calloc(1, sizeof(*cev));
        cev->callback  = rtc_compare_cb;
        cev->user_data = &soc->rtc0;
        soc->rtc0.compare_event[i] = cev;
    }
    arm_register_io(&plat->cpu, NRF_RTC0_BASE, NRF_RTC0_SIZE,
                    nrf_rtc_read, nrf_rtc_write, &soc->rtc0);

    soc->rtc1.irq_num = NRF_RTC1_IRQ;
    soc->rtc1.base    = NRF_RTC1_BASE;
    soc->rtc1.soc     = soc;
    cpu_event_t *tev1 = (cpu_event_t *)calloc(1, sizeof(*tev1));
    tev1->callback  = rtc_event_cb;
    tev1->user_data = &soc->rtc1;
    soc->rtc1.tick_event = tev1;
    for (int i = 0; i < 4; i++) {
        cpu_event_t *cev = (cpu_event_t *)calloc(1, sizeof(*cev));
        cev->callback  = rtc_compare_cb;
        cev->user_data = &soc->rtc1;
        soc->rtc1.compare_event[i] = cev;
    }
    arm_register_io(&plat->cpu, NRF_RTC1_BASE, NRF_RTC1_SIZE,
                    nrf_rtc_read, nrf_rtc_write, &soc->rtc1);

    /* RADIO — on-chip 2.4 GHz multi-protocol radio. */
    soc->radio.irq_num = NRF_RADIO_IRQ;
    soc->radio.state   = NRF_RADIO_STATE_DISABLED;
    soc->radio.power   = 1;       /* powered up by default */
    {
        /* Deferred-disable event: fires EVENTS_DISABLED after a short
         * ramp-down delay so the firmware's clear-then-poll pattern works
         * (see radio_trigger_task TASKS_DISABLE). */
        extern void nrf_radio_disable_cb(void *, cpu_event_t *);
        cpu_event_t *dev = (cpu_event_t *)calloc(1, sizeof(*dev));
        dev->callback  = nrf_radio_disable_cb;
        dev->user_data = soc;
        soc->radio.disable_event = dev;
        /* Deferred TX-completion event (PHYEND one air-time after START). */
        cpu_event_t *txe = (cpu_event_t *)calloc(1, sizeof(*txe));
        txe->callback  = nrf_radio_tx_done_cb;
        txe->user_data = soc;
        soc->radio.tx_event = txe;
        /* Deferred RXEN/TXEN ramp-up event (RXRU/TXRU → idle after 40 µs). */
        cpu_event_t *rue = (cpu_event_t *)calloc(1, sizeof(*rue));
        rue->callback  = nrf_radio_ramp_cb;
        rue->user_data = soc;
        soc->radio.ramp_event = rue;
        /* Deferred chip auto-ACK (192 µs after RX end). */
        cpu_event_t *ake = (cpu_event_t *)calloc(1, sizeof(*ake));
        ake->callback  = nrf_radio_ack_cb;
        ake->user_data = soc;
        soc->radio.ack_event = ake;
    }
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
    {
        static const int timer_irqs[5] = { 8, 9, 10, 26, 27 };
        for (int i = 0; i < 5; i++) {
            soc->timer[i].base    = timer_bases[i];
            soc->timer[i].soc     = soc;
            soc->timer[i].irq_num = timer_irqs[i];
            soc->timer[i].bitmode = 0;
            soc->timer[i].next_cc = -1;
            cpu_event_t *cev = (cpu_event_t *)calloc(1, sizeof(*cev));
            cev->callback  = timer_compare_event_cb;
            cev->user_data = &soc->timer[i];
            soc->timer[i].compare_event = cev;
            arm_register_io(&plat->cpu, timer_bases[i], 0x1000,
                            nrf_timer_read, nrf_timer_write, &soc->timer[i]);
        }
    }

    /* RNG */
    soc->rng.prng_state = 0xDEADBEEF;
    soc->rng.irq_num    = NRF_RNG_IRQ;
    soc->rng.soc        = soc;
    cpu_event_t *rev = (cpu_event_t *)calloc(1, sizeof(*rev));
    rev->callback   = nrf_rng_event_cb;
    rev->user_data  = &soc->rng;
    soc->rng.valrdy_event = rev;
    arm_register_io(&plat->cpu, NRF_RNG_BASE, 0x1000,
                    nrf_rng_read, nrf_rng_write, &soc->rng);

    /* PPI — event→task routing nrf_802154 uses to ramp the radio. */
    soc->ppi.soc = soc;
    arm_register_io(&plat->cpu, NRF_PPI_BASE, 0x1000,
                    nrf_ppi_read, nrf_ppi_write, &soc->ppi);

    /* EGU0/SWI0 — the hop in the RADIO_DISABLED → EGU → RADIO_RXEN PPI chain. */
    soc->egu0.soc     = soc;
    soc->egu0.irq_num = NRF_EGU_IRQ;
    arm_register_io(&plat->cpu, NRF_EGU0_BASE, 0x1000,
                    nrf_egu_read, nrf_egu_write, &soc->egu0);

    /* FICR — default values; harness patches DEVICEADDR0/1 per node. */
    soc->ficr.deviceaddr0 = 0x00000000;
    soc->ficr.deviceaddr1 = 0x0000F4CE;     /* upper bytes used as MAC OUI fragment */
    arm_register_io(&plat->cpu, NRF_FICR_BASE, 0x1000,
                    nrf_ficr_read, nrf_ficr_write, &soc->ficr);

    /* TEMP — die temperature sensor (nrf_802154 radio calibration). */
    soc->temp.soc = soc;
    soc->temp.irq_num = 12;     /* TEMP IRQ on nRF52840 */
    arm_register_io(&plat->cpu, NRF_TEMP_BASE, 0x1000,
                    nrf_temp_read, nrf_temp_write, &soc->temp);
}

static void nrf52840_soc_destroy(arm_platform_t *plat) {
    nrf52840_soc_t *soc = (nrf52840_soc_t *)plat->soc;
    if (soc) {
        nrf_rtc_state_t *rtcs[2] = { &soc->rtc0, &soc->rtc1 };
        for (int r = 0; r < 2; r++) {
            if (rtcs[r]->tick_event) {
                arm_cancel_event(&plat->cpu, (arm_event_t *)rtcs[r]->tick_event);
                free(rtcs[r]->tick_event);
            }
            for (int i = 0; i < 4; i++)
                if (rtcs[r]->compare_event[i]) {
                    arm_cancel_event(&plat->cpu, (arm_event_t *)rtcs[r]->compare_event[i]);
                    free(rtcs[r]->compare_event[i]);
                }
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
