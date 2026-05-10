/*
 * nRF52840 SoC peripheral bundle.
 *
 * Currently models the bare minimum to run firmware up through console
 * output: CLOCK (0x40000000) for the HFCLK/LFCLK start handshake, and
 * the legacy UART0 register window of UARTE0 (0x40002000) for byte
 * transmit. Real EasyDMA UARTE, GPIO, GPIOTE, RTC, TIMER, RADIO, PPI
 * etc. land later.
 *
 * Plugs into `arm_platform_t` through the `arm_soc_ops_t` vtable defined
 * in `arm_platform.h`.
 */
#ifndef NRF52840_SOC_H
#define NRF52840_SOC_H

#include "arm_platform.h"

/* CLOCK + POWER share base 0x40000000 on nRF52840 (peripheral ID 0).
 * We only model the four start-task / event pairs the boot path uses. */
typedef struct nrf_clock_state {
    uint32_t hfclkstarted;   /* offset 0x100 — set when HFCLK is up */
    uint32_t lfclkstarted;   /* offset 0x104 — set when LFCLK is up */
} nrf_clock_state_t;

/* Legacy UART0 register window of the UARTE0 peripheral at 0x40002000.
 * Contiki's nrf52840 platform uses the legacy API when NRF52840_NATIVE_USB=0:
 *   write byte to TXD (0x51C)
 *   spin on EVENTS_TXDRDY (0x11C) until set
 *   clear EVENTS_TXDRDY (write 0). */
typedef struct nrf_uart_state {
    uint32_t enable;          /* offset 0x500 */
    uint32_t txdrdy;          /* offset 0x11C */
    arm_uart_tx_callback tx_cb;
    void                *tx_user;
} nrf_uart_state_t;

/* RTC0 at 0x4000B000.
 *
 * 24-bit counter clocked from LFCLK (32 768 Hz) through a 12-bit
 * prescaler. Three events relevant to Contiki:
 *   - TICK             : fires on every COUNTER++
 *   - OVRFLW           : fires when COUNTER wraps (every 512 s at PRESCALER=0)
 *   - COMPARE[0..3]    : fires when COUNTER == CC[i]
 *
 * Contiki's clock_init enables TICK interrupts; the IRQ handler drives
 * the etimer / clock subsystem. We model TICK via a recurring event
 * scheduled on the ARM event queue, and let the existing WFI handling
 * advance simulated time between ticks. */
typedef struct nrf_rtc_state {
    /* Mapped registers */
    uint32_t prescaler;       /* offset 0x508 */
    uint32_t intenset;        /* offset 0x304 (bit 0 = TICK, 1 = OVRFLW, 16..19 = COMPARE[0..3]) */
    uint32_t evtenset;        /* offset 0x340 */
    uint32_t cc[4];           /* offset 0x540..0x54C */

    /* Latched events */
    uint32_t evt_tick;        /* offset 0x100 */
    uint32_t evt_ovrflw;      /* offset 0x104 */
    uint32_t evt_compare[4];  /* offset 0x140..0x14C */

    /* Internal counter / scheduling */
    bool     running;
    int64_t  counter_anchor_cycles;   /* cpu->cycles at COUNTER=0 reference point */
    uint32_t counter_at_anchor;       /* COUNTER value at the anchor */
    uint32_t tick_period_cycles;      /* CPU cycles per RTC tick after prescaler */
    int      irq_num;                 /* RTC0 IRQ = 11 on nRF52840 */
    /* Recurring TICK event, re-scheduled by its own callback. */
    void    *tick_event;              /* opaque arm_event_t * */
    void    *soc;                     /* back-pointer for the event callback */
} nrf_rtc_state_t;

typedef struct nrf52840_soc {
    nrf_clock_state_t clock;
    nrf_uart_state_t  uart0;
    nrf_rtc_state_t   rtc0;
    arm_platform_t   *plat;   /* back-pointer for event-callback NVIC access */
} nrf52840_soc_t;

extern const arm_soc_ops_t nrf52840_soc_ops;

static inline nrf52840_soc_t *arm_platform_nrf52840(arm_platform_t *plat) {
    if (!plat || !plat->config || plat->config->soc_ops != &nrf52840_soc_ops)
        return NULL;
    return (nrf52840_soc_t *)plat->soc;
}

#endif /* NRF52840_SOC_H */
