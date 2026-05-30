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

/* RADIO at 0x40001000.
 *
 * 2.4 GHz multi-protocol radio. In 802.15.4 mode the state machine is:
 *
 *   DISABLED →[TXEN]→ TXRU →(auto)→ TXIDLE →[START]→ TX →(auto)→ TXIDLE
 *   DISABLED →[RXEN]→ RXRU →(auto)→ RXIDLE →[START]→ RX
 *
 * SHORTS register chains transitions automatically:
 *   READY_START         — when ramp-up completes, trigger START
 *   TXREADY_START       — same, 802.15.4-specific
 *   RXREADY_START       — same, 802.15.4-specific
 *   END_DISABLE         — when frame done, trigger DISABLE
 *   DISABLED_TXEN       — restart in TX after DISABLE
 *   DISABLED_RXEN       — restart in RX after DISABLE
 *
 * For the first cut we model: STATE register, task → state transitions
 * (instant — no rampup delay), READY / END / DISABLED / TXREADY /
 * RXREADY / FRAMESTART events, SHORTS auto-chaining, EVENTS firing
 * NVIC IRQ (vector 1) when INTENSET allows.
 *
 * No EasyDMA frame data movement yet: TX bytes are dropped, RX never
 * fires END from "external" data. That keeps the firmware's TX/RX state
 * machines progressing without spin-locking; multinode delivery comes
 * once we wire RX into the radio_medium.
 */
typedef struct nrf_radio_state {
    /* State machine */
    uint32_t state;           /* current STATE; see NRF_RADIO_STATE_* */
    /* Mapped configuration registers (kept verbatim for firmware to
     * read back; only some are interpreted). */
    uint32_t mode;            /* 0x510 — protocol mode (15.4 mode = 15) */
    uint32_t frequency;       /* 0x508 */
    uint32_t txpower;         /* 0x50C */
    uint32_t pcnf0;           /* 0x514 */
    uint32_t pcnf1;           /* 0x518 */
    uint32_t base0;           /* 0x51C */
    uint32_t base1;           /* 0x520 */
    uint32_t prefix0;         /* 0x524 */
    uint32_t prefix1;         /* 0x528 */
    uint32_t txaddress;       /* 0x52C */
    uint32_t rxaddresses;     /* 0x530 */
    uint32_t crccnf;          /* 0x534 */
    uint32_t crcpoly;         /* 0x538 */
    uint32_t crcinit;         /* 0x53C */
    uint32_t shorts;          /* 0x200 */
    uint32_t intenset;        /* 0x304 */
    uint32_t packetptr;       /* 0x504 — EasyDMA buffer pointer */
    uint32_t modecnf0;        /* 0x650 */
    uint32_t ccactrl;         /* 0x66C */
    uint32_t power;           /* 0xFFC */

    /* Latched events (offsets relative to RADIO base) */
    uint32_t evt_ready;        /* 0x100 */
    uint32_t evt_address;      /* 0x104 */
    uint32_t evt_payload;      /* 0x108 */
    uint32_t evt_end;          /* 0x10C */
    uint32_t evt_disabled;     /* 0x110 */
    uint32_t evt_devmatch;     /* 0x114 */
    uint32_t evt_devmiss;      /* 0x118 */
    uint32_t evt_rssiend;      /* 0x11C */
    uint32_t evt_bcmatch;      /* 0x128 */
    uint32_t evt_crcok;        /* 0x130 */
    uint32_t evt_crcerror;     /* 0x134 */
    uint32_t evt_framestart;   /* 0x138 */
    uint32_t evt_edend;        /* 0x13C */
    uint32_t evt_edstopped;    /* 0x140 */
    uint32_t evt_ccaidle;      /* 0x144 */
    uint32_t evt_ccabusy;      /* 0x148 */
    uint32_t evt_ccastopped;   /* 0x14C */
    uint32_t evt_rateboost;    /* 0x150 */
    uint32_t evt_txready;      /* 0x154 */
    uint32_t evt_rxready;      /* 0x158 */
    uint32_t evt_mhrmatch;     /* 0x15C */
    uint32_t evt_phyend;       /* 0x16C */

    int      irq_num;          /* RADIO IRQ = 1 */
    /* RX byte parser state — incoming on-air bytes feed this. */
    int      rx_phase;         /* nrf_rx_phase_t enum */
    int      rx_remaining;
    int      rx_offset;
    /* Cooja-style per-byte delivery: an auto-ACK arriving 192 µs after
     * our own TX can land before the driver has flipped state back to
     * RX.  Buffer bytes received outside RX state and replay them when
     * the radio transitions to RX.  Cleared on TX entry so the buffer
     * never carries stale bytes across a TX cycle. */
    uint8_t  rx_incoming[256];
    int      rx_incoming_len;
} nrf_radio_state_t;

/* TIMER0..4 at 0x40008000 / 0x40009000 / 0x4000A000 / 0x4001A000 / 0x4001B000.
 *
 * Contiki's rtimer_arch_now() captures TIMER0 CC[0] and reads it back.
 * That's the only access pattern we model in this first cut — counter
 * advances based on simulated cycles, capture latches into CC[i], read
 * returns the latched value.  Compare / IRQ paths land later if firmware
 * needs them. */
typedef struct nrf_timer_state {
    uint32_t base;            /* peripheral base address (for IO range) */
    bool     running;
    int64_t  start_cycles;    /* cpu->cycles when TASKS_START fired */
    uint32_t counter_at_start;
    uint32_t prescaler;       /* 0..9; tick rate = 16 MHz >> prescaler */
    uint32_t bitmode;         /* 0=16, 1=8, 2=24, 3=32-bit */
    uint32_t mode;            /* 0=Timer (default), 1=Counter */
    uint32_t cc[6];
    uint32_t evt_compare[6];
    uint32_t intenset;
    uint32_t shorts;
    struct nrf52840_soc *soc; /* back-pointer for cpu access */
} nrf_timer_state_t;

/* RNG at 0x4000D000 — random byte generator. */
typedef struct nrf_rng_state {
    uint32_t evt_valrdy;      /* offset 0x100 */
    uint32_t intenset;        /* offset 0x304 (bit 0 = VALRDY) */
    uint32_t shorts;          /* offset 0x200 */
    bool     running;
    uint32_t prng_state;      /* xorshift32 seeded per-node */
} nrf_rng_state_t;

/* FICR at 0x10000000 — factory information. csim only models the
 * DEVICEADDR pair (used by the platform glue to derive the IEEE
 * EUI-64). The multinode harness writes per-node values here. */
typedef struct nrf_ficr_state {
    uint32_t deviceaddr0;     /* 0x100000A4 — low 32 bits */
    uint32_t deviceaddr1;     /* 0x100000A8 — top 16 bits */
} nrf_ficr_state_t;

/* Radio TX byte listener — installed by the multinode runner so air
 * bytes get pushed into radio_medium. Receives bytes one at a time
 * starting from the preamble. */
typedef void (*nrf_radio_tx_listener_t)(void *user_data, uint8_t byte);

typedef struct nrf52840_soc {
    nrf_clock_state_t clock;
    nrf_uart_state_t  uart0;
    nrf_rtc_state_t   rtc0;
    nrf_radio_state_t radio;
    nrf_timer_state_t timer[5];
    nrf_rng_state_t   rng;
    nrf_ficr_state_t  ficr;
    arm_platform_t   *plat;   /* back-pointer for event-callback NVIC access */
    /* Radio TX listener installed by the harness. */
    nrf_radio_tx_listener_t radio_tx_cb;
    void                   *radio_tx_user;
} nrf52840_soc_t;

/* Public hook used by the multinode harness to install the TX listener
 * and to push received bytes into the radio. */
void nrf_radio_set_tx_listener(nrf52840_soc_t *soc, nrf_radio_tx_listener_t cb, void *user_data);
void nrf_radio_receive_byte(nrf52840_soc_t *soc, uint8_t byte);

/* nRF52840 RADIO state enum values (per PS v1.7 §6.20.15.39). */
#define NRF_RADIO_STATE_DISABLED   0
#define NRF_RADIO_STATE_RXRU       1
#define NRF_RADIO_STATE_RXIDLE     2
#define NRF_RADIO_STATE_RX         3
#define NRF_RADIO_STATE_RXDISABLE  4
#define NRF_RADIO_STATE_TXRU       9
#define NRF_RADIO_STATE_TXIDLE    10
#define NRF_RADIO_STATE_TX        11
#define NRF_RADIO_STATE_TXDISABLE 12

extern const arm_soc_ops_t nrf52840_soc_ops;

static inline nrf52840_soc_t *arm_platform_nrf52840(arm_platform_t *plat) {
    if (!plat || !plat->config || plat->config->soc_ops != &nrf52840_soc_ops)
        return NULL;
    return (nrf52840_soc_t *)plat->soc;
}

#endif /* NRF52840_SOC_H */
