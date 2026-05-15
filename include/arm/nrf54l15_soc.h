/*
 * nRF54L15 SoC peripheral bundle — first cut.
 *
 * Models the bare minimum to clear the Reset_Handler boot path. Right
 * now that's exactly one peripheral:
 *
 *   - GLOBAL_CLOCK at 0x5010_E000 (offsets discovered empirically with
 *     hello-world.nrf54l15-dk — see devices/nrf54l15-dk/STATUS.md).
 *     Handles the HFXO start handshake: firmware writes 1 to
 *     TASKS_HFXOSTART (offset 0), then spins reading
 *     EVENTS_HFXOSTARTED (offset 0x100) until non-zero. We latch the
 *     event immediately so nrfx_clock_start exits.
 *
 * Everything else (GRTC, UARTE20, GPIO/GPIOTE, RADIO, DPPI, …) lands
 * in subsequent commits, driven by what firmware actually traps on.
 *
 * Plugs into `arm_platform_t` through the `arm_soc_ops_t` vtable in
 * `arm_platform.h`. Note: the address range is 0x5xxx_xxxx, NOT
 * 0x4xxx_xxxx as on nRF52 — different peripheral region entirely.
 */
#ifndef NRF54L15_SOC_H
#define NRF54L15_SOC_H

#include "arm_platform.h"

/* GLOBAL_CLOCK at 0x5010_E000 (peripheral ID 0x10E in the application
 * domain).  Offsets discovered empirically — see the trace comment in
 * `nrf54l15_soc.c`.  We only model the single event the boot path
 * polls; other registers accept writes as no-ops and read 0. */
typedef struct nrf54l_global_clock_state {
    /* Latched events.  Each is set to 1 when the corresponding TASKS_*
     * register is written with value 1, and cleared by an explicit
     * write of 0 (firmware acknowledging the event).
     *
     * The two task/event pair sets observed during boot:
     *
     *   nrf_802154_clock_init:
     *     0x000 TASKS_HFCLKSTART → 0x100 EVENTS_HFCLKSTARTED  (`hfclkstarted`)
     *     0x008 TASKS_LFCLKSTART → 0x104 EVENTS_LFCLKSTARTED  (`lfclkstarted`)
     *
     *   nrfx_clock_start:
     *     0x010 TASKS_HFXOSTART  → 0x108 EVENTS_HFXOSTARTED   (`hfxostarted`)
     */
    uint32_t hfclkstarted;      /* 0x100 */
    uint32_t lfclkstarted;      /* 0x104 */
    uint32_t hfxostarted;       /* 0x108 */
    uint32_t domain_enable_440; /* 0x440 — clock-domain enable */
} nrf54l_global_clock_state_t;

/* UARTE20 at 0x500C_6000.  Pure EasyDMA — no legacy register window
 * (nrf54l15 does not expose one).
 *
 * Register subset modelled (per the SVD):
 *   0x050  TASKS_DMA.TX.START   — write 1 → emit MAXCNT bytes from PTR
 *   0x054  TASKS_DMA.TX.STOP    — write 1 → latch EVENTS_TXSTOPPED
 *   0x130  EVENTS_TXSTOPPED     — set by START/STOP (SHORTS_ENDTX_STOPTX)
 *   0x168  EVENTS_DMA.TX.END    — set by START after byte emission
 *   0x500  ENABLE               — 8 = UARTE EasyDMA mode (accepted)
 *   0x73C  DMA.TX.PTR           — SRAM source pointer
 *   0x740  DMA.TX.MAXCNT        — byte count for next transfer
 *   0x744  DMA.TX.AMOUNT        — bytes transferred (read-back)
 *
 * RX path, INTEN, PSEL, BAUDRATE, CONFIG, FRAMETIMEOUT, PUBLISH/SUBSCRIBE,
 * and the matching RX-DMA registers are accepted as no-ops.  Real
 * hardware needs hundreds of µs per byte at 115200 baud; csim emits the
 * full buffer synchronously on TASKS_DMA.TX.START, latches the events,
 * and returns — matches the zero-latency shortcut philosophy of the
 * GLOBAL_CLOCK stub.
 */
typedef struct nrf54l_uarte_state {
    arm_platform_t      *plat;         /* back-pointer for arm_read8 */
    uint32_t             tx_ptr;
    uint32_t             tx_maxcnt;
    uint32_t             tx_amount;    /* bytes most-recently emitted */
    uint32_t             evt_txstopped;
    uint32_t             evt_dma_tx_end;
    uint32_t             enable;
    arm_uart_tx_callback tx_cb;
    void                *tx_user;
} nrf54l_uarte_state_t;

/* DPPI (Distributed Programmable Peripheral Interconnect) — a 32-channel
 * routing fabric.  Peripherals publish events on a channel (via their
 * PUBLISH_<event> register: bit 31 = EN, bits 4..0 = channel ID) and
 * subscribe tasks to channels (via SUBSCRIBE_<task>, same shape).  When
 * a publisher fires and its channel is enabled (DPPIC.CHEN[channel]),
 * every subscriber's task callback runs.
 *
 * Nordic's nrf54l15 actually has multiple DPPI controllers (DPPIC00 at
 * 0x5004_2000, DPPIC10 at 0x5008_2000, DPPIC20 at 0x500C_2000, DPPIC30
 * at 0x5010_2000) each with their own 32-channel space, bridged across
 * domains by PPIB.  In csim we collapse all of them into one global
 * fabric — firmware-level channel IDs end up in the same 32-slot table
 * regardless of which DPPIC owns them.  This works as long as channel
 * allocations don't collide across domains, which Nordic's allocator
 * ensures in practice.
 *
 * Subscribers register via `nrf54l_dppi_subscribe` (small linked list
 * per channel since the same channel can drive multiple tasks); they
 * de-register via `_unsubscribe`.  Publishers call `_publish(channel)`
 * which walks the subscriber list and invokes each callback iff the
 * channel is enabled. */
#define NRF54L_DPPI_NUM_CHANNELS  32

typedef void (*nrf54l_dppi_sub_cb)(void *user);

typedef struct nrf54l_dppi_subscriber {
    nrf54l_dppi_sub_cb         cb;
    void                      *user;
    struct nrf54l_dppi_subscriber *next;
} nrf54l_dppi_subscriber_t;

typedef struct nrf54l_dppi_state {
    uint32_t                   chen;     /* CHEN — bitmap of enabled channels */
    nrf54l_dppi_subscriber_t  *subs[NRF54L_DPPI_NUM_CHANNELS];
} nrf54l_dppi_state_t;

/* GRTC (Global Real-Time Counter) at 0x500E_2000 — Contiki's tick source
 * + the timer Nordic's nrf_802154 driver hands MPSL for timeslot grants.
 * Without this, MPSL never fires `on_timeslot_started`, so RADIO is
 * never programmed.
 *
 * Subset modelled:
 *   0x060  TASKS_START
 *   0x068  TASKS_CLEAR
 *   0x100..0x12C  EVENTS_COMPARE[0..11]
 *   0x300  INTEN0 / 0x304 INTENSET0 / 0x308 INTENCLR0
 *   0x520..0x5BC  CC[0..11] (CCL/CCH/CCADD/CCEN, stride 0x10)
 *   0x720+0x000  SYSCOUNTER[0].SYSCOUNTERL  (== 0x720 / 0x740 / ...)
 *   0x720+0x004  SYSCOUNTER[0].SYSCOUNTERH
 *   0x720+0x028  SYSCOUNTER capture-ready bit (write 1 to trigger capture)
 *
 * The counter ticks at 1 MHz per Contiki's clock-arch.c (GRTC_TICK_FREQUENCY_HZ).
 * Compare events use CCADD (relative offset from current counter), not
 * absolute CC.  When firmware writes CC[n].CCADD and sets CCEN=1, we
 * schedule a CPU event `CCADD * 1000 ns` in the future that latches
 * EVENTS_COMPARE[n] and raises the GRTC IRQ. */
#define NRF54L_GRTC_NUM_CC   12

typedef struct nrf54l_grtc_cc {
    uint32_t ccl;
    uint32_t cch;
    uint32_t ccadd;
    uint32_t ccen;
    void    *event;       /* cpu_event_t* — null if not armed */
} nrf54l_grtc_cc_t;

typedef struct nrf54l_grtc_state {
    arm_platform_t      *plat;
    nrf54l_dppi_state_t *dppi;          /* shared DPPI fabric */
    bool                 running;
    int64_t              start_anchor_ns;
    nrf54l_grtc_cc_t     cc[NRF54L_GRTC_NUM_CC];
    uint32_t             evt_compare[NRF54L_GRTC_NUM_CC];
    uint32_t             publish_compare[NRF54L_GRTC_NUM_CC];  /* PUBLISH_COMPARE[n] */
    uint32_t             inten;          /* INTEN0 — bit n = COMPARE[n] */
    uint32_t             captured_lo;    /* SYSCOUNTERL latched value */
    uint32_t             captured_hi;    /* SYSCOUNTERH latched value */
    int                  irq_num;        /* GRTC_2_IRQn = 228 */
} nrf54l_grtc_state_t;

/* RADIO at 0x5008_A000 — 2.4 GHz 802.15.4 transceiver.
 *
 * Register layout (per nrf54lv10a_enga_application.svd) is DRAMATICALLY
 * different from nRF52840.  Key offset shifts:
 *
 *   nRF52840              nrf54l15
 *   --------              --------
 *   TASKS:    0x000+      TASKS:        0x000+
 *   EVENTS:   0x100+      SUBSCRIBE:    0x100+
 *   SHORTS:   0x200       EVENTS:       0x200+
 *   INTENSET: 0x304       PUBLISH:      0x300+
 *   PACKETPTR:0x504       SHORTS:       0x400
 *   FREQUENCY:0x508       INTENSET00:   0x488
 *   PCNF0:    0x514       INTENSET10:   0x4A8 (second IRQ line)
 *   STATE:    0x550       MODE:         0x500, STATE: 0x520
 *                         FREQUENCY:    0x708
 *                         PCNF0:        0xE20
 *                         PACKETPTR:    0xED0
 *
 * Plus nrf54l15 routes its task triggers via DPPI exclusively —
 * Nordic's nrf_802154 driver doesn't write TASKS_TXEN directly.  Each
 * SUBSCRIBE_<task> register binds a DPPI channel; when something
 * publishes on that channel (typically GRTC compare or a previous
 * RADIO event via PUBLISH_<event>), the corresponding task fires.
 */
#define NRF54L_RADIO_NUM_SUBSCRIBES 12   /* TXEN..CCASTOP, contiguous 4-byte slots */
#define NRF54L_RADIO_NUM_PUBLISHES  24   /* READY..CTEPRESENT */

typedef struct nrf54l_radio_state {
    arm_platform_t      *plat;
    nrf54l_dppi_state_t *dppi;
    int                  irq_num_0;   /* RADIO_0 IRQn = 138 */
    int                  irq_num_1;   /* RADIO_1 IRQn = 139 */

    uint32_t state;                   /* 0=DISABLED, 2=RXIDLE, 3=RX, 10=TXIDLE, 11=TX */
    uint32_t shorts;
    uint32_t intenset00;
    uint32_t intenset10;

    /* Config — kept verbatim for read-back; only a few are interpreted. */
    uint32_t mode, frequency, txpower, timing;
    uint32_t pcnf0, pcnf1, crccnf, crcpoly, crcinit;
    uint32_t txaddress, rxaddresses;
    uint32_t bcc;                     /* bit-counter compare (in bits, not bytes) */
    uint32_t packetptr;

    /* SUBSCRIBE channels (one per task slot 0..11, value 0 = disabled, else
     * 0x80000000 | channel_id).  We also track the bound channel
     * separately so we can unsubscribe before re-subscribing. */
    uint32_t subscribe[NRF54L_RADIO_NUM_SUBSCRIBES];
    int      sub_channel[NRF54L_RADIO_NUM_SUBSCRIBES];   /* -1 = unbound */

    /* PUBLISH channels (one per event slot 0..23). */
    uint32_t publish[NRF54L_RADIO_NUM_PUBLISHES];

    /* Latched events.  Each evt_<x> is 1 after the event fires until
     * firmware writes 0 to the corresponding offset. */
    uint32_t evt_ready, evt_txready, evt_rxready;
    uint32_t evt_address, evt_framestart, evt_payload;
    uint32_t evt_end,    evt_phyend,    evt_disabled;
    uint32_t evt_devmatch, evt_devmiss;
    uint32_t evt_crcok,  evt_crcerror, evt_bcmatch;
    uint32_t evt_edend,  evt_edstopped;
    uint32_t evt_ccaidle, evt_ccabusy, evt_ccastopped;
    uint32_t evt_rateboost, evt_mhrmatch, evt_sync;
    uint32_t evt_ctepresent;

    /* RX byte-stream parser state. */
    int rx_phase;
    int rx_remaining;
    int rx_offset;

    /* TX byte listener — installed by the multinode harness. */
    void (*tx_cb)(void *user, uint8_t byte);
    void  *tx_user;
} nrf54l_radio_state_t;

/* EGU (Event Generator Unit) — 16-channel software-triggerable
 * event source.  nrf_802154 uses it as the "kick" point for the
 * ramp-up DPPI chain: CPU writes TASKS_TRIGGER[n] → EVENTS_TRIGGERED[n]
 * fires → PUBLISH_TRIGGERED[n] publishes on its bound channel →
 * RADIO.SUBSCRIBE_RXEN (or TXEN) is gated on that channel → state
 * machine starts.
 *
 * One csim model instance covers all four EGU bases (00/10/20/30);
 * they share the global DPPI fabric like everything else.
 */
#define NRF54L_EGU_NUM_CHANNELS 16

typedef struct nrf54l_egu_state {
    arm_platform_t      *plat;
    nrf54l_dppi_state_t *dppi;
    uint32_t             events[NRF54L_EGU_NUM_CHANNELS];
    uint32_t             publish[NRF54L_EGU_NUM_CHANNELS];
    uint32_t             subscribe[NRF54L_EGU_NUM_CHANNELS];
    int                  sub_channel[NRF54L_EGU_NUM_CHANNELS];
    uint32_t             inten;
    int                  irq_num;
} nrf54l_egu_state_t;

/* TIMER (instances TIMER00/10/20/21/22/23/24).  Up to 6 CC channels.
 * Backing model: 32-bit counter at 16 MHz / 2^PRESCALER, derived from
 * sim_time_ns when needed.  Compare events fire from the CPU event queue. */
#define NRF54L_TIMER_NUM_CC  6
typedef struct nrf54l_timer_state {
    arm_platform_t      *plat;
    nrf54l_dppi_state_t *dppi;
    uint32_t             cc[NRF54L_TIMER_NUM_CC];
    uint32_t             events_compare[NRF54L_TIMER_NUM_CC];
    uint32_t             publish_compare[NRF54L_TIMER_NUM_CC];
    uint32_t             subscribe_capture[NRF54L_TIMER_NUM_CC];
    int                  sub_capture_ch[NRF54L_TIMER_NUM_CC];
    uint32_t             subscribe_start;
    uint32_t             subscribe_stop;
    uint32_t             subscribe_count;
    uint32_t             subscribe_clear;
    uint32_t             shorts;
    uint32_t             inten;
    uint32_t             mode;
    uint32_t             bitmode;          /* 0=16, 1=8, 2=24, 3=32 */
    uint32_t             prescaler;        /* counter clock = 16 MHz >> prescaler */
    uint32_t             counter;          /* used in Counter mode */
    int64_t              t0_ns;            /* sim_time_ns when started/cleared */
    uint32_t             snapshot;         /* counter at last stop */
    bool                 running;
    int                  irq_num;
    arm_event_t          ev_compare[NRF54L_TIMER_NUM_CC];
    uint32_t             base_addr;        /* for IO routing */
} nrf54l_timer_state_t;

typedef struct nrf54l15_soc {
    nrf54l_global_clock_state_t global_clock;
    nrf54l_uarte_state_t        uarte20;
    nrf54l_grtc_state_t         grtc;
    nrf54l_dppi_state_t         dppi;
    nrf54l_radio_state_t        radio;
    /* Three EGU instances at 0x5001_5000 (EGU00), 0x5008_7000 (EGU10),
     * 0x500C_7000 (EGU20).  Channel allocation across instances is
     * domain-local on real HW; csim collapses to the global DPPI. */
    nrf54l_egu_state_t          egu00;
    nrf54l_egu_state_t          egu10;
    nrf54l_egu_state_t          egu20;
    /* TIMER instances.  TIMER10/20 are the ones used by the nrf_802154
     * lptimer backend. */
    nrf54l_timer_state_t        timer10;
    nrf54l_timer_state_t        timer20;
    arm_platform_t             *plat;
} nrf54l15_soc_t;

/* nrf54l15 RADIO STATE enum (from SVD). */
#define NRF54L_RADIO_STATE_DISABLED   0
#define NRF54L_RADIO_STATE_RXRU       1
#define NRF54L_RADIO_STATE_RXIDLE     2
#define NRF54L_RADIO_STATE_RX         3
#define NRF54L_RADIO_STATE_RXDISABLE  4
#define NRF54L_RADIO_STATE_TXRU       9
#define NRF54L_RADIO_STATE_TXIDLE     10
#define NRF54L_RADIO_STATE_TX         11
#define NRF54L_RADIO_STATE_TXDISABLE  12

/* Public DPPI API for other peripherals to wire publish/subscribe. */
void nrf54l_dppi_subscribe(nrf54l_dppi_state_t *d, int channel,
                           nrf54l_dppi_sub_cb cb, void *user);
void nrf54l_dppi_unsubscribe(nrf54l_dppi_state_t *d, int channel,
                             nrf54l_dppi_sub_cb cb, void *user);
void nrf54l_dppi_publish(nrf54l_dppi_state_t *d, int channel);

/* Public radio hooks for the multinode harness. */
typedef void (*nrf54l_radio_tx_listener_t)(void *user, uint8_t byte);
void nrf54l_radio_set_tx_listener(nrf54l15_soc_t *soc,
                                   nrf54l_radio_tx_listener_t cb, void *user);
void nrf54l_radio_receive_byte(nrf54l15_soc_t *soc, uint8_t byte);


extern const arm_soc_ops_t nrf54l15_soc_ops;

static inline nrf54l15_soc_t *arm_platform_nrf54l15(arm_platform_t *plat) {
    if (!plat || !plat->config || plat->config->soc_ops != &nrf54l15_soc_ops)
        return NULL;
    return (nrf54l15_soc_t *)plat->soc;
}

#endif /* NRF54L15_SOC_H */
