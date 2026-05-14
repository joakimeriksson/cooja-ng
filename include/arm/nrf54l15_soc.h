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

typedef struct nrf54l15_soc {
    nrf54l_global_clock_state_t global_clock;
    nrf54l_uarte_state_t        uarte20;
    nrf54l_grtc_state_t         grtc;
    nrf54l_dppi_state_t         dppi;
    arm_platform_t             *plat;
} nrf54l15_soc_t;

/* Public DPPI API for other peripherals to wire publish/subscribe. */
void nrf54l_dppi_subscribe(nrf54l_dppi_state_t *d, int channel,
                           nrf54l_dppi_sub_cb cb, void *user);
void nrf54l_dppi_unsubscribe(nrf54l_dppi_state_t *d, int channel,
                             nrf54l_dppi_sub_cb cb, void *user);
void nrf54l_dppi_publish(nrf54l_dppi_state_t *d, int channel);

extern const arm_soc_ops_t nrf54l15_soc_ops;

static inline nrf54l15_soc_t *arm_platform_nrf54l15(arm_platform_t *plat) {
    if (!plat || !plat->config || plat->config->soc_ops != &nrf54l15_soc_ops)
        return NULL;
    return (nrf54l15_soc_t *)plat->soc;
}

#endif /* NRF54L15_SOC_H */
