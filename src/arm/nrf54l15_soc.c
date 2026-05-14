/*
 * nRF54L15 SoC — minimum-viable peripheral set.
 *
 * Models just enough to get past the Reset_Handler busy-wait in
 * nrfx_clock_start. Right now: GLOBAL_CLOCK at 0x5010_E000 only.
 * Subsequent commits add GRTC (Contiki tick source), UARTE20
 * (console), GPIO/GPIOTE, and eventually the 802.15.4 RADIO.
 *
 * Addresses come from the empirical trace in
 * devices/nrf54l15-dk/STATUS.md and the Contiki-NG nrf54l15 port
 * source — the Product Specification pages will be cross-referenced
 * as additional registers are modelled.
 *
 * NOTE: peripheral region is 0x50000000, NOT 0x40000000. That's the
 * single biggest map-level difference from nRF52.
 */
#include "nrf54l15_soc.h"
#include "arm_cpu.h"
#include "arm_nvic.h"
#include <stdio.h>
#include <stdlib.h>

/* sim_host shims — identical to nRF52's. Off-SoC chip drivers reach
 * the CPU's event scheduler through this vtable.  No external chips
 * are wired on either nrf54l15 board today, but the host fields must
 * still be valid for anything that touches plat->host. */
static int64_t nrf54l_host_now_ns(void *cpu) {
    return ((arm_cpu_t *)cpu)->sim_time_ns;
}
static void nrf54l_host_schedule_ns(void *cpu, cpu_event_t *ev, int64_t fire_ns) {
    arm_schedule_event_ns((arm_cpu_t *)cpu, ev, fire_ns);
}
static void nrf54l_host_cancel(void *cpu, cpu_event_t *ev) {
    arm_cancel_event((arm_cpu_t *)cpu, ev);
}

/* ============================================================
 * GLOBAL_CLOCK (0x5010_E000)
 *
 * Replaces the nRF52 CLOCK + POWER pair.  Offsets discovered
 * empirically by tracing the access pattern of nrfx_clock_start +
 * nrfx_clock_hfclk_start in the Contiki-NG nrf54l15 port:
 *
 *   write 1 → 0x440  (sub-block enable / clock domain config; ignored)
 *   read   ← 0x44C   (status; returns 0 — firmware accepts)
 *   read   ← 0x448   (status; returns 0 — firmware accepts)
 *   write 1 → 0x440  (re-poke after status check; ignored)
 *   write 0 → 0x108  (clear EVENTS_HFXOSTARTED)
 *   read   ← 0x108   (peek before kick)
 *   write 1 → 0x010  (TASKS_HFXOSTART — kick the crystal)
 *   loop:
 *     read 0x108     (EVENTS_HFXOSTARTED); if == 0 spin
 *
 * On real hardware the HFXO crystal takes ~700 µs to stabilise;
 * csim latches the event immediately on TASKS_HFXOSTART, same
 * shortcut we use for nRF52's HFCLK.  Sufficient for polling
 * firmware; IRQ-driven firmware also needs NVIC wiring (later).
 *
 * NOTE: offsets are NOT the canonical 0x000/0x100 Nordic uses on
 * nRF52 — nrf54l15 puts each task/event group inside a sub-block
 * with its own base offset.  Don't infer offsets from nRF52
 * register layouts; verify empirically before adding new ones.
 * ============================================================ */
#define NRF54L_GLOBAL_CLOCK_BASE          0x5010E000u
#define NRF54L_GLOBAL_CLOCK_SIZE          0x1000u

#define NRF54L_GC_TASKS_HFCLKSTART        0x000   /* nrf_802154 path */
#define NRF54L_GC_TASKS_LFCLKSTART        0x008   /* nrf_802154 path */
#define NRF54L_GC_TASKS_HFXOSTART         0x010   /* nrfx_clock_start path */
#define NRF54L_GC_EVENTS_HFCLKSTARTED     0x100
#define NRF54L_GC_EVENTS_LFCLKSTARTED     0x104
#define NRF54L_GC_EVENTS_HFXOSTARTED      0x108
#define NRF54L_GC_DOMAIN_ENABLE           0x440   /* WR 1 → bring clock domain up */
#define NRF54L_GC_DOMAIN_STATUS           0x44C   /* RD: bit 16 = domain running */

#define NRF54L_GC_DOMAIN_STATUS_RUNNING   (1u << 16)

static int nrf54l_global_clock_read(void *user_data, uint32_t addr) {
    nrf54l_global_clock_state_t *gc = (nrf54l_global_clock_state_t *)user_data;
    uint32_t off = addr - NRF54L_GLOBAL_CLOCK_BASE;
    switch (off) {
        case NRF54L_GC_EVENTS_HFCLKSTARTED:  return (int)gc->hfclkstarted;
        case NRF54L_GC_EVENTS_LFCLKSTARTED:  return (int)gc->lfclkstarted;
        case NRF54L_GC_EVENTS_HFXOSTARTED:   return (int)gc->hfxostarted;
        case NRF54L_GC_DOMAIN_STATUS:
            /* Bit 16 = "clock domain running".  clock_init's tight spin
             * (PC 0x152c..0x1534 in hello-world.nrf54l15-dk) polls this
             * via `lsls r3, #15; bpl` — i.e. waits for bit 16 to be set.
             * We return RUNNING once the domain has been enabled at
             * 0x440. Other status bits stay 0 until firmware needs
             * them. */
            return gc->domain_enable_440 ? (int)NRF54L_GC_DOMAIN_STATUS_RUNNING : 0;
        default:                             return 0;
    }
}

static void nrf54l_global_clock_write(void *user_data, uint32_t addr, uint32_t value) {
    nrf54l_global_clock_state_t *gc = (nrf54l_global_clock_state_t *)user_data;
    uint32_t off = addr - NRF54L_GLOBAL_CLOCK_BASE;
    switch (off) {
        case NRF54L_GC_TASKS_HFCLKSTART:
            if (value == 1) gc->hfclkstarted = 1;
            break;
        case NRF54L_GC_TASKS_LFCLKSTART:
            if (value == 1) gc->lfclkstarted = 1;
            break;
        case NRF54L_GC_TASKS_HFXOSTART:
            if (value == 1) gc->hfxostarted = 1;
            break;
        case NRF54L_GC_EVENTS_HFCLKSTARTED:
            gc->hfclkstarted = value & 1;
            break;
        case NRF54L_GC_EVENTS_LFCLKSTARTED:
            gc->lfclkstarted = value & 1;
            break;
        case NRF54L_GC_EVENTS_HFXOSTARTED:
            gc->hfxostarted = value & 1;
            break;
        case NRF54L_GC_DOMAIN_ENABLE:
            /* Writing 1 brings the corresponding clock domain up; the
             * status bit at 0x44c becomes visible the next read. */
            gc->domain_enable_440 = value;
            break;
        default:
            /* Other offsets seen during init (e.g. 0x034 sub-enable,
             * 0x448 secondary status) are accepted as no-ops —
             * firmware reads but doesn't gate on them. */
            break;
    }
}

/* ============================================================
 * UARTE20 EasyDMA (0x500C_6000)
 *
 * Offsets confirmed from nrf54lv10a_enga_application.svd in
 * arch/cpu/nrf/lib/nrfx/mdk/ (Contiki-NG nrfx submodule).
 * ============================================================ */
#define NRF54L_UARTE20_BASE              0x500C6000u
#define NRF54L_UARTE20_SIZE              0x1000u

#define NRF54L_UARTE_TASKS_DMA_TX_START  0x050
#define NRF54L_UARTE_TASKS_DMA_TX_STOP   0x054
#define NRF54L_UARTE_EVENTS_TXSTOPPED    0x130
#define NRF54L_UARTE_EVENTS_DMA_TX_END   0x168
#define NRF54L_UARTE_ENABLE              0x500
#define NRF54L_UARTE_DMA_TX_PTR          0x73C
#define NRF54L_UARTE_DMA_TX_MAXCNT       0x740
#define NRF54L_UARTE_DMA_TX_AMOUNT       0x744

static int nrf54l_uarte_read(void *user_data, uint32_t addr) {
    nrf54l_uarte_state_t *u = (nrf54l_uarte_state_t *)user_data;
    uint32_t off = addr - NRF54L_UARTE20_BASE;
    switch (off) {
        case NRF54L_UARTE_EVENTS_TXSTOPPED:   return (int)u->evt_txstopped;
        case NRF54L_UARTE_EVENTS_DMA_TX_END:  return (int)u->evt_dma_tx_end;
        case NRF54L_UARTE_ENABLE:             return (int)u->enable;
        case NRF54L_UARTE_DMA_TX_PTR:         return (int)u->tx_ptr;
        case NRF54L_UARTE_DMA_TX_MAXCNT:      return (int)u->tx_maxcnt;
        case NRF54L_UARTE_DMA_TX_AMOUNT:      return (int)u->tx_amount;
        default:                              return 0;
    }
}

static void nrf54l_uarte_write(void *user_data, uint32_t addr, uint32_t value) {
    nrf54l_uarte_state_t *u = (nrf54l_uarte_state_t *)user_data;
    uint32_t off = addr - NRF54L_UARTE20_BASE;
    switch (off) {
        case NRF54L_UARTE_TASKS_DMA_TX_START:
            /* Kick TX: stream MAXCNT bytes from SRAM at PTR through the
             * console callback, then latch both EVENTS_DMA.TX.END and
             * EVENTS_TXSTOPPED (because firmware programs
             * SHORTS_ENDTX_STOPTX → STOPTX fires on END). */
            if (value == 1 && u->plat) {
                for (uint32_t i = 0; i < u->tx_maxcnt; i++) {
                    uint8_t b = arm_read8(&u->plat->cpu, u->tx_ptr + i);
                    if (u->tx_cb) u->tx_cb(u->tx_user, b);
                }
                u->tx_amount = u->tx_maxcnt;
                u->evt_dma_tx_end = 1;
                u->evt_txstopped  = 1;
            }
            break;
        case NRF54L_UARTE_TASKS_DMA_TX_STOP:
            /* Defensive STOP after START.  Latch TXSTOPPED so a
             * stop-then-poll firmware path also escapes its loop. */
            if (value == 1) u->evt_txstopped = 1;
            break;
        case NRF54L_UARTE_EVENTS_TXSTOPPED:   u->evt_txstopped  = value & 1; break;
        case NRF54L_UARTE_EVENTS_DMA_TX_END:  u->evt_dma_tx_end = value & 1; break;
        case NRF54L_UARTE_ENABLE:             u->enable         = value;     break;
        case NRF54L_UARTE_DMA_TX_PTR:         u->tx_ptr         = value;     break;
        case NRF54L_UARTE_DMA_TX_MAXCNT:      u->tx_maxcnt      = value;     break;
        case NRF54L_UARTE_DMA_TX_AMOUNT:      u->tx_amount      = value;     break;
        default:
            /* Accept every other register write as a no-op: SHORTS,
             * BAUDRATE, CONFIG, FRAMETIMEOUT, PSEL.*, INTEN*,
             * PUBLISH_*, SUBSCRIBE_*, the RX-DMA cluster, etc.  None of
             * these have observed read-back semantics that would gate
             * boot progress in this firmware. */
            break;
    }
}

/* ============================================================
 * DPPI fabric — collapsed to one 32-channel global state
 * ============================================================ */
#define NRF54L_DPPI_CHEN          0x500
#define NRF54L_DPPI_CHENSET       0x504
#define NRF54L_DPPI_CHENCLR       0x508

/* All four DPPIC base addresses share the same state in our model.
 * Real HW has separate per-domain controllers; channel allocation in
 * Nordic's allocator keeps IDs unique across domains, so collapsing
 * them is safe for the firmware paths csim emulates. */
static const uint32_t nrf54l_dppic_bases[] = {
    0x50042000u,  /* DPPIC00 — paired with peripherals at 0x5004_xxxx */
    0x50082000u,  /* DPPIC10 — RADIO at 0x5008_A000 lives here */
    0x500C2000u,  /* DPPIC20 — UARTE20 at 0x500C_6000 lives here */
    0x50102000u,  /* DPPIC30 — GRTC at 0x500E_2000 + SPU/MPC at 0x501x_xxxx */
};
#define NRF54L_DPPIC_SIZE  0x1000u

void nrf54l_dppi_subscribe(nrf54l_dppi_state_t *d, int channel,
                           nrf54l_dppi_sub_cb cb, void *user) {
    if (channel < 0 || channel >= NRF54L_DPPI_NUM_CHANNELS) return;
    nrf54l_dppi_subscriber_t *s =
        (nrf54l_dppi_subscriber_t *)calloc(1, sizeof(*s));
    s->cb   = cb;
    s->user = user;
    s->next = d->subs[channel];
    d->subs[channel] = s;
}

void nrf54l_dppi_unsubscribe(nrf54l_dppi_state_t *d, int channel,
                             nrf54l_dppi_sub_cb cb, void *user) {
    if (channel < 0 || channel >= NRF54L_DPPI_NUM_CHANNELS) return;
    nrf54l_dppi_subscriber_t **slot = &d->subs[channel];
    while (*slot) {
        if ((*slot)->cb == cb && (*slot)->user == user) {
            nrf54l_dppi_subscriber_t *dead = *slot;
            *slot = dead->next;
            free(dead);
            return;
        }
        slot = &(*slot)->next;
    }
}

void nrf54l_dppi_publish(nrf54l_dppi_state_t *d, int channel) {
    if (channel < 0 || channel >= NRF54L_DPPI_NUM_CHANNELS) return;
    if (!(d->chen & (1u << channel))) return;   /* channel gated off */
    for (nrf54l_dppi_subscriber_t *s = d->subs[channel]; s; s = s->next)
        s->cb(s->user);
}

static int nrf54l_dppic_read(void *user_data, uint32_t addr) {
    nrf54l_dppi_state_t *d = (nrf54l_dppi_state_t *)user_data;
    /* DPPIC bases overlap by region in addressing; the meaningful sub-
     * offset is the low 12 bits. */
    uint32_t off = addr & 0xFFFu;
    switch (off) {
        case NRF54L_DPPI_CHEN:
        case NRF54L_DPPI_CHENSET:
            return (int)d->chen;
        default: return 0;
    }
}

static void nrf54l_dppic_write(void *user_data, uint32_t addr, uint32_t value) {
    nrf54l_dppi_state_t *d = (nrf54l_dppi_state_t *)user_data;
    uint32_t off = addr & 0xFFFu;
    switch (off) {
        case NRF54L_DPPI_CHEN:    d->chen  = value;  break;
        case NRF54L_DPPI_CHENSET: d->chen |= value;  break;
        case NRF54L_DPPI_CHENCLR: d->chen &= ~value; break;
        default: /* TASKS_CHG, SUBSCRIBE_CHG, CHG cluster: no-op */ break;
    }
}

/* ============================================================
 * Peripheral aliases on nrf54l
 *
 * Contiki's nrf54l15 build maps every peripheral via the 0x5xxx_xxxx
 * alias (the secure-mapped window), not the 0x4xxx_xxxx non-secure
 * base.  Confirmed by `objdump -d` of the firmware: no 0x4008A000
 * literals, but 0x5008A000 (RADIO_S), 0x500E2000 (GRTC_S), 0x500C6000
 * (UARTE20_S).  Stick with 0x5xxx for every nrf54l peripheral.
 * ============================================================ */

/* ============================================================
 * GRTC (Global Real-Time Counter) at 0x500E_2000
 *
 * Drives Contiki's clock_arch tick (1 MHz syscounter) and is the
 * timer MPSL hands to nrf_802154 for timeslot grants.  Without GRTC
 * compare events firing, the radio driver never enables RX/TX.
 *
 * Register offsets (per nrf54lv10a_enga_application.svd):
 *   0x000+4n  TASKS_CAPTURE[0..11]   — latch counter into CC[n].CCL/CCH
 *   0x060     TASKS_START
 *   0x064     TASKS_STOP
 *   0x068     TASKS_CLEAR
 *   0x100+4n  EVENTS_COMPARE[0..11]
 *   0x300     INTEN0   (bit n = COMPARE[n] enable)
 *   0x304     INTENSET0
 *   0x308     INTENCLR0
 *   0x30C     INTPEND0
 *   0x310+10*k INTEN1/2/3 (one per GRTC IRQ line; we only fire GRTC_2)
 *   0x510     MODE
 *   0x520+10n CC[n].CCL    (compare value low 32 bits)
 *   0x524+10n CC[n].CCH    (compare value high 32 bits)
 *   0x528+10n CC[n].CCADD  (relative offset from now — used by nrfx_grtc
 *                            _syscounter_cc_relative_set; firmware writes
 *                            this then sets CCEN, model schedules event)
 *   0x52C+10n CC[n].CCEN   (write 1 to arm, 0 to disarm)
 *   0x720     SYSCOUNTER[0].SYSCOUNTERL  (32-bit low half)
 *   0x724     SYSCOUNTER[0].SYSCOUNTERH  (32-bit high half + LOADED bit)
 *   0x748     SYSCOUNTER[0] capture-ready (write 1 → latch L/H)
 *
 * Counter rate is 1 MHz (1 µs per tick) per Contiki's
 * GRTC_TICK_FREQUENCY_HZ = 1_000_000.  We compute the counter from
 * sim_time_ns: counter = (sim_time_ns - start_anchor_ns) / 1000.
 * ============================================================ */
#define NRF54L_GRTC_BASE                  0x500E2000u
#define NRF54L_GRTC_SIZE                  0x1000u

#define NRF54L_GRTC_TASKS_CAPTURE_BASE    0x000
#define NRF54L_GRTC_TASKS_START           0x060
#define NRF54L_GRTC_TASKS_STOP            0x064
#define NRF54L_GRTC_TASKS_CLEAR           0x068
#define NRF54L_GRTC_EVENTS_COMPARE_BASE   0x100
#define NRF54L_GRTC_PUBLISH_COMPARE_BASE  0x180
#define NRF54L_GRTC_INTEN0                0x300
#define NRF54L_GRTC_INTENSET0             0x304
#define NRF54L_GRTC_INTENCLR0             0x308
#define NRF54L_GRTC_INTPEND0              0x30C
#define NRF54L_GRTC_INTEN_BANKS_END       0x320   /* INTEN1/2/3 follow at 0x310/0x318/0x320 */
#define NRF54L_GRTC_CC_BASE               0x520
#define NRF54L_GRTC_CC_STRIDE             0x10
#define NRF54L_GRTC_CC_END                (NRF54L_GRTC_CC_BASE + NRF54L_GRTC_NUM_CC * NRF54L_GRTC_CC_STRIDE)
#define NRF54L_GRTC_SYSCOUNTERL           0x720
#define NRF54L_GRTC_SYSCOUNTERH           0x724
#define NRF54L_GRTC_SYSCOUNTER_CAPTURE    0x748

#define NRF54L_GRTC_TICK_NS               1000    /* 1 MHz syscounter */
#define NRF54L_GRTC_IRQ                   228     /* GRTC_2_IRQn on app core */

static uint64_t grtc_counter_now(nrf54l_grtc_state_t *grtc) {
    if (!grtc->running) return 0;
    int64_t elapsed = grtc->plat->cpu.sim_time_ns - grtc->start_anchor_ns;
    if (elapsed < 0) elapsed = 0;
    return (uint64_t)elapsed / NRF54L_GRTC_TICK_NS;
}

/* Forward declaration */
static void nrf54l_grtc_compare_fire(void *user_data, cpu_event_t *event);

static void nrf54l_grtc_arm_cc(nrf54l_grtc_state_t *grtc, int idx) {
    nrf54l_grtc_cc_t *cc = &grtc->cc[idx];
    if (!cc->event) {
        cc->event = (cpu_event_t *)calloc(1, sizeof(cpu_event_t));
        ((cpu_event_t *)cc->event)->callback  = nrf54l_grtc_compare_fire;
        /* user_data = grtc; the callback iterates cc[] to find which
         * slot owns the fired event pointer. */
        ((cpu_event_t *)cc->event)->user_data = grtc;
    }
    arm_cancel_event(&grtc->plat->cpu, (arm_event_t *)cc->event);
    /* Compute firing wall-clock time in ns.  Firmware uses CCADD as a
     * relative offset from the current counter, so the event fires
     * CCADD ticks from now. */
    int64_t fire_ns = grtc->plat->cpu.sim_time_ns +
                      (int64_t)cc->ccadd * NRF54L_GRTC_TICK_NS;
    arm_schedule_event_ns(&grtc->plat->cpu, (arm_event_t *)cc->event, fire_ns);
}

static void nrf54l_grtc_compare_fire(void *user_data, cpu_event_t *event) {
    nrf54l_grtc_state_t *grtc = (nrf54l_grtc_state_t *)user_data;
    /* Find which CC slot owns this event. */
    int idx = -1;
    for (int i = 0; i < NRF54L_GRTC_NUM_CC; i++) {
        if (grtc->cc[i].event == event) { idx = i; break; }
    }
    if (idx < 0) return;
    /* Latch the event and pend the IRQ if enabled. */
    grtc->evt_compare[idx] = 1;
    if (grtc->inten & (1u << idx)) {
        arm_nvic_set_pending(&grtc->plat->nvic, grtc->irq_num);
    }
    /* Publish on the DPPI channel configured by PUBLISH_COMPARE[idx]
     * (bit 31 = EN, bits 4..0 = channel ID).  This is how MPSL routes
     * GRTC ticks to RADIO TASKS_TXEN / _RXEN. */
    uint32_t pub = grtc->publish_compare[idx];
    if (grtc->dppi && (pub & 0x80000000u))
        nrf54l_dppi_publish(grtc->dppi, (int)(pub & 0x1Fu));
    /* CCEN remains set in our model — firmware re-arms by writing
     * a new CCADD value, so we don't auto-disarm. */
}

static int nrf54l_grtc_read(void *user_data, uint32_t addr) {
    nrf54l_grtc_state_t *grtc = (nrf54l_grtc_state_t *)user_data;
    uint32_t off = addr - NRF54L_GRTC_BASE;

    /* EVENTS_COMPARE[0..11] */
    if (off >= NRF54L_GRTC_EVENTS_COMPARE_BASE &&
        off < NRF54L_GRTC_EVENTS_COMPARE_BASE + 4 * NRF54L_GRTC_NUM_CC) {
        int idx = (off - NRF54L_GRTC_EVENTS_COMPARE_BASE) / 4;
        return (int)grtc->evt_compare[idx];
    }

    /* PUBLISH_COMPARE[0..11] */
    if (off >= NRF54L_GRTC_PUBLISH_COMPARE_BASE &&
        off < NRF54L_GRTC_PUBLISH_COMPARE_BASE + 4 * NRF54L_GRTC_NUM_CC) {
        int idx = (off - NRF54L_GRTC_PUBLISH_COMPARE_BASE) / 4;
        return (int)grtc->publish_compare[idx];
    }

    /* CC[0..11].(CCL/CCH/CCADD/CCEN) */
    if (off >= NRF54L_GRTC_CC_BASE && off < NRF54L_GRTC_CC_END) {
        int idx = (off - NRF54L_GRTC_CC_BASE) / NRF54L_GRTC_CC_STRIDE;
        int sub = (off - NRF54L_GRTC_CC_BASE) % NRF54L_GRTC_CC_STRIDE;
        switch (sub) {
            case 0x0: return (int)grtc->cc[idx].ccl;
            case 0x4: return (int)grtc->cc[idx].cch;
            case 0x8: return (int)grtc->cc[idx].ccadd;
            case 0xC: return (int)grtc->cc[idx].ccen;
        }
    }

    switch (off) {
        case NRF54L_GRTC_SYSCOUNTERL: {
            /* If a capture was requested, return latched value;
             * otherwise compute live counter low half. */
            if (grtc->captured_lo || grtc->captured_hi) return (int)grtc->captured_lo;
            uint64_t c = grtc_counter_now(grtc);
            return (int)(uint32_t)c;
        }
        case NRF54L_GRTC_SYSCOUNTERH: {
            if (grtc->captured_lo || grtc->captured_hi) {
                /* SYSCOUNTERH carries a LOADED status bit (bit 31) per the SVD.
                 * Real HW: LOADED=1 means the value is valid. */
                return (int)(grtc->captured_hi | (1u << 31));
            }
            uint64_t c = grtc_counter_now(grtc);
            return (int)((uint32_t)(c >> 32) | (1u << 31));
        }
        case NRF54L_GRTC_SYSCOUNTER_CAPTURE:
            /* Reads return 1 to indicate ready/busy=ready. */
            return 1;
        case NRF54L_GRTC_INTEN0:
        case NRF54L_GRTC_INTENSET0:
            return (int)grtc->inten;
        case NRF54L_GRTC_INTPEND0: {
            uint32_t pending = 0;
            for (int i = 0; i < NRF54L_GRTC_NUM_CC; i++) {
                if (grtc->evt_compare[i] && (grtc->inten & (1u << i)))
                    pending |= (1u << i);
            }
            return (int)pending;
        }
        default: return 0;
    }
}

static void nrf54l_grtc_write(void *user_data, uint32_t addr, uint32_t value) {
    nrf54l_grtc_state_t *grtc = (nrf54l_grtc_state_t *)user_data;
    uint32_t off = addr - NRF54L_GRTC_BASE;

    /* EVENTS_COMPARE clears */
    if (off >= NRF54L_GRTC_EVENTS_COMPARE_BASE &&
        off < NRF54L_GRTC_EVENTS_COMPARE_BASE + 4 * NRF54L_GRTC_NUM_CC) {
        int idx = (off - NRF54L_GRTC_EVENTS_COMPARE_BASE) / 4;
        grtc->evt_compare[idx] = value & 1;
        return;
    }

    /* PUBLISH_COMPARE writes — store channel routing for compare-fire. */
    if (off >= NRF54L_GRTC_PUBLISH_COMPARE_BASE &&
        off < NRF54L_GRTC_PUBLISH_COMPARE_BASE + 4 * NRF54L_GRTC_NUM_CC) {
        int idx = (off - NRF54L_GRTC_PUBLISH_COMPARE_BASE) / 4;
        grtc->publish_compare[idx] = value;
        return;
    }

    /* CC[0..11] writes */
    if (off >= NRF54L_GRTC_CC_BASE && off < NRF54L_GRTC_CC_END) {
        int idx = (off - NRF54L_GRTC_CC_BASE) / NRF54L_GRTC_CC_STRIDE;
        int sub = (off - NRF54L_GRTC_CC_BASE) % NRF54L_GRTC_CC_STRIDE;
        switch (sub) {
            case 0x0: grtc->cc[idx].ccl   = value; break;
            case 0x4: grtc->cc[idx].cch   = value; break;
            case 0x8: grtc->cc[idx].ccadd = value; break;
            case 0xC:
                grtc->cc[idx].ccen = value & 1;
                if (grtc->cc[idx].ccen && grtc->running)
                    nrf54l_grtc_arm_cc(grtc, idx);
                else if (!grtc->cc[idx].ccen && grtc->cc[idx].event)
                    arm_cancel_event(&grtc->plat->cpu,
                                      (arm_event_t *)grtc->cc[idx].event);
                break;
        }
        return;
    }

    switch (off) {
        case NRF54L_GRTC_TASKS_START:
            if (value == 1) {
                grtc->running = true;
                grtc->start_anchor_ns = grtc->plat->cpu.sim_time_ns;
                /* Re-arm any pre-loaded CCs. */
                for (int i = 0; i < NRF54L_GRTC_NUM_CC; i++)
                    if (grtc->cc[i].ccen) nrf54l_grtc_arm_cc(grtc, i);
            }
            break;
        case NRF54L_GRTC_TASKS_STOP:
            if (value == 1) {
                grtc->running = false;
                for (int i = 0; i < NRF54L_GRTC_NUM_CC; i++)
                    if (grtc->cc[i].event)
                        arm_cancel_event(&grtc->plat->cpu,
                                          (arm_event_t *)grtc->cc[i].event);
            }
            break;
        case NRF54L_GRTC_TASKS_CLEAR:
            if (value == 1) grtc->start_anchor_ns = grtc->plat->cpu.sim_time_ns;
            break;
        case NRF54L_GRTC_SYSCOUNTER_CAPTURE:
            /* Write 1 = trigger capture; write 0 = clear ready flag. */
            if (value == 1) {
                uint64_t c = grtc_counter_now(grtc);
                grtc->captured_lo = (uint32_t)c;
                grtc->captured_hi = (uint32_t)(c >> 32);
            } else {
                grtc->captured_lo = 0;
                grtc->captured_hi = 0;
            }
            break;
        case NRF54L_GRTC_INTEN0:    grtc->inten  = value;          break;
        case NRF54L_GRTC_INTENSET0: grtc->inten |= value;          break;
        case NRF54L_GRTC_INTENCLR0: grtc->inten &= ~value;         break;
        default:
            /* Accept SHORTS, PUBLISH/SUBSCRIBE, MODE, INTEN1/2/3,
             * TASKS_CAPTURE[n], etc. as no-ops. */
            break;
    }
}

/* ============================================================
 * SoC lifecycle
 * ============================================================ */
static void nrf54l15_soc_init(arm_platform_t *plat) {
    nrf54l15_soc_t *soc = (nrf54l15_soc_t *)calloc(1, sizeof(*soc));
    plat->soc = soc;
    soc->plat = plat;

    /* Per-board VTOR override (both DK and XIAO use 0 — no bootloader). */
    if (plat->config && plat->config->vtor_override)
        plat->cpu.vtor_default = plat->config->vtor_override;

    /* Minimum host vtable so anything reaching plat->host finds valid
     * function pointers even before peripherals are wired. */
    plat->host.cpu         = &plat->cpu;
    plat->host.gpio        = NULL;
    plat->host.now_ns      = nrf54l_host_now_ns;
    plat->host.schedule_ns = nrf54l_host_schedule_ns;
    plat->host.cancel      = nrf54l_host_cancel;

    /* GLOBAL_CLOCK — first peripheral firmware touches after reset. */
    arm_register_io(&plat->cpu,
                    NRF54L_GLOBAL_CLOCK_BASE, NRF54L_GLOBAL_CLOCK_SIZE,
                    nrf54l_global_clock_read, nrf54l_global_clock_write,
                    &soc->global_clock);

    /* UARTE20 — pure EasyDMA console. */
    soc->uarte20.plat = plat;
    arm_register_io(&plat->cpu,
                    NRF54L_UARTE20_BASE, NRF54L_UARTE20_SIZE,
                    nrf54l_uarte_read, nrf54l_uarte_write, &soc->uarte20);

    /* DPPI fabric — single 32-channel state aliased by all four
     * DPPIC base addresses (DPPIC00/10/20/30).  Subscribers register
     * via nrf54l_dppi_subscribe(); publishers call nrf54l_dppi_publish().
     * No state to initialise — channels start disabled, subs list empty. */
    for (size_t i = 0; i < sizeof(nrf54l_dppic_bases) / sizeof(nrf54l_dppic_bases[0]); i++) {
        arm_register_io(&plat->cpu,
                        nrf54l_dppic_bases[i], NRF54L_DPPIC_SIZE,
                        nrf54l_dppic_read, nrf54l_dppic_write, &soc->dppi);
    }

    /* GRTC — Contiki tick source + MPSL timeslot timer. */
    soc->grtc.plat    = plat;
    soc->grtc.dppi    = &soc->dppi;
    soc->grtc.irq_num = NRF54L_GRTC_IRQ;
    arm_register_io(&plat->cpu,
                    NRF54L_GRTC_BASE, NRF54L_GRTC_SIZE,
                    nrf54l_grtc_read, nrf54l_grtc_write, &soc->grtc);
}

static void nrf54l15_soc_destroy(arm_platform_t *plat) {
    nrf54l15_soc_t *soc = (nrf54l15_soc_t *)plat->soc;
    if (soc) {
        /* Cancel + free any armed GRTC CC events. */
        for (int i = 0; i < NRF54L_GRTC_NUM_CC; i++) {
            if (soc->grtc.cc[i].event) {
                arm_cancel_event(&plat->cpu,
                                  (arm_event_t *)soc->grtc.cc[i].event);
                free(soc->grtc.cc[i].event);
                soc->grtc.cc[i].event = NULL;
            }
        }
        /* Free any remaining DPPI subscriber linked-list nodes. */
        for (int i = 0; i < NRF54L_DPPI_NUM_CHANNELS; i++) {
            nrf54l_dppi_subscriber_t *s = soc->dppi.subs[i];
            while (s) {
                nrf54l_dppi_subscriber_t *next = s->next;
                free(s);
                s = next;
            }
        }
        free(soc);
        plat->soc = NULL;
    }
}

static void nrf54l15_soc_set_console(arm_platform_t *plat,
                                      arm_uart_tx_callback cb, void *user_data) {
    nrf54l15_soc_t *soc = (nrf54l15_soc_t *)plat->soc;
    soc->uarte20.tx_cb   = cb;
    soc->uarte20.tx_user = user_data;
}

const arm_soc_ops_t nrf54l15_soc_ops = {
    .name        = "nrf54l15",
    .init        = nrf54l15_soc_init,
    .destroy     = nrf54l15_soc_destroy,
    .set_console = nrf54l15_soc_set_console,
};
