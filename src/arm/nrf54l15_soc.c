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
/* GRTC has 4 IRQ lines (GRTC_0..3) with their own INTEN bank each.
 * Contiki nrf54l15 routes its tick through GRTC_2 (IRQ 228), so we
 * only care about INTEN2.  Other banks are accepted as no-ops. */
#define NRF54L_GRTC_INTEN0                0x300
#define NRF54L_GRTC_INTENSET0             0x304
#define NRF54L_GRTC_INTENCLR0             0x308
#define NRF54L_GRTC_INTEN2                0x320
#define NRF54L_GRTC_INTENSET2             0x324
#define NRF54L_GRTC_INTENCLR2             0x328
#define NRF54L_GRTC_CC_BASE               0x520
#define NRF54L_GRTC_CC_STRIDE             0x10
#define NRF54L_GRTC_CC_END                (NRF54L_GRTC_CC_BASE + NRF54L_GRTC_NUM_CC * NRF54L_GRTC_CC_STRIDE)
/* SYSCOUNTER cluster at 0x720, dim=4, dimIncrement=0x10.  Each entry
 * is one CPU's view of the 52-bit counter (L+H pair):
 *   SYSCOUNTER[0]: 0x720/0x724  (secure)
 *   SYSCOUNTER[1]: 0x730/0x734
 *   SYSCOUNTER[2]: 0x740/0x744  (application core — what Contiki reads)
 *   SYSCOUNTER[3]: 0x750/0x754  (FLPR)
 * All four return the same logical counter in csim (we don't simulate
 * per-CPU views).  Bit 29 = LOADED, 30 = BUSY, 31 = OVERFLOW. */
#define NRF54L_GRTC_SYSCOUNTER_BASE       0x720
#define NRF54L_GRTC_SYSCOUNTER_END        0x760

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
    /* CCADD bit 31 selects the reference (SYSCOUNTER vs CC value) and
     * must not be folded into the delay. */
    uint32_t delay_ticks = cc->ccadd & 0x7FFFFFFFu;
    int64_t fire_ns = grtc->plat->cpu.sim_time_ns +
                      (int64_t)delay_ticks * NRF54L_GRTC_TICK_NS;
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

    /* SYSCOUNTER[n] cluster — any of the 4 CPUs' views maps to the same
     * counter in our model.  Offsets 0x720..0x75F. */
    if (off >= NRF54L_GRTC_SYSCOUNTER_BASE && off < NRF54L_GRTC_SYSCOUNTER_END) {
        int sub = (off - NRF54L_GRTC_SYSCOUNTER_BASE) % 0x10;
        if (sub == 0x0) {  /* SYSCOUNTERL */
            if (grtc->captured_lo || grtc->captured_hi) return (int)grtc->captured_lo;
            uint64_t c = grtc_counter_now(grtc);
            return (int)(uint32_t)c;
        }
        if (sub == 0x4) {  /* SYSCOUNTERH */
            if (grtc->captured_lo || grtc->captured_hi)
                return (int)((grtc->captured_hi & 0xFFFFFu) | (1u << 29));
            uint64_t c = grtc_counter_now(grtc);
            return (int)(((uint32_t)(c >> 32) & 0xFFFFFu) | (1u << 29));
        }
        if (sub == 0x8) return 1;  /* CAPTURE control — always ready */
        return 0;
    }
    switch (off) {
        case NRF54L_GRTC_INTEN0:
        case NRF54L_GRTC_INTENSET0:
            return 0;       /* GRTC_0 IRQ unused by Contiki */
        case NRF54L_GRTC_INTEN2:
        case NRF54L_GRTC_INTENSET2:
            return (int)grtc->inten;
        case 0x32C: {       /* INTPEND2 — pending+enabled bitmap */
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

    /* SYSCOUNTER[n] writes — write 1 to the +0x8 control reg captures
     * the counter into the L/H pair. */
    if (off >= NRF54L_GRTC_SYSCOUNTER_BASE && off < NRF54L_GRTC_SYSCOUNTER_END) {
        int sub = (off - NRF54L_GRTC_SYSCOUNTER_BASE) % 0x10;
        if (sub == 0x8) {
            if (value == 1) {
                uint64_t c = grtc_counter_now(grtc);
                grtc->captured_lo = (uint32_t)c;
                grtc->captured_hi = (uint32_t)(c >> 32);
            } else {
                grtc->captured_lo = 0;
                grtc->captured_hi = 0;
            }
        }
        return;
    }

    /* CC[0..11] writes */
    if (off >= NRF54L_GRTC_CC_BASE && off < NRF54L_GRTC_CC_END) {
        int idx = (off - NRF54L_GRTC_CC_BASE) / NRF54L_GRTC_CC_STRIDE;
        int sub = (off - NRF54L_GRTC_CC_BASE) % NRF54L_GRTC_CC_STRIDE;
        switch (sub) {
            case 0x0: grtc->cc[idx].ccl   = value; break;
            case 0x4: grtc->cc[idx].cch   = value; break;
            case 0x8:
                grtc->cc[idx].ccadd = value;
                if (grtc->cc[idx].ccen && grtc->running)
                    nrf54l_grtc_arm_cc(grtc, idx);
                break;
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
        case NRF54L_GRTC_INTEN2:    grtc->inten  = value;          break;
        case NRF54L_GRTC_INTENSET2: grtc->inten |= value;          break;
        case NRF54L_GRTC_INTENCLR2: grtc->inten &= ~value;         break;
        default:
            /* Accept SHORTS, PUBLISH/SUBSCRIBE, MODE, INTEN0/1/3,
             * TASKS_CAPTURE[n], etc. as no-ops. */
            break;
    }
}

/* ============================================================
 * EGU (Event Generator Unit) — software-trigger → DPPI bridge.
 *
 * Three instances on nrf54l15: EGU00 (0x5001_5000), EGU10 (0x5008_7000,
 * paired with RADIO), EGU20 (0x500C_7000).  The Nordic nrf_802154
 * driver uses EGU10 as the "kick" point for ramp-up: CPU writes
 * TASKS_TRIGGER[15] → EVENTS_TRIGGERED[15] fires → PUBLISH_TRIGGERED[15]
 * publishes on its bound channel → RADIO.SUBSCRIBE_RXEN catches it.
 *
 * Register layout (per the SVD):
 *   0x000+4n  TASKS_TRIGGER[0..15]
 *   0x080+4n  SUBSCRIBE_TRIGGER[0..15]
 *   0x100+4n  EVENTS_TRIGGERED[0..15]
 *   0x180+4n  PUBLISH_TRIGGERED[0..15]
 *   0x300/4/8 INTEN / INTENSET / INTENCLR
 *   0x30C     INTPEND
 * ============================================================ */
#define NRF54L_EGU00_BASE  0x50015000u
#define NRF54L_EGU10_BASE  0x50087000u
#define NRF54L_EGU20_BASE  0x500C7000u
#define NRF54L_EGU_SIZE    0x1000u

#define E_TASKS_BASE       0x000
#define E_SUBSCRIBE_BASE   0x080
#define E_EVENTS_BASE      0x100
#define E_PUBLISH_BASE     0x180
#define E_INTEN            0x300
#define E_INTENSET         0x304
#define E_INTENCLR         0x308
#define E_INTPEND          0x30C

/* Forward decl */
static void egu_fire_event(nrf54l_egu_state_t *e, int n);

/* SUBSCRIBE binding storage — one slot per (EGU, channel) — used to
 * dispatch DPPI publishes back into the right TASKS_TRIGGER[n]. */
typedef struct {
    nrf54l_egu_state_t *egu;
    int                 task_n;
} egu_sub_binding_t;
static egu_sub_binding_t egu_sub_bindings[3 /* EGU instances */ * NRF54L_EGU_NUM_CHANNELS];

static void egu_sub_cb(void *user) {
    egu_sub_binding_t *b = (egu_sub_binding_t *)user;
    egu_fire_event(b->egu, b->task_n);
}

static void egu_fire_event(nrf54l_egu_state_t *e, int n) {
    if (n < 0 || n >= NRF54L_EGU_NUM_CHANNELS) return;
    e->events[n] = 1;
    if ((e->inten & (1u << n)) && e->irq_num >= 0)
        arm_nvic_set_pending(&e->plat->nvic, e->irq_num);
    uint32_t pub = e->publish[n];
    if ((pub & 0x80000000u) && e->dppi)
        nrf54l_dppi_publish(e->dppi, (int)(pub & 0x1Fu));
}

static int nrf54l_egu_read(void *user_data, uint32_t addr) {
    nrf54l_egu_state_t *e = (nrf54l_egu_state_t *)user_data;
    uint32_t off = addr & 0xFFFu;
    if (off >= E_SUBSCRIBE_BASE && off < E_SUBSCRIBE_BASE + 4 * NRF54L_EGU_NUM_CHANNELS)
        return (int)e->subscribe[(off - E_SUBSCRIBE_BASE) / 4];
    if (off >= E_EVENTS_BASE && off < E_EVENTS_BASE + 4 * NRF54L_EGU_NUM_CHANNELS)
        return (int)e->events[(off - E_EVENTS_BASE) / 4];
    if (off >= E_PUBLISH_BASE && off < E_PUBLISH_BASE + 4 * NRF54L_EGU_NUM_CHANNELS)
        return (int)e->publish[(off - E_PUBLISH_BASE) / 4];
    switch (off) {
        case E_INTEN:
        case E_INTENSET: return (int)e->inten;
        default:         return 0;
    }
}

static void nrf54l_egu_write(void *user_data, uint32_t addr, uint32_t value) {
    nrf54l_egu_state_t *e = (nrf54l_egu_state_t *)user_data;
    uint32_t off = addr & 0xFFFu;
    /* TASKS_TRIGGER[0..15] — write 1 to fire. */
    if (off < E_TASKS_BASE + 4 * NRF54L_EGU_NUM_CHANNELS) {
        if (value == 1) egu_fire_event(e, (off - E_TASKS_BASE) / 4);
        return;
    }
    /* SUBSCRIBE_TRIGGER[n] — bind/unbind DPPI channel. */
    if (off >= E_SUBSCRIBE_BASE && off < E_SUBSCRIBE_BASE + 4 * NRF54L_EGU_NUM_CHANNELS) {
        int idx = (off - E_SUBSCRIBE_BASE) / 4;
        e->subscribe[idx] = value;
        /* Find this instance's binding slot. */
        int instance_off = 0;
        nrf54l15_soc_t *soc = (nrf54l15_soc_t *)e->plat->soc;
        if (e == &soc->egu00)      instance_off = 0;
        else if (e == &soc->egu10) instance_off = NRF54L_EGU_NUM_CHANNELS;
        else                        instance_off = 2 * NRF54L_EGU_NUM_CHANNELS;
        egu_sub_binding_t *binding = &egu_sub_bindings[instance_off + idx];
        if (e->sub_channel[idx] >= 0) {
            nrf54l_dppi_unsubscribe(e->dppi, e->sub_channel[idx], egu_sub_cb, binding);
            e->sub_channel[idx] = -1;
        }
        if (value & 0x80000000u) {
            int ch = (int)(value & 0x1Fu);
            binding->egu     = e;
            binding->task_n  = idx;
            nrf54l_dppi_subscribe(e->dppi, ch, egu_sub_cb, binding);
            e->sub_channel[idx] = ch;
        }
        return;
    }
    /* EVENTS_TRIGGERED[n] — firmware clears. */
    if (off >= E_EVENTS_BASE && off < E_EVENTS_BASE + 4 * NRF54L_EGU_NUM_CHANNELS) {
        e->events[(off - E_EVENTS_BASE) / 4] = value & 1;
        return;
    }
    /* PUBLISH_TRIGGERED[n] — store for later. */
    if (off >= E_PUBLISH_BASE && off < E_PUBLISH_BASE + 4 * NRF54L_EGU_NUM_CHANNELS) {
        e->publish[(off - E_PUBLISH_BASE) / 4] = value;
        return;
    }
    switch (off) {
        case E_INTEN:    e->inten  = value;  break;
        case E_INTENSET: e->inten |= value;  break;
        case E_INTENCLR: e->inten &= ~value; break;
        default: break;
    }
}

/* ============================================================
 * RADIO (0x5008_A000)
 *
 * IEEE 802.15.4 PHY.  Programmed via Nordic's nrf_802154 SDK driver,
 * which routes every task trigger through DPPI rather than writing
 * task registers directly.  We model:
 *
 *   - State machine: DISABLED ↔ RXIDLE/RX ↔ TXIDLE/TX (no rampup
 *     delays — entry events fire immediately on state change).
 *   - SUBSCRIBE_<task>: write `0x80000000 | channel` to bind the
 *     task to a DPPI channel; we register a DPPI subscriber that
 *     triggers the task when the channel fires.
 *   - PUBLISH_<event>: write `0x80000000 | channel` to publish the
 *     event on a DPPI channel; we call `dppi_publish(channel)`
 *     whenever the event latches.
 *   - SHORTS (0x400): READY_START / TXREADY_START / RXREADY_START
 *     / PHYEND_DISABLE / END_DISABLE bits auto-chain on entry.
 *   - INTENSET00 / INTENSET10: per-event masks for two NVIC IRQ
 *     lines (RADIO_0 = 138, RADIO_1 = 139).
 *   - EasyDMA TX: TASKS_START in TXIDLE walks PACKETPTR (PHR +
 *     payload), computes IEEE 802.15.4 FCS, emits preamble + SFD +
 *     PHR + payload + FCS to the multinode TX listener.
 *   - EasyDMA RX: byte stream parser feeds PACKETPTR; on frame
 *     end, fires CRCOK / END / PHYEND, sends auto-ACK if frame is
 *     a unicast Data with ACK_REQUEST set.
 *
 * Same FCS algorithm + auto-ACK shortcut as `nrf52840_soc.c`'s
 * radio model — just different register offsets and the DPPI
 * indirection.
 * ============================================================ */
#define NRF54L_RADIO_BASE              0x5008A000u
#define NRF54L_RADIO_SIZE              0x2000u
#define NRF54L_RADIO_IRQ_0             138
#define NRF54L_RADIO_IRQ_1             139

/* Task / SUBSCRIBE / EVENT / PUBLISH register offsets. */
#define R_TASKS_TXEN                   0x000
#define R_TASKS_RXEN                   0x004
#define R_TASKS_START                  0x008
#define R_TASKS_STOP                   0x00C
#define R_TASKS_DISABLE                0x010
#define R_TASKS_RSSISTART              0x014
#define R_TASKS_BCSTART                0x018
#define R_TASKS_BCSTOP                 0x01C
#define R_TASKS_EDSTART                0x020
#define R_TASKS_EDSTOP                 0x024
#define R_TASKS_CCASTART               0x028
#define R_TASKS_CCASTOP                0x02C

#define R_SUBSCRIBE_BASE               0x100
#define R_SUBSCRIBE_END                (R_SUBSCRIBE_BASE + 4 * NRF54L_RADIO_NUM_SUBSCRIBES)
#define R_SUBSCRIBE_INDEX(off)         (((off) - R_SUBSCRIBE_BASE) / 4)

#define R_EVENTS_READY                 0x200
#define R_EVENTS_TXREADY               0x204
#define R_EVENTS_RXREADY               0x208
#define R_EVENTS_ADDRESS               0x20C
#define R_EVENTS_FRAMESTART            0x210
#define R_EVENTS_PAYLOAD               0x214
#define R_EVENTS_END                   0x218
#define R_EVENTS_PHYEND                0x21C
#define R_EVENTS_DISABLED              0x220
#define R_EVENTS_DEVMATCH              0x224
#define R_EVENTS_DEVMISS               0x228
#define R_EVENTS_CRCOK                 0x22C
#define R_EVENTS_CRCERROR              0x230
#define R_EVENTS_BCMATCH               0x238
#define R_EVENTS_EDEND                 0x23C
#define R_EVENTS_EDSTOPPED             0x240
#define R_EVENTS_CCAIDLE               0x244
#define R_EVENTS_CCABUSY               0x248
#define R_EVENTS_CCASTOPPED            0x24C
#define R_EVENTS_RATEBOOST             0x250
#define R_EVENTS_MHRMATCH              0x254
#define R_EVENTS_SYNC                  0x258
#define R_EVENTS_CTEPRESENT            0x25C

#define R_PUBLISH_BASE                 0x300
#define R_PUBLISH_END                  (R_PUBLISH_BASE + 4 * NRF54L_RADIO_NUM_PUBLISHES)

#define R_SHORTS                       0x400
#define R_INTENSET00                   0x488
#define R_INTENCLR00                   0x490
#define R_INTENSET10                   0x4A8
#define R_INTENCLR10                   0x4B0
#define R_MODE                         0x500
#define R_STATE                        0x520
#define R_TIMING                       0x704
#define R_FREQUENCY                    0x708
#define R_TXPOWER                      0x710
#define R_TIFS                         0x714
#define R_PCNF0                        0xE20
#define R_PCNF1                        0xE28
#define R_CRCCNF                       0xE44
#define R_CRCPOLY                      0xE48
#define R_CRCINIT                      0xE4C
#define R_BCC                          0xE94
#define R_PACKETPTR                    0xED0
#define R_CRCSTATUS                    0xE0C
#define R_RXMATCH                      0xE10

/* INTENSET bit positions (per the SVD field ordering — bits are
 * assigned in event-list order starting at bit 0). */
enum {
    INT_READY = 0,    INT_TXREADY,  INT_RXREADY,  INT_ADDRESS,
    INT_FRAMESTART,   INT_PAYLOAD,  INT_END,      INT_PHYEND,
    INT_DISABLED,     INT_DEVMATCH, INT_DEVMISS,  INT_CRCOK,
    INT_CRCERROR,     INT_BCMATCH,  INT_EDEND,    INT_EDSTOPPED,
    INT_CCAIDLE,      INT_CCABUSY,  INT_CCASTOPPED,
    INT_RATEBOOST,    INT_MHRMATCH, INT_SYNC,     INT_CTEPRESENT,
};

/* PUBLISH index ↔ event mapping.  Both clusters are indexed by the
 * same enum (READY = 0 through CTEPRESENT). */
enum {
    PUB_READY = 0, PUB_TXREADY, PUB_RXREADY, PUB_ADDRESS,
    PUB_FRAMESTART, PUB_PAYLOAD, PUB_END, PUB_PHYEND,
    PUB_DISABLED, PUB_DEVMATCH, PUB_DEVMISS, PUB_CRCOK,
    PUB_CRCERROR, /* skip */ PUB_BCMATCH = 14, PUB_EDEND, PUB_EDSTOPPED,
    PUB_CCAIDLE, PUB_CCABUSY, PUB_CCASTOPPED, PUB_RATEBOOST,
    PUB_MHRMATCH, PUB_SYNC, PUB_CTEPRESENT
};
/* (BCMATCH is at 0x338 not 0x334 due to a gap — index 13 is unused.) */

/* SHORTS bits — taken from <lsb> entries in the SVD (do NOT match the
 * sequential order of field declarations; bits 1, 7-9, etc. are
 * reserved). */
#define R_SHORT_READY_START          (1u <<  0)
#define R_SHORT_DISABLED_TXEN        (1u <<  2)
#define R_SHORT_DISABLED_RXEN        (1u <<  3)
#define R_SHORT_ADDRESS_RSSISTART    (1u <<  4)
#define R_SHORT_END_START            (1u <<  5)
#define R_SHORT_ADDRESS_BCSTART      (1u <<  6)
#define R_SHORT_RXREADY_CCASTART     (1u << 10)
#define R_SHORT_CCAIDLE_TXEN         (1u << 11)
#define R_SHORT_CCABUSY_DISABLE      (1u << 12)
#define R_SHORT_FRAMESTART_BCSTART   (1u << 13)
#define R_SHORT_READY_EDSTART        (1u << 14)
#define R_SHORT_EDEND_DISABLE        (1u << 15)
#define R_SHORT_CCAIDLE_STOP         (1u << 16)
#define R_SHORT_TXREADY_START        (1u << 17)
#define R_SHORT_RXREADY_START        (1u << 18)
#define R_SHORT_PHYEND_DISABLE       (1u << 19)
#define R_SHORT_PHYEND_START         (1u << 20)

#define IEEE802154_PREAMBLE_BYTE  0x00
#define IEEE802154_PREAMBLE_LEN   4
#define IEEE802154_SFD            0x7A   /* Standard IEEE 802.15.4 SFD value
                                          * (matches nrf52840_soc, cc2538_rfcore,
                                          * and the multinode medium's PCAP
                                          * writer).  An earlier "0xA7 reversed"
                                          * value silently broke cross-node
                                          * frame delivery. */

/* RX byte parser phases. */
enum nrf54l_rx_phase {
    NRF54L_RX_WAIT_PREAMBLE = 0,
    NRF54L_RX_WAIT_SFD,
    NRF54L_RX_READ_PHR,
    NRF54L_RX_READ_PAYLOAD
};

/* SUBSCRIBE-slot binding — one per task-index 0..11, lives inside
 * the radio state.  DPPI subscriber callback receives a pointer to
 * one of these and triggers the task at `task_off`. */
typedef struct {
    nrf54l_radio_state_t *radio;
    uint32_t              task_off;
} radio_sub_binding_t;
static radio_sub_binding_t radio_sub_bindings[NRF54L_RADIO_NUM_SUBSCRIBES];
static const uint32_t radio_sub_task_off[NRF54L_RADIO_NUM_SUBSCRIBES] = {
    R_TASKS_TXEN, R_TASKS_RXEN, R_TASKS_START, R_TASKS_STOP,
    R_TASKS_DISABLE, R_TASKS_RSSISTART, R_TASKS_BCSTART, R_TASKS_BCSTOP,
    R_TASKS_EDSTART, R_TASKS_EDSTOP, R_TASKS_CCASTART, R_TASKS_CCASTOP,
};

/* Forward declarations. */
static void nrf54l_radio_trigger_task(nrf54l_radio_state_t *r, uint32_t task_off);
static void nrf54l_radio_apply_shorts(nrf54l_radio_state_t *r, uint32_t after_event_bit);
static void nrf54l_radio_publish_event(nrf54l_radio_state_t *r, int pub_idx);
static void nrf54l_radio_set_state(nrf54l_radio_state_t *r, uint32_t new_state);
static void nrf54l_radio_emit_tx(nrf54l_radio_state_t *r);

/* CCITT-16 CRC matching IEEE 802.15.4 FCS (same as cc2420.c / nrf52840). */
static uint16_t nrf54l_crc_add(uint16_t crc, uint8_t data) {
    uint16_t n = ((crc >> 8) & 0xff) | ((crc << 8) & 0xffff);
    n ^= data;
    n ^= (n & 0xff) >> 4;
    n ^= (n << 12) & 0xffff;
    n ^= (n & 0xff) << 5;
    return n & 0xffff;
}

/* Fire an event: latch the bit, raise IRQs whose INTENSET matches, and
 * publish on the configured DPPI channel.  `pub_idx` is the PUBLISH
 * cluster index (PUB_READY..PUB_CTEPRESENT). */
static void nrf54l_radio_fire_event(nrf54l_radio_state_t *r,
                                     uint32_t *evt, int int_bit, int pub_idx) {
    *evt = 1;
    if (r->intenset00 & (1u << int_bit))
        arm_nvic_set_pending(&r->plat->nvic, r->irq_num_0);
    if (r->intenset10 & (1u << int_bit))
        arm_nvic_set_pending(&r->plat->nvic, r->irq_num_1);
    nrf54l_radio_publish_event(r, pub_idx);
}

static void nrf54l_radio_publish_event(nrf54l_radio_state_t *r, int pub_idx) {
    if (pub_idx < 0 || pub_idx >= NRF54L_RADIO_NUM_PUBLISHES) return;
    uint32_t pub = r->publish[pub_idx];
    if ((pub & 0x80000000u) && r->dppi)
        nrf54l_dppi_publish(r->dppi, (int)(pub & 0x1Fu));
}

static void nrf54l_radio_apply_shorts(nrf54l_radio_state_t *r, uint32_t after_event_bit) {
    if (!(r->shorts & after_event_bit)) return;
    switch (after_event_bit) {
        case R_SHORT_READY_START:
        case R_SHORT_TXREADY_START:
        case R_SHORT_RXREADY_START:
        case R_SHORT_END_START:
        case R_SHORT_PHYEND_START:
            nrf54l_radio_trigger_task(r, R_TASKS_START);    break;
        case R_SHORT_DISABLED_TXEN:
            nrf54l_radio_trigger_task(r, R_TASKS_TXEN);     break;
        case R_SHORT_DISABLED_RXEN:
            nrf54l_radio_trigger_task(r, R_TASKS_RXEN);     break;
        case R_SHORT_PHYEND_DISABLE:
        case R_SHORT_CCABUSY_DISABLE:
        case R_SHORT_EDEND_DISABLE:
            nrf54l_radio_trigger_task(r, R_TASKS_DISABLE);  break;
        case R_SHORT_CCAIDLE_TXEN:
            nrf54l_radio_trigger_task(r, R_TASKS_TXEN);     break;
        case R_SHORT_CCAIDLE_STOP:
            nrf54l_radio_trigger_task(r, R_TASKS_STOP);     break;
        default: break;
    }
}

static void nrf54l_radio_set_state(nrf54l_radio_state_t *r, uint32_t new_state) {
    r->state = new_state;
    switch (new_state) {
        case NRF54L_RADIO_STATE_RXIDLE:
            nrf54l_radio_fire_event(r, &r->evt_ready,   INT_READY,   PUB_READY);
            nrf54l_radio_fire_event(r, &r->evt_rxready, INT_RXREADY, PUB_RXREADY);
            nrf54l_radio_apply_shorts(r, R_SHORT_READY_START);
            nrf54l_radio_apply_shorts(r, R_SHORT_RXREADY_START);
            break;
        case NRF54L_RADIO_STATE_TXIDLE:
            nrf54l_radio_fire_event(r, &r->evt_ready,   INT_READY,   PUB_READY);
            nrf54l_radio_fire_event(r, &r->evt_txready, INT_TXREADY, PUB_TXREADY);
            nrf54l_radio_apply_shorts(r, R_SHORT_READY_START);
            nrf54l_radio_apply_shorts(r, R_SHORT_TXREADY_START);
            break;
        case NRF54L_RADIO_STATE_DISABLED:
            nrf54l_radio_fire_event(r, &r->evt_disabled, INT_DISABLED, PUB_DISABLED);
            nrf54l_radio_apply_shorts(r, R_SHORT_DISABLED_TXEN);
            nrf54l_radio_apply_shorts(r, R_SHORT_DISABLED_RXEN);
            break;
        default: break;
    }
}

static void nrf54l_radio_trigger_task(nrf54l_radio_state_t *r, uint32_t task_off) {
    /* Recursion guard.  Each task can synchronously fire SHORTS or
     * DPPI publishes that trigger more tasks.  Real hardware spaces
     * these out by µs of ramp-up / transmit / disable time; csim
     * fires them all in the same cycle, so without a depth limit
     * we burn into infinite cycles after the first PHYEND. */
    static int depth = 0;
    if (depth >= 8) return;
    depth++;
    if (getenv("NRF54L_RADIO_TASK_TRACE")) {
        static int n = 0;
        if (n++ < 100) {
            const char *name = "?";
            switch (task_off) {
                case R_TASKS_TXEN: name = "TXEN"; break;
                case R_TASKS_RXEN: name = "RXEN"; break;
                case R_TASKS_START: name = "START"; break;
                case R_TASKS_STOP: name = "STOP"; break;
                case R_TASKS_DISABLE: name = "DISABLE"; break;
                case R_TASKS_CCASTART: name = "CCASTART"; break;
            }
            fprintf(stderr, "[radio task %s state=%u cyc=%lld]\n",
                    name, r->state, (long long)r->plat->cpu.cycles);
        }
    }
    switch (task_off) {
        case R_TASKS_TXEN:
            if (r->state != NRF54L_RADIO_STATE_TX &&
                r->state != NRF54L_RADIO_STATE_TXRU)
                nrf54l_radio_set_state(r, NRF54L_RADIO_STATE_TXIDLE);
            break;
        case R_TASKS_RXEN:
            if (r->state != NRF54L_RADIO_STATE_RX &&
                r->state != NRF54L_RADIO_STATE_RXRU)
                nrf54l_radio_set_state(r, NRF54L_RADIO_STATE_RXIDLE);
            break;
        case R_TASKS_START:
            if (r->state == NRF54L_RADIO_STATE_TXIDLE) {
                r->state = NRF54L_RADIO_STATE_TX;
                nrf54l_radio_emit_tx(r);
                nrf54l_radio_fire_event(r, &r->evt_address,    INT_ADDRESS,    PUB_ADDRESS);
                nrf54l_radio_fire_event(r, &r->evt_framestart, INT_FRAMESTART, PUB_FRAMESTART);
                nrf54l_radio_fire_event(r, &r->evt_payload,    INT_PAYLOAD,    PUB_PAYLOAD);
                nrf54l_radio_fire_event(r, &r->evt_end,        INT_END,        PUB_END);
                nrf54l_radio_fire_event(r, &r->evt_phyend,     INT_PHYEND,     PUB_PHYEND);
                r->state = NRF54L_RADIO_STATE_TXIDLE;
                nrf54l_radio_apply_shorts(r, R_SHORT_END_START);
                nrf54l_radio_apply_shorts(r, R_SHORT_PHYEND_START);
                nrf54l_radio_apply_shorts(r, R_SHORT_PHYEND_DISABLE);
            } else if (r->state == NRF54L_RADIO_STATE_RXIDLE) {
                r->state    = NRF54L_RADIO_STATE_RX;
                r->rx_phase = NRF54L_RX_WAIT_PREAMBLE;
            }
            break;
        case R_TASKS_STOP:
            if (r->state == NRF54L_RADIO_STATE_RX) r->state = NRF54L_RADIO_STATE_RXIDLE;
            if (r->state == NRF54L_RADIO_STATE_TX) r->state = NRF54L_RADIO_STATE_TXIDLE;
            break;
        case R_TASKS_DISABLE:
            nrf54l_radio_set_state(r, NRF54L_RADIO_STATE_DISABLED);
            break;
        case R_TASKS_CCASTART:
            /* No interference model — always idle. */
            nrf54l_radio_fire_event(r, &r->evt_ccaidle, INT_CCAIDLE, PUB_CCAIDLE);
            nrf54l_radio_apply_shorts(r, R_SHORT_CCAIDLE_TXEN);
            nrf54l_radio_apply_shorts(r, R_SHORT_CCAIDLE_STOP);
            break;
        case R_TASKS_RSSISTART:
        case R_TASKS_EDSTART:
            nrf54l_radio_fire_event(r, &r->evt_edend, INT_EDEND, PUB_EDEND);
            break;
        default: break;
    }
    depth--;
}

/* DPPI subscriber callback — bound once per SUBSCRIBE slot. */
static void radio_sub_cb(void *user) {
    radio_sub_binding_t *b = (radio_sub_binding_t *)user;
    nrf54l_radio_trigger_task(b->radio, b->task_off);
}

static void nrf54l_radio_emit_tx(nrf54l_radio_state_t *r) {
    if (!r->tx_cb) return;
    arm_cpu_t *cpu = &r->plat->cpu;
    if (r->packetptr < cpu->sram_base || r->packetptr + 128 > cpu->sram_end)
        return;
    uint8_t phr = arm_read8(cpu, r->packetptr);
    if (phr < 2 || phr > 127) return;
    uint8_t payload[125];
    int payload_len = (int)phr - 2;
    for (int i = 0; i < payload_len; i++)
        payload[i] = arm_read8(cpu, r->packetptr + 1 + (uint32_t)i);
    uint16_t crc = 0;
    for (int i = 0; i < payload_len; i++) crc = nrf54l_crc_add(crc, payload[i]);
    void (*cb)(void *, uint8_t) = r->tx_cb;
    void *ud = r->tx_user;
    for (int i = 0; i < IEEE802154_PREAMBLE_LEN; i++)
        cb(ud, IEEE802154_PREAMBLE_BYTE);
    cb(ud, IEEE802154_SFD);
    cb(ud, phr);
    for (int i = 0; i < payload_len; i++) cb(ud, payload[i]);
    cb(ud, (uint8_t)(crc & 0xFF));
    cb(ud, (uint8_t)((crc >> 8) & 0xFF));
}

void nrf54l_radio_set_tx_listener(nrf54l15_soc_t *soc,
                                   nrf54l_radio_tx_listener_t cb, void *user) {
    soc->radio.tx_cb   = cb;
    soc->radio.tx_user = user;
}

void nrf54l_radio_receive_byte(nrf54l15_soc_t *soc, uint8_t byte) {
    nrf54l_radio_state_t *r = &soc->radio;
    if (r->state != NRF54L_RADIO_STATE_RX) return;
    arm_cpu_t *cpu = &r->plat->cpu;
    if (r->packetptr < cpu->sram_base || r->packetptr + 128 > cpu->sram_end)
        return;
    switch (r->rx_phase) {
        case NRF54L_RX_WAIT_PREAMBLE:
            if (byte == IEEE802154_PREAMBLE_BYTE)
                r->rx_phase = NRF54L_RX_WAIT_SFD;
            break;
        case NRF54L_RX_WAIT_SFD:
            if (byte == IEEE802154_SFD) {
                r->rx_phase = NRF54L_RX_READ_PHR;
                nrf54l_radio_fire_event(r, &r->evt_address,    INT_ADDRESS,    PUB_ADDRESS);
                nrf54l_radio_fire_event(r, &r->evt_framestart, INT_FRAMESTART, PUB_FRAMESTART);
            } else if (byte != IEEE802154_PREAMBLE_BYTE) {
                r->rx_phase = NRF54L_RX_WAIT_PREAMBLE;
            }
            break;
        case NRF54L_RX_READ_PHR:
            if (byte < 2 || byte > 127) {
                r->rx_phase = NRF54L_RX_WAIT_PREAMBLE;
                break;
            }
            arm_write8(cpu, r->packetptr, byte);
            r->rx_remaining = byte;
            r->rx_offset    = 1;
            r->rx_phase     = NRF54L_RX_READ_PAYLOAD;
            break;
        case NRF54L_RX_READ_PAYLOAD:
            arm_write8(cpu, r->packetptr + (uint32_t)r->rx_offset, byte);
            r->rx_offset++;
            r->rx_remaining--;
            if (r->rx_remaining == 0) {
                nrf54l_radio_fire_event(r, &r->evt_payload,  INT_PAYLOAD,  PUB_PAYLOAD);
                nrf54l_radio_fire_event(r, &r->evt_end,      INT_END,      PUB_END);
                nrf54l_radio_fire_event(r, &r->evt_phyend,   INT_PHYEND,   PUB_PHYEND);
                nrf54l_radio_fire_event(r, &r->evt_crcok,    INT_CRCOK,    PUB_CRCOK);

                /* Hardware-style auto-ACK — same logic and rationale
                 * as nrf52840's radio model. PACKETPTR layout:
                 * [PHR][FCF0][FCF1][DSN][...]. */
                if (r->tx_cb && r->rx_offset > 4) {
                    uint8_t fcf0       = arm_read8(cpu, r->packetptr + 1);
                    uint8_t dsn        = arm_read8(cpu, r->packetptr + 3);
                    int     frame_type = fcf0 & 0x07;
                    int     ack_req    = (fcf0 >> 5) & 1;
                    if (frame_type == 0x1 && ack_req) {
                        uint8_t ack_fcf0 = 0x02;
                        uint8_t ack_fcf1 = 0x00;
                        uint16_t crc = 0;
                        crc = nrf54l_crc_add(crc, ack_fcf0);
                        crc = nrf54l_crc_add(crc, ack_fcf1);
                        crc = nrf54l_crc_add(crc, dsn);
                        void (*cb)(void *, uint8_t) = r->tx_cb;
                        void *ud = r->tx_user;
                        for (int i = 0; i < IEEE802154_PREAMBLE_LEN; i++)
                            cb(ud, IEEE802154_PREAMBLE_BYTE);
                        cb(ud, IEEE802154_SFD);
                        cb(ud, 5);
                        cb(ud, ack_fcf0);
                        cb(ud, ack_fcf1);
                        cb(ud, dsn);
                        cb(ud, (uint8_t)(crc & 0xFF));
                        cb(ud, (uint8_t)((crc >> 8) & 0xFF));
                    }
                }
                r->state    = NRF54L_RADIO_STATE_RXIDLE;
                r->rx_phase = NRF54L_RX_WAIT_PREAMBLE;
                nrf54l_radio_apply_shorts(r, R_SHORT_END_START);
                nrf54l_radio_apply_shorts(r, R_SHORT_PHYEND_DISABLE);
            }
            break;
    }
}

static int nrf54l_radio_read(void *user_data, uint32_t addr) {
    nrf54l_radio_state_t *r = (nrf54l_radio_state_t *)user_data;
    uint32_t off = addr - NRF54L_RADIO_BASE;
    if (off >= R_SUBSCRIBE_BASE && off < R_SUBSCRIBE_END)
        return (int)r->subscribe[R_SUBSCRIBE_INDEX(off)];
    if (off >= R_PUBLISH_BASE && off < R_PUBLISH_END)
        return (int)r->publish[(off - R_PUBLISH_BASE) / 4];
    switch (off) {
        case R_EVENTS_READY:        return (int)r->evt_ready;
        case R_EVENTS_TXREADY:      return (int)r->evt_txready;
        case R_EVENTS_RXREADY:      return (int)r->evt_rxready;
        case R_EVENTS_ADDRESS:      return (int)r->evt_address;
        case R_EVENTS_FRAMESTART:   return (int)r->evt_framestart;
        case R_EVENTS_PAYLOAD:      return (int)r->evt_payload;
        case R_EVENTS_END:          return (int)r->evt_end;
        case R_EVENTS_PHYEND:       return (int)r->evt_phyend;
        case R_EVENTS_DISABLED:     return (int)r->evt_disabled;
        case R_EVENTS_DEVMATCH:     return (int)r->evt_devmatch;
        case R_EVENTS_DEVMISS:      return (int)r->evt_devmiss;
        case R_EVENTS_CRCOK:        return (int)r->evt_crcok;
        case R_EVENTS_CRCERROR:     return (int)r->evt_crcerror;
        case R_EVENTS_BCMATCH:      return (int)r->evt_bcmatch;
        case R_EVENTS_EDEND:        return (int)r->evt_edend;
        case R_EVENTS_CCAIDLE:      return (int)r->evt_ccaidle;
        case R_EVENTS_CCABUSY:      return (int)r->evt_ccabusy;
        case R_SHORTS:              return (int)r->shorts;
        case R_INTENSET00:
        case R_INTENCLR00:          return (int)r->intenset00;
        case R_INTENSET10:
        case R_INTENCLR10:          return (int)r->intenset10;
        case R_MODE:                return (int)r->mode;
        case R_STATE:               return (int)r->state;
        case R_FREQUENCY:           return (int)r->frequency;
        case R_TXPOWER:             return (int)r->txpower;
        case R_TIMING:              return (int)r->timing;
        case R_PCNF0:               return (int)r->pcnf0;
        case R_PCNF1:               return (int)r->pcnf1;
        case R_CRCCNF:              return (int)r->crccnf;
        case R_CRCPOLY:             return (int)r->crcpoly;
        case R_CRCINIT:             return (int)r->crcinit;
        case R_PACKETPTR:           return (int)r->packetptr;
        case R_BCC:                 return (int)r->bcc;
        case R_CRCSTATUS:           return 1;     /* always CRC OK */
        case R_RXMATCH:             return 0;
        default:                    return 0;
    }
}

static void nrf54l_radio_write(void *user_data, uint32_t addr, uint32_t value) {
    nrf54l_radio_state_t *r = (nrf54l_radio_state_t *)user_data;
    uint32_t off = addr - NRF54L_RADIO_BASE;

    /* Tasks: trigger on write of 1. */
    if (off <= R_TASKS_CCASTOP) {
        if (value == 1) nrf54l_radio_trigger_task(r, off);
        return;
    }

    /* SUBSCRIBE — bind/unbind DPPI channel. */
    if (off >= R_SUBSCRIBE_BASE && off < R_SUBSCRIBE_END) {
        int idx = R_SUBSCRIBE_INDEX(off);
        if (getenv("NRF54L_RADIO_TASK_TRACE") && (value & 0x80000000u)) {
            const char *names[] = {
                "TXEN","RXEN","START","STOP","DISABLE","RSSISTART",
                "BCSTART","BCSTOP","EDSTART","EDSTOP","CCASTART","CCASTOP"
            };
            fprintf(stderr, "[radio SUBSCRIBE_%s = ch%d cyc=%lld]\n",
                    idx < 12 ? names[idx] : "?", value & 0x1F,
                    (long long)r->plat->cpu.cycles);
        }
        r->subscribe[idx] = value;
        /* Unbind previous channel if any. */
        if (r->sub_channel[idx] >= 0) {
            nrf54l_dppi_unsubscribe(r->dppi, r->sub_channel[idx],
                                     radio_sub_cb, &radio_sub_bindings[idx]);
            r->sub_channel[idx] = -1;
        }
        /* Bind new channel if EN set. */
        if (value & 0x80000000u) {
            int ch = (int)(value & 0x1Fu);
            radio_sub_bindings[idx].radio    = r;
            radio_sub_bindings[idx].task_off = radio_sub_task_off[idx];
            nrf54l_dppi_subscribe(r->dppi, ch,
                                   radio_sub_cb, &radio_sub_bindings[idx]);
            r->sub_channel[idx] = ch;
        }
        return;
    }

    /* PUBLISH — just store; we publish on event-fire by looking up. */
    if (off >= R_PUBLISH_BASE && off < R_PUBLISH_END) {
        int idx = (off - R_PUBLISH_BASE) / 4;
        r->publish[idx] = value;
        return;
    }

    /* EVENTS — firmware writes 0 to ack. */
    switch (off) {
        case R_EVENTS_READY:        r->evt_ready      = value & 1; return;
        case R_EVENTS_TXREADY:      r->evt_txready    = value & 1; return;
        case R_EVENTS_RXREADY:      r->evt_rxready    = value & 1; return;
        case R_EVENTS_ADDRESS:      r->evt_address    = value & 1; return;
        case R_EVENTS_FRAMESTART:   r->evt_framestart = value & 1; return;
        case R_EVENTS_PAYLOAD:      r->evt_payload    = value & 1; return;
        case R_EVENTS_END:          r->evt_end        = value & 1; return;
        case R_EVENTS_PHYEND:       r->evt_phyend     = value & 1; return;
        case R_EVENTS_DISABLED:     r->evt_disabled   = value & 1; return;
        case R_EVENTS_DEVMATCH:     r->evt_devmatch   = value & 1; return;
        case R_EVENTS_DEVMISS:      r->evt_devmiss    = value & 1; return;
        case R_EVENTS_CRCOK:        r->evt_crcok      = value & 1; return;
        case R_EVENTS_CRCERROR:     r->evt_crcerror   = value & 1; return;
        case R_EVENTS_BCMATCH:      r->evt_bcmatch    = value & 1; return;
        case R_EVENTS_EDEND:        r->evt_edend      = value & 1; return;
        case R_EVENTS_CCAIDLE:      r->evt_ccaidle    = value & 1; return;
        case R_EVENTS_CCABUSY:      r->evt_ccabusy    = value & 1; return;

        case R_SHORTS:              r->shorts       = value; return;
        case R_INTENSET00:          r->intenset00  |= value; return;
        case R_INTENCLR00:          r->intenset00 &= ~value; return;
        case R_INTENSET10:          r->intenset10  |= value; return;
        case R_INTENCLR10:          r->intenset10 &= ~value; return;
        case R_MODE:                r->mode      = value;    return;
        case R_FREQUENCY:           r->frequency = value;    return;
        case R_TXPOWER:             r->txpower   = value;    return;
        case R_TIMING:              r->timing    = value;    return;
        case R_PCNF0:               r->pcnf0     = value;    return;
        case R_PCNF1:               r->pcnf1     = value;    return;
        case R_CRCCNF:              r->crccnf    = value;    return;
        case R_CRCPOLY:             r->crcpoly   = value;    return;
        case R_CRCINIT:             r->crcinit   = value;    return;
        case R_PACKETPTR:           r->packetptr = value;    return;
        case R_BCC:                 r->bcc       = value;    return;
        default: return;
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

    /* RADIO — 802.15.4 transceiver. */
    soc->radio.plat       = plat;
    soc->radio.dppi       = &soc->dppi;
    soc->radio.irq_num_0  = NRF54L_RADIO_IRQ_0;
    soc->radio.irq_num_1  = NRF54L_RADIO_IRQ_1;
    soc->radio.state      = NRF54L_RADIO_STATE_DISABLED;
    for (int i = 0; i < NRF54L_RADIO_NUM_SUBSCRIBES; i++)
        soc->radio.sub_channel[i] = -1;
    arm_register_io(&plat->cpu,
                    NRF54L_RADIO_BASE, NRF54L_RADIO_SIZE,
                    nrf54l_radio_read, nrf54l_radio_write, &soc->radio);

    /* EGU00/10/20 — software-trigger sources for the DPPI fabric.
     * EGU10 is the one nrf_802154 uses to kick radio ramp-up. IRQs
     * unused by Contiki, so irq_num = -1 (no NVIC pending). */
    nrf54l_egu_state_t *egus[3]   = { &soc->egu00, &soc->egu10, &soc->egu20 };
    uint32_t            bases[3]  = { NRF54L_EGU00_BASE, NRF54L_EGU10_BASE, NRF54L_EGU20_BASE };
    for (int i = 0; i < 3; i++) {
        egus[i]->plat    = plat;
        egus[i]->dppi    = &soc->dppi;
        egus[i]->irq_num = -1;
        for (int n = 0; n < NRF54L_EGU_NUM_CHANNELS; n++)
            egus[i]->sub_channel[n] = -1;
        arm_register_io(&plat->cpu, bases[i], NRF54L_EGU_SIZE,
                        nrf54l_egu_read, nrf54l_egu_write, egus[i]);
    }
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
