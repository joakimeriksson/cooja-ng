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
#include "ieee_802154.h"
#include "nrf_radio_common.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* sim_host shims — identical to nRF52's. Off-SoC chip drivers reach
 * the CPU's event scheduler through this vtable.  The SPI chips on the
 * DK (flash on SPIM00, ENC28J60 on SPIM22) get the `live_host` copy
 * built in soc_init, whose now_ns is cycle-derived. */
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
 * FICR (0x00FFC000) — INFO.DEVICEID is the only field firmware
 * reads on the boot path. linkaddr-arch.c combines it with the
 * Nordic OUI (f4:ce:36) to derive the link-layer EUI-64. The two
 * 32-bit DEVICEID words sit at offsets 0x304 / 0x308 (FICR.INFO
 * starts at 0x300, DEVICEID is at +4 inside INFO). Other FICR
 * fields read 0xFFFFFFFF on real HW (the reset value of unprogrammed
 * factory cells); we return the same so any code that gates on
 * "value != 0xFFFFFFFF" doesn't mistake silence for a programmed
 * value.
 * ============================================================ */
#define NRF54L_FICR_BASE                  0x00FFC000u
#define NRF54L_FICR_SIZE                  0x1000u
#define NRF54L_FICR_INFO_DEVICEID0        0x304
#define NRF54L_FICR_INFO_DEVICEID1        0x308

static int nrf54l_ficr_read(void *user_data, uint32_t addr) {
    nrf54l_ficr_state_t *ficr = (nrf54l_ficr_state_t *)user_data;
    uint32_t off = addr - NRF54L_FICR_BASE;
    switch (off) {
        case NRF54L_FICR_INFO_DEVICEID0: return (int)ficr->deviceid0;
        case NRF54L_FICR_INFO_DEVICEID1: return (int)ficr->deviceid1;
        default:                          return -1; /* 0xFFFFFFFF */
    }
}

static void nrf54l_ficr_write(void *user_data, uint32_t addr, uint32_t value) {
    /* FICR is read-only on real HW; the harness writes via the
     * struct directly, firmware never traps here. */
    (void)user_data; (void)addr; (void)value;
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

/* Receive-side register offsets (NRF_UARTE_Type on this SoC). */
#define NRF54L_UARTE_TASKS_DMA_RX_START   0x028
#define NRF54L_UARTE_TASKS_DMA_RX_STOP    0x02C
#define NRF54L_UARTE_EVENTS_RXDRDY        0x110
#define NRF54L_UARTE_EVENTS_DMA_RX_END    0x14C
#define NRF54L_UARTE_EVENTS_DMA_RX_READY  0x150
#define NRF54L_UARTE_INTEN                0x300
#define NRF54L_UARTE_INTENSET             0x304
#define NRF54L_UARTE_INTENCLR             0x308
#define NRF54L_UARTE_DMA_RX_PTR           0x704
#define NRF54L_UARTE_DMA_RX_MAXCNT        0x708
#define NRF54L_UARTE_DMA_RX_AMOUNT        0x70C
#define NRF54L_UARTE_INTEN_RXDRDY         (1u << 4)
#define NRF54L_UARTE_INTEN_DMARXEND       (1u << 19)
#define NRF54L_UARTE_INTEN_DMARXREADY     (1u << 20)
#define NRF54L_UARTE20_IRQ     198   /* SERIAL20_IRQn */

/* One character time at the console's 115200 baud, 8N1: ten bit periods.
 * The model does not decode the BAUDRATE register — every board here runs the
 * console at 115200 — but the pacing itself matters, which is why this is a
 * real interval and not zero. Delivering an injected line as fast as the
 * firmware can re-arm its single-byte transfer overwrites the buffer under
 * the driver, and the shell reads back a line that matches no command. */
#define NRF54L_UARTE_CHAR_NS   86800LL

static int nrf54l_trace_flag(int *cache, const char *name);
static int trc54_uartrx = -1;   /* NRF54L_UART_RX_TRACE */
static void nrf54l_uarte_rx_pace_cb(void *user, arm_event_t *ev);
static void nrf54l_uarte_rx_schedule(nrf54l_uarte_state_t *u);

/* Arrange for the next queued byte to arrive one character time from now.
 * Delivery happens only from the paced callback: bytes arrive on a wire at
 * the baud rate, and the firmware re-arms its single-byte transfer from the
 * completion handler. Handing it the next byte the instant it re-arms
 * overwrites the buffer under the driver, which then reads the line back as
 * garbage — the whole line lands before the shell has consumed a character. */
static void nrf54l_uarte_rx_schedule(nrf54l_uarte_state_t *u) {
    if (!u->plat || u->rx_pace_scheduled) return;
    if (u->rx_head == u->rx_tail) return;
    arm_cpu_t *cpu = &u->plat->cpu;
    u->rx_pace_event.callback  = nrf54l_uarte_rx_pace_cb;
    u->rx_pace_event.user_data = u;
    u->rx_pace_scheduled = 1;
    arm_schedule_event(cpu, &u->rx_pace_event,
                       cpu->cycles + cpu_ns_to_cycles(NRF54L_UARTE_CHAR_NS,
                                                      cpu->cpu_freq_hz));
}

static void nrf54l_uarte_rx_drain(nrf54l_uarte_state_t *u) {
    if (!u->plat) return;
    arm_cpu_t *cpu = &u->plat->cpu;
    if (u->rx_active && u->rx_head != u->rx_tail && u->rx_amount < u->rx_maxcnt) {
        uint8_t byte = u->rx_fifo[u->rx_tail];
        u->rx_tail = (u->rx_tail + 1) % (int)sizeof(u->rx_fifo);
        arm_write8(cpu, u->rx_ptr + u->rx_amount, byte);
        if (nrf54l_trace_flag(&trc54_uartrx, "NRF54L_UART_RX_TRACE"))
            fprintf(stderr, "[uart-rx] deliver 0x%02x '%c' -> 0x%08x amount=%u/%u\n",
                    byte, (byte >= 32 && byte < 127) ? byte : '.',
                    u->rx_ptr + u->rx_amount, u->rx_amount + 1, u->rx_maxcnt);
        u->rx_amount++;
        u->evt_rxdrdy = 1;
        if (u->inten & NRF54L_UARTE_INTEN_RXDRDY)
            arm_nvic_set_pending(&u->plat->nvic, u->irq_num);
        if (u->rx_amount >= u->rx_maxcnt) {
            u->rx_active = false;
            u->evt_dma_rx_end = 1;
            if (u->inten & NRF54L_UARTE_INTEN_DMARXEND)
                arm_nvic_set_pending(&u->plat->nvic, u->irq_num);
        }
    }
    nrf54l_uarte_rx_schedule(u);
}

static void nrf54l_uarte_rx_pace_cb(void *user, arm_event_t *ev) {
    (void)ev;
    nrf54l_uarte_state_t *u = (nrf54l_uarte_state_t *)user;
    u->rx_pace_scheduled = 0;
    nrf54l_uarte_rx_drain(u);
}

void nrf54l_uarte_feed_rx(struct nrf54l15_soc *soc, const uint8_t *buf, int len) {
    nrf54l_uarte_state_t *u = &((nrf54l15_soc_t *)soc)->uarte20;
    for (int i = 0; i < len; i++) {
        int next = (u->rx_head + 1) % (int)sizeof(u->rx_fifo);
        if (next == u->rx_tail) break;      /* ring full: drop, as hardware would */
        u->rx_fifo[u->rx_head] = buf[i];
        u->rx_head = next;
    }
    nrf54l_uarte_rx_schedule(u);
}

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
        case NRF54L_UARTE_EVENTS_RXDRDY:        return (int)u->evt_rxdrdy;
        case NRF54L_UARTE_EVENTS_DMA_RX_END:    return (int)u->evt_dma_rx_end;
        case NRF54L_UARTE_EVENTS_DMA_RX_READY:  return (int)u->evt_dma_rx_ready;
        case NRF54L_UARTE_DMA_RX_PTR:           return (int)u->rx_ptr;
        case NRF54L_UARTE_DMA_RX_MAXCNT:        return (int)u->rx_maxcnt;
        case NRF54L_UARTE_DMA_RX_AMOUNT:        return (int)u->rx_amount;
        case NRF54L_UARTE_INTEN:
        case NRF54L_UARTE_INTENSET:
        case NRF54L_UARTE_INTENCLR:             return (int)u->inten;
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
             * EVENTS_TXSTOPPED.  Trigger on any bit-0 write; nrfx
             * sometimes writes non-1 values (e.g. flag bits via fp). */
            if ((value & 1) && u->plat) {
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
            if (value & 1) u->evt_txstopped = 1;
            break;
        case NRF54L_UARTE_EVENTS_TXSTOPPED:   u->evt_txstopped  = value & 1; break;
        case NRF54L_UARTE_EVENTS_DMA_TX_END:  u->evt_dma_tx_end = value & 1; break;
        case NRF54L_UARTE_ENABLE:             u->enable         = value;     break;
        case NRF54L_UARTE_DMA_TX_PTR:         u->tx_ptr         = value;     break;
        case NRF54L_UARTE_DMA_TX_MAXCNT:      u->tx_maxcnt      = value;     break;
        case NRF54L_UARTE_DMA_TX_AMOUNT:      u->tx_amount      = value;     break;
        case NRF54L_UARTE_TASKS_DMA_RX_START:
            if (value & 1) {
                u->rx_amount = 0;
                u->rx_active = true;
                nrf54l_uarte_rx_schedule(u);
            }
            break;
        case NRF54L_UARTE_TASKS_DMA_RX_STOP:
            if (value & 1) u->rx_active = false;
            break;
        case NRF54L_UARTE_EVENTS_RXDRDY:        u->evt_rxdrdy     = value & 1; break;
        case NRF54L_UARTE_EVENTS_DMA_RX_END:    u->evt_dma_rx_end = value & 1; break;
        case NRF54L_UARTE_EVENTS_DMA_RX_READY:  u->evt_dma_rx_ready = value & 1; break;
        case NRF54L_UARTE_DMA_RX_PTR:           u->rx_ptr         = value;     break;
        case NRF54L_UARTE_DMA_RX_MAXCNT:        u->rx_maxcnt      = value;     break;
        case NRF54L_UARTE_DMA_RX_AMOUNT:        u->rx_amount      = value;     break;
        case NRF54L_UARTE_INTEN:                u->inten  = value;  break;
        case NRF54L_UARTE_INTENSET:             u->inten |= value;  break;
        case NRF54L_UARTE_INTENCLR:             u->inten &= ~value; break;
        default:
            /* Accept every other register write as a no-op: SHORTS,
             * BAUDRATE, CONFIG, FRAMETIMEOUT, PSEL.*, PUBLISH_*,
             * SUBSCRIBE_*, etc.  None of these have observed read-back
             * semantics that would gate boot progress in this firmware. */
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

/* Live "now" in ns derived from CPU cycles. cpu->sim_time_ns is only
 * synced at arm_step / arm_step_until boundaries, so reads from
 * within an IRQ handler (e.g. nrfx GRTC re-arming the next compare)
 * see the value frozen at the start of the current step — making
 * every fire_ns we compute one-step-of-staleness late, which on the
 * multinode harness manifests as a ~50× tick-rate slowdown.
 * Same root cause as the cc2538 sleeptimer fix. */
static inline int64_t grtc_now_ns(nrf54l_grtc_state_t *grtc) {
    arm_cpu_t *cpu = &grtc->plat->cpu;
    return arm_cycles_to_ns(cpu->cycles, cpu->cpu_freq_hz);
}

static uint64_t grtc_counter_now(nrf54l_grtc_state_t *grtc) {
    if (!grtc->running) return 0;
    int64_t elapsed = grtc_now_ns(grtc) - grtc->start_anchor_ns;
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
    /* CCADD bit 31 selects the reference:
     *   1 (NRFX_GRTC_CC_RELATIVE_SYSCOUNTER) → fire at now + delay
     *   0 (NRFX_GRTC_CC_RELATIVE_COMPARE)    → fire at last_scheduled + delay
     * The COMPARE mode is drift-free: it ignores IRQ-handler latency
     * between the previous fire and this re-arm. Critical for the
     * Contiki etimer tick (clock-arch.c re-arms every 7813 µs in
     * COMPARE mode); using SYSCOUNTER semantics for it stretches the
     * effective tick by the IRQ overhead and slows wall-clock by ~1.7×. */
    int64_t  now_ns      = grtc_now_ns(grtc);
    int64_t  fire_ns;
    if (cc->absolute_mode) {
        /* Absolute mode: CCL/CCH form a 52-bit syscounter target.
         * Convert to ns from the GRTC start anchor and let
         * arm_schedule_event_ns figure out the fire cycle. If the
         * target is already in the past relative to now, the firmware
         * scheduled it past-due; fire on the next event check, which
         * is what real-HW comparator-already-matched semantics
         * produce. */
        uint64_t target = (uint64_t)cc->ccl |
                          ((uint64_t)(cc->cch & 0x000FFFFFu) << 32);
        fire_ns = grtc->start_anchor_ns +
                  (int64_t)(target * (uint64_t)NRF54L_GRTC_TICK_NS);
        if (fire_ns < now_ns) fire_ns = now_ns;
        cc->scheduled_ns = fire_ns;
        arm_schedule_event_ns(&grtc->plat->cpu, (arm_event_t *)cc->event, fire_ns);
        return;
    }
    uint32_t delay_ticks = cc->ccadd & 0x7FFFFFFFu;
    int64_t  delay_ns    = (int64_t)delay_ticks * NRF54L_GRTC_TICK_NS;
    if ((cc->ccadd & 0x80000000u) || cc->scheduled_ns == 0) {
        fire_ns = now_ns + delay_ns;
    } else {
        /* RELATIVE_COMPARE: fire at scheduled_ns + delay regardless of
         * whether that time is already in the past. Real HW behaviour:
         * if the new compare target has already been passed by the
         * SYSCOUNTER, the comparator matches immediately and the IRQ
         * fires "back-to-back" with the previous one. Earlier code
         * clamped past targets forward to `now + delay`, which silently
         * dropped clock-tick events whenever an IRQ handler ran long.
         * That manifested as Contiki's CLOCK_SECOND counter advancing
         * at 56% of sim time and the udp-client etimer never reaching
         * its 10 s SEND_INTERVAL.
         *
         * arm_schedule_event_ns will fire the event on the next event
         * check after the CPU's cycle passes `fire_ns`, so handing it a
         * past timestamp just means "fire ASAP" — which is what real
         * comparator-already-matched semantics produce. */
        fire_ns = cc->scheduled_ns + delay_ns;
    }
    cc->scheduled_ns = fire_ns;
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
    /* FLPR's IRQ group (INTEN0 / GRTC_0): raise the RV32E machine external
     * interrupt + wake it from WFI. Zephyr drives the VPR system tick exactly
     * this way (nrf_grtc_timer on GRTC_0). The firmware's mie/MIE gate it. */
    if ((grtc->inten_flpr & (1u << idx)) && grtc->plat->cpu.coproc_raise_irq)
        grtc->plat->cpu.coproc_raise_irq(grtc->plat->cpu.coproc, 1u << 11); /* MEIP */
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
        /* SYSCOUNTERL/H always return the live 1 MHz counter. nrfx
         * issues an atomic L→H read pair; we don't model the
         * roll-over hazard since both halves come from the same
         * grtc_counter_now() snapshot in 64-bit. The captured_lo/hi
         * scratch slots are populated by the capture-trigger write
         * (sub == 0x8) but never gate reads — earlier code did and
         * pinned SYSCOUNTERL to the boot-time capture, breaking
         * every nrfx_grtc_syscounter_get() after the first one. */
        if (sub == 0x0) {  /* SYSCOUNTERL */
            uint64_t c = grtc_counter_now(grtc);
            return (int)(uint32_t)c;
        }
        if (sub == 0x4) {  /* SYSCOUNTERH — bits[19:0] VALUE, bit 30 BUSY,
                            * bit 31 OVERFLOW. We keep BUSY=0/OVERFLOW=0
                            * and return just the masked VALUE bits.
                            * An earlier draft set bit 29 with a stale
                            * "LOADED" label, but no such bit exists on
                            * nrf54l15 silicon; firmware code that doesn't
                            * mask the read (e.g. the Contiki
                            * clock_arch_get_syscounter loop) would
                            * interpret it as a ~2.3×10^18 counter. */
            uint64_t c = grtc_counter_now(grtc);
            return (int)((uint32_t)(c >> 32) & 0xFFFFFu);
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
            case 0x0:
                grtc->cc[idx].ccl = value;
                grtc->cc[idx].absolute_mode = 1;
                break;
            case 0x4:
                grtc->cc[idx].cch = value;
                grtc->cc[idx].absolute_mode = 1;
                break;
            case 0x8:
                grtc->cc[idx].ccadd = value;
                grtc->cc[idx].absolute_mode = 0;
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
                grtc->start_anchor_ns = grtc_now_ns(grtc);
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
            if (value == 1) grtc->start_anchor_ns = grtc_now_ns(grtc);
            break;
        case NRF54L_GRTC_INTEN2:    grtc->inten  = value;          break;
        case NRF54L_GRTC_INTENSET2: grtc->inten |= value;          break;
        case NRF54L_GRTC_INTENCLR2: grtc->inten &= ~value;         break;
        /* INTEN0 = the FLPR's IRQ group (GRTC_0). Compares enabled here fire to
         * the RV32E core, not the M33 NVIC. */
        case NRF54L_GRTC_INTEN0:    grtc->inten_flpr  = value;     break;
        case NRF54L_GRTC_INTENSET0: grtc->inten_flpr |= value;     break;
        case NRF54L_GRTC_INTENCLR0: grtc->inten_flpr &= ~value;    break;
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
#define NRF54L_EGU10_IRQ   135    /* EGU10_IRQn — SWI dispatch for nrf_802154 */
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

/* SUBSCRIBE binding — the storage lives per-instance in
 * nrf54l_egu_state_t::sub_bindings (see the header); the DPPI callback
 * dispatches the publish back into the right instance's TASKS_TRIGGER[n]. */
static void egu_sub_cb(void *user) {
    nrf54l_egu_sub_binding_t *b = (nrf54l_egu_sub_binding_t *)user;
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
        /* Binding storage lives in this instance — no cross-node sharing. */
        nrf54l_egu_sub_binding_t *binding = &e->sub_bindings[idx];
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

/* INTENSET bit positions per the SVD.  Note bit 13 is reserved
 * (intentional gap), so BCMATCH is at bit 14 — confirmed against
 * nrf54l15_application.svd.  Get this wrong and BCMATCH interrupts
 * never fire (and the bit-13 EDEND would fire spuriously). */
enum {
    INT_READY      = 0,
    INT_TXREADY    = 1,
    INT_RXREADY    = 2,
    INT_ADDRESS    = 3,
    INT_FRAMESTART = 4,
    INT_PAYLOAD    = 5,
    INT_END        = 6,
    INT_PHYEND     = 7,
    INT_DISABLED   = 8,
    INT_DEVMATCH   = 9,
    INT_DEVMISS    = 10,
    INT_CRCOK      = 11,
    INT_CRCERROR   = 12,
    /* bit 13 reserved */
    INT_BCMATCH    = 14,
    INT_EDEND      = 15,
    INT_EDSTOPPED  = 16,
    INT_CCAIDLE    = 17,
    INT_CCABUSY    = 18,
    INT_CCASTOPPED = 19,
    INT_RATEBOOST  = 20,
    INT_MHRMATCH   = 21,
    INT_SYNC       = 26,
    INT_CTEPRESENT = 27,
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

/* IEEE 802.15.4 PHY constants (PREAMBLE_BYTE/LEN/SFD) and the RX
 * phase enum live in include/common/ieee_802154.h — shared with cc2420,
 * cc2538_rfcore, nrf52840.  Local NRF54L_RX_* aliases avoid churn in
 * the existing switch statements. */
#define NRF54L_RX_WAIT_PREAMBLE IEEE802154_RX_WAIT_PREAMBLE
#define NRF54L_RX_WAIT_SFD      IEEE802154_RX_WAIT_SFD
#define NRF54L_RX_READ_PHR      IEEE802154_RX_READ_PHR
#define NRF54L_RX_READ_PAYLOAD  IEEE802154_RX_READ_PAYLOAD

/* SUBSCRIBE-slot binding storage lives per-instance in
 * nrf54l_radio_state_t::sub_bindings (see the header).  The DPPI subscriber
 * callback receives a pointer to one of these and triggers the task at
 * `task_off`. */
static const uint32_t radio_sub_task_off[NRF54L_RADIO_NUM_SUBSCRIBES] = {
    R_TASKS_TXEN, R_TASKS_RXEN, R_TASKS_START, R_TASKS_STOP,
    R_TASKS_DISABLE, R_TASKS_RSSISTART, R_TASKS_BCSTART, R_TASKS_BCSTOP,
    R_TASKS_EDSTART, R_TASKS_EDSTOP, R_TASKS_CCASTART, R_TASKS_CCASTOP,
};

/* Forward declarations. */
static void nrf54l_radio_trigger_task(nrf54l_radio_state_t *r, uint32_t task_off);
static void nrf54l_radio_tx_end_cb(void *user, arm_event_t *ev);
static void nrf54l_radio_rx_disable_timeout_cb(void *user, arm_event_t *ev);
static void nrf54l_radio_disabled_event_fire_cb(void *user, arm_event_t *ev);
static void nrf54l_radio_apply_shorts(nrf54l_radio_state_t *r, uint32_t after_event_bit);
static void nrf54l_radio_publish_event(nrf54l_radio_state_t *r, int pub_idx);
static void nrf54l_radio_set_state(nrf54l_radio_state_t *r, uint32_t new_state);
static void nrf54l_radio_emit_tx(nrf54l_radio_state_t *r);

/* CCITT-16 CRC matching IEEE 802.15.4 FCS — shared with cc2420 /
 * cc2538_rfcore / nrf52840 via ieee_802154.h. */

/* NRF54L_RADIO_TRACE=1 — driver-level trace of radio activity (state
 * transitions, tasks, events, IRQs).  Cached on first call.  Optionally
 * filter by node tag (low 16 bits of cpu pointer) via NRF54L_RADIO_NODE. */
static int nrf54l_radio_trace_enabled = -1;
static uint32_t nrf54l_radio_trace_node = 0;
/* Debug-trace env flags, latched once per process — unlatched getenv() in
 * MMIO/byte-rate paths measured as the dominant simulation cost elsewhere
 * (see arm_cpu.c ARM_WILD_TRAP / nrf52840_soc.c nrf_trace_flag). */
static int nrf54l_trace_flag(int *cache, const char *name) {
    if (__builtin_expect(*cache < 0, 0)) *cache = getenv(name) ? 1 : 0;
    return *cache;
}
static int trc54_rxstall = -1, trc54_rxdrop = -1, trc54_task = -1,
           trc54_timer = -1, trc54_gpio = -1;

static void nrf54l_radio_trace_probe(void) {
    const char *e = getenv("NRF54L_RADIO_TRACE");
    nrf54l_radio_trace_enabled = (e && *e && *e != '0') ? 1 : 0;
    const char *n = getenv("NRF54L_RADIO_NODE");
    if (n) nrf54l_radio_trace_node = (uint32_t)strtoul(n, NULL, 0);
}
static inline int nrf54l_radio_trace_active(nrf54l_radio_state_t *r) {
    if (nrf54l_radio_trace_enabled < 0) nrf54l_radio_trace_probe();
    if (!nrf54l_radio_trace_enabled) return 0;
    if (nrf54l_radio_trace_node) {
        uint32_t tag = (uint32_t)((uintptr_t)&r->plat->cpu & 0xFFFF);
        if (tag != nrf54l_radio_trace_node) return 0;
    }
    return 1;
}
static const char *nrf54l_radio_state_name(uint32_t s) {
    switch (s) {
        case NRF54L_RADIO_STATE_DISABLED:  return "DISABLED";
        case NRF54L_RADIO_STATE_RXRU:      return "RXRU";
        case NRF54L_RADIO_STATE_RXIDLE:    return "RXIDLE";
        case NRF54L_RADIO_STATE_RX:        return "RX";
        case NRF54L_RADIO_STATE_RXDISABLE: return "RXDISABLE";
        case NRF54L_RADIO_STATE_TXRU:      return "TXRU";
        case NRF54L_RADIO_STATE_TXIDLE:    return "TXIDLE";
        case NRF54L_RADIO_STATE_TX:        return "TX";
        case NRF54L_RADIO_STATE_TXDISABLE: return "TXDISABLE";
        default:                            return "?";
    }
}
static const char *nrf54l_radio_event_name(int int_bit) {
    switch (int_bit) {
        case INT_READY:      return "READY";
        case INT_TXREADY:    return "TXREADY";
        case INT_RXREADY:    return "RXREADY";
        case INT_ADDRESS:    return "ADDRESS";
        case INT_FRAMESTART: return "FRAMESTART";
        case INT_PAYLOAD:    return "PAYLOAD";
        case INT_END:        return "END";
        case INT_PHYEND:     return "PHYEND";
        case INT_DISABLED:   return "DISABLED";
        case INT_DEVMATCH:   return "DEVMATCH";
        case INT_DEVMISS:    return "DEVMISS";
        case INT_CRCOK:      return "CRCOK";
        case INT_CRCERROR:   return "CRCERROR";
        case INT_BCMATCH:    return "BCMATCH";
        case INT_EDEND:      return "EDEND";
        case INT_CCAIDLE:    return "CCAIDLE";
        case INT_CCABUSY:    return "CCABUSY";
        default:              return "?";
    }
}
static const char *nrf54l_radio_task_name(uint32_t off) {
    switch (off) {
        case R_TASKS_TXEN:      return "TXEN";
        case R_TASKS_RXEN:      return "RXEN";
        case R_TASKS_START:     return "START";
        case R_TASKS_STOP:      return "STOP";
        case R_TASKS_DISABLE:   return "DISABLE";
        case R_TASKS_RSSISTART: return "RSSISTART";
        case R_TASKS_BCSTART:   return "BCSTART";
        case R_TASKS_BCSTOP:    return "BCSTOP";
        case R_TASKS_CCASTART:  return "CCASTART";
        default:                 return "?";
    }
}

/* Fire an event: latch the bit, raise IRQs whose INTENSET matches, and
 * publish on the configured DPPI channel.  `pub_idx` is the PUBLISH
 * cluster index (PUB_READY..PUB_CTEPRESENT). */
static void nrf54l_radio_fire_event(nrf54l_radio_state_t *r,
                                     uint32_t *evt, int int_bit, int pub_idx) {
    *evt = 1;
    int irq_raised = 0;
    if (r->intenset00 & (1u << int_bit)) {
        arm_nvic_set_pending(&r->plat->nvic, r->irq_num_0);
        irq_raised |= 1;
    }
    if (r->intenset10 & (1u << int_bit)) {
        arm_nvic_set_pending(&r->plat->nvic, r->irq_num_1);
        irq_raised |= 2;
    }
    nrf54l_radio_publish_event(r, pub_idx);
    if (nrf54l_radio_trace_active(r)) {
        uint32_t pub = (pub_idx >= 0 && pub_idx < NRF54L_RADIO_NUM_PUBLISHES)
                       ? r->publish[pub_idx] : 0;
        fprintf(stderr, "[radio cpu=0x%04x cyc=%lld EVENT %-10s irq=%d pub=0x%x state=%s]\n",
                (unsigned)((uintptr_t)&r->plat->cpu & 0xFFFF),
                (long long)r->plat->cpu.cycles,
                nrf54l_radio_event_name(int_bit), irq_raised, pub,
                nrf54l_radio_state_name(r->state));
    }
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
    if (nrf54l_radio_trace_active(r) && new_state != r->state) {
        fprintf(stderr, "[radio cpu=0x%04x cyc=%lld STATE  %-10s -> %s]\n",
                (unsigned)((uintptr_t)&r->plat->cpu & 0xFFFF),
                (long long)r->plat->cpu.cycles,
                nrf54l_radio_state_name(r->state),
                nrf54l_radio_state_name(new_state));
    }
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
            /* Apply DISABLED_TXEN / DISABLED_RXEN shorts synchronously —
             * those are deterministic register-level chains and don't
             * involve IRQs racing against the firmware. */
            nrf54l_radio_apply_shorts(r, R_SHORT_DISABLED_TXEN);
            nrf54l_radio_apply_shorts(r, R_SHORT_DISABLED_RXEN);
            /* Defer EVENTS_DISABLED firing by ~8 µs — wide enough to let
             * the firmware PHYEND ISR write SUBSCRIBE_TXEN=0 and tear
             * down the DISABLED→EGU→TXEN DPPI chain (the nrf_802154
             * driver wires `RADIO.PUBLISH_DISABLED = ch7 → EGU TRIGGER
             * → EGU.PUBLISH_TRIGGERED = ch23 → RADIO.SUBSCRIBE_TXEN`
             * at TX setup so a short turnaround can re-TX without ISR
             * latency) before our model collapses-and-fires it. Also
             * wide enough to clear the earlier wait_until_radio_is_
             * disabled / trx.c:360 race where the DISABLED IRQ dispatches
             * before the firmware enters its polling loop. With the
             * previous ~250 ns defer the DPPI chain fired in a single
             * synchronous burst before the ISR advanced one instruction,
             * so every broadcast TX produced two on-air frames from the
             * same PACKETPTR and corrupted the receiver's parser. */
            if (!r->disabled_event_defer_scheduled) {
                arm_cpu_t *cpu = &r->plat->cpu;
                r->disabled_event_defer.callback  = nrf54l_radio_disabled_event_fire_cb;
                r->disabled_event_defer.user_data = r;
                r->disabled_event_defer_scheduled = 1;
                int64_t fire = cpu->cycles +
                    cpu_ns_to_cycles(8000LL, cpu->cpu_freq_hz);
                arm_schedule_event(cpu, &r->disabled_event_defer, fire);
            }
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
    if (nrf54l_radio_trace_active(r)) {
        fprintf(stderr, "[radio cpu=0x%04x cyc=%lld TASK   %-10s state=%s]\n",
                (unsigned)((uintptr_t)&r->plat->cpu & 0xFFFF),
                (long long)r->plat->cpu.cycles,
                nrf54l_radio_task_name(task_off),
                nrf54l_radio_state_name(r->state));
    }
    switch (task_off) {
        case R_TASKS_TXEN:
            /* DPPI fans TXEN out the same way it does START — multiple
             * subscribers fire in one cycle. Dedup so a second TXEN in
             * the same cycle doesn't clobber a state the firmware just
             * settled (e.g. DISABLED post-PHYEND_DISABLE). */
            if (r->plat->cpu.cycles == r->last_txen_cycle)
                break;
            r->last_txen_cycle = r->plat->cpu.cycles;
            /* Arm BEFORE set_state — set_state fires READY/TXREADY → can
             * synchronously trigger TASKS_START via shorts, which must
             * see tx_armed=1 to actually emit. */
            r->tx_armed = 1;
            if (r->state != NRF54L_RADIO_STATE_TX &&
                r->state != NRF54L_RADIO_STATE_TXRU)
                nrf54l_radio_set_state(r, NRF54L_RADIO_STATE_TXIDLE);
            break;
        case R_TASKS_RXEN:
            if (r->plat->cpu.cycles == r->last_rxen_cycle)
                break;
            r->last_rxen_cycle = r->plat->cpu.cycles;
            if (r->state != NRF54L_RADIO_STATE_RX &&
                r->state != NRF54L_RADIO_STATE_RXRU)
                nrf54l_radio_set_state(r, NRF54L_RADIO_STATE_RXIDLE);
            break;
        case R_TASKS_START:
            if (r->state == NRF54L_RADIO_STATE_TXIDLE && r->tx_armed &&
                r->plat->cpu.cycles != r->last_tx_emit_cycle) {
                arm_cpu_t *cpu = &r->plat->cpu;
                r->tx_armed = 0;
                r->last_tx_emit_cycle = cpu->cycles;
                r->state = NRF54L_RADIO_STATE_TX;
                /* Defer PHYEND by a small fixed delay — just enough for
                 * the driver's TX-setup critical section to exit and
                 * re-enable the RADIO IRQ in NVIC.  Using the full
                 * on-air duration (preamble+SFD+PHR+payload+FCS) * 32 µs
                 * would more closely match real hardware, but the
                 * multinode harness delivers bytes between nodes at
                 * synchronous (tx-start-anchored) timestamps — so a
                 * full-air-time defer leaves Node A's radio still in TX
                 * state when Node B's ACK arrives.  100 µs is well
                 * above the driver's typical critical-section length
                 * and well below the 192 µs ACK turnaround. */
                int64_t air_dur_ns = 100000LL;
                nrf54l_radio_emit_tx(r);
                nrf54l_radio_fire_event(r, &r->evt_address,    INT_ADDRESS,    PUB_ADDRESS);
                nrf54l_radio_fire_event(r, &r->evt_framestart, INT_FRAMESTART, PUB_FRAMESTART);
                /* Defer PAYLOAD/END/PHYEND + state→TXIDLE + END/PHYEND
                 * shorts to actual air-time end.  See the tx_end_event
                 * comment in the header for why this matters. */
                if (r->tx_end_scheduled)
                    arm_cancel_event(cpu, &r->tx_end_event);
                r->tx_end_event.callback  = nrf54l_radio_tx_end_cb;
                r->tx_end_event.user_data = r;
                r->tx_end_scheduled = 1;
                /* Schedule by CYCLES, not _ns: sim_time_ns can lag the live
                 * cycle counter, and arm_schedule_event_ns computes the fire
                 * cycle off sim_time_ns — so `now_ns + air_dur_ns` often
                 * landed in the past and PHYEND fired ~1 cycle after START
                 * instead of ~100 µs later.  That collapsed the whole TX into
                 * one cycle, so the driver's DPPI TXEN/START fan-out (which
                 * fires the same task a few µs apart) started a SECOND full
                 * emit of the same frame — two frames back-to-back on air, so
                 * a per-byte receiver read the second SFD as the PHR and every
                 * frame failed CRC.  Cycle-based scheduling keeps the radio in
                 * TX for the whole air-time window, absorbing the redundant
                 * START (state != TXIDLE) exactly as real HW does. */
                int64_t air_cycles = cpu_ns_to_cycles(air_dur_ns,
                                                      cpu->cpu_freq_hz);
                arm_schedule_event(cpu, &r->tx_end_event,
                                   cpu->cycles + air_cycles);
            } else if (r->state == NRF54L_RADIO_STATE_RXIDLE) {
                r->state    = NRF54L_RADIO_STATE_RX;
                r->rx_phase = NRF54L_RX_WAIT_PREAMBLE;
                r->bcc_last_fired = 0;
            }
            break;
        case R_TASKS_STOP:
            if (r->state == NRF54L_RADIO_STATE_RX) r->state = NRF54L_RADIO_STATE_RXIDLE;
            if (r->state == NRF54L_RADIO_STATE_TX) r->state = NRF54L_RADIO_STATE_TXIDLE;
            break;
        case R_TASKS_DISABLE:
            /* If a frame is mid-flight on RX, defer the actual state
             * change until the parser hits end-of-frame — see the
             * rx_disable_pending comment in the header. Anything else
             * disables immediately. */
            if (r->state == NRF54L_RADIO_STATE_RX &&
                r->rx_phase != NRF54L_RX_WAIT_PREAMBLE &&
                r->rx_phase != NRF54L_RX_WAIT_SFD) {
                r->rx_disable_pending = 1;
                /* Schedule the safety-net timeout — see
                 * rx_disable_timeout_event in the header. 100 µs sits
                 * comfortably inside the driver's ~250 µs rampdown
                 * window and well above the 32 µs IEEE 802.15.4
                 * byte-period, so a normally-progressing parser still
                 * wins the race. */
                if (!r->rx_disable_timeout_scheduled) {
                    /* Schedule the safety-net timeout — see
                     * rx_disable_timeout_event in the header. Fire after
                     * ~5 µs of CPU time. Use the cycle-based scheduler
                     * (not _ns) because sim_time_ns can lag the live
                     * cycle counter — the _ns scheduler computes
                     * fire_cycle off sim_time_ns and either fires
                     * immediately or far too late. 5 µs at 128 MHz =
                     * 640 cycles, well below the driver's ~50 µs
                     * MAX_RAMPDOWN_CYCLES busy-wait window and well
                     * above the IEEE 802.15.4 byte period (32 µs) so a
                     * normally-progressing parser still has time to
                     * complete first via end-of-frame. */
                    arm_cpu_t *cpu = &r->plat->cpu;
                    r->rx_disable_timeout_event.callback  = nrf54l_radio_rx_disable_timeout_cb;
                    r->rx_disable_timeout_event.user_data = r;
                    r->rx_disable_timeout_scheduled = 1;
                    int64_t fire_cycle = cpu->cycles +
                        cpu_ns_to_cycles(5000LL, cpu->cpu_freq_hz);
                    arm_schedule_event(cpu, &r->rx_disable_timeout_event,
                                       fire_cycle);
                }
            } else {
                nrf54l_radio_set_state(r, NRF54L_RADIO_STATE_DISABLED);
            }
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
    nrf54l_radio_sub_binding_t *b = (nrf54l_radio_sub_binding_t *)user;
    nrf54l_radio_trigger_task(b->radio, b->task_off);
}

/* Fired (frame_air_dur) ns after TASKS_START — see the tx_end_event
 * field comment in the header.  Handles the END / PHYEND firing and
 * the TX→TXIDLE→(DISABLED) state dance that used to be inline in
 * R_TASKS_START.  Real hardware fires PHYEND at on-air completion;
 * doing the same here gives the driver's TX setup (still inside its
 * NVIC-disabling critical section when emit_tx ran) time to finish
 * and re-enable the RADIO IRQ before PHYEND triggers it. */
static void nrf54l_radio_tx_end_cb(void *user, arm_event_t *ev) {
    (void)ev;
    nrf54l_radio_state_t *r = (nrf54l_radio_state_t *)user;
    r->tx_end_scheduled = 0;
    /* Clear tx_armed before applying END_START / PHYEND_START shorts —
     * those trigger TASKS_START, and we must not let a stale tx_armed
     * (from some TXEN that fired while the TX was in flight) re-emit
     * the same frame.  Real HW edge-triggers the START off TXREADY,
     * not off END/PHYEND; chained TX needs a fresh TXREADY cycle. */
    r->tx_armed = 0;
    nrf54l_radio_fire_event(r, &r->evt_payload, INT_PAYLOAD, PUB_PAYLOAD);
    nrf54l_radio_fire_event(r, &r->evt_end,     INT_END,     PUB_END);
    nrf54l_radio_fire_event(r, &r->evt_phyend,  INT_PHYEND,  PUB_PHYEND);
    r->state = NRF54L_RADIO_STATE_TXIDLE;
    nrf54l_radio_apply_shorts(r, R_SHORT_END_START);
    nrf54l_radio_apply_shorts(r, R_SHORT_PHYEND_START);
    nrf54l_radio_apply_shorts(r, R_SHORT_PHYEND_DISABLE);
}

/* Emit the on-air byte sequence from PACKETPTR.  Body lives in
 * nrf_radio_common — shared with nrf52840 RADIO. */
static void nrf54l_radio_emit_tx(nrf54l_radio_state_t *r) {
    nrf_radio_emit_ieee802154_frame(&r->plat->cpu, r->packetptr,
                                     r->tx_cb, r->tx_user);
}

/* Fired ~100 µs after a deferred TASKS_DISABLE when the byte parser
 * was mid-frame. If end-of-frame still hasn't completed the disable
 * (peer aborted, no more bytes arriving — see rx_disable_timeout_event
 * in the header), force STATE→DISABLED here so the driver's
 * wait_until_radio_is_disabled() busy-wait at trx.c:328 returns
 * radio_is_disabled=true instead of asserting at trx.c:360. */
static void nrf54l_radio_rx_disable_timeout_cb(void *user, arm_event_t *ev) {
    (void)ev;
    nrf54l_radio_state_t *r = (nrf54l_radio_state_t *)user;
    r->rx_disable_timeout_scheduled = 0;
    if (!r->rx_disable_pending) return;        /* parser already completed */
    r->rx_disable_pending = 0;
    r->rx_phase           = NRF54L_RX_WAIT_PREAMBLE;
    /* Fire PHYEND before snapping to DISABLED. The deferred-disable path
     * is reached only when the parser was past WAIT_SFD when TASKS_DISABLE
     * arrived (see TASKS_DISABLE handler above) — which means BCMATCH
     * already fired and the nrf_802154 driver flipped its
     * `psdu_being_received` flag to true. Without a matching PHYEND/END
     * to drive the driver back into `rxframe_finish_psdu_is_not_being_
     * received()`, that flag stays set forever, and every later
     * `nrf_802154_transmit_raw` returns `BUSY_CHANNEL` from
     * current_operation_terminate -> can_terminate_current_operation
     * (psdu_being_received_now). On Node 3 of the 3-node chain — where
     * we deliver bytes from two neighbours continuously — that translates
     * to "Node 3 can never TX" and breaks RPL convergence past 2 hops. */
    nrf54l_radio_fire_event(r, &r->evt_phyend, INT_PHYEND, PUB_PHYEND);
    nrf54l_radio_set_state(r, NRF54L_RADIO_STATE_DISABLED);
}

/* Frame-stall recovery — see the header-struct comment. M9.5: the
 * deadline lives on the kernel's radio bus as a *sim-time* event ("no
 * RF byte for this mote in SIM_RADIO_RX_STALL_NS"), not on the chip's
 * cpu-cycle queue.  The previous cpu-cycle watchdog needed a 50 ms
 * band-aid because the receiver's cycle clock stands still while a
 * synchronously-emitted frame traverses the parser; global sim time
 * advances with the bytes' air times, so the bus window can be tight.
 *
 * We do NOT fire CRCERROR — real HW only does that on CRC mismatch of a
 * fully-received frame. Firing it on stall would trick the driver into
 * treating an aborted neighbour's TX as a CRC-failed frame addressed to
 * us, which would dirty its statistics. PHYEND alone is enough to make
 * the SHORTS chain take the radio back to DISABLED.
 */
void nrf54l_radio_rx_stall(nrf54l15_soc_t *soc) {
    nrf54l_radio_state_t *r = &soc->radio;
    /* Only fire if we're still mid-frame; the parser may have completed
     * naturally since the bus saw the last byte. */
    if (r->state != NRF54L_RADIO_STATE_RX ||
        r->rx_phase == NRF54L_RX_WAIT_PREAMBLE ||
        r->rx_phase == NRF54L_RX_WAIT_SFD)
        return;
    if (nrf54l_radio_trace_active(r) || nrf54l_trace_flag(&trc54_rxstall, "NRF54L_RX_STALL_TRACE"))
        fprintf(stderr, "[radio cpu=0x%04x cyc=%lld RX_STALL phase=%d remaining=%d]\n",
                (unsigned)((uintptr_t)&r->plat->cpu & 0xFFFF),
                (long long)r->plat->cpu.cycles, r->rx_phase, r->rx_remaining);
    /* Abandon the frame with a terminal CRC failure.  The parser had locked
     * SFD, so ADDRESS/FRAMESTART already fired and the nrf_802154 driver set
     * psdu_being_received; firing only PHYEND (no RX interrupt) leaves that flag
     * set forever, and every later nrf_802154_transmit_raw then returns
     * BUSY_CHANNEL (psdu_being_received_now) so the node can never TX/ACK/forward
     * again — stalling RPL past the first hop.  Emit END+PHYEND+CRCERROR:
     * CRCERROR raises the RX IRQ, the driver runs rxframe_finish() and clears the
     * flag.  Physically accurate too — a receiver that locks SFD then loses signal
     * clocks in noise and fails the FCS. */
    r->rx_phase = NRF54L_RX_WAIT_PREAMBLE;
    r->state    = NRF54L_RADIO_STATE_RXIDLE;
    r->crcstatus = 0;
    nrf54l_radio_fire_event(r, &r->evt_end,      INT_END,      PUB_END);
    nrf54l_radio_fire_event(r, &r->evt_phyend,   INT_PHYEND,   PUB_PHYEND);
    nrf54l_radio_fire_event(r, &r->evt_crcerror, INT_CRCERROR, PUB_CRCERROR);
    nrf54l_radio_apply_shorts(r, R_SHORT_PHYEND_DISABLE);
}

static void nrf54l_radio_disabled_event_fire_cb(void *user, arm_event_t *ev) {
    (void)ev;
    nrf54l_radio_state_t *r = (nrf54l_radio_state_t *)user;
    r->disabled_event_defer_scheduled = 0;
    /* If the radio left DISABLED between schedule and fire, don't fire
     * a stale event. (e.g. firmware triggered RXEN right after the wait
     * succeeded, taking state to RXIDLE/RX before our 32-cycle defer.) */
    if (r->state != NRF54L_RADIO_STATE_DISABLED) return;
    nrf54l_radio_fire_event(r, &r->evt_disabled, INT_DISABLED, PUB_DISABLED);
}

/* WFI fast-forward guard. Returns non-zero when the radio holds state
 * that has to advance instruction-by-instruction:
 *   - Mid-frame RX (parser past WAIT_SFD): the frame's remaining
 *     bytes (or the bus's RX-stall recovery) need normal stepping.
 *   - TX in progress / tx_armed: skipping would let the firmware
 *     observe TX-done before the air time has elapsed, breaking the
 *     channel-busy window in the medium model.
 * Anything else (DISABLED, RXIDLE, RX-WAIT_PREAMBLE) is genuinely idle
 * from the CPU's point of view and the WFI can fast-forward to the
 * next scheduled CPU event. */
int nrf54l_wfi_skip_guard(void *user) {
    nrf54l15_soc_t *soc = (nrf54l15_soc_t *)user;
    nrf54l_radio_state_t *r = &soc->radio;
    if (r->state == NRF54L_RADIO_STATE_TX ||
        r->state == NRF54L_RADIO_STATE_TXRU ||
        r->state == NRF54L_RADIO_STATE_TXDISABLE)
        return 1;
    if (r->tx_armed)
        return 1;
    if (r->state == NRF54L_RADIO_STATE_RX &&
        r->rx_phase != NRF54L_RX_WAIT_PREAMBLE)
        return 1;
    if (r->rx_disable_pending)
        return 1;
    return 0;
}

void nrf54l_radio_set_tx_listener(nrf54l15_soc_t *soc,
                                   nrf54l_radio_tx_listener_t cb, void *user) {
    soc->radio.tx_cb   = cb;
    soc->radio.tx_user = user;
}

void nrf54l_radio_receive_byte(nrf54l15_soc_t *soc, uint8_t byte) {
    nrf54l_radio_state_t *r = &soc->radio;
    if (r->state != NRF54L_RADIO_STATE_RX) {
        if (nrf54l_radio_trace_active(r) || nrf54l_trace_flag(&trc54_rxdrop, "NRF54L_RX_DROP_TRACE"))
            fprintf(stderr, "[radio cpu=0x%04x cyc=%lld RX_DROP state=%s byte=0x%02x]\n",
                    (unsigned)((uintptr_t)&r->plat->cpu & 0xFFFF),
                    (long long)r->plat->cpu.cycles,
                    nrf54l_radio_state_name(r->state), byte);
        return;
    }
    if (nrf54l_radio_trace_active(r) && byte != 0x00 &&
        r->rx_phase == NRF54L_RX_WAIT_PREAMBLE)
        fprintf(stderr, "[radio cpu=0x%04x cyc=%lld RX_BYTE state=%s phase=%d byte=0x%02x]\n",
                (unsigned)((uintptr_t)&r->plat->cpu & 0xFFFF),
                (long long)r->plat->cpu.cycles,
                nrf54l_radio_state_name(r->state), r->rx_phase, byte);
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
            r->rx_crc       = 0;
            r->rx_phase     = NRF54L_RX_READ_PAYLOAD;
            /* nrf_802154 typically programs BCC=8 (after FRAMESTART) so it
             * gets a BCMATCH the moment PHR is in the buffer. The check
             * lives both here and in READ_PAYLOAD so the milestone fires
             * at the exact rx_offset the driver expects (1 = PHR boundary,
             * higher = subsequent header milestones the IRQ re-arms for).
             * `bcc_last_fired` suppresses re-fires for the same BCC value
             * while leaving `r->bcc` intact for firmware read-back. */
            if (r->bcc != 0 && r->bcc != r->bcc_last_fired &&
                (uint32_t)(r->rx_offset * 8) >= r->bcc) {
                r->bcc_last_fired = r->bcc;
                nrf54l_radio_fire_event(r, &r->evt_bcmatch, INT_BCMATCH, PUB_BCMATCH);
            }
            break;
        case NRF54L_RX_READ_PAYLOAD:
            arm_write8(cpu, r->packetptr + (uint32_t)r->rx_offset, byte);
            if (r->rx_remaining > 2)
                r->rx_crc = ieee802154_crc_add_bitrev(r->rx_crc, byte);
            else
                r->rx_fcs[2 - r->rx_remaining] = byte;
            r->rx_offset++;
            r->rx_remaining--;
            /* BCMATCH fires when BCC bits have been received.  BCC is in
             * bits and the Nordic 802154 driver programs successive
             * milestones (e.g. PHR + FCF + DST + ...) to inspect headers
             * as bytes arrive. Use `bcc_last_fired` instead of clearing
             * `r->bcc` so the driver can still read it via the BCC
             * register in its IRQ handler. */
            if (r->bcc != 0 && r->bcc != r->bcc_last_fired &&
                (uint32_t)(r->rx_offset * 8) >= r->bcc) {
                r->bcc_last_fired = r->bcc;
                nrf54l_radio_fire_event(r, &r->evt_bcmatch, INT_BCMATCH, PUB_BCMATCH);
            }
            if (r->rx_remaining == 0) {
                /* Frame complete — verify the FCS over the delivered
                 * bytes so collision-garbled assemblies fail CRC exactly
                 * as on silicon (see nrf52840_soc.c). */
                int crc_ok =
                    r->rx_fcs[0] == ieee802154_bitrev((uint8_t)((r->rx_crc >> 8) & 0xFF)) &&
                    r->rx_fcs[1] == ieee802154_bitrev((uint8_t)(r->rx_crc & 0xFF));
                r->crcstatus = crc_ok ? 1 : 0;
                nrf54l_radio_fire_event(r, &r->evt_payload,  INT_PAYLOAD,  PUB_PAYLOAD);
                nrf54l_radio_fire_event(r, &r->evt_end,      INT_END,      PUB_END);
                nrf54l_radio_fire_event(r, &r->evt_phyend,   INT_PHYEND,   PUB_PHYEND);
                if (crc_ok)
                    nrf54l_radio_fire_event(r, &r->evt_crcok,    INT_CRCOK,    PUB_CRCOK);
                else
                    nrf54l_radio_fire_event(r, &r->evt_crcerror, INT_CRCERROR, PUB_CRCERROR);

                /* No hardware-style auto-ACK: the Nordic 802.15.4 driver
                 * schedules its own ACK via TIMER+PPI in response to
                 * CRCOK with AR bit set, and now that emit_tx defers
                 * PHYEND to actual air-time end the driver has time to
                 * exit its critical section and arm the ACK on time.
                 * Emitting one from the chip too produces a duplicate
                 * ACK frame at the sender. */
                r->state    = NRF54L_RADIO_STATE_RXIDLE;
                r->rx_phase = NRF54L_RX_WAIT_PREAMBLE;
                nrf54l_radio_apply_shorts(r, R_SHORT_END_START);
                nrf54l_radio_apply_shorts(r, R_SHORT_PHYEND_DISABLE);
                if (r->rx_disable_pending) {
                    r->rx_disable_pending = 0;
                    if (r->rx_disable_timeout_scheduled) {
                        arm_cancel_event(&r->plat->cpu, &r->rx_disable_timeout_event);
                        r->rx_disable_timeout_scheduled = 0;
                    }
                    nrf54l_radio_set_state(r, NRF54L_RADIO_STATE_DISABLED);
                }
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
        case R_CRCSTATUS:           return (int)r->crcstatus;
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
        if (nrf54l_trace_flag(&trc54_task, "NRF54L_RADIO_TASK_TRACE") && (value & 0x80000000u)) {
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
                                     radio_sub_cb, &r->sub_bindings[idx]);
            r->sub_channel[idx] = -1;
        }
        /* Bind new channel if EN set. */
        if (value & 0x80000000u) {
            int ch = (int)(value & 0x1Fu);
            r->sub_bindings[idx].radio    = r;
            r->sub_bindings[idx].task_off = radio_sub_task_off[idx];
            nrf54l_dppi_subscribe(r->dppi, ch,
                                   radio_sub_cb, &r->sub_bindings[idx]);
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
        case R_INTENSET00:
            r->intenset00 |= value;
            if (nrf54l_radio_trace_active(r))
                fprintf(stderr, "[radio cpu=0x%04x cyc=%lld INTSET00 +0x%x => 0x%x state=%s]\n",
                        (unsigned)((uintptr_t)&r->plat->cpu & 0xFFFF),
                        (long long)r->plat->cpu.cycles, value, r->intenset00,
                        nrf54l_radio_state_name(r->state));
            return;
        case R_INTENCLR00:
            r->intenset00 &= ~value;
            if (nrf54l_radio_trace_active(r))
                fprintf(stderr, "[radio cpu=0x%04x cyc=%lld INTCLR00 -0x%x => 0x%x state=%s]\n",
                        (unsigned)((uintptr_t)&r->plat->cpu & 0xFFFF),
                        (long long)r->plat->cpu.cycles, value, r->intenset00,
                        nrf54l_radio_state_name(r->state));
            return;
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
        case R_BCC:           r->bcc       = value;    return;
        default: return;
    }
}
/* ============================================================
 * TIMER (TIMER00/10/20/21/22/23/24)
 *
 * Minimal model targeting the nrf_802154 lptimer backend, which uses
 * TIMER20 at 1 MHz in Timer mode and reads CC[0] via TASKS_CAPTURE[0].
 * We back the counter with sim_time_ns; compare matches schedule a CPU
 * event that fires EVENTS_COMPARE[n], optionally publishing on DPPI.
 *
 * Register layout (from SVD, derived from GLOBAL_TIMER00_NS):
 *   0x000 TASKS_START   0x004 STOP   0x008 COUNT   0x00C CLEAR
 *   0x040+4n  TASKS_CAPTURE[n]
 *   0x080+4n  SUBSCRIBE_<START/STOP/COUNT/CLEAR>
 *   0x0C0+4n  SUBSCRIBE_CAPTURE[n]
 *   0x140+4n  EVENTS_COMPARE[n]
 *   0x1C0+4n  PUBLISH_COMPARE[n]
 *   0x200     SHORTS
 *   0x300/304/308  INTEN/INTENSET/INTENCLR
 *   0x504 MODE   0x508 BITMODE   0x510 PRESCALER
 *   0x540+4n  CC[n]
 * ============================================================ */
#define NRF54L_TIMER10_BASE  0x50085000u
#define NRF54L_TIMER20_BASE  0x500CA000u
#define NRF54L_TIMER_SIZE    0x1000u

/* IRQ numbers from SVD interrupts blocks (nrf54l15_application.svd). */
#define NRF54L_TIMER10_IRQ   133
#define NRF54L_TIMER20_IRQ   164

/* Base TIMER clock on nrf54l15 is 16 MHz; PRESCALER divides by 2^PRESCALER. */
#define NRF54L_TIMER_BASE_HZ  16000000u

static inline uint64_t nrf54l_timer_tick_ns(nrf54l_timer_state_t *t) {
    uint32_t pres = t->prescaler & 0xF;
    /* tick_ns = 1e9 / (16e6 / 2^pres) = (1<<pres) * 1000 / 16 */
    return ((uint64_t)(1u << pres) * 1000u) / 16u;
}

static inline uint32_t nrf54l_timer_mask(nrf54l_timer_state_t *t) {
    switch (t->bitmode & 3) {
        case 0: return 0x0000FFFFu;  /* 16-bit */
        case 1: return 0x000000FFu;  /* 8-bit  */
        case 2: return 0x00FFFFFFu;  /* 24-bit */
        default: return 0xFFFFFFFFu; /* 32-bit */
    }
}

/* Live "now" in ns from CPU cycles — same rationale as GRTC: a busy-wait
 * inside the firmware (e.g. nrf_802154_time_get + wait_for_flag) calls
 * this from within an arm_step batch; reading cpu.sim_time_ns would see
 * the value frozen at the start of the step, so the deadline never
 * advances and the busy-wait spins until the wall-clock timeout. */
static inline int64_t nrf54l_timer_now_ns(nrf54l_timer_state_t *t) {
    arm_cpu_t *cpu = &t->plat->cpu;
    return arm_cycles_to_ns(cpu->cycles, cpu->cpu_freq_hz);
}

static uint32_t nrf54l_timer_counter_now(nrf54l_timer_state_t *t) {
    if (!t->running) return t->snapshot & nrf54l_timer_mask(t);
    int64_t now = nrf54l_timer_now_ns(t);
    uint64_t tn = nrf54l_timer_tick_ns(t);
    if (tn == 0) return 0;
    uint64_t ticks = (uint64_t)(now - t->t0_ns) / tn + t->snapshot;
    return (uint32_t)(ticks & nrf54l_timer_mask(t));
}

static void nrf54l_timer_compare_fired(void *user, cpu_event_t *ev) {
    (void)ev;
    struct { nrf54l_timer_state_t *t; int n; } *ctx = user;
    nrf54l_timer_state_t *t = ctx->t;
    int n = ctx->n;
    t->events_compare[n] = 1;
    int irq_fired = (t->inten & (1u << (16 + n))) && t->irq_num >= 0;
    if (irq_fired)
        arm_nvic_set_pending(&t->plat->nvic, t->irq_num);
    uint32_t pub = t->publish_compare[n];
    int published = ((pub & 0x80000000u) && t->dppi);
    if (published)
        nrf54l_dppi_publish(t->dppi, (int)(pub & 0x1Fu));
    if (nrf54l_trace_flag(&trc54_timer, "NRF54L_TIMER_TRACE")) {
        fprintf(stderr, "[timer cpu=0x%04x cyc=%lld base=0x%x CC[%d]=%u irq=%d pub=0x%x]\n",
                (unsigned)((uintptr_t)&t->plat->cpu & 0xFFFF),
                (long long)t->plat->cpu.cycles,
                t->base_addr, n, t->cc[n], irq_fired, pub);
    }
    /* SHORTS: COMPARE_CLEAR (bit n), COMPARE_STOP (bit 8+n). */
    if (t->shorts & (1u << n)) {
        t->snapshot = 0;
        t->t0_ns = nrf54l_timer_now_ns(t);
    }
    if (t->shorts & (1u << (8 + n))) {
        if (t->running) t->snapshot = nrf54l_timer_counter_now(t);
        t->running = false;
    }
}

/* Per-CC firing context lives in the event's user pointer — storage is
 * per-instance in nrf54l_timer_state_t::cc_ctx (see the header), so two
 * nRF54L15 nodes' timers never share it. */

static void nrf54l_timer_schedule_cc(nrf54l_timer_state_t *t, int n) {
    arm_cancel_event(&t->plat->cpu, &t->ev_compare[n]);
    if (!t->running) return;
    /* Skip CCs that the firmware hasn't programmed (CC=0 with no INTEN
     * and no PUBLISH wired up).  Spurious fires waste cycles and confuse
     * the driver. */
    bool inten   = (t->inten & (1u << (16 + n))) != 0;
    bool publish = (t->publish_compare[n] & 0x80000000u) != 0;
    bool shorts  = (t->shorts & ((1u << n) | (1u << (8 + n)))) != 0;
    if (!inten && !publish && !shorts) return;
    uint32_t now = nrf54l_timer_counter_now(t);
    uint32_t target = t->cc[n] & nrf54l_timer_mask(t);
    uint32_t delta = (target - now) & nrf54l_timer_mask(t);
    if (delta == 0) delta = nrf54l_timer_mask(t) + 1u;  /* full wrap */
    uint64_t tn = nrf54l_timer_tick_ns(t);
    int64_t fire_ns = nrf54l_timer_now_ns(t) + (int64_t)(delta * tn);
    arm_schedule_event_ns(&t->plat->cpu, &t->ev_compare[n], fire_ns);
}

static void nrf54l_timer_capture(nrf54l_timer_state_t *t, int n) {
    if (n < 0 || n >= NRF54L_TIMER_NUM_CC) return;
    t->cc[n] = nrf54l_timer_counter_now(t);
}

static void nrf54l_timer_task(nrf54l_timer_state_t *t, uint32_t off) {
    switch (off) {
        case 0x000: /* START */
            if (!t->running) {
                t->t0_ns   = nrf54l_timer_now_ns(t);
                t->running = true;
                for (int n = 0; n < NRF54L_TIMER_NUM_CC; n++)
                    nrf54l_timer_schedule_cc(t, n);
            }
            break;
        case 0x004: /* STOP */
            if (t->running) {
                t->snapshot = nrf54l_timer_counter_now(t);
                t->running  = false;
            }
            for (int n = 0; n < NRF54L_TIMER_NUM_CC; n++)
                arm_cancel_event(&t->plat->cpu, &t->ev_compare[n]);
            break;
        case 0x008: /* COUNT */
            if (t->mode == 1 /* Counter */ || t->mode == 2 /* LowPowerCounter */) {
                t->counter = (t->counter + 1) & nrf54l_timer_mask(t);
                for (int n = 0; n < NRF54L_TIMER_NUM_CC; n++) {
                    if (t->counter == (t->cc[n] & nrf54l_timer_mask(t)))
                        nrf54l_timer_compare_fired((void *)&t->cc_ctx[n], NULL);
                }
            }
            break;
        case 0x00C: /* CLEAR */
            t->snapshot = 0;
            t->t0_ns    = nrf54l_timer_now_ns(t);
            t->counter  = 0;
            for (int n = 0; n < NRF54L_TIMER_NUM_CC; n++)
                if (t->running) nrf54l_timer_schedule_cc(t, n);
            break;
        default:
            if (off >= 0x040 && off < 0x040 + 4 * NRF54L_TIMER_NUM_CC)
                nrf54l_timer_capture(t, (off - 0x040) / 4);
            break;
    }
}

static void timer_capture_sub_cb(void *user) {
    nrf54l_timer_cc_ctx_t *ctx = (nrf54l_timer_cc_ctx_t *)user;
    nrf54l_timer_capture(ctx->t, ctx->n);
}
static void timer_start_sub_cb(void *user) {
    nrf54l_timer_task((nrf54l_timer_state_t *)user, 0x000);
}
static void timer_stop_sub_cb(void *user) {
    nrf54l_timer_task((nrf54l_timer_state_t *)user, 0x004);
}
static void timer_clear_sub_cb(void *user) {
    nrf54l_timer_task((nrf54l_timer_state_t *)user, 0x00C);
}
static void timer_count_sub_cb(void *user) {
    nrf54l_timer_task((nrf54l_timer_state_t *)user, 0x008);
}

static int nrf54l_timer_read(void *user_data, uint32_t addr) {
    nrf54l_timer_state_t *t = (nrf54l_timer_state_t *)user_data;
    uint32_t off = addr & 0xFFFu;
    if (off >= 0x140 && off < 0x140 + 4 * NRF54L_TIMER_NUM_CC)
        return (int)t->events_compare[(off - 0x140) / 4];
    if (off >= 0x1C0 && off < 0x1C0 + 4 * NRF54L_TIMER_NUM_CC)
        return (int)t->publish_compare[(off - 0x1C0) / 4];
    if (off >= 0x540 && off < 0x540 + 4 * NRF54L_TIMER_NUM_CC)
        return (int)t->cc[(off - 0x540) / 4];
    if (off >= 0x0C0 && off < 0x0C0 + 4 * NRF54L_TIMER_NUM_CC)
        return (int)t->subscribe_capture[(off - 0x0C0) / 4];
    switch (off) {
        case 0x080: return (int)t->subscribe_start;
        case 0x084: return (int)t->subscribe_stop;
        case 0x088: return (int)t->subscribe_count;
        case 0x08C: return (int)t->subscribe_clear;
        case 0x200: return (int)t->shorts;
        case 0x300:
        case 0x304:
        case 0x308: return (int)t->inten;
        case 0x504: return (int)t->mode;
        case 0x508: return (int)t->bitmode;
        case 0x510: return (int)t->prescaler;
        default:    return 0;
    }
}

static void nrf54l_timer_write(void *user_data, uint32_t addr, uint32_t value) {
    nrf54l_timer_state_t *t = (nrf54l_timer_state_t *)user_data;
    uint32_t off = addr & 0xFFFu;
    if (nrf54l_trace_flag(&trc54_timer, "NRF54L_TIMER_TRACE")) {
        const char *kind = "?";
        if (off < 0x80) kind = "TASK";
        else if (off >= 0x140 && off < 0x180) kind = "EVENTS_COMPARE";
        else if (off >= 0x540 && off < 0x560) kind = "CC";
        else if (off == 0x510) kind = "PRESCALER";
        else if (off == 0x508) kind = "BITMODE";
        else if (off == 0x504) kind = "MODE";
        else if (off == 0x200) kind = "SHORTS";
        else if (off >= 0x300 && off <= 0x308) kind = "INTEN*";
        else kind = "other";
        fprintf(stderr, "[timerW cpu=0x%04x cyc=%lld base=0x%x off=0x%x val=0x%x (%s)]\n",
                (unsigned)((uintptr_t)&t->plat->cpu & 0xFFFF),
                (long long)t->plat->cpu.cycles,
                t->base_addr, off, value, kind);
    }
    /* Tasks region. */
    if (off < 0x080) {
        if (value == 1) nrf54l_timer_task(t, off);
        return;
    }
    /* SUBSCRIBE for START/STOP/COUNT/CLEAR. */
    if (off >= 0x080 && off <= 0x08C) {
        nrf54l_dppi_sub_cb cb = NULL;
        switch (off) {
            case 0x080: t->subscribe_start = value; cb = timer_start_sub_cb; break;
            case 0x084: t->subscribe_stop  = value; cb = timer_stop_sub_cb;  break;
            case 0x088: t->subscribe_count = value; cb = timer_count_sub_cb; break;
            case 0x08C: t->subscribe_clear = value; cb = timer_clear_sub_cb; break;
        }
        if ((value & 0x80000000u) && t->dppi)
            nrf54l_dppi_subscribe(t->dppi, (int)(value & 0x1Fu), cb, t);
        return;
    }
    if (off >= 0x0C0 && off < 0x0C0 + 4 * NRF54L_TIMER_NUM_CC) {
        int n = (off - 0x0C0) / 4;
        if (t->sub_capture_ch[n] >= 0)
            nrf54l_dppi_unsubscribe(t->dppi, t->sub_capture_ch[n],
                                     timer_capture_sub_cb, &t->cc_ctx[n]);
        t->subscribe_capture[n] = value;
        t->sub_capture_ch[n] = -1;
        if ((value & 0x80000000u) && t->dppi) {
            int ch = (int)(value & 0x1Fu);
            t->sub_capture_ch[n] = ch;
            nrf54l_dppi_subscribe(t->dppi, ch, timer_capture_sub_cb,
                                   &t->cc_ctx[n]);
        }
        return;
    }
    if (off >= 0x140 && off < 0x140 + 4 * NRF54L_TIMER_NUM_CC) {
        t->events_compare[(off - 0x140) / 4] = value & 1;
        return;
    }
    if (off >= 0x1C0 && off < 0x1C0 + 4 * NRF54L_TIMER_NUM_CC) {
        int n = (off - 0x1C0) / 4;
        t->publish_compare[n] = value;
        if (t->running) nrf54l_timer_schedule_cc(t, n);
        return;
    }
    if (off >= 0x540 && off < 0x540 + 4 * NRF54L_TIMER_NUM_CC) {
        int n = (off - 0x540) / 4;
        t->cc[n] = value;
        if (t->running) nrf54l_timer_schedule_cc(t, n);
        return;
    }
    switch (off) {
        case 0x200: t->shorts = value;
                    if (t->running)
                        for (int n=0; n<NRF54L_TIMER_NUM_CC; n++) nrf54l_timer_schedule_cc(t, n);
                    return;
        case 0x300: t->inten  = value;
                    if (t->running)
                        for (int n=0; n<NRF54L_TIMER_NUM_CC; n++) nrf54l_timer_schedule_cc(t, n);
                    return;
        case 0x304: t->inten |= value;
                    if (t->running)
                        for (int n=0; n<NRF54L_TIMER_NUM_CC; n++) nrf54l_timer_schedule_cc(t, n);
                    return;
        case 0x308: t->inten &= ~value; return;
        case 0x504: t->mode   = value; return;
        case 0x508:
            t->bitmode = value;
            return;
        case 0x510:
            t->prescaler = value & 0xF;
            return;
        default: return;
    }
}

static void nrf54l_timer_setup(nrf54l_timer_state_t *t, arm_platform_t *plat,
                                nrf54l_dppi_state_t *dppi, uint32_t base,
                                int irq_num) {
    t->plat      = plat;
    t->dppi      = dppi;
    t->irq_num   = irq_num;
    t->base_addr = base;
    t->bitmode   = 0; /* 16-bit default per SVD */
    for (int n = 0; n < NRF54L_TIMER_NUM_CC; n++) {
        t->sub_capture_ch[n] = -1;
        t->cc_ctx[n].t = t;
        t->cc_ctx[n].n = n;
        t->ev_compare[n].callback  = nrf54l_timer_compare_fired;
        t->ev_compare[n].user_data = &t->cc_ctx[n];
    }
    arm_register_io(&plat->cpu, base, NRF54L_TIMER_SIZE,
                    nrf54l_timer_read, nrf54l_timer_write, t);
}



/* ============================================================
 * VPR00 — FLPR (RV32E) launch control + SPU00 permission gate
 *
 * The M33 releases the FLPR from reset (Zephyr nordic_vpr_launcher
 * sequence). We latch INITPC/CPURUN and the SPU SECATTR bit; on the
 * CPURUN 0->1 edge, with SECATTR set, the RV32E core is started over
 * the shared SRAM. See docs/design/riscv-vpr-plan.md.
 * ============================================================ */
#define NRF54L_VPR00_BASE     0x5004C000u
#define NRF54L_VPR00_SIZE     0x1000u
#define NRF54L_VPR_CPURUN     0x800u   /* offset within the VPR page */
#define NRF54L_VPR_INITPC     0x808u

#define NRF54L_SPU00_BASE     0x50040000u
#define NRF54L_SPU00_SIZE     0x1000u
#define NRF54L_SPU_PERIPH12   0x530u   /* PERIPH[12].PERM = 0x500 + 12*4 */
#define NRF54L_SPU_SECATTR    (1u << 4)

/* Implemented in src/riscv/nrf54l_vpr.c: instantiate + co-step the RV32E
 * FLPR core over the shared bus. */
void nrf54l_vpr_launch(nrf54l_vpr_state_t *vpr);

static int nrf54l_vpr_read(void *user, uint32_t addr) {
    nrf54l_vpr_state_t *vpr = user;
    switch (addr - NRF54L_VPR00_BASE) {
        case NRF54L_VPR_CPURUN: return (int)vpr->cpurun;
        case NRF54L_VPR_INITPC: return (int)vpr->initpc;
        default:                return 0;
    }
}

static void nrf54l_vpr_write(void *user, uint32_t addr, uint32_t value) {
    nrf54l_vpr_state_t *vpr = user;
    switch (addr - NRF54L_VPR00_BASE) {
        case NRF54L_VPR_INITPC:
            vpr->initpc = value;
            return;
        case NRF54L_VPR_CPURUN:
            /* Rising edge releases the FLPR. The real VPR only fetches when
             * its SPU SECATTR matches the (Secure) access — mirror that gate
             * so a missing SECATTR write surfaces instead of silently running. */
            if ((value & 1u) && !(vpr->cpurun & 1u)) {
                vpr->cpurun = value;
                if (vpr->spu_periph12 & NRF54L_SPU_SECATTR)
                    nrf54l_vpr_launch(vpr);
                else
                    fprintf(stderr, "[VPR] CPURUN=1 but SPU SECATTR clear — "
                                    "FLPR will not fetch (as on HW)\n");
            } else {
                vpr->cpurun = value;
            }
            return;
        default:
            return;
    }
}

static int nrf54l_spu_read(void *user, uint32_t addr) {
    nrf54l_vpr_state_t *vpr = user;
    if ((addr - NRF54L_SPU00_BASE) == NRF54L_SPU_PERIPH12)
        return (int)vpr->spu_periph12;
    return 0;
}

static void nrf54l_spu_write(void *user, uint32_t addr, uint32_t value) {
    nrf54l_vpr_state_t *vpr = user;
    if ((addr - NRF54L_SPU00_BASE) == NRF54L_SPU_PERIPH12)
        vpr->spu_periph12 = value;
}

/* ============================================================
 * GPIO P0/P1/P2 — minimal OUT/DIR model (drives + observes LEDs)
 * Register map (NRF_GPIO_Type): OUT 0x00, OUTSET 0x04, OUTCLR 0x08,
 * IN 0x0c, DIR 0x10, DIRSET 0x14, DIRCLR 0x18. Ports: P0 0x5010A000,
 * P1 0x500D8200 (LED1=P1.10), P2 0x50050400 (LED0=P2.9).
 * ============================================================ */
#define NRF54L_P0_BASE     0x5010A000u
#define NRF54L_P1_BASE     0x500D8200u
#define NRF54L_P2_BASE     0x50050400u
#define NRF54L_GPIO_SIZE   0x100u

static inline void nrf54l_gpio_changed(nrf54l_gpio_state_t *g) {
    if (g->change_cb) g->change_cb(g->change_user, g);
}

static void nrf54l_gpio_set_out(nrf54l_gpio_state_t *g, uint32_t newout) {
    uint32_t changed = (g->out ^ newout) & g->dir;
    if (changed) {
        for (uint32_t m = changed; m; m &= m - 1) g->out_toggles++;
        if (nrf54l_trace_flag(&trc54_gpio, "CSIM_GPIO_TRACE")) {
            int64_t t = g->plat ? g->plat->cpu.sim_time_ns : 0;
            fprintf(stderr, "[GPIO] %9.3f P%d.OUT 0x%08x -> 0x%08x (changed 0x%08x)\n",
                    t / 1e9, g->port, g->out, newout, changed);
        }
    }
    bool any = g->out != newout;
    g->out = newout;
    if (any) nrf54l_gpio_changed(g);
}

/* DIR and PIN_CNF[n].DIR are two views of the same bit (NRF_GPIO_Type:
 * DIR @ 0x10, PIN_CNF[32] @ 0x80, DIR = bit 0).  nrf_gpio_cfg_output —
 * what Contiki's gpio_hal_arch_pin_set_output expands to — only writes
 * PIN_CNF, so a chip-select pin never shows up in DIR unless the two
 * are kept in sync here. */
static void nrf54l_gpio_set_dir(nrf54l_gpio_state_t *g, uint32_t newdir) {
    if (g->dir == newdir) return;
    uint32_t changed = g->dir ^ newdir;
    g->dir = newdir;
    for (int n = 0; n < 32; n++)
        if (changed & (1u << n))
            g->pin_cnf[n] = (g->pin_cnf[n] & ~1u) | ((newdir >> n) & 1u);
    nrf54l_gpio_changed(g);
}

static int nrf54l_gpio_read(void *user, uint32_t addr) {
    nrf54l_gpio_state_t *g = user;
    uint32_t off = addr - g->base;
    if (off >= 0x80 && off < 0x80 + 32 * 4)
        return (int)g->pin_cnf[(off - 0x80) / 4];
    switch (off) {
        case 0x00: case 0x04: case 0x08: return (int)g->out;  /* OUT/SET/CLR */
        case 0x0c: return (int)g->out;                        /* IN (loopback) */
        case 0x10: case 0x14: case 0x18: return (int)g->dir;  /* DIR/SET/CLR */
        default:   return 0;
    }
}

static void nrf54l_gpio_write(void *user, uint32_t addr, uint32_t value) {
    nrf54l_gpio_state_t *g = user;
    uint32_t off = addr - g->base;
    if (off >= 0x80 && off < 0x80 + 32 * 4) {                          /* PIN_CNF[n] */
        int n = (int)((off - 0x80) / 4);
        g->pin_cnf[n] = value;
        uint32_t dir = (value & 1u) ? (g->dir | (1u << n)) : (g->dir & ~(1u << n));
        if (dir != g->dir) nrf54l_gpio_set_dir(g, dir);
        else nrf54l_gpio_changed(g);
        return;
    }
    switch (off) {
        case 0x00: nrf54l_gpio_set_out(g, value);            return; /* OUT    */
        case 0x04: nrf54l_gpio_set_out(g, g->out |  value);  return; /* OUTSET */
        case 0x08: nrf54l_gpio_set_out(g, g->out & ~value);  return; /* OUTCLR */
        case 0x10: nrf54l_gpio_set_dir(g, value);            return; /* DIR    */
        case 0x14: nrf54l_gpio_set_dir(g, g->dir |  value);  return; /* DIRSET */
        case 0x18: nrf54l_gpio_set_dir(g, g->dir & ~value);  return; /* DIRCLR */
        default:   return;
    }
}

/* ============================================================
 * SPIM00 / SPIM22 / SPIM30 + the off-SoC SPI chips on them.
 * The register model lives in nrf54l15_spim.c; this section binds it
 * to the CPU (EasyDMA through arm_read8/arm_write8, completion events
 * on the live clock) and does the board-level part: which chip sees a
 * byte (chip-select routing) and chip-select edge forwarding.
 * ============================================================ */
static int64_t nrf54l_host_now_ns_live(void *cpu) {
    arm_cpu_t *c = cpu;
    return arm_cycles_to_ns(c->cycles, c->cpu_freq_hz);
}

static uint8_t nrf54l_spim_mem_read8(void *mem, uint32_t addr) {
    return arm_read8((arm_cpu_t *)mem, addr);
}
static void nrf54l_spim_mem_write8(void *mem, uint32_t addr, uint8_t value) {
    arm_write8((arm_cpu_t *)mem, addr, value);
}

static int nrf54l_spim_mmio_read(void *user, uint32_t addr) {
    nrf54l_spim_t *s = user;
    return (int)nrf54l_spim_read(s, addr & (NRF54L_SPIM_SIZE - 1));
}
static void nrf54l_spim_mmio_write(void *user, uint32_t addr, uint32_t value) {
    nrf54l_spim_t *s = user;
    nrf54l_spim_write(s, addr & (NRF54L_SPIM_SIZE - 1), value);
}

static const struct { int id; uint32_t base; } nrf54l_spim_instances[NRF54L_NUM_SPIM] = {
    {  0, NRF54L_SPIM00_BASE },
    { 22, NRF54L_SPIM22_BASE },
    { 30, NRF54L_SPIM30_BASE },
};

nrf54l_spim_t *nrf54l15_soc_spim(nrf54l15_soc_t *soc, int spim_id) {
    for (int i = 0; i < NRF54L_NUM_SPIM; i++)
        if (nrf54l_spim_instances[i].id == spim_id) return &soc->spim[i];
    return NULL;
}

/* Byte router: the chip whose CS is low on this instance answers. */
static uint8_t nrf54l_soc_spi_exchange(void *user, int instance, uint8_t mosi) {
    nrf54l15_soc_t *soc = user;
    for (int i = 0; i < NRF54L_MAX_SPI_CHIPS; i++) {
        nrf54l_spi_chip_t *c = &soc->spi_chips[i];
        if (c->name[0] && c->spim == instance && c->cs_low && c->exchange)
            return c->exchange(c->chip, mosi);
    }
    return 0xFF;
}

/* GPIO change → chip-select edges.  Selected = pin is an output AND
 * driven low; an unconfigured pin (input, external pull-up on the DK)
 * leaves the chip deselected. */
static void nrf54l_soc_gpio_changed(void *user, nrf54l_gpio_state_t *g) {
    nrf54l15_soc_t *soc = user;
    for (int i = 0; i < NRF54L_MAX_SPI_CHIPS; i++) {
        nrf54l_spi_chip_t *c = &soc->spi_chips[i];
        if (!c->name[0] || c->cs_port != g->port) continue;
        uint32_t bit = 1u << c->cs_pin;
        bool low = (g->dir & bit) && !(g->out & bit);
        if (low != c->cs_low) {
            c->cs_low = low;
            if (c->set_cs) c->set_cs(c->chip, low);
        }
    }
}

/* Chip kinds are registered per level: L2 adds mx25r6435f, L3 enc28j60. */
static int nrf54l_soc_bind_spi_chip(nrf54l_spi_chip_t *c, const char *name,
                                    const sim_host_t *host) {
    (void)c; (void)name; (void)host;
    return -1;
}

int nrf54l15_soc_attach_spi_chip(nrf54l15_soc_t *soc, const char *name,
                                 int spim_id, int cs_port, int cs_pin) {
    if (!soc || !name || !nrf54l15_soc_spim(soc, spim_id) ||
        cs_port < 0 || cs_port > 2 || cs_pin < 0 || cs_pin > 31)
        return -1;
    int slot = -1;
    for (int i = 0; i < NRF54L_MAX_SPI_CHIPS; i++)
        if (!soc->spi_chips[i].name[0]) { slot = i; break; }
    if (slot < 0) return -1;
    nrf54l_spi_chip_t *c = &soc->spi_chips[slot];
    memset(c, 0, sizeof(*c));
    if (nrf54l_soc_bind_spi_chip(c, name, &soc->live_host) != 0) {
        fprintf(stderr, "nrf54l15: unknown SPI chip '%s'\n", name);
        return -1;
    }
    snprintf(c->name, sizeof(c->name), "%s", name);
    c->spim    = spim_id;
    c->cs_port = cs_port;
    c->cs_pin  = cs_pin;
    c->cs_low  = false;
    /* Pick up the current pin level in case the firmware configured CS
     * before the chip was attached (config applied after boot). */
    nrf54l_soc_gpio_changed(soc, &soc->gpio[cs_port]);
    return slot;
}

void nrf54l15_soc_clear_spi_chips(nrf54l15_soc_t *soc) {
    if (!soc) return;
    for (int i = 0; i < NRF54L_MAX_SPI_CHIPS; i++) {
        nrf54l_spi_chip_t *c = &soc->spi_chips[i];
        if (c->name[0] && c->destroy) c->destroy(c->chip);
        memset(c, 0, sizeof(*c));
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

    /* WFI fast-forward guard. See arm_set_wfi_skip_guard / the WFI
     * handler in arm_cpu.c::arm_step. Returns non-zero whenever the
     * radio has cycle-tight state in flight; the WFI handler then
     * stays at full speed so CSMA / collision timing stay accurate. */
    extern int nrf54l_wfi_skip_guard(void *user);
    arm_set_wfi_skip_guard(&plat->cpu, nrf54l_wfi_skip_guard, soc);

    /* GLOBAL_CLOCK — first peripheral firmware touches after reset. */
    arm_register_io(&plat->cpu,
                    NRF54L_GLOBAL_CLOCK_BASE, NRF54L_GLOBAL_CLOCK_SIZE,
                    nrf54l_global_clock_read, nrf54l_global_clock_write,
                    &soc->global_clock);

    /* FICR — INFO.DEVICEID for the link-layer EUI-64. Default to
     * 0 here; the multinode harness overwrites with per-node values
     * before populate_link_address() runs. */
    arm_register_io(&plat->cpu,
                    NRF54L_FICR_BASE, NRF54L_FICR_SIZE,
                    nrf54l_ficr_read, nrf54l_ficr_write, &soc->ficr);

    /* UARTE20 — pure EasyDMA console. */
    soc->uarte20.plat    = plat;
    soc->uarte20.irq_num = NRF54L_UARTE20_IRQ;
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

    /* GPIO P0/P1/P2 — drives + observes the demo LEDs (LED0=P2.9 FLPR,
     * LED1=P1.10 M33) and carries the SPI chip-selects. */
    uint32_t gpio_bases[3] = { NRF54L_P0_BASE, NRF54L_P1_BASE, NRF54L_P2_BASE };
    for (int p = 0; p < 3; p++) {
        soc->gpio[p].plat = plat;
        soc->gpio[p].base = gpio_bases[p];
        soc->gpio[p].port = p;
        for (int n = 0; n < 32; n++) soc->gpio[p].pin_cnf[n] = 0x2;  /* GPIO_PIN_CNF_ResetValue */
        soc->gpio[p].change_cb   = nrf54l_soc_gpio_changed;
        soc->gpio[p].change_user = soc;
        arm_register_io(&plat->cpu, gpio_bases[p], NRF54L_GPIO_SIZE,
                        nrf54l_gpio_read, nrf54l_gpio_write, &soc->gpio[p]);
    }

    /* Live-time host vtable for the SPIM completion events and the SPI
     * chips: same as plat->host but now_ns comes from CPU cycles, so an
     * event armed mid-step lands bytes*8/bit-rate ahead of *now*, not of
     * the step's start. */
    soc->live_host        = plat->host;
    soc->live_host.now_ns = nrf54l_host_now_ns_live;

    /* SPIM00 / SPIM22 / SPIM30 — SPI masters with EasyDMA. */
    for (int i = 0; i < NRF54L_NUM_SPIM; i++) {
        nrf54l_spim_t *s = &soc->spim[i];
        nrf54l_spim_init(s, &soc->live_host, nrf54l_spim_instances[i].id);
        s->mem_read8     = nrf54l_spim_mem_read8;
        s->mem_write8    = nrf54l_spim_mem_write8;
        s->mem           = &plat->cpu;
        s->exchange      = nrf54l_soc_spi_exchange;
        s->exchange_user = soc;
        arm_register_io(&plat->cpu, nrf54l_spim_instances[i].base, NRF54L_SPIM_SIZE,
                        nrf54l_spim_mmio_read, nrf54l_spim_mmio_write, s);
    }

    /* VPR00 launch control + SPU00 permission gate — FLPR coprocessor.
     * spu_periph12 reset default 0x8001000a matches HW so the flpr-host
     * "SPU PERIPH[12] before=... after=..." log reads identically. */
    soc->vpr.plat         = plat;
    soc->vpr.spu_periph12 = 0x8001000au;
    soc->vpr.flpr         = NULL;
    arm_register_io(&plat->cpu, NRF54L_VPR00_BASE, NRF54L_VPR00_SIZE,
                    nrf54l_vpr_read, nrf54l_vpr_write, &soc->vpr);
    arm_register_io(&plat->cpu, NRF54L_SPU00_BASE, NRF54L_SPU00_SIZE,
                    nrf54l_spu_read, nrf54l_spu_write, &soc->vpr);

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
     * EGU10 dispatches the nrf_802154 SWI handler: the driver maps
     * RADIO.EVENTS_DISABLED → DPPI → EGU10.SUBSCRIBE_TRIGGER →
     * EGU10.EVENTS_TRIGGERED → IRQ 135 → SWI_IRQHandler → rxframe
     * notification. Without an actual NVIC pending bit the SWI
     * never runs and the upper layer never sees received frames. */
    nrf54l_egu_state_t *egus[3]    = { &soc->egu00, &soc->egu10, &soc->egu20 };
    uint32_t            bases[3]   = { NRF54L_EGU00_BASE, NRF54L_EGU10_BASE, NRF54L_EGU20_BASE };
    int                 irqs[3]    = { -1, NRF54L_EGU10_IRQ, -1 };
    for (int i = 0; i < 3; i++) {
        egus[i]->plat    = plat;
        egus[i]->dppi    = &soc->dppi;
        egus[i]->irq_num = irqs[i];
        for (int n = 0; n < NRF54L_EGU_NUM_CHANNELS; n++)
            egus[i]->sub_channel[n] = -1;
        arm_register_io(&plat->cpu, bases[i], NRF54L_EGU_SIZE,
                        nrf54l_egu_read, nrf54l_egu_write, egus[i]);
    }

    /* TIMER10/20 — used by nrf_802154 lptimer backend (TIMER20). */
    nrf54l_timer_setup(&soc->timer10, plat, &soc->dppi,
                       NRF54L_TIMER10_BASE, NRF54L_TIMER10_IRQ);
    nrf54l_timer_setup(&soc->timer20, plat, &soc->dppi,
                       NRF54L_TIMER20_BASE, NRF54L_TIMER20_IRQ);
}

static void nrf54l15_soc_destroy(arm_platform_t *plat) {
    nrf54l15_soc_t *soc = (nrf54l15_soc_t *)plat->soc;
    if (soc) {
        nrf54l15_soc_clear_spi_chips(soc);
        for (int i = 0; i < NRF54L_NUM_SPIM; i++)
            if (soc->spim[i].busy) arm_cancel_event(&plat->cpu, &soc->spim[i].xfer_event);
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
