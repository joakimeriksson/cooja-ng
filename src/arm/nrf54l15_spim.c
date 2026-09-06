/*
 * nrf54l15_spim — SPIM (SPI master + EasyDMA) model.  See the header
 * for the register map provenance and the driver path this covers.
 *
 * Transfer semantics (nRF54L15 SPIM, as exercised by nrfx_spim blocking
 * mode):
 *   - TASKS_START with ENABLE == 7 latches DMA.TX.MAXCNT / DMA.RX.MAXCNT
 *     and clocks max(TX, RX) bytes.  Each byte is taken from DMA.TX.PTR
 *     while TX bytes remain, ORC afterwards; the byte returned on MISO
 *     is stored to DMA.RX.PTR while RX bytes remain, dropped afterwards.
 *   - EVENTS_STARTED fires at START; EVENTS_END, EVENTS_DMA.TX.END and
 *     EVENTS_DMA.RX.END fire when the last byte has been clocked, with
 *     DMA.*.AMOUNT holding the byte counts.
 *   - The wire time is bytes * 8 / (base_freq / PRESCALER).  The
 *     completion is scheduled that far ahead rather than done inline so
 *     a 256-byte transfer at 8 MHz takes 256 µs of simulated time, not
 *     zero — the only fidelity the polling driver can observe.
 *   - TASKS_STOP raises EVENTS_STOPPED.  With no transfer in flight
 *     that is all it does; with one in flight the model completes the
 *     remaining bytes first (nrfx only STOPs after END in blocking mode,
 *     so the truncated-AMOUNT path is not modelled).
 *   - START while disabled is ignored, as on silicon.
 *
 * Time is read from the host and the event is scheduled through it, so
 * on the real platform the SoC hands over a "live" vtable (cycle-derived
 * now) — see the stale-sim_time_ns pitfall in docs/porting-a-device.md.
 */
#include "nrf54l15_spim.h"
#include <string.h>

static void spim_raise(nrf54l_spim_t *s, uint32_t int_bit) {
    if ((s->inten & int_bit) && s->irq) s->irq(s->irq_user);
}

uint32_t nrf54l_spim_bit_rate_hz(const nrf54l_spim_t *s) {
    uint32_t div = s->prescaler & SPIM_PRESCALER_MSK;
    if (div < s->prescaler_min) div = s->prescaler_min;
    return s->base_freq_hz / div;
}

int64_t nrf54l_spim_transfer_ns(const nrf54l_spim_t *s, uint32_t nbytes) {
    uint32_t hz = nrf54l_spim_bit_rate_hz(s);
    if (hz == 0) hz = 1;
    /* bits * 1e9 / hz, in 64-bit to survive 65535 * 8 * 1e9. */
    return (int64_t)(((uint64_t)nbytes * 8u * 1000000000ull) / hz);
}

/* Clock the latched transfer through the exchange callback. */
static void spim_complete(nrf54l_spim_t *s) {
    uint32_t n = s->xfer_len;
    for (uint32_t i = 0; i < n; i++) {
        uint8_t mosi = (i < s->tx_maxcnt && s->mem_read8)
                       ? s->mem_read8(s->mem, s->tx_ptr + i)
                       : (uint8_t)(s->orc & 0xFFu);
        uint8_t miso = s->exchange
                       ? s->exchange(s->exchange_user, s->instance, mosi)
                       : 0xFFu;                /* nothing on the bus → floats high */
        if (i < s->rx_maxcnt && s->mem_write8)
            s->mem_write8(s->mem, s->rx_ptr + i, miso);
    }
    s->tx_amount = s->tx_maxcnt;
    s->rx_amount = s->rx_maxcnt;
    s->busy      = false;
    s->stat_transfers++;
    s->stat_bytes += n;

    s->evt_tx_end = 1; spim_raise(s, SPIM_INT_DMATXEND);
    s->evt_rx_end = 1; spim_raise(s, SPIM_INT_DMARXEND);
    s->evt_end    = 1; spim_raise(s, SPIM_INT_END);
    /* SHORTS.END_START would re-trigger START here.  nrfx never sets it
     * in blocking mode and a self-restarting transfer with the same
     * buffers is not something the drivers under test do, so it is
     * deliberately not chained. */
}

static void spim_xfer_event_cb(void *user, cpu_event_t *ev) {
    (void)ev;
    nrf54l_spim_t *s = user;
    if (s->busy) spim_complete(s);
}

static void spim_task_start(nrf54l_spim_t *s) {
    if (s->enable != SPIM_ENABLE_ENABLED) return;   /* task ignored while disabled */
    if (s->busy) return;                            /* START during a transfer: no-op */

    uint32_t tx = s->tx_maxcnt & SPIM_MAXCNT_MSK;
    uint32_t rx = s->rx_maxcnt & SPIM_MAXCNT_MSK;
    s->xfer_len   = tx > rx ? tx : rx;
    s->busy       = true;
    s->evt_started  = 1; spim_raise(s, SPIM_INT_STARTED);
    s->evt_tx_ready = 1; spim_raise(s, SPIM_INT_DMATXREADY);
    s->evt_rx_ready = 1; spim_raise(s, SPIM_INT_DMARXREADY);

    int64_t now  = HOST_NOW_NS(s);
    int64_t fire = now + nrf54l_spim_transfer_ns(s, s->xfer_len);
    if (fire <= now) fire = now + 1;               /* zero-length: still asynchronous */
    s->xfer_event.callback  = spim_xfer_event_cb;
    s->xfer_event.user_data = s;
    HOST_SCHEDULE_NS(s, &s->xfer_event, fire);
}

static void spim_task_stop(nrf54l_spim_t *s) {
    if (s->busy) {
        HOST_CANCEL(s, &s->xfer_event);
        spim_complete(s);
    }
    s->evt_stopped = 1;
    spim_raise(s, SPIM_INT_STOPPED);
}

void nrf54l_spim_init(nrf54l_spim_t *s, const sim_host_t *host, int instance) {
    memset(s, 0, sizeof(*s));
    s->host     = host;
    s->instance = instance;
    if (instance == 0) {            /* SPIM00: 128 MHz core, divisor 4..126 */
        s->base_freq_hz  = 128000000u;
        s->prescaler_min = 4;
    } else {                        /* SPIM2x/30: 16 MHz core, divisor 2..126 */
        s->base_freq_hz  = 16000000u;
        s->prescaler_min = 2;
    }
    s->prescaler = SPIM_PRESCALER_RESET;
    s->psel_sck = s->psel_mosi = s->psel_miso = SPIM_PSEL_DISCONNECTED;
    s->psel_dcx = s->psel_csn  = SPIM_PSEL_DISCONNECTED;
    s->rxdelay  = 2;                /* SPIM00_RXDELAY_RESET_VALUE */
}

uint32_t nrf54l_spim_read(nrf54l_spim_t *s, uint32_t off) {
    switch (off) {
        case SPIM_SUBSCRIBE_START:      return s->subscribe_start;
        case SPIM_SUBSCRIBE_STOP:       return s->subscribe_stop;
        case SPIM_EVENTS_STARTED:       return s->evt_started;
        case SPIM_EVENTS_STOPPED:       return s->evt_stopped;
        case SPIM_EVENTS_END:           return s->evt_end;
        case SPIM_EVENTS_DMA_RX_END:    return s->evt_rx_end;
        case SPIM_EVENTS_DMA_RX_READY:  return s->evt_rx_ready;
        case SPIM_EVENTS_DMA_TX_END:    return s->evt_tx_end;
        case SPIM_EVENTS_DMA_TX_READY:  return s->evt_tx_ready;
        case SPIM_PUBLISH_STARTED:      return s->publish_started;
        case SPIM_PUBLISH_STOPPED:      return s->publish_stopped;
        case SPIM_PUBLISH_END:          return s->publish_end;
        case SPIM_SHORTS:               return s->shorts;
        case SPIM_INTENSET:
        case SPIM_INTENCLR:             return s->inten;
        case SPIM_ENABLE:               return s->enable;
        case SPIM_PRESCALER:            return s->prescaler;
        case SPIM_CONFIG:               return s->config;
        case SPIM_IFTIMING_RXDELAY:     return s->rxdelay;
        case SPIM_IFTIMING_CSNDUR:      return s->csndur;
        case SPIM_DCXCNT:               return s->dcxcnt;
        case SPIM_CSNPOL:               return s->csnpol;
        case SPIM_ORC:                  return s->orc;
        case SPIM_PSEL_SCK:             return s->psel_sck;
        case SPIM_PSEL_MOSI:            return s->psel_mosi;
        case SPIM_PSEL_MISO:            return s->psel_miso;
        case SPIM_PSEL_DCX:             return s->psel_dcx;
        case SPIM_PSEL_CSN:             return s->psel_csn;
        case SPIM_DMA_RX_PTR:           return s->rx_ptr;
        case SPIM_DMA_RX_MAXCNT:        return s->rx_maxcnt;
        case SPIM_DMA_RX_AMOUNT:        return s->rx_amount;
        case SPIM_DMA_RX_LIST:          return s->rx_list;
        case SPIM_DMA_TX_PTR:           return s->tx_ptr;
        case SPIM_DMA_TX_MAXCNT:        return s->tx_maxcnt;
        case SPIM_DMA_TX_AMOUNT:        return s->tx_amount;
        case SPIM_DMA_TX_LIST:          return s->tx_list;
        default:                        return 0;   /* tasks, bus-error, match, errata pokes */
    }
}

void nrf54l_spim_write(nrf54l_spim_t *s, uint32_t off, uint32_t value) {
    switch (off) {
        case SPIM_TASKS_START:   if (value & 1u) spim_task_start(s); return;
        case SPIM_TASKS_STOP:    if (value & 1u) spim_task_stop(s);  return;
        case SPIM_TASKS_SUSPEND:
        case SPIM_TASKS_RESUME:  return;             /* not used by the blocking driver */

        case SPIM_SUBSCRIBE_START:      s->subscribe_start = value; return;
        case SPIM_SUBSCRIBE_STOP:       s->subscribe_stop  = value; return;

        /* Events: firmware writes 0 to clear (nrf_spim_event_clear). */
        case SPIM_EVENTS_STARTED:       s->evt_started  = value & 1u; return;
        case SPIM_EVENTS_STOPPED:       s->evt_stopped  = value & 1u; return;
        case SPIM_EVENTS_END:           s->evt_end      = value & 1u; return;
        case SPIM_EVENTS_DMA_RX_END:    s->evt_rx_end   = value & 1u; return;
        case SPIM_EVENTS_DMA_RX_READY:  s->evt_rx_ready = value & 1u; return;
        case SPIM_EVENTS_DMA_TX_END:    s->evt_tx_end   = value & 1u; return;
        case SPIM_EVENTS_DMA_TX_READY:  s->evt_tx_ready = value & 1u; return;

        case SPIM_PUBLISH_STARTED:      s->publish_started = value; return;
        case SPIM_PUBLISH_STOPPED:      s->publish_stopped = value; return;
        case SPIM_PUBLISH_END:          s->publish_end     = value; return;

        case SPIM_SHORTS:               s->shorts = value;   return;
        case SPIM_INTENSET:             s->inten |= value;   return;
        case SPIM_INTENCLR:             s->inten &= ~value;  return;

        case SPIM_ENABLE:               s->enable    = value & 0xFu; return;
        case SPIM_PRESCALER:            s->prescaler = value & SPIM_PRESCALER_MSK; return;
        case SPIM_CONFIG:               s->config    = value & 0x7u; return;
        case SPIM_IFTIMING_RXDELAY:     s->rxdelay   = value & 0x7u; return;
        case SPIM_IFTIMING_CSNDUR:      s->csndur    = value & 0xFFu; return;
        case SPIM_DCXCNT:               s->dcxcnt    = value & 0xFu; return;
        case SPIM_CSNPOL:               s->csnpol    = value & 1u; return;
        case SPIM_ORC:                  s->orc       = value & 0xFFu; return;

        case SPIM_PSEL_SCK:             s->psel_sck  = value; return;
        case SPIM_PSEL_MOSI:            s->psel_mosi = value; return;
        case SPIM_PSEL_MISO:            s->psel_miso = value; return;
        case SPIM_PSEL_DCX:             s->psel_dcx  = value; return;
        case SPIM_PSEL_CSN:             s->psel_csn  = value; return;

        case SPIM_DMA_RX_PTR:           s->rx_ptr    = value; return;
        case SPIM_DMA_RX_MAXCNT:        s->rx_maxcnt = value & SPIM_MAXCNT_MSK; return;
        case SPIM_DMA_RX_LIST:          s->rx_list   = value; return;
        case SPIM_DMA_TX_PTR:           s->tx_ptr    = value; return;
        case SPIM_DMA_TX_MAXCNT:        s->tx_maxcnt = value & SPIM_MAXCNT_MSK; return;
        case SPIM_DMA_TX_LIST:          s->tx_list   = value; return;

        default:
            /* AMOUNT (read-only), bus-error / pattern-match registers,
             * and the errata 55 / 8 workaround pokes at +0xC80/+0xC84:
             * accepted and dropped. */
            return;
    }
}
