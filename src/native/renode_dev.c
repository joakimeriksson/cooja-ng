/*
 * renode_dev — the register window csim exposes to Renode.
 * See include/native/renode_dev.h for the map and the design rules.
 */
#include "renode_dev.h"

#include <string.h>

/* ============================================================
 * Lifecycle
 * ============================================================ */

static void dev_clear_state(renode_dev_t *d) {
    d->ctrl          = 0;
    d->status_sticky = 0;
    d->channel       = RENODE_DEV_CHANNEL_ANY;
    d->txpower       = 31;

    memset(d->tx_buf, 0, sizeof(d->tx_buf));
    d->tx_len = 0;
    d->tx_wr  = 0;
    d->tx_count = 0;

    d->rx_head = 0;
    d->rx_count = 0;
    d->rx_rd = 0;
    d->rx_dropped = 0;

    d->uo_head = 0;
    d->uo_count = 0;
    d->uart_dropped = 0;
    d->ui_count = 0;
}

void renode_dev_init(renode_dev_t *d) {
    if (!d) return;
    memset(d, 0, sizeof(*d));
    dev_clear_state(d);
    d->uart_node = 0;
    d->self_id   = -1;
    d->node_count = 0;
}

void renode_dev_reset(renode_dev_t *d) {
    if (!d) return;
    dev_clear_state(d);
    /* self_id / node_count / uart_node are identity, not state: a machine
     * reset in Renode does not renumber csim's nodes. */
}

/* ============================================================
 * Queues
 * ============================================================ */

int renode_dev_rx_push(renode_dev_t *d, const uint8_t *frame, int len,
                       int64_t start_ns, int from_id, int channel,
                       int8_t rssi) {
    if (!d || !frame || len <= 0)
        return -1;
    if (len > RENODE_DEV_MAX_FRAME)
        len = RENODE_DEV_MAX_FRAME;
    if (d->rx_count >= RENODE_DEV_RX_QUEUE) {
        /* Drop the NEW frame, not the oldest: a guest reading in order sees
         * a prefix of the truth rather than a gap in the middle. */
        d->rx_dropped++;
        d->status_sticky |= RENODE_STATUS_RX_OVERFLOW;
        return -1;
    }
    int slot = (d->rx_head + d->rx_count) % RENODE_DEV_RX_QUEUE;
    renode_dev_rx_t *e = &d->rx[slot];
    memcpy(e->frame, frame, (size_t)len);
    e->len      = len;
    e->start_ns = start_ns;
    e->from_id  = from_id;
    e->channel  = channel;
    e->rssi     = rssi;
    d->rx_count++;
    return 0;
}

static void rx_pop(renode_dev_t *d) {
    if (d->rx_count <= 0) return;
    d->rx_head = (d->rx_head + 1) % RENODE_DEV_RX_QUEUE;
    d->rx_count--;
    d->rx_rd = 0;
}

static const renode_dev_rx_t *rx_peek(const renode_dev_t *d) {
    if (d->rx_count <= 0) return NULL;
    return &d->rx[d->rx_head];
}

void renode_dev_uart_push(renode_dev_t *d, uint8_t byte) {
    if (!d) return;
    if (d->uo_count >= RENODE_DEV_UART_RING) {
        d->uart_dropped++;
        d->status_sticky |= RENODE_STATUS_UART_OVERFLOW;
        return;
    }
    int slot = (d->uo_head + d->uo_count) % RENODE_DEV_UART_RING;
    d->uart_out[slot] = byte;
    d->uo_count++;
}

void renode_dev_uart_retry(renode_dev_t *d) {
    if (!d || d->ui_count <= 0 || !d->uart_inject)
        return;
    int n = d->uart_inject(d->user, d->uart_node, d->uart_in, d->ui_count);
    if (n <= 0)
        return;                       /* node could not take them; keep */
    if (n >= d->ui_count) {
        d->ui_count = 0;
        return;
    }
    memmove(d->uart_in, d->uart_in + n, (size_t)(d->ui_count - n));
    d->ui_count -= n;
}

bool renode_dev_irq_level(const renode_dev_t *d) {
    if (!d) return false;
    if ((d->ctrl & RENODE_CTRL_IRQ_RX_EN) && d->rx_count > 0)
        return true;
    if ((d->ctrl & RENODE_CTRL_IRQ_UART_EN) && d->uo_count > 0)
        return true;
    return false;
}

/* ============================================================
 * Bus access
 * ============================================================ */

static uint32_t dev_status(const renode_dev_t *d) {
    uint32_t s = d->status_sticky;
    if (d->rx_count > 0) s |= RENODE_STATUS_RX_AVAIL;
    if (d->uo_count > 0) s |= RENODE_STATUS_UART_AVAIL;
    if (d->ui_count > 0) s |= RENODE_STATUS_UART_IN_PENDING;
    return s;
}

static uint32_t dev_irq_status(const renode_dev_t *d) {
    uint32_t s = 0;
    if (d->rx_count > 0) s |= RENODE_IRQ_SRC_RX;
    if (d->uo_count > 0) s |= RENODE_IRQ_SRC_UART;
    return s;
}

static int64_t dev_now(const renode_dev_t *d) {
    return d->now_ns ? d->now_ns(d->user) : 0;
}

/* Read `width` bytes out of the head frame, little-endian, zero-padded past
 * the end.  Advances the read cursor so a guest can stream a frame with
 * consecutive accesses of any width. */
static uint64_t rx_data_read(renode_dev_t *d, int width) {
    const renode_dev_rx_t *e = rx_peek(d);
    uint64_t v = 0;
    for (int i = 0; i < width; i++) {
        uint8_t b = 0;
        if (e && d->rx_rd < e->len)
            b = e->frame[d->rx_rd];
        if (e && d->rx_rd < e->len)
            d->rx_rd++;
        v |= (uint64_t)b << (8 * i);
    }
    return v;
}

uint64_t renode_dev_read(renode_dev_t *d, uint64_t off, int width) {
    if (!d || width <= 0)
        return 0;
    if (width > 8) width = 8;

    /* The two FIFO data registers are byte streams: an access of any width
     * transfers that many bytes.  Everything else is a 32-bit register. */
    if (off == RENODE_REG_RX_DATA)
        return rx_data_read(d, width);

    const renode_dev_rx_t *e = rx_peek(d);
    uint32_t v = 0;
    switch (off) {
    case RENODE_REG_ID:          v = RENODE_DEV_ID; break;
    case RENODE_REG_VERSION:     v = RENODE_DEV_VERSION; break;
    case RENODE_REG_CTRL:        v = d->ctrl & RENODE_CTRL_PERSISTENT; break;
    case RENODE_REG_STATUS:      v = dev_status(d); break;
    case RENODE_REG_CHANNEL:     v = (uint32_t)(int32_t)d->channel; break;
    case RENODE_REG_TXPOWER:     v = (uint32_t)d->txpower; break;
    case RENODE_REG_TX_LEN:      v = (uint32_t)d->tx_len; break;
    case RENODE_REG_TX_COUNT:    v = d->tx_count; break;
    case RENODE_REG_RX_COUNT:    v = (uint32_t)d->rx_count; break;
    case RENODE_REG_RX_LEN:      v = e ? (uint32_t)e->len : 0; break;
    case RENODE_REG_RX_RSSI:     v = (uint32_t)(int32_t)(e ? e->rssi : 0); break;
    case RENODE_REG_RX_FROM:     v = (uint32_t)(int32_t)(e ? e->from_id : -1); break;
    case RENODE_REG_RX_CHANNEL:  v = (uint32_t)(int32_t)(e ? e->channel : -1); break;
    case RENODE_REG_RX_TIME_LO:  v = e ? (uint32_t)((uint64_t)e->start_ns) : 0; break;
    case RENODE_REG_RX_TIME_HI:  v = e ? (uint32_t)((uint64_t)e->start_ns >> 32) : 0; break;
    case RENODE_REG_RX_DROPPED:  v = d->rx_dropped; break;
    case RENODE_REG_SIM_TIME_LO: v = (uint32_t)((uint64_t)dev_now(d)); break;
    case RENODE_REG_SIM_TIME_HI: v = (uint32_t)((uint64_t)dev_now(d) >> 32); break;
    case RENODE_REG_NODE_COUNT:  v = (uint32_t)d->node_count; break;
    case RENODE_REG_SELF_ID:     v = (uint32_t)(int32_t)d->self_id; break;
    case RENODE_REG_UART_NODE:   v = (uint32_t)d->uart_node; break;
    case RENODE_REG_UART_COUNT:  v = (uint32_t)d->uo_count; break;
    case RENODE_REG_IRQ_STATUS:  v = dev_irq_status(d); break;
    case RENODE_REG_UART_DATA:
        if (d->uo_count > 0) {
            v = d->uart_out[d->uo_head];
            d->uo_head = (d->uo_head + 1) % RENODE_DEV_UART_RING;
            d->uo_count--;
            /* Reading the last byte clears the sticky overflow: the guest
             * has now seen everything that survived. */
            if (d->uo_count == 0)
                d->status_sticky &= ~RENODE_STATUS_UART_OVERFLOW;
        } else {
            v = 0xFFFFFFFFU;          /* empty */
        }
        break;
    default:
        v = 0;                        /* unmapped offsets read as zero */
        break;
    }

    if (width >= 4)
        return v;
    /* A narrow access sees the low bytes of the 32-bit register. */
    return v & ((1ULL << (8 * width)) - 1ULL);
}

static void tx_append(renode_dev_t *d, int width, uint64_t value) {
    for (int i = 0; i < width; i++) {
        if (d->tx_wr >= d->tx_len || d->tx_wr >= RENODE_DEV_MAX_FRAME)
            return;                   /* full: extra bytes ignored */
        d->tx_buf[d->tx_wr++] = (uint8_t)(value >> (8 * i));
    }
}

static void tx_fire(renode_dev_t *d) {
    /* A guest driver bug (TX_CTRL with no length, or a short buffer) must
     * not put a malformed frame on the air, and must not crash us. */
    if (d->tx_len <= 0 || d->tx_wr < d->tx_len)
        return;
    if (d->tx)
        d->tx(d->user, d->tx_buf, d->tx_len);
    d->tx_count++;
    d->tx_wr = 0;                     /* ready for the next frame */
}

void renode_dev_write(renode_dev_t *d, uint64_t off, int width,
                      uint64_t value) {
    if (!d || width <= 0)
        return;
    if (width > 8) width = 8;

    if (off == RENODE_REG_TX_DATA) {
        tx_append(d, width, value);
        return;
    }

    uint32_t v32 = (uint32_t)value;
    switch (off) {
    case RENODE_REG_CTRL: {
        d->ctrl = v32 & RENODE_CTRL_PERSISTENT;
        if (v32 & RENODE_CTRL_RX_FLUSH) {
            d->rx_head = 0;
            d->rx_count = 0;
            d->rx_rd = 0;
            d->status_sticky &= ~RENODE_STATUS_RX_OVERFLOW;
        }
        if (v32 & RENODE_CTRL_TX_RESET) {
            d->tx_wr = 0;
            d->tx_len = 0;
        }
        break;
    }
    case RENODE_REG_CHANNEL: {
        int ch = (int)(int32_t)v32;
        /* Out-of-band values other than the wildcard are ignored rather than
         * pushed into the medium, where they would silently mute the node. */
        if (ch != RENODE_DEV_CHANNEL_ANY && (ch < 11 || ch > 26))
            break;
        d->channel = ch;
        if (d->set_channel && ch != RENODE_DEV_CHANNEL_ANY)
            d->set_channel(d->user, ch);
        break;
    }
    case RENODE_REG_TXPOWER: {
        int p = (int)(v32 & 0x1f);
        d->txpower = p;
        if (d->set_power)
            d->set_power(d->user, p);
        break;
    }
    case RENODE_REG_TX_LEN: {
        int len = (int)(int32_t)v32;
        if (len < 0 || len > RENODE_DEV_MAX_FRAME)
            break;                    /* ignore, keep the old length */
        d->tx_len = len;
        d->tx_wr = 0;                 /* a new length starts a new frame */
        break;
    }
    case RENODE_REG_TX_CTRL:
        if (v32 & 1U)
            tx_fire(d);
        break;
    case RENODE_REG_RX_POP:
        if (v32 & 1U)
            rx_pop(d);
        break;
    case RENODE_REG_UART_NODE:
        d->uart_node = (int)(int32_t)v32;
        break;
    case RENODE_REG_UART_DATA:
        if (d->ui_count < RENODE_DEV_UART_IN)
            d->uart_in[d->ui_count++] = (uint8_t)(v32 & 0xff);
        break;
    default:
        break;                        /* read-only / unmapped: ignored */
    }
}
