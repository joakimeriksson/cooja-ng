/*
 * renode_dev — the register window csim exposes to Renode.
 *
 * Renode maps this device into its emulated CPU's address space (a
 * `CoSimulated.CoSimulatedPeripheral` in a .repl).  Guest firmware running
 * in Renode therefore sees csim's whole 802.15.4 network as one memory-
 * mapped radio: write a frame into the TX FIFO and it goes on the air in
 * csim's medium; a frame csim delivers here lands in the RX FIFO and raises
 * the device's interrupt line.
 *
 * This file is the device model alone — no sockets, no kernel, no mote.
 * Every simulation-facing action goes through the hooks below, which
 * src/motes/renode_mote.c fills in with the runner's env glue.  That keeps
 * it unit-testable (test_renode_cosim.c) and keeps the ordering rule
 * visible: a bus access must never reach into the kernel by itself.
 *
 * Design: docs/design/renode-cosim-plan.md.  Wire codec:
 * include/common/renode_proto.h.  Guest-side mirror of this map:
 * examples/renode/csim_dev.h.
 */
#ifndef RENODE_DEV_H
#define RENODE_DEV_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Size of the mapped window (must match the .repl's `+0x100`). */
#define RENODE_DEV_WINDOW      0x100

/* Frames are handed to the runner's frame hook, which PHY-wraps into a
 * 160-byte buffer.  Same cap, and same reason, as EXT_NODE_MAX_FRAME. */
#define RENODE_DEV_MAX_FRAME   152

/* Frames waiting for the guest to read them.  Same depth as the external
 * node's queue: deep enough for a burst, shallow enough that a guest that
 * never drains is noticed. */
#define RENODE_DEV_RX_QUEUE    16

/* Console bytes waiting to go to Renode's log. */
#define RENODE_DEV_UART_RING   1024

/* Console bytes from Renode still to be pushed into a csim node. */
#define RENODE_DEV_UART_IN     64

/* ---- Register offsets (32-bit registers; see the map in the plan doc) ---- */
#define RENODE_REG_ID          0x00  /* R   0x4353494D "CSIM"                */
#define RENODE_REG_VERSION     0x04  /* R   register-map version             */
#define RENODE_REG_CTRL        0x08  /* RW  see RENODE_CTRL_*                */
#define RENODE_REG_STATUS      0x0C  /* R   see RENODE_STATUS_*              */
#define RENODE_REG_CHANNEL     0x10  /* RW  802.15.4 channel, -1 = any       */
#define RENODE_REG_TXPOWER     0x14  /* RW  0..31 (CC2420 PA_LEVEL scale)    */
#define RENODE_REG_TX_LEN      0x18  /* RW  MAC frame length, excl. FCS      */
#define RENODE_REG_TX_DATA     0x1C  /* W   append `width` bytes             */
#define RENODE_REG_TX_CTRL     0x20  /* W   1 = transmit now                 */
#define RENODE_REG_TX_COUNT    0x24  /* R   frames transmitted               */
#define RENODE_REG_RX_COUNT    0x28  /* R   frames queued                    */
#define RENODE_REG_RX_LEN      0x2C  /* R   head frame length                */
#define RENODE_REG_RX_DATA     0x30  /* R   next `width` bytes of head       */
#define RENODE_REG_RX_POP      0x34  /* W   1 = drop head                    */
#define RENODE_REG_RX_RSSI     0x38  /* R   head RSSI, sign-extended         */
#define RENODE_REG_RX_FROM     0x3C  /* R   head sender node id, -1 unknown  */
#define RENODE_REG_RX_CHANNEL  0x40  /* R   head sender channel              */
#define RENODE_REG_RX_TIME_LO  0x44  /* R   head on-air start ns, low  32    */
#define RENODE_REG_RX_TIME_HI  0x48  /* R   head on-air start ns, high 32    */
#define RENODE_REG_RX_DROPPED  0x4C  /* R   frames dropped, queue full       */
#define RENODE_REG_SIM_TIME_LO 0x50  /* R   csim now_ns, low  32             */
#define RENODE_REG_SIM_TIME_HI 0x54  /* R   csim now_ns, high 32             */
#define RENODE_REG_NODE_COUNT  0x58  /* R   csim node count                  */
#define RENODE_REG_SELF_ID     0x5C  /* R   this device's csim node id       */
#define RENODE_REG_UART_NODE   0x60  /* RW  slot whose console is bridged    */
#define RENODE_REG_UART_DATA   0x64  /* RW  R: pop byte / W: inject byte     */
#define RENODE_REG_UART_COUNT  0x68  /* R   console bytes available          */
#define RENODE_REG_IRQ_STATUS  0x6C  /* R   unmasked interrupt sources       */
#define RENODE_REG_CCA         0x70  /* R   1 = csim's medium is busy here    */

#define RENODE_DEV_ID          0x4353494DU   /* "CSIM" */
#define RENODE_DEV_VERSION     1U

/* CTRL bits */
#define RENODE_CTRL_IRQ_RX_EN   (1U << 0)
#define RENODE_CTRL_IRQ_UART_EN (1U << 1)
#define RENODE_CTRL_RX_FLUSH    (1U << 2)  /* write-1, self-clearing */
#define RENODE_CTRL_TX_RESET    (1U << 3)  /* write-1, self-clearing */
#define RENODE_CTRL_PERSISTENT  (RENODE_CTRL_IRQ_RX_EN | RENODE_CTRL_IRQ_UART_EN)

/* STATUS bits */
#define RENODE_STATUS_RX_AVAIL       (1U << 0)
#define RENODE_STATUS_RX_OVERFLOW    (1U << 1)  /* sticky until RX_FLUSH  */
#define RENODE_STATUS_UART_AVAIL     (1U << 2)
#define RENODE_STATUS_UART_OVERFLOW  (1U << 3)  /* sticky until read      */
#define RENODE_STATUS_UART_IN_PENDING (1U << 4)

/* IRQ_STATUS bits (unmasked sources) */
#define RENODE_IRQ_SRC_RX   (1U << 0)
#define RENODE_IRQ_SRC_UART (1U << 1)

/* The device's single interrupt line, wired in the .repl as `0 -> nvic@N`. */
#define RENODE_DEV_IRQ_INDEX 0

/* Channel value meaning "no channel selected yet" — the medium's wildcard. */
#define RENODE_DEV_CHANNEL_ANY (-1)

typedef struct renode_dev_rx {
    int64_t start_ns;    /* frame's start on the air (bus frame_start_ns) */
    int     from_id;     /* sender's csim node id, -1 unknown             */
    int     channel;     /* sender's channel, -1 unknown                  */
    int8_t  rssi;        /* per-receiver RSSI the medium computed         */
    int     len;
    uint8_t frame[RENODE_DEV_MAX_FRAME];
} renode_dev_rx_t;

typedef struct renode_dev {
    /* ---- guest-visible state ---- */
    uint32_t ctrl;
    uint32_t status_sticky;      /* the two overflow bits only */
    int      channel;
    int      txpower;

    uint8_t  tx_buf[RENODE_DEV_MAX_FRAME];
    int      tx_len;             /* TX_LEN as written */
    int      tx_wr;              /* bytes appended so far */
    uint32_t tx_count;

    renode_dev_rx_t rx[RENODE_DEV_RX_QUEUE];
    int      rx_head;            /* index of the oldest queued frame */
    int      rx_count;
    int      rx_rd;              /* read cursor inside the head frame */
    uint32_t rx_dropped;

    uint8_t  uart_out[RENODE_DEV_UART_RING];   /* csim node -> Renode */
    int      uo_head, uo_count;
    uint32_t uart_dropped;

    uint8_t  uart_in[RENODE_DEV_UART_IN];      /* Renode -> csim node */
    int      ui_count;
    int      uart_node;          /* slot whose console is bridged */

    int      self_id;            /* this device's csim node id */
    int      node_count;         /* csim node count (informational) */

    /* ---- simulation hooks (filled by the mote; all may be NULL) ---- */
    int64_t (*now_ns)(void *user);
    void    (*tx)(void *user, const uint8_t *frame, int len);
    void    (*set_channel)(void *user, int channel);
    void    (*set_power)(void *user, int indicator);
    /* Push bytes into node `slot`'s console.  Returns bytes consumed. */
    int     (*uart_inject)(void *user, int slot, const uint8_t *buf, int len);
    /* Carrier sense: is any in-range neighbour transmitting right now?
     *
     * A frame-level peer (Renode's radio model has no air time) cannot know
     * this by itself, and injecting on top of a transmission already in
     * flight is not a collision csim reports -- the receiver's radio is busy
     * with the first frame, drops the opening bytes of the second, then locks
     * SFD partway through it and fails CRC.  Exposing the medium's own CCA
     * is what lets such a peer defer instead. */
    bool    (*channel_busy)(void *user);
    void    *user;
} renode_dev_t;

/* Zero the guest-visible state and set the defaults (channel = any,
 * txpower = max).  Does NOT touch the hooks or `user` — call before wiring
 * them, or use renode_dev_reset() afterwards. */
void renode_dev_init(renode_dev_t *d);

/* resetPeripheral: clear FIFOs, counters and CTRL, keep the hooks and the
 * identity fields (self_id, node_count, uart_node). */
void renode_dev_reset(renode_dev_t *d);

/* Bus access.  `width` is 1/2/4/8; the FIFO data registers transfer that
 * many bytes, every other register is a 32-bit value masked to `width`. */
uint64_t renode_dev_read(renode_dev_t *d, uint64_t off, int width);
void     renode_dev_write(renode_dev_t *d, uint64_t off, int width,
                          uint64_t value);

/* Queue a frame the medium delivered to this node.  Returns 0 on success,
 * -1 when the queue is full (the frame is dropped and counted). */
int  renode_dev_rx_push(renode_dev_t *d, const uint8_t *frame, int len,
                        int64_t start_ns, int from_id, int channel,
                        int8_t rssi);

/* Queue one console byte from the bridged csim node. */
void renode_dev_uart_push(renode_dev_t *d, uint8_t byte);

/* Push whatever the guest wrote to UART_DATA into the csim node.  Called at
 * the quantum boundary: a node that could not take the bytes now is retried
 * next time rather than losing them. */
void renode_dev_uart_retry(renode_dev_t *d);

/* Current level of the device's interrupt line. */
bool renode_dev_irq_level(const renode_dev_t *d);

#ifdef __cplusplus
}
#endif

#endif /* RENODE_DEV_H */
