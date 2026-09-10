/*
 * csim_dev.h — guest-side view of csim's co-simulation register window.
 *
 * Include this in firmware running under Renode to talk to a csim network
 * mapped as a CoSimulated.CoSimulatedPeripheral.  From the guest's point of
 * view csim is one memory-mapped 802.15.4 radio whose antenna reaches every
 * node in the csim simulation.
 *
 * This is a mirror of include/native/renode_dev.h.  The two must agree; the
 * VERSION register exists so a guest can check at run time.
 *
 * Base address is whatever the .repl maps the peripheral at.
 */
#ifndef CSIM_DEV_H
#define CSIM_DEV_H

#include <stdint.h>

#ifndef CSIM_DEV_BASE
#define CSIM_DEV_BASE 0x40100000UL
#endif

#define CSIM_REG(off) (*(volatile uint32_t *)(CSIM_DEV_BASE + (off)))

/* Identity */
#define CSIM_ID          CSIM_REG(0x00)   /* reads 0x4353494D, "CSIM" */
#define CSIM_VERSION     CSIM_REG(0x04)
#define CSIM_ID_MAGIC    0x4353494DUL
#define CSIM_VERSION_EXPECTED 1U

/* Control and status */
#define CSIM_CTRL        CSIM_REG(0x08)
#define CSIM_STATUS      CSIM_REG(0x0C)

#define CSIM_CTRL_IRQ_RX_EN    (1U << 0)  /* interrupt when a frame arrives  */
#define CSIM_CTRL_IRQ_UART_EN  (1U << 1)  /* interrupt on console output     */
#define CSIM_CTRL_RX_FLUSH     (1U << 2)  /* write 1: drop every queued frame */
#define CSIM_CTRL_TX_RESET     (1U << 3)  /* write 1: abandon the TX buffer   */

#define CSIM_STATUS_RX_AVAIL        (1U << 0)
#define CSIM_STATUS_RX_OVERFLOW     (1U << 1)  /* sticky until RX_FLUSH */
#define CSIM_STATUS_UART_AVAIL      (1U << 2)
#define CSIM_STATUS_UART_OVERFLOW   (1U << 3)
#define CSIM_STATUS_UART_IN_PENDING (1U << 4)

/* Radio configuration */
#define CSIM_CHANNEL     CSIM_REG(0x10)   /* 11..26; -1 = not selected */
#define CSIM_TXPOWER     CSIM_REG(0x14)   /* 0..31, scales range in the medium */

/* Transmit: set the length, stream the bytes, then trigger. */
#define CSIM_TX_LEN      CSIM_REG(0x18)   /* MAC frame length, excluding FCS */
#define CSIM_TX_DATA     CSIM_REG(0x1C)   /* an access appends its own width  */
#define CSIM_TX_CTRL     CSIM_REG(0x20)   /* write 1 to transmit              */
#define CSIM_TX_COUNT    CSIM_REG(0x24)

/* Receive: read the head's metadata and bytes, then pop it. */
#define CSIM_RX_COUNT    CSIM_REG(0x28)
#define CSIM_RX_LEN      CSIM_REG(0x2C)
#define CSIM_RX_DATA     CSIM_REG(0x30)   /* an access consumes its own width */
#define CSIM_RX_POP      CSIM_REG(0x34)   /* write 1 to drop the head         */
#define CSIM_RX_RSSI     CSIM_REG(0x38)   /* signed dBm, as heard HERE        */
#define CSIM_RX_FROM     CSIM_REG(0x3C)   /* sending csim node id, -1 unknown */
#define CSIM_RX_CHANNEL  CSIM_REG(0x40)
#define CSIM_RX_TIME_LO  CSIM_REG(0x44)   /* frame's start ON THE AIR, ns     */
#define CSIM_RX_TIME_HI  CSIM_REG(0x48)
#define CSIM_RX_DROPPED  CSIM_REG(0x4C)   /* frames lost to a full queue      */

/* Simulation */
#define CSIM_SIM_TIME_LO CSIM_REG(0x50)   /* csim's clock, ns                 */
#define CSIM_SIM_TIME_HI CSIM_REG(0x54)
#define CSIM_NODE_COUNT  CSIM_REG(0x58)
#define CSIM_SELF_ID     CSIM_REG(0x5C)

/* Console bridge to one csim node */
#define CSIM_UART_NODE   CSIM_REG(0x60)   /* which node's console (slot index) */
#define CSIM_UART_DATA   CSIM_REG(0x64)   /* read: a byte, or 0xFFFFFFFF if
                                           * empty.  write: inject a byte     */
#define CSIM_UART_COUNT  CSIM_REG(0x68)
#define CSIM_IRQ_STATUS  CSIM_REG(0x6C)   /* bit 0 RX, bit 1 console          */
#define CSIM_CCA         CSIM_REG(0x70)   /* 1 = csim's medium is busy here   */

#define CSIM_IRQ_SRC_RX   (1U << 0)
#define CSIM_IRQ_SRC_UART (1U << 1)

/* Longest MAC frame the window accepts. */
#define CSIM_MAX_FRAME 152

/* --- convenience -------------------------------------------------------- */

static inline int csim_present(void) {
    return CSIM_ID == CSIM_ID_MAGIC;
}

static inline uint64_t csim_sim_time_ns(void) {
    /* The clock only moves at a quantum boundary, never during a burst of
     * bus accesses, so the two halves cannot tear. */
    uint32_t lo = CSIM_SIM_TIME_LO;
    uint32_t hi = CSIM_SIM_TIME_HI;
    return ((uint64_t)hi << 32) | lo;
}

static inline void csim_send(const uint8_t *frame, uint32_t len) {
    if (len == 0 || len > CSIM_MAX_FRAME)
        return;
    CSIM_TX_LEN = len;
    for (uint32_t i = 0; i < len; i++)
        *(volatile uint8_t *)(CSIM_DEV_BASE + 0x1C) = frame[i];
    CSIM_TX_CTRL = 1;
}

/* Copy the oldest queued frame into `buf` and drop it.  Returns its length,
 * or 0 when nothing is queued. */
static inline uint32_t csim_receive(uint8_t *buf, uint32_t cap) {
    if (CSIM_RX_COUNT == 0)
        return 0;
    uint32_t len = CSIM_RX_LEN;
    uint32_t n = (len < cap) ? len : cap;
    for (uint32_t i = 0; i < n; i++)
        buf[i] = (uint8_t)(*(volatile uint8_t *)(CSIM_DEV_BASE + 0x30));
    CSIM_RX_POP = 1;
    return n;
}

#endif /* CSIM_DEV_H */
