/*
 * sim_radio_bus — RF routing state and helpers (Phase 1 milestone 9).
 *
 * M9.1/9.2 slice: the runner's per-sender frame assembly, per-receiver
 * deferred-RX queues, and TX byte buffers move behind one struct owned
 * by the kernel (sim_runtime_t.radio_bus points here).  Dispatch logic
 * (mixed_rf_tx_handler_radio, emu_deliver_bytes, …) still lives in the
 * runner and migrates in M9.3/9.4; this header is the data model they
 * will move onto.  See docs/design/refactor-plan.md §3.9 / §Phase 5.
 */
#ifndef SIM_RADIO_BUS_H
#define SIM_RADIO_BUS_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SIM_RADIO_BUS_MAX_NODES 128

/* RF byte buffer for deferred delivery */
#define RF_BUF_SIZE 4096
typedef struct {
    uint8_t bytes[RF_BUF_SIZE];
    int count;
} rf_buffer_t;

/* TX frame assembler: detects complete frame boundaries in a sender's
 * raw byte stream. Two on-air formats are supported:
 *
 *   - 802.15.4 (CC2420 / cc2538_rfcore): 4×0x00 preamble + 0x7A SFD +
 *     1-byte length (PHY hdr) + payload.
 *   - 802.15.4g (CC1200): 4×0x55 preamble + 32-bit sync word
 *     0x6E4E904E + 2-byte PHR (length = (phra & 0x07)<<8 | phrb) +
 *     payload.
 *
 * The state machine sniffs which one it's looking at on the fly, since
 * the CC2420 callback and the CC1200 callback both feed the same
 * TX handler (per the per-node fan-out). */
#define TX_ASM_PREAMBLE   0
#define TX_ASM_LENGTH     1
#define TX_ASM_PAYLOAD    2
#define TX_ASM_SUBGHZ_PHR 3   /* CC1200: past sync word, expecting 2-byte PHR */

typedef struct {
    int state;
    int zero_count;        /* 0x00 preamble bytes (802.15.4) */
    uint32_t sync_match;   /* sliding 32-bit register for 802.15.4g sync */
    int phr_lo;            /* CC1200: top 3 bits of length stashed here */
    int expected_len;      /* PHY length value (PHR for CC1200) */
    int payload_count;     /* bytes received after length byte */
    bool subghz;           /* true once sync word matched (sub-GHz frame) */
    int subghz_phr_len;    /* CC1200 PHR width: 1 or 2 bytes */
    int64_t first_byte_ns; /* sim time of first preamble byte */
} tx_frame_asm_t;

typedef struct {
    uint8_t bytes[256];
    int len;
} tx_frame_capture_t;

/* Per-receiver deferred RX frame queue.  64 holds ~1 second of
 * full-rate sub-GHz traffic, comfortably more than the firmware's
 * worst-case process latency. */
#define EMU_RX_QUEUE_SIZE 64
#define EMU_RX_FRAME_MAX  160  /* preamble(4)+SFD(1)+len(1)+payload(≤127)+CRC(2) */

typedef struct {
    uint8_t data[EMU_RX_FRAME_MAX];
    int     len;
    int8_t  rssi;
    int64_t arrival_ns;
    int64_t end_ns;
    bool    collided;
    bool    subghz;     /* CC1200 frame — use 160 µs/byte for re-delivery */
} emu_rx_frame_t;

typedef struct {
    emu_rx_frame_t frames[EMU_RX_QUEUE_SIZE];
    int head, count;
} emu_rx_queue_t;

/* Per-sender RF outgoing buffer (threaded mode: written only by the
 * sender's thread) */
typedef struct {
    uint8_t bytes[RF_BUF_SIZE];
    int count;
} rf_outgoing_t;

/* The bus: all RF routing state, one instance per runtime. */
typedef struct sim_radio_bus {
    rf_buffer_t        rf_pending[SIM_RADIO_BUS_MAX_NODES];   /* per-receiver byte staging   */
    tx_frame_asm_t     tx_asm[SIM_RADIO_BUS_MAX_NODES];       /* per-sender frame assembler  */
    tx_frame_capture_t tx_cap[SIM_RADIO_BUS_MAX_NODES];       /* sender-side frame capture   */
    emu_rx_queue_t     emu_rx_queue[SIM_RADIO_BUS_MAX_NODES]; /* per-receiver deferred queue */
    int64_t            emu_rx_end_ns[SIM_RADIO_BUS_MAX_NODES];/* last RX end per receiver    */
    rf_outgoing_t      rf_outgoing[SIM_RADIO_BUS_MAX_NODES];  /* threaded-mode TX staging    */
} sim_radio_bus_t;

/* 802.15.4 byte duration at 250 kbps (2.4 GHz CC2420 / CC2538 RFCore) =
 * 32 µs/byte. 802.15.4g over CC1200 at 50 kbps = 160 µs/byte. Using the
 * 2.4 GHz value for CC1200 frames (5× too fast) made hidden-terminal
 * collisions look real and starved RPL convergence — see
 * devices/zoul-firefly/SPEC.md L6. */
#define IEEE802154_BYTE_NS    32000LL
#define CC1200_50KBPS_BYTE_NS 160000LL

static inline int64_t byte_period_ns(bool subghz) {
    return subghz ? CC1200_50KBPS_BYTE_NS : IEEE802154_BYTE_NS;
}

/* FIFO bytes a receiving chip needs free to accept a buffered on-air
 * frame; sentinel > any FIFO size when the buffer is too short to read
 * the length field (caller queues the frame instead). */
int frame_fifo_bytes(const uint8_t *data, int len, bool subghz);

#ifdef __cplusplus
}
#endif

#endif /* SIM_RADIO_BUS_H */
