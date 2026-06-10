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

#include "radio_medium.h"

#ifdef __cplusplus
extern "C" {
#endif

struct sim_runtime;

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

/* Mote radio endpoint ops (M9.3) — the bus delivers RF bytes through
 * these instead of switching on node type.  Registered per node by the
 * runner at init; folds into the full mote vtable in Phase 2. */
typedef struct mote_radio_ops {
    /* Deliver one on-air byte to every radio on this mote (multi-radio
     * motes like Firefly fan out internally; chips whose framer doesn't
     * recognize the preamble ignore the byte). */
    void (*receive_byte)(void *mote, uint8_t byte, int8_t rssi);
    /* Free RXFIFO bytes on the most-constrained radio; the bus queues
     * frames that don't fit. */
    int  (*rxfifo_available)(void *mote);
    /* True while a frame is mid-reception (delivery now would corrupt
     * it; the bus queues instead). */
    bool (*rx_busy)(void *mote);
} mote_radio_ops_t;

/* How TX bytes reach a registered receiver (M9.4).  Chosen by the runner
 * at registration time from platform knowledge:
 *   SYNC     — native (Cooja-protocol) motes: feed receive_byte
 *              synchronously from the dispatch loop (frame assembly
 *              happens mote-side).
 *   PER_BYTE — chips with an rx_incoming buffer + state guard (CC2420,
 *              cc2538_rfcore, nrf54l15): one SIM_EV_RX_BYTE kernel event
 *              per on-air byte at its air time.
 *   BATCH    — chips that need whole frames (nrf52840's DMA-style RADIO
 *              writes straight to PACKETPTR; per-byte regressed 4-node
 *              RPL convergence): stage bytes in rf_pending[], the host's
 *              frame_complete hook delivers the assembled frame. */
typedef enum sim_radio_delivery_mode {
    SIM_RADIO_DELIVERY_NONE = 0,
    SIM_RADIO_DELIVERY_SYNC,
    SIM_RADIO_DELIVERY_PER_BYTE,
    SIM_RADIO_DELIVERY_BATCH,
} sim_radio_delivery_mode_t;

/* Host (runner) hooks the TX path calls out to (M9.4).  These cover the
 * pieces that stay runner-side until Phase 2: node lifecycle, native
 * channel pulls, debug stats, and the frame-complete delivery machinery
 * (which mini-steps receivers — forbidden inside the bus per §3.9).
 * Optional hooks may be NULL; node_active and frame_complete are
 * required for a functional bus. */
typedef struct sim_radio_bus_host {
    void *user;
    /* False while a node hasn't reached its startup-delay time (no RF). */
    bool (*node_active)(void *user, int idx);
    /* Pull a native mote's channel into the medium before filtering
     * (chip motes push synchronously through their FSCTRL callbacks). */
    void (*sync_channel)(void *user, int idx);
    /* Optional: one call per TX byte before dispatch.  depth > 0 means
     * the byte re-entered from a receiver's auto-ACK. */
    void (*on_tx_byte)(void *user, int sender, uint8_t byte,
                       int64_t byte_time_ns, int depth);
    /* Optional: one call per (sender, receiver) byte the medium accepted
     * for a non-SYNC receiver (debug stats/traces). */
    void (*on_byte_accepted)(void *user, int sender, int receiver,
                             uint8_t byte, int depth);
    /* The sender's frame assembler completed a frame: deliver staged
     * rf_pending[] frames, run collision/FIFO policy, flush auto-ACKs.
     * Runs with the bus's tx_depth already raised, so chip TX callbacks
     * fired from inside (auto-ACKs) re-enter sim_radio_bus_tx_byte as
     * depth > 0 and are staged, not re-dispatched as frames. */
    void (*frame_complete)(void *user, int sender, int sender_radio,
                           int64_t byte_time_ns, int64_t sender_byte_ns);
} sim_radio_bus_host_t;

/* The bus: all RF routing state, one instance per runtime. */
typedef struct sim_radio_bus {
    const mote_radio_ops_t *ops[SIM_RADIO_BUS_MAX_NODES];
    void                   *mote[SIM_RADIO_BUS_MAX_NODES];
    sim_radio_delivery_mode_t delivery[SIM_RADIO_BUS_MAX_NODES];
    int                node_count;   /* registration high-water mark + 1   */
    int                tx_depth;     /* >0: inside frame_complete delivery */
    sim_radio_bus_host_t host;       /* runner hooks (M9.4)                */
    rf_buffer_t        rf_pending[SIM_RADIO_BUS_MAX_NODES];   /* per-receiver byte staging   */
    tx_frame_asm_t     tx_asm[SIM_RADIO_BUS_MAX_NODES];       /* per-sender frame assembler  */
    tx_frame_capture_t tx_cap[SIM_RADIO_BUS_MAX_NODES];       /* sender-side frame capture   */
    emu_rx_queue_t     emu_rx_queue[SIM_RADIO_BUS_MAX_NODES]; /* per-receiver deferred queue */
    int64_t            emu_rx_end_ns[SIM_RADIO_BUS_MAX_NODES];/* last RX end per receiver    */
    rf_outgoing_t      rf_outgoing[SIM_RADIO_BUS_MAX_NODES];  /* threaded-mode TX staging    */
} sim_radio_bus_t;

/* Register node idx's radio endpoint + delivery mode.  Call once per
 * node after platform init, before the main loop. */
void sim_radio_bus_register(sim_radio_bus_t *bus, int idx,
                            const mote_radio_ops_t *ops, void *mote,
                            sim_radio_delivery_mode_t mode);

/* Install the runner's host hooks (copied by value). */
void sim_radio_bus_set_host(sim_radio_bus_t *bus,
                            const sim_radio_bus_host_t *host);

/* TX entry point (M9.4, §3.9): chips/native/JS senders emit each on-air
 * byte here.  The bus arms the sender's byte clock, captures the byte,
 * dispatches it to every receiver the medium accepts (scheduling
 * SIM_EV_RX_BYTE / staging in rf_pending / feeding natives per the
 * receiver's delivery mode), feeds the sender's frame assembler, and
 * invokes the host's frame_complete hook when a frame ends.  Re-entrant
 * calls (auto-ACK bytes emitted during frame_complete) are dispatched
 * to receivers but never fed to the assembler — the outer
 * frame_complete flushes them. */
void sim_radio_bus_tx_byte(sim_radio_bus_t *bus, struct sim_runtime *sim,
                           int sender_idx, int sender_radio, uint8_t byte);

/* Per-sender on-air frame assembler.  feed() returns true when `byte`
 * completes a frame; sticky fields (subghz, expected_len,
 * subghz_phr_len) stay readable until the next frame's first preamble
 * byte. */
bool sim_radio_bus_asm_feed(tx_frame_asm_t *a, uint8_t byte);
void sim_radio_bus_asm_reset(tx_frame_asm_t *a);

/* Receiver radio slot matching a sender's (node, radio) emission per
 * the medium's spectrum registration, or -1 to drop (no slot on the
 * receiver shares the sender's spectrum). */
int sim_radio_bus_pick_receiver_radio(const radio_medium_t *medium,
                                      int sender_idx, int sender_radio,
                                      int receiver_idx);

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
