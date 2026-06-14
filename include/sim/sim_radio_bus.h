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
    /* Optional (M9.5): the bus saw no RF byte for this mote within
     * SIM_RADIO_RX_STALL_NS of sim time after the last delivered byte.
     * If the radio is still mid-frame, the stream died (collision /
     * aborted sender) — abandon the frame the way real HW's signal-loss
     * detector would.  NULL when the platform has no stalled-parser
     * hazard; the bus only arms the timer when this op is set. */
    void (*rx_stall)(void *mote);
    /* Optional (M28): pull-model channel for motes with no push callback
     * (native Cooja reads simRadioChannel).  Returns the current channel,
     * or a value < -1 ("no channel reported") that the bus ignores.  The
     * bus syncs it into the medium before filtering.  NULL for motes that
     * push their channel synchronously through a chip callback. */
    int (*current_channel)(void *mote);
    /* Optional (M28): mark this mote's own queued RX frames that overlap
     * [start_ns, end_ns) as collided; returns the number newly marked.
     * Only motes that keep their RX queue mote-side (native Cooja) set
     * this; the bus marks its own emu_rx_queue directly for others. */
    int (*mark_collisions)(void *mote, int64_t start_ns, int64_t end_ns);
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

/* Per-receiver/per-sender delivery capability flags (M25).  These
 * encode platform RF-delivery quirks the runner previously expressed
 * as `nodes[].type == NODE_MSP430` checks, so the delivery code can be
 * type-blind ahead of moving into the bus.  Each src/motes module sets
 * its own caps at registration; today only the MSP430 module sets any.
 *
 *   RX_TICKING_STEP   — in the synchronous RX path, when the receiver
 *                       is the currently-executing node, step it ~1 ms
 *                       so the chip's symbol/ACK work runs in-line.
 *   DRAIN_MINI_STEP   — in the RX-queue drain, when the RXFIFO is full,
 *                       mini-step the receiver (up to 4×) to drain it.
 *   WAKE_SENDER_POST_TX — after a frame completes, schedule the sender's
 *                       own wakeup so its radio finishes the TX→RX turn. */
#define SIM_RADIO_CAP_RX_TICKING_STEP    (1u << 0)
#define SIM_RADIO_CAP_DRAIN_MINI_STEP    (1u << 1)
#define SIM_RADIO_CAP_WAKE_SENDER_POST_TX (1u << 2)

/* Host (runner) hooks the TX path calls out to (M9.4).  These cover the
 * pieces that stay runner-side until Phase 2: node lifecycle, native
 * channel pulls, debug stats, and the frame-complete delivery machinery
 * (which mini-steps receivers — forbidden inside the bus per §3.9).
 * Optional hooks may be NULL; node_active and frame_complete are
 * required for a functional bus. */
/* Per-frame context shared by the M27 observability hooks. */
typedef struct sim_radio_frame_info {
    int      sender_idx;
    int      sender_radio;
    int      expected_len;     /* tx_asm.expected_len (PHY length value)  */
    int      total_air_bytes;  /* preamble..payload(+CRC) byte budget     */
    bool     subghz;
    int64_t  tx_start_ns;      /* accurate frame air-time start           */
    int64_t  tx_end_ns;        /* accurate frame air-time end             */
    int64_t  air_dur_ns;       /* tx_end - tx_start                       */
    int64_t  sender_byte_ns;   /* per-byte air period for this frame      */
    const uint8_t *capture;    /* sender-side captured on-air bytes       */
    int      capture_len;
} sim_radio_frame_info_t;

/* on_rx_frame outcome codes (the bus's per-receiver delivery decision). */
typedef enum sim_radio_rx_outcome {
    SIM_RADIO_RX_STALE_BUFFER = 0, /* staged byte count != frame size; dropped */
    SIM_RADIO_RX_COLLIDED,         /* overlapped a prior frame; dropped        */
    SIM_RADIO_RX_DIRECT,           /* delivered synchronously                  */
    SIM_RADIO_RX_QUEUED,           /* deferred to the emu RX queue             */
} sim_radio_rx_outcome_t;

typedef struct sim_radio_bus_host {
    void *user;
    /* False while a node hasn't reached its startup-delay time (no RF). */
    bool (*node_active)(void *user, int idx);
    /* (M28: the native channel pull moved to the current_channel mote
     * op + the bus's bus_sync_channel; the sync_channel hook is gone.) */
    /* Optional: one call per TX byte before dispatch.  depth > 0 means
     * the byte re-entered from a receiver's auto-ACK. */
    void (*on_tx_byte)(void *user, int sender, uint8_t byte,
                       int64_t byte_time_ns, int depth);
    /* Optional: one call per (sender, receiver) byte the medium accepted
     * for a non-SYNC receiver (debug stats/traces). */
    void (*on_byte_accepted)(void *user, int sender, int receiver,
                             uint8_t byte, int depth);
    /* M27 frame-completion observability (the delivery policy moved into
     * sim_radio_bus_frame_complete; these notify the runner so the
     * TX/RX timeline, PCAP, packet-analyzer prints, and traces stay
     * runner-side until Phase 6).  Defined below the host struct.
     * frame_observed: once per completed frame, before delivery.
     * on_rx_frame:    once per emulated receiver, with the bus's outcome.
     * on_ack:         the data delivery to a receiver triggered an ACK. */
    void (*frame_observed)(void *user, const struct sim_radio_frame_info *fi);
    void (*on_rx_frame)(void *user, const struct sim_radio_frame_info *fi,
                        int receiver, int outcome,
                        const uint8_t *data, int len,
                        int64_t coll_start_ns, int64_t prev_rx_end_ns);
    void (*on_ack)(void *user, const struct sim_radio_frame_info *fi,
                   int data_receiver, int64_t ack_start_ns, int ack_byte_count);
    /* M26: the bus delivered/queued a frame's worth of on-air bytes to
     * emulated receiver `idx` occupying the air window [start_ns,
     * end_ns].  The runner emits the RX timeline + radio state from
     * this (observability stays runner-side until Phase 6). */
    void (*on_rx)(void *user, int idx, int64_t start_ns, int64_t end_ns);
} sim_radio_bus_host_t;

/* Bus-owned RF delivery counters (M26): incremented by the emulated-RX
 * delivery/queue paths the bus now owns; the runner reads them at end of
 * run and zeroes them per sim restart. */
typedef struct sim_radio_bus_stats {
    int rx_direct;    /* frames delivered synchronously               */
    int rx_queued;    /* frames pushed to an emu RX queue (RXFIFO busy)*/
    int rx_drained;   /* frames delivered out of an emu RX queue       */
    int rx_dropped;   /* frames dropped because the queue was full     */
    int rx_collided;  /* frames dropped due to an RF collision         */
    /* M28: frame-level (native/JS) RX path counters (ex the runner's
     * stat_rx_frames_*). */
    int frame_queued;     /* frames delivered/queued to a native/JS mote */
    int frame_queue_full; /* of those, the mote's RX queue was full      */
    int frame_collided;   /* native queued frames marked collided        */
} sim_radio_bus_stats_t;

/* The bus: all RF routing state, one instance per runtime. */
typedef struct sim_radio_bus {
    const mote_radio_ops_t *ops[SIM_RADIO_BUS_MAX_NODES];
    void                   *mote[SIM_RADIO_BUS_MAX_NODES];
    sim_radio_delivery_mode_t delivery[SIM_RADIO_BUS_MAX_NODES];
    uint32_t                caps[SIM_RADIO_BUS_MAX_NODES];   /* SIM_RADIO_CAP_* */
    int                node_count;   /* registration high-water mark + 1   */
    int                tx_depth;     /* >0: inside frame_complete delivery */
    sim_radio_bus_host_t host;       /* runner hooks (M9.4)                */
    rf_buffer_t        rf_pending[SIM_RADIO_BUS_MAX_NODES];   /* per-receiver byte staging   */
    tx_frame_asm_t     tx_asm[SIM_RADIO_BUS_MAX_NODES];       /* per-sender frame assembler  */
    tx_frame_capture_t tx_cap[SIM_RADIO_BUS_MAX_NODES];       /* sender-side frame capture   */
    emu_rx_queue_t     emu_rx_queue[SIM_RADIO_BUS_MAX_NODES]; /* per-receiver deferred queue */
    int64_t            emu_rx_end_ns[SIM_RADIO_BUS_MAX_NODES];/* last RX end per receiver    */
    /* RX-stall timer bookkeeping (M9.5): one lazy SIM_EV_RADIO_TIMER
     * pending per receiver, deadline extended per delivered byte. */
    int64_t            rx_stall_deadline_ns[SIM_RADIO_BUS_MAX_NODES];
    bool               rx_stall_pending[SIM_RADIO_BUS_MAX_NODES];
    /* M26: the node whose CPU is currently executing (was the runner's
     * ticking_node_idx).  The synchronous RX path skips per-byte clock
     * sync for this node to avoid a re-entrant step.  -1 = none. */
    int                executing_node;
    sim_radio_bus_stats_t stats;
    /* M27: per-sender medium-busy deadline (ex node_tx_busy_until_ns) —
     * a frame occupies the sender's channel until this sim time; CCA
     * queries (cc1200) read it. */
    int64_t            tx_busy_until_ns[SIM_RADIO_BUS_MAX_NODES];
    /* M27: true while frame_complete is delivering synchronously, so the
     * runner suppresses async chip state callbacks (ex
     * suppress_state_callback). */
    bool               in_delivery;
} sim_radio_bus_t;

/* Register node idx's radio endpoint + delivery mode + capability
 * flags (SIM_RADIO_CAP_*).  Call once per node after platform init,
 * before the main loop (re-called on reboot, so caps re-apply). */
void sim_radio_bus_register(sim_radio_bus_t *bus, int idx,
                            const mote_radio_ops_t *ops, void *mote,
                            sim_radio_delivery_mode_t mode, uint32_t caps);

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

/* Called by the runner when a SIM_EV_RADIO_TIMER pops for node_idx.
 * Returns true when the receiver's RX-stall deadline truly expired —
 * the caller should sync the mote's CPU to time_ns and invoke
 * ops->rx_stall.  Returns false when fresher RF bytes extended the
 * deadline; the bus re-arms the timer and the popped event is stale. */
bool sim_radio_bus_rx_stall_expired(sim_radio_bus_t *bus,
                                    struct sim_runtime *sim,
                                    int node_idx, int64_t time_ns);

/* Clear per-node bus state on mote re-init/reboot (pairs with the
 * runner's sim_cancel_mote_events purge, which also drops any pending
 * SIM_EV_RADIO_TIMER for the slot). */
void sim_radio_bus_reset_node(sim_radio_bus_t *bus, int idx);

/* M26: mark the node whose CPU is currently executing (-1 = none).
 * The runner's execute adapters + serial-injection path set this around
 * any CPU step so the synchronous RX delivery path can avoid a
 * re-entrant step for that node. */
void sim_radio_bus_set_executing(sim_radio_bus_t *bus, int idx);

/* ============================================================
 * Emulated-receiver RX delivery (M26 — moved from the runner).
 *
 * These own the byte-burst delivery to an emulated mote's radio, the
 * deferred-RX queue, and the queue drain.  CPU motion goes through the
 * mote's sim_mote_ops_t (rx_byte_sync / step_until / cycles / freq_hz)
 * via sim_runtime_mote(); the runner stays responsible only for the
 * RX timeline/state (the on_rx host hook) and the SIM_EV_RX_BYTE event
 * dispatch (deliver_rx_byte) whose CC2420 trace needs chip headers.
 * ============================================================ */

/* Push a complete frame into emulated node idx's deferred RX queue
 * (no collision detection here — the caller owns that). */
void sim_radio_bus_queue_frame(sim_radio_bus_t *bus, int idx,
                               const uint8_t *data, int len, int8_t rssi,
                               int64_t arrival_ns, int64_t end_ns,
                               bool subghz);

/* Deliver a frame's on-air bytes to emulated node idx's radio, spaced
 * on the air-time byte clock, pre-syncing the receiver's CPU per byte;
 * queues instead if the radio is mid-frame.  No-op for non-emulated
 * receivers (rx_byte_sync == NULL). */
void sim_radio_bus_deliver_bytes(sim_radio_bus_t *bus, struct sim_runtime *sim,
                                 int idx, const uint8_t *data, int len,
                                 int8_t rssi, int64_t air_time_ns,
                                 bool subghz);

/* Deliver at most one queued frame to emulated node idx (skips collided
 * frames; mini-steps the receiver to free RXFIFO when CAP_DRAIN_MINI_STEP
 * is set). */
void sim_radio_bus_drain_rx(sim_radio_bus_t *bus, struct sim_runtime *sim,
                            int idx);

/* M27: schedule node idx's next wakeup after RF activity — re-run at the
 * current sim time if it has queued RX frames, else at its next CPU
 * event (mote sched_hint_ns op).  Ex the runner's schedule_emulated_wakeup;
 * emulated motes only. */
void sim_radio_bus_wake_mote(sim_radio_bus_t *bus, struct sim_runtime *sim,
                             int idx);

/* M28: deliver a complete frame from a native/JS sender to its
 * frame-consuming neighbours (the ones with a receive_frame mote op) and
 * mark interference-range collisions.  Ex the runner's
 * mixed_rf_frame_handler delivery loop; the runner wrapper keeps the
 * TX-side bookkeeping (stat_rf_frames, channels_dirty, last_tx_ns). */
void sim_radio_bus_tx_frame(sim_radio_bus_t *bus, struct sim_runtime *sim,
                            int sender_idx, const uint8_t *frame, int len);

/* M28: push a (radio_idx, channel) change for node idx into the medium —
 * auto-registers the slot's spectrum on first push (slot 0 = 2.4 GHz,
 * slot 1 = sub-GHz) and mirrors the legacy single-channel alias the CCA
 * query still reads.  Ex the runner's mixed_node_radio_set_channel body.
 * The runner wrapper keeps the idx bound + the trace. */
void sim_radio_bus_push_channel(sim_radio_bus_t *bus, struct sim_runtime *sim,
                                int idx, int radio_idx, int channel);

/* 802.15.4 byte duration at 250 kbps (2.4 GHz CC2420 / CC2538 RFCore) =
 * 32 µs/byte. 802.15.4g over CC1200 at 50 kbps = 160 µs/byte. Using the
 * 2.4 GHz value for CC1200 frames (5× too fast) made hidden-terminal
 * collisions look real and starved RPL convergence — see
 * devices/zoul-firefly/SPEC.md L6. */
#define IEEE802154_BYTE_NS    32000LL
#define CC1200_50KBPS_BYTE_NS 160000LL

/* RX-stall window (M9.5): sim-time after a receiver's last delivered RF
 * byte before ops->rx_stall fires.  Must exceed the inter-byte air gap
 * (32 µs at 250 kbps) with margin for senders that emit a frame's bytes
 * incrementally across their own execution slices; 200 µs (~6 byte
 * periods) is generous on both counts.  This replaces the nrf54l15
 * chip-internal cpu-cycle watchdog whose 50 ms band-aid existed only
 * because the receiver's local cycle clock stands still while a
 * synchronously-emitted frame traverses the parser — global sim time
 * does not have that problem. */
#define SIM_RADIO_RX_STALL_NS 200000LL

static inline int64_t byte_period_ns(bool subghz) {
    return subghz ? CC1200_50KBPS_BYTE_NS : IEEE802154_BYTE_NS;
}

/* Window (sim ns) over which a completing frame can collide with queued
 * frames on interference-range neighbours (M27 — the runner's 1 ms
 * TIME_STEP_NS, kept as the interference overlap span). */
#define SIM_RADIO_INTERFERENCE_WINDOW_NS 1000000LL

/* FIFO bytes a receiving chip needs free to accept a buffered on-air
 * frame; sentinel > any FIFO size when the buffer is too short to read
 * the length field (caller queues the frame instead). */
int frame_fifo_bytes(const uint8_t *data, int len, bool subghz);

#ifdef __cplusplus
}
#endif

#endif /* SIM_RADIO_BUS_H */
