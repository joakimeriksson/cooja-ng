/*
 * sim_control — the one implementation of every live-simulation mutation.
 *
 * Before this module the runner carried three copies of "move / send /
 * remove / add / reboot a node" (the JSON action executor, the JS action
 * executor, and the WebSocket UI's move handler) plus a UI-private
 * pause/speed state.  The interactive shell (docs/shell.md) would have been
 * a fourth.  sim_control owns the *semantics* once — id→slot lookup, the
 * "reboot = destroy+init, re-seed the mote clock, open the start gate"
 * sequence, the "send = inject, then wake the CPU out of WFI" idiom, the
 * add-a-new-node path — on top of a small bundle of runner primitives.
 *
 * The runner still owns the node table and the RF-side per-slot state
 * (refactor-plan §3.4: storage stays with the registrant), so it populates
 * `sim_control_ops_t` with one thin adapter per primitive, exactly the
 * pattern the timeline/progress/UI services already use for describe/move
 * callbacks.  Everything above the bundle is runner-agnostic and unit-
 * testable against a mock bundle (test/test_shell.c).
 *
 * Run state: sim_control is the only writer of sim_runtime_t.run_state
 * PAUSED/RUNNING (the pump writes RUNNING on entry; sim_control_after_pump
 * re-asserts PAUSED when a run-for / step budget has been reached).  The
 * loop consults sim_control_paused() where it used to read the UI's flag.
 *
 * Determinism: nothing here reads the wall clock.  Wall-clock pacing is a
 * loop concern; this module only stores the target ratio.
 */
#ifndef SIM_CONTROL_H
#define SIM_CONTROL_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#include "sim_runtime.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Snapshot of one node slot, filled by the runner's describe primitive. */
typedef struct sim_control_node_info {
    int         index;        /* slot index                                 */
    int         id;           /* Cooja node id (what users type)            */
    const char *type;         /* mote-kind label: "MSP430", "ARM", ...      */
    const char *firmware;     /* firmware path the slot was booted from     */
    const char *secure_firmware; /* TrustZone-M Secure-world ELF, "" = none */
    double      x, y;         /* radio-medium position (metres)             */
    int64_t     start_ns;     /* start gate; INT64_MAX = removed            */
    bool        active;       /* started (now >= start_ns) and not removed  */
    bool        removed;
    int64_t     sim_time_ns;  /* the mote's own clock                       */
    int64_t     cycles;
    uint32_t    freq_hz;
} sim_control_node_info_t;

/* Runner-populated primitives.  Each wraps ONE runner static; none of them
 * carries policy.  `user` is passed back verbatim.  A NULL member means the
 * primitive is unavailable and the corresponding sim_control_* call fails
 * with -1 (the unit-test mock leaves the ones it doesn't need NULL). */
typedef struct sim_control_ops {
    void *user;
    /* Number of node slots in use (including removed ones). */
    int   (*node_count)(void *u);
    /* Fill `out` for slot idx; false if idx is out of range. */
    bool  (*describe)(void *u, int idx, sim_control_node_info_t *out);
    /* Console input into slot idx; returns bytes consumed (may be < len). */
    int   (*inject_serial)(void *u, int idx, const uint8_t *buf, int len);
    /* Set the slot's position AND recompute the medium's neighbour table. */
    void  (*set_position)(void *u, int idx, double x, double y);
    /* Destroy + re-initialize the slot from its own firmware (no clock
     * re-seed, no start gate — those are `start`).  0 on success. */
    int   (*reboot)(void *u, int idx);
    /* Make the slot live at the current sim time: re-seed the mote's clock
     * (ops->reset_time) and open its start gate (node_start_ns = now). */
    void  (*start)(void *u, int idx);
    /* Close the slot's start gate forever (node_start_ns = INT64_MAX).  The
     * slot keeps its id so a later reboot/add can revive it. */
    void  (*remove)(void *u, int idx);
    /* Create a NEW slot booted from `firmware` (plus an optional TrustZone-M
     * Secure-world image, NULL/"" = single image) with Cooja id node_id, and
     * grow every runner-side node count.  Returns the slot index or -1. */
    int   (*add)(void *u, const char *firmware, const char *secure_firmware,
                 int node_id);
    /* Config v2 mote-type name → firmware path, or NULL if unknown.  When
     * non-NULL, *secure_firmware receives the type's Secure-world image
     * (NULL or "" = none). */
    const char *(*firmware_for_type)(void *u, const char *type_name,
                                     const char **secure_firmware);
    /* sim_mote_ops_t.get_interface pass-through (SIM_MOTE_IFACE_*). */
    void *(*get_interface)(void *u, int idx, int iface);
    /* Write the LIVE configuration (the --save-config writer).  0 on ok. */
    int   (*save_config)(void *u, const char *path);
} sim_control_ops_t;

/* Flags for the mutation calls. */
enum {
    /* After the mutation, schedule a wakeup for the slot so its CPU leaves
     * WFI and services the console/boot promptly.  send: at now + 1 ms
     * (room for one execute slice, see the runner's CSIM_NODE<n>_INPUT
     * path); reboot/add: at now.  The JSON action executor passes 0 —
     * it never woke the node, and existing configs stay byte-identical. */
    SIM_CONTROL_WAKE  = 1u << 0,
    /* send: keep bytes the mote did not consume and retry them on the
     * next sim_control_send_flush_pending() (MSP430 baud-paced injection
     * returns partial counts).  Without it the remainder is dropped, as
     * the JSON/JS executors always did. */
    SIM_CONTROL_RETRY = 1u << 1,
    /* add: also `start` the new slot (clock re-seed + start gate) so it
     * runs from now.  The JS executor omits it because it reboots the new
     * slot right afterwards (historical behaviour, kept byte-identical). */
    SIM_CONTROL_START = 1u << 2,
};

#define SIM_CONTROL_PENDING_MAX 512

typedef struct sim_control {
    sim_runtime_t     *sim;
    sim_control_ops_t  ops;

    /* Wall-clock pacing target: sim seconds per wall second.  0 = unpaced
     * (run as fast as possible).  Only stored here; the loop paces. */
    double             speed_ratio;

    /* Auto-pause horizon for `run <duration>`: INT64_MAX = none.  The loop
     * clamps its slice to it (sim_control_slice_cap) and after_pump pauses
     * once now_ns reaches it. */
    int64_t            pause_at_ns;
    /* `step N`: dispatched-event budget; > 0 means pause when it hits 0.
     * The loop's dispatcher calls sim_control_note_event() per event. */
    int                step_events_left;
    bool               step_armed;

    /* Per-slot unconsumed console bytes (SIM_CONTROL_RETRY). */
    struct {
        uint8_t buf[SIM_CONTROL_PENDING_MAX];
        int     len;
    }                  pending[SIM_EQ_MAX_NODES];
} sim_control_t;

void sim_control_init(sim_control_t *c, sim_runtime_t *sim,
                      const sim_control_ops_t *ops);

/* --- node lookup ------------------------------------------------------- */

/* Slot index for a Cooja node id (removed slots included), or -1. */
int  sim_control_index_of_id(const sim_control_t *c, int node_id);
int  sim_control_node_count(const sim_control_t *c);
bool sim_control_describe(const sim_control_t *c, int idx,
                          sim_control_node_info_t *out);
/* True if the slot is started and not removed at the current sim time. */
bool sim_control_node_active(const sim_control_t *c, int idx);

/* --- mutations (all take Cooja node ids; return 0 on success) ---------- */

int  sim_control_move(sim_control_t *c, int node_id, double x, double y);
int  sim_control_remove(sim_control_t *c, int node_id);
/* Destroy + re-init the slot, re-seed its clock, open its start gate. */
int  sim_control_reboot(sim_control_t *c, int node_id, unsigned flags);
/* Create a new slot.  `fw_or_type` is a firmware path or a config v2
 * mote-type name (whose Secure-world image, if any, is used); for a path,
 * `secure_fw` (may be NULL) names the Secure-world image.  x/y are applied
 * when either is non-zero (the JS executor's rule).  Returns the new slot
 * index or -1. */
int  sim_control_add(sim_control_t *c, const char *fw_or_type,
                     const char *secure_fw, int node_id,
                     double x, double y, unsigned flags);
/* Console input into one node.  Returns bytes consumed (or queued when
 * SIM_CONTROL_RETRY), -1 if the id is unknown. */
int  sim_control_send(sim_control_t *c, int node_id, const uint8_t *buf,
                      int len, unsigned flags);
/* Console input into every ACTIVE slot, ascending slot order. */
void sim_control_send_all(sim_control_t *c, const uint8_t *buf, int len,
                          unsigned flags);
/* Retry SIM_CONTROL_RETRY leftovers.  Call once per loop iteration, from
 * the outer loop (never from an observer: MSP430 injection steps the CPU). */
void sim_control_send_flush_pending(sim_control_t *c);
/* Bytes still queued for a slot (0 when nothing is pending). */
int  sim_control_pending_len(const sim_control_t *c, int idx);

/* --- run state --------------------------------------------------------- */

void sim_control_pause(sim_control_t *c);
void sim_control_resume(sim_control_t *c);          /* clears run-for/step */
/* Resume and pause again once now_ns >= now + dur_ns. */
void sim_control_run_for(sim_control_t *c, int64_t dur_ns);
/* Resume and pause again after n dispatched events (n >= 1). */
void sim_control_step_events(sim_control_t *c, int n);
bool sim_control_paused(const sim_control_t *c);
static inline int64_t sim_control_pause_at(const sim_control_t *c) {
    return c->pause_at_ns;
}
/* Request the run to end (sim_runtime_request_stop); sticky. */
void sim_control_request_exit(sim_control_t *c);

void   sim_control_set_speed(sim_control_t *c, double ratio);  /* 0 = max */
static inline double sim_control_speed(const sim_control_t *c) {
    return c->speed_ratio;
}
static inline bool sim_control_pacing(const sim_control_t *c) {
    return c->speed_ratio > 0.0;
}

/* --- loop hooks -------------------------------------------------------- */

/* Clamp a slice horizon to the run-for target. */
int64_t sim_control_slice_cap(const sim_control_t *c, int64_t cap_ns);
/* The loop's dispatcher reports each dispatched event (step budget). */
void    sim_control_note_event(sim_control_t *c);
/* After sim_runtime_run_until(): re-assert PAUSED when a run-for horizon
 * or step budget has been reached.  Returns true if it paused. */
bool    sim_control_after_pump(sim_control_t *c);

#ifdef __cplusplus
}
#endif

#endif /* SIM_CONTROL_H */
