/*
 * Co-simulation interface — communication layer for external simulation
 * coordinators.
 *
 * Protocol: JSON over 4-byte big-endian length-prefixed TCP.
 * Enables external tools to control Cooja-NG's time stepping and
 * route all radio traffic through an external channel model.
 *
 * Messages (coordinator → Cooja-NG):
 *   time_advance  — deterministic time step
 *   step_to       — DES time step
 *   run_until     — run at full speed until target time (pause on TX)
 *   continue      — resume after TX pause (after RX injections)
 *   rx            — inject RX packet into node
 *   sim_control   — shutdown
 *
 * Messages (Cooja-NG → coordinator):
 *   init          — handshake with node info
 *   tx            — node transmitted a frame
 *   tx_done       — TX occurred during run_until (pause point)
 *   time_ack      — time step complete
 *   node_idle     — DES per-node idle report
 *   console       — per-node UART output
 */
#ifndef COSIM_H
#define COSIM_H

#include <stdint.h>
#include <stdbool.h>

#define COSIM_MAX_NODES     128
#define COSIM_MAX_RX_QUEUE  16
#define COSIM_MAX_FRAME_LEN 160
#define COSIM_MAX_TIMERS    16
#define COSIM_NODE_ID_LEN   64

/* Command types from coordinator */
typedef enum {
    COSIM_CMD_NONE = 0,
    COSIM_CMD_TIME_ADVANCE,   /* deterministic mode */
    COSIM_CMD_STEP_TO,        /* DES mode */
    COSIM_CMD_STOP,           /* shutdown */
    COSIM_CMD_RUN_UNTIL,      /* run at full speed until target (pause on TX) */
    COSIM_CMD_CONTINUE        /* resume after TX pause */
} cosim_cmd_type_t;

/* Time command from coordinator */
typedef struct {
    cosim_cmd_type_t type;
    int64_t target_time_ns;
    int64_t quantum_ns;
} cosim_time_cmd_t;

/* RX packet to inject into a node */
typedef struct {
    int     node_index;
    int     source_index;  /* sender node index (for local ACK delivery, -1 if unknown) */
    uint8_t payload[COSIM_MAX_FRAME_LEN];
    int     payload_len;
    int8_t  rssi_dbm;
    int     channel;
} cosim_rx_inject_t;

/* Per-node info for init handshake */
typedef struct {
    char    node_id[COSIM_NODE_ID_LEN];
    int     node_index;
} cosim_node_info_t;

/*
 * Initialize co-simulation: connect to coordinator as TCP client.
 * Returns 0 on success, -1 on error.
 */
int cosim_init(const char *addr, int port);

/*
 * Close the co-simulation connection.
 */
void cosim_close(void);

/*
 * Send init handshake with node list.
 * Called once after connection, before main loop.
 */
int cosim_send_init(const cosim_node_info_t *nodes, int node_count);

/*
 * Send TX event: a node transmitted a frame.
 * payload is the raw 802.15.4 frame (without preamble/SFD).
 */
int cosim_send_tx(int node_index, const char *node_id,
                  const uint8_t *payload, int len,
                  int tx_power_dbm, int channel,
                  uint64_t timestamp_ns);

/*
 * Send time_ack: all nodes have advanced to target time (deterministic mode).
 */
int cosim_send_time_ack(int64_t target_time_ns);

/*
 * Send per-node node_idle report (DES mode).
 * timer_times_ns: array of next scheduled timer times (nanoseconds).
 */
int cosim_send_node_idle(const char *node_id,
                         const uint64_t *timer_times_ns,
                         int timer_count);

/*
 * Send tx_done: a TX occurred during run_until, pausing at current time.
 * The coordinator will inject RX packets and send continue.
 */
int cosim_send_tx_done(int64_t time_ns);

/*
 * Send per-node console output (UART line).
 */
int cosim_send_console(const char *node_id, const char *text,
                       uint64_t timestamp_ns);

/*
 * Block waiting for next command from coordinator.
 * Fills cmd with the command type and timing info.
 * Fills rx_queue with any RX packets to inject (up to rx_max).
 * Returns number of RX packets, or -1 on error.
 */
int cosim_wait_command(cosim_time_cmd_t *cmd,
                       cosim_rx_inject_t *rx_queue, int rx_max);

#endif /* COSIM_H */
