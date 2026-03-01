/*
 * Native Cooja mote support — dlopen-based Contiki-NG node execution
 *
 * Loads TARGET=cooja firmware as shared libraries. Each node gets an
 * independent copy (dlopen with RTLD_LOCAL) and is advanced via
 * cooja_tick() calls with simCurrentTime/simRtimerCurrentTicks updated
 * to match the global simulation time.
 *
 * Copyright (c) 2024, All rights reserved.
 * BSD 3-clause license.
 */
#ifndef NATIVE_NODE_H
#define NATIVE_NODE_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward-declare the frame queue used for native-to-native delivery */
typedef struct native_pending_frame {
    uint8_t data[128];
    int     len;
    bool    valid;
} native_pending_frame_t;

/* State machine for reassembling byte-stream into 802.15.4 frames */
typedef enum {
    RX_ASM_PREAMBLE = 0,
    RX_ASM_SFD,
    RX_ASM_LENGTH,
    RX_ASM_PAYLOAD
} rx_asm_state_t;

typedef struct {
    uint8_t       buf[256];
    int           count;
    rx_asm_state_t state;
    int           zero_count;
    int           expected_len;  /* PHY length byte (payload + FCS) */
} native_rx_assembler_t;

/* Main native node struct */
typedef struct native_node {
    void *dl_handle;
    char  dl_path[256];
    bool  dl_path_is_temp;

    /* Function pointers (dlsym) */
    int  (*cooja_init)(void);
    void (*cooja_tick)(void);

    /* Pointers to shared variables (dlsym) */
    uint64_t *simCurrentTime;             /* ms ticks (CLOCK_SECOND=1000) */
    uint64_t *simRtimerCurrentTicks;      /* us ticks (RTIMER_ARCH_SECOND=1M) */
    int      *simEtimerPending;
    uint64_t *simEtimerNextExpirationTime;
    int      *simRtimerPending;
    uint64_t *simRtimerNextExpirationTime;
    int      *simProcessRunValue;
    char     *simInDataBuffer;            /* RX frame buffer [128] */
    int      *simInSize;
    char     *simOutDataBuffer;           /* TX frame buffer [128] */
    int      *simOutSize;
    char     *simRadioHWOn;
    int      *simRadioChannel;
    char     *simReceiving;
    char     *simLoggedData;
    int      *simLoggedLength;
    char     *simLoggedFlag;
    int      *simMoteID;
    char     *simMoteIDChanged;
    int      *simRandomSeed;

    /* Simulation state */
    int64_t  sim_time_ns;
    int      node_id;

    /* Radio state */
    native_pending_frame_t pending_rx_frame;   /* queued frame for delivery */
    native_rx_assembler_t  rx_asm;             /* byte-stream reassembler */

    /* UART/log callback */
    void (*log_callback)(void *user_data, uint8_t byte);
    void *log_callback_data;

    /* RF TX callback (for byte-stream bridging to emulated nodes) */
    void (*rf_tx_callback)(void *user_data, uint8_t byte);
    void *rf_tx_callback_data;

    /* RF frame callback (for native-to-native frame delivery) */
    void (*rf_frame_callback)(void *user_data, const uint8_t *frame, int len);
    void *rf_frame_callback_data;
} native_node_t;

/* --- Public API --- */

/* Initialize a native node: copy firmware, dlopen, resolve symbols, cooja_init */
int native_node_init(native_node_t *node, const char *firmware_path, int node_id);

/* Destroy a native node: dlclose, remove temp file */
void native_node_destroy(native_node_t *node);

/* Step the node forward until target_ns (calls cooja_tick as needed) */
void native_step_until_ns(native_node_t *node, int64_t target_ns);

/* Check and drain log output (calls log_callback for each byte) */
void native_check_log_output(native_node_t *node);

/* Check and handle TX output after a tick */
void native_check_radio_tx(native_node_t *node);

/* Deliver a complete frame to this native node (native-to-native) */
void native_deliver_frame(native_node_t *node, const uint8_t *frame, int len);

/* Feed a byte from emulated radio byte-stream into the reassembler */
void native_rx_assembler_feed(native_node_t *node, uint8_t byte);

/* --- Radio bridging helpers (in native_radio.c) --- */

/* Convert a native frame to 802.15.4 byte stream with preamble/SFD/FCS */
int native_frame_to_bytes(const uint8_t *frame, int frame_len,
                          uint8_t *out, int out_max);

/* Reset the byte-to-frame reassembler */
void native_rx_assembler_reset(native_rx_assembler_t *asm_state);

/* CRC-CCITT-16 with bit reversal (same as CC2420) */
uint16_t native_crc16(const uint8_t *data, int len);

#ifdef __cplusplus
}
#endif

#endif /* NATIVE_NODE_H */
