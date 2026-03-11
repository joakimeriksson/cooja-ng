/*
 * Simulation timeline event logger
 *
 * Records timestamped state transitions (radio, LEDs, packets) in a
 * ring buffer for UI visualization and post-mortem analysis.
 */
#ifndef TIMELINE_H
#define TIMELINE_H

#include <stdint.h>

/* Event types — matches Cooja Classic timeline semantics */
typedef enum {
    TL_RADIO_OFF  = 0,   /* Radio transceiver off (white) */
    TL_RADIO_ON   = 1,   /* Radio on, listening (gray) */
    TL_RADIO_TX   = 2,   /* Transmitting (blue) */
    TL_RADIO_RX   = 3,   /* Receiving data (green) */
    TL_RADIO_INTF = 4,   /* Interference / collision (red) */
    TL_LED_ON     = 5,   /* LED turned on */
    TL_LED_OFF    = 6,   /* LED turned off */
    TL_RF_FRAME   = 7,   /* Complete RF frame TX or RX */
    TL_LOG        = 8,   /* Console log line */
} tl_event_type_t;

/* Single timeline event */
typedef struct {
    int64_t  time_ns;     /* Simulation time in nanoseconds */
    uint8_t  node_id;     /* Node index */
    uint8_t  type;        /* tl_event_type_t */
    uint8_t  value;       /* LED number, radio state, etc. */
    uint8_t  extra_len;   /* Length of extra data (packet summary, log line) */
    char     extra[128];  /* Extra string data */
} tl_event_t;

/* Ring buffer capacity */
#define TL_MAX_EVENTS 65536

/* Timeline instance */
typedef struct {
    tl_event_t events[TL_MAX_EVENTS];
    int head;            /* Oldest event index */
    int count;           /* Number of events in buffer */
    int new_count;       /* Events since last flush (for UI delta) */
    int enabled;         /* 1 = recording events */
} timeline_t;

/* Initialize timeline */
void tl_init(timeline_t *tl);

/* Record events */
void tl_radio_event(timeline_t *tl, int node_id, int64_t time_ns,
                    tl_event_type_t type);
void tl_led_event(timeline_t *tl, int node_id, int64_t time_ns,
                  int led_num, int on);
void tl_frame_event(timeline_t *tl, int node_id, int64_t time_ns,
                    int is_tx, const char *summary);
void tl_log_event(timeline_t *tl, int node_id, int64_t time_ns,
                  const char *line);

/*
 * Get new events since last call to tl_flush_new().
 * Returns pointer to internal array and sets *count.
 * Caller must process before next tl_ call.
 */
const tl_event_t *tl_get_new(timeline_t *tl, int *count);

/* Mark all current events as "sent" (resets new_count) */
void tl_flush_new(timeline_t *tl);

/*
 * Serialize new timeline events to JSON array string.
 * Caller must free() returned string.
 * Does NOT reset new_count — caller must call tl_flush_new().
 */
char *tl_events_to_json(timeline_t *tl);

/*
 * Serialize ALL events in the ring buffer to JSON array string.
 * Used for full-state sync on client connect/reload.
 * Caller must free() returned string.
 */
char *tl_all_events_to_json(timeline_t *tl);

#endif /* TIMELINE_H */
