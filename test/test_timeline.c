/*
 * Timeline subsystem unit tests
 *
 * Validates event recording, JSON serialization, and ring buffer behavior
 * without requiring full simulation. Tests cover:
 * - Basic event push/get/flush
 * - JSON serialization accuracy (timestamps, value field, extra data)
 * - Ring buffer wrap-around
 * - Sub-millisecond timestamp precision
 * - All event types (radio, LED, frame, log)
 */
#include "timeline.h"
#include "cJSON.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

static int test_count = 0;
static int pass_count = 0;

/* Heap-allocated timeline (struct is ~9MB with 65536 events, too large for stack) */
static timeline_t *tl_heap;

#define CHECK(cond, msg) do { \
    test_count++; \
    if (cond) { pass_count++; } \
    else { printf("  FAIL: %s (line %d)\n", msg, __LINE__); } \
} while(0)

/* Re-initialize the shared timeline */
static void tl_reset(void) {
    tl_init(tl_heap);
}

/* Parse JSON array from timeline and return cJSON*, caller must free */
static cJSON *get_timeline_json(void) {
    char *json_str = tl_events_to_json(tl_heap);
    if (!json_str) return NULL;
    cJSON *arr = cJSON_Parse(json_str);
    free(json_str);
    return arr;
}

/* --- Test: Basic radio events --- */
static void test_radio_events(void) {
    printf("  Radio events...\n");
    tl_reset();

    tl_radio_event(tl_heap, 1, 1000000000LL, TL_RADIO_ON);   /* 1.0s */
    tl_radio_event(tl_heap, 1, 1500000000LL, TL_RADIO_TX);   /* 1.5s */
    tl_radio_event(tl_heap, 1, 1600000000LL, TL_RADIO_ON);   /* 1.6s */
    tl_radio_event(tl_heap, 2, 1500000000LL, TL_RADIO_RX);   /* 1.5s node 2 */
    tl_radio_event(tl_heap, 2, 1600000000LL, TL_RADIO_ON);   /* 1.6s node 2 */

    CHECK(tl_heap->count == 5, "5 events in buffer");
    CHECK(tl_heap->new_count == 5, "5 new events");

    cJSON *arr = get_timeline_json();
    CHECK(arr != NULL, "JSON serialized");
    CHECK(cJSON_GetArraySize(arr) == 5, "5 events in JSON");

    /* Check first event */
    cJSON *ev0 = cJSON_GetArrayItem(arr, 0);
    CHECK(ev0 != NULL, "event 0 exists");
    cJSON *t0 = cJSON_GetObjectItem(ev0, "t");
    CHECK(t0 && fabs(t0->valuedouble - 1000.0) < 0.001, "time is 1000.0 ms");
    cJSON *n0 = cJSON_GetObjectItem(ev0, "n");
    CHECK(n0 && n0->valueint == 1, "node is 1");
    cJSON *e0 = cJSON_GetObjectItem(ev0, "e");
    CHECK(e0 && strcmp(e0->valuestring, "on") == 0, "event type is 'on'");

    /* Check TX event */
    cJSON *ev1 = cJSON_GetArrayItem(arr, 1);
    cJSON *e1 = cJSON_GetObjectItem(ev1, "e");
    CHECK(e1 && strcmp(e1->valuestring, "tx") == 0, "event type is 'tx'");

    /* Check RX event for node 2 */
    cJSON *ev3 = cJSON_GetArrayItem(arr, 3);
    cJSON *n3 = cJSON_GetObjectItem(ev3, "n");
    cJSON *e3 = cJSON_GetObjectItem(ev3, "e");
    CHECK(n3 && n3->valueint == 2, "node 2 RX event");
    CHECK(e3 && strcmp(e3->valuestring, "rx") == 0, "event type is 'rx'");

    cJSON_Delete(arr);

    /* After serialization, new_count should still be set (caller must flush) */
    CHECK(tl_heap->new_count == 5, "new_count preserved until flush");
    tl_flush_new(tl_heap);
    CHECK(tl_heap->new_count == 0, "new_count zero after flush");

    /* No new events after flush */
    char *json2 = tl_events_to_json(tl_heap);
    CHECK(json2 == NULL, "no JSON after flush");
}

/* --- Test: Sub-millisecond timestamp precision --- */
static void test_sub_ms_precision(void) {
    printf("  Sub-ms timestamp precision...\n");
    tl_reset();

    /* Events at 0.1ms, 0.2ms, 0.5ms apart — must not collapse to same ms */
    tl_radio_event(tl_heap, 0, 100000LL, TL_RADIO_ON);    /* 0.1 ms */
    tl_radio_event(tl_heap, 0, 200000LL, TL_RADIO_TX);    /* 0.2 ms */
    tl_radio_event(tl_heap, 0, 500000LL, TL_RADIO_ON);    /* 0.5 ms */

    cJSON *arr = get_timeline_json();
    CHECK(arr != NULL, "JSON serialized");

    cJSON *ev0 = cJSON_GetArrayItem(arr, 0);
    cJSON *ev1 = cJSON_GetArrayItem(arr, 1);
    cJSON *ev2 = cJSON_GetArrayItem(arr, 2);

    double t0 = cJSON_GetObjectItem(ev0, "t")->valuedouble;
    double t1 = cJSON_GetObjectItem(ev1, "t")->valuedouble;
    double t2 = cJSON_GetObjectItem(ev2, "t")->valuedouble;

    CHECK(fabs(t0 - 0.1) < 0.001, "t0 = 0.1 ms");
    CHECK(fabs(t1 - 0.2) < 0.001, "t1 = 0.2 ms");
    CHECK(fabs(t2 - 0.5) < 0.001, "t2 = 0.5 ms");

    /* Verify they are distinct */
    CHECK(t0 != t1, "t0 != t1 (sub-ms distinct)");
    CHECK(t1 != t2, "t1 != t2 (sub-ms distinct)");

    cJSON_Delete(arr);
    tl_flush_new(tl_heap);
}

/* --- Test: value field always present (LED 0, RX frames) --- */
static void test_value_field(void) {
    printf("  Value field serialization...\n");
    tl_reset();

    /* LED 0 on — value=0, must still have "v" in JSON */
    tl_led_event(tl_heap, 1, 1000000LL, 0, 1);  /* LED 0 ON */
    tl_led_event(tl_heap, 1, 2000000LL, 0, 0);  /* LED 0 OFF */
    tl_led_event(tl_heap, 1, 3000000LL, 1, 1);  /* LED 1 ON */

    /* RX frame — value=0 (is_tx=0) */
    tl_frame_event(tl_heap, 2, 4000000LL, 0, "Data #1 50B N1->N2 DIO");
    /* TX frame — value=1 (is_tx=1) */
    tl_frame_event(tl_heap, 1, 5000000LL, 1, "Data #2 60B N1->bcast DIO");

    cJSON *arr = get_timeline_json();
    CHECK(arr != NULL, "JSON serialized");
    CHECK(cJSON_GetArraySize(arr) == 5, "5 events");

    /* LED 0 ON: v must be 0, not absent */
    cJSON *ev0 = cJSON_GetArrayItem(arr, 0);
    cJSON *v0 = cJSON_GetObjectItem(ev0, "v");
    CHECK(v0 != NULL, "LED 0 has 'v' field");
    CHECK(v0 && v0->valueint == 0, "LED 0 v=0");

    /* LED 1 ON: v must be 1 */
    cJSON *ev2 = cJSON_GetArrayItem(arr, 2);
    cJSON *v2 = cJSON_GetObjectItem(ev2, "v");
    CHECK(v2 && v2->valueint == 1, "LED 1 v=1");

    /* RX frame: v must be 0 (not absent) */
    cJSON *ev3 = cJSON_GetArrayItem(arr, 3);
    cJSON *v3 = cJSON_GetObjectItem(ev3, "v");
    CHECK(v3 != NULL, "RX frame has 'v' field");
    CHECK(v3 && v3->valueint == 0, "RX frame v=0");

    /* TX frame: v must be 1 */
    cJSON *ev4 = cJSON_GetArrayItem(arr, 4);
    cJSON *v4 = cJSON_GetObjectItem(ev4, "v");
    CHECK(v4 && v4->valueint == 1, "TX frame v=1");

    /* Frame events have summary string */
    cJSON *s3 = cJSON_GetObjectItem(ev3, "s");
    CHECK(s3 != NULL, "RX frame has summary");
    CHECK(s3 && strstr(s3->valuestring, "DIO") != NULL, "RX frame summary contains DIO");

    cJSON_Delete(arr);
    tl_flush_new(tl_heap);
}

/* --- Test: All event types --- */
static void test_all_event_types(void) {
    printf("  All event types...\n");
    tl_reset();

    tl_radio_event(tl_heap, 0, 1000000LL, TL_RADIO_OFF);
    tl_radio_event(tl_heap, 0, 2000000LL, TL_RADIO_ON);
    tl_radio_event(tl_heap, 0, 3000000LL, TL_RADIO_TX);
    tl_radio_event(tl_heap, 0, 4000000LL, TL_RADIO_RX);
    tl_radio_event(tl_heap, 0, 5000000LL, TL_RADIO_INTF);
    tl_led_event(tl_heap, 0, 6000000LL, 0, 1);
    tl_led_event(tl_heap, 0, 7000000LL, 0, 0);
    tl_frame_event(tl_heap, 0, 8000000LL, 1, "test TX");
    tl_frame_event(tl_heap, 0, 9000000LL, 0, "test RX");
    tl_log_event(tl_heap, 0, 10000000LL, "hello world");

    cJSON *arr = get_timeline_json();
    CHECK(arr != NULL, "JSON serialized");
    CHECK(cJSON_GetArraySize(arr) == 10, "10 events");

    const char *expected_types[] = {
        "off", "on", "tx", "rx", "intf",
        "led_on", "led_off", "frame", "frame", "log"
    };

    for (int i = 0; i < 10; i++) {
        cJSON *ev = cJSON_GetArrayItem(arr, i);
        cJSON *e = cJSON_GetObjectItem(ev, "e");
        char msg[64];
        snprintf(msg, sizeof(msg), "event %d type = '%s'", i, expected_types[i]);
        CHECK(e && strcmp(e->valuestring, expected_types[i]) == 0, msg);
    }

    /* Log event has string */
    cJSON *ev9 = cJSON_GetArrayItem(arr, 9);
    cJSON *s9 = cJSON_GetObjectItem(ev9, "s");
    CHECK(s9 && strcmp(s9->valuestring, "hello world") == 0, "log has correct string");

    cJSON_Delete(arr);
    tl_flush_new(tl_heap);
}

/* --- Test: Ring buffer wrap-around --- */
static void test_ring_buffer(void) {
    printf("  Ring buffer wrap-around...\n");
    tl_reset();

    /* Fill beyond capacity to test wrap-around */
    int over = 100;
    for (int i = 0; i < TL_MAX_EVENTS + over; i++) {
        tl_radio_event(tl_heap, 0, (int64_t)i * 1000000LL, TL_RADIO_ON);
    }

    CHECK(tl_heap->count == TL_MAX_EVENTS, "count capped at TL_MAX_EVENTS");
    CHECK(tl_heap->new_count <= TL_MAX_EVENTS, "new_count <= TL_MAX_EVENTS");

    /* Verify oldest events were dropped — newest event should be last */
    cJSON *arr = get_timeline_json();
    CHECK(arr != NULL, "JSON serialized after wrap");
    int n = cJSON_GetArraySize(arr);
    CHECK(n > 0, "has events");

    /* Last event should have time = (TL_MAX_EVENTS + over - 1) ms */
    cJSON *last = cJSON_GetArrayItem(arr, n - 1);
    cJSON *t_last = cJSON_GetObjectItem(last, "t");
    double expected_ms = (double)(TL_MAX_EVENTS + over - 1);
    CHECK(t_last && fabs(t_last->valuedouble - expected_ms) < 0.001,
          "last event has correct timestamp");

    cJSON_Delete(arr);
    tl_flush_new(tl_heap);
}

/* --- Test: Flush only after explicit call --- */
static void test_flush_semantics(void) {
    printf("  Flush semantics...\n");
    tl_reset();

    tl_radio_event(tl_heap, 0, 1000000LL, TL_RADIO_ON);
    tl_radio_event(tl_heap, 0, 2000000LL, TL_RADIO_TX);

    /* First serialization */
    char *json1 = tl_events_to_json(tl_heap);
    CHECK(json1 != NULL, "first serialization succeeds");
    free(json1);

    /* Without flush, serialization returns same events */
    char *json2 = tl_events_to_json(tl_heap);
    CHECK(json2 != NULL, "second serialization without flush succeeds");
    free(json2);

    /* After flush, no new events */
    tl_flush_new(tl_heap);
    char *json3 = tl_events_to_json(tl_heap);
    CHECK(json3 == NULL, "no events after flush");

    /* New events appear after flush */
    tl_radio_event(tl_heap, 0, 3000000LL, TL_RADIO_ON);
    char *json4 = tl_events_to_json(tl_heap);
    CHECK(json4 != NULL, "new event after flush serialized");
    cJSON *arr = cJSON_Parse(json4);
    CHECK(arr && cJSON_GetArraySize(arr) == 1, "only 1 new event");
    cJSON_Delete(arr);
    free(json4);

    tl_flush_new(tl_heap);
}

/* --- Test: Disabled timeline doesn't record --- */
static void test_disabled(void) {
    printf("  Disabled timeline...\n");
    tl_reset();
    tl_heap->enabled = 0;

    tl_radio_event(tl_heap, 0, 1000000LL, TL_RADIO_ON);
    tl_led_event(tl_heap, 0, 2000000LL, 0, 1);
    tl_frame_event(tl_heap, 0, 3000000LL, 1, "test");
    tl_log_event(tl_heap, 0, 4000000LL, "hello");

    CHECK(tl_heap->count == 0, "no events when disabled");
    CHECK(tl_heap->new_count == 0, "no new events when disabled");

    char *json = tl_events_to_json(tl_heap);
    CHECK(json == NULL, "no JSON when disabled");
}

/* --- Test: Interference events --- */
static void test_interference(void) {
    printf("  Interference events...\n");
    tl_reset();

    tl_radio_event(tl_heap, 1, 1000000LL, TL_RADIO_RX);
    tl_radio_event(tl_heap, 1, 1500000LL, TL_RADIO_INTF);  /* collision mid-RX */
    tl_radio_event(tl_heap, 1, 2000000LL, TL_RADIO_ON);

    cJSON *arr = get_timeline_json();
    CHECK(arr != NULL, "JSON serialized");

    cJSON *ev1 = cJSON_GetArrayItem(arr, 1);
    cJSON *e1 = cJSON_GetObjectItem(ev1, "e");
    CHECK(e1 && strcmp(e1->valuestring, "intf") == 0, "interference event type");

    cJSON_Delete(arr);
    tl_flush_new(tl_heap);
}

/* --- Test: Multi-node interleaved events --- */
static void test_multinode_interleave(void) {
    printf("  Multi-node interleaved events...\n");
    tl_reset();

    /* Simulate 3 nodes with interleaved TX/RX */
    tl_radio_event(tl_heap, 1, 1000000LL, TL_RADIO_TX);    /* N1 TX start */
    tl_radio_event(tl_heap, 2, 1000000LL, TL_RADIO_RX);    /* N2 RX start */
    tl_radio_event(tl_heap, 3, 1000000LL, TL_RADIO_RX);    /* N3 RX start */
    tl_radio_event(tl_heap, 1, 1500000LL, TL_RADIO_ON);    /* N1 TX end */
    tl_radio_event(tl_heap, 2, 1500000LL, TL_RADIO_ON);    /* N2 RX end */
    tl_radio_event(tl_heap, 3, 1500000LL, TL_RADIO_INTF);  /* N3 collision */
    tl_radio_event(tl_heap, 3, 2000000LL, TL_RADIO_ON);    /* N3 intf end */

    /* TX frame for N1, RX frame for N2 */
    tl_frame_event(tl_heap, 1, 1500000LL, 1, "Data #0 50B N1->bcast DIO");
    tl_frame_event(tl_heap, 2, 1500000LL, 0, "Data #0 50B N1->bcast DIO");

    cJSON *arr = get_timeline_json();
    CHECK(arr != NULL, "JSON serialized");
    CHECK(cJSON_GetArraySize(arr) == 9, "9 events total");

    /* Count per-node events */
    int counts[4] = {0};
    int frame_tx = 0, frame_rx = 0;
    for (int i = 0; i < cJSON_GetArraySize(arr); i++) {
        cJSON *ev = cJSON_GetArrayItem(arr, i);
        int nid = cJSON_GetObjectItem(ev, "n")->valueint;
        counts[nid]++;
        const char *e = cJSON_GetObjectItem(ev, "e")->valuestring;
        if (strcmp(e, "frame") == 0) {
            int v = cJSON_GetObjectItem(ev, "v")->valueint;
            if (v == 1) frame_tx++;
            else frame_rx++;
        }
    }
    CHECK(counts[1] == 3, "N1: 3 events (TX, ON, frame_TX)");
    CHECK(counts[2] == 3, "N2: 3 events (RX, ON, frame_RX)");
    CHECK(counts[3] == 3, "N3: 3 events (RX, INTF, ON)");
    CHECK(frame_tx == 1, "1 TX frame event");
    CHECK(frame_rx == 1, "1 RX frame event");

    cJSON_Delete(arr);
    tl_flush_new(tl_heap);
}

/* --- Test: Long extra string truncation --- */
static void test_extra_truncation(void) {
    printf("  Extra string truncation...\n");
    tl_reset();

    /* Create a string longer than extra[128] */
    char long_str[256];
    memset(long_str, 'A', 255);
    long_str[255] = '\0';

    tl_frame_event(tl_heap, 0, 1000000LL, 1, long_str);

    cJSON *arr = get_timeline_json();
    CHECK(arr != NULL, "JSON serialized");
    cJSON *ev = cJSON_GetArrayItem(arr, 0);
    cJSON *s = cJSON_GetObjectItem(ev, "s");
    CHECK(s != NULL, "has summary");
    CHECK(s && (int)strlen(s->valuestring) <= 127, "summary truncated to fit");

    cJSON_Delete(arr);
    tl_flush_new(tl_heap);
}

/* --- Test: tl_all_events_to_json returns full buffer --- */
static void test_all_events_json(void) {
    printf("  All events JSON (full state)...\n");
    tl_reset();

    tl_radio_event(tl_heap, 0, 1000000LL, TL_RADIO_ON);
    tl_radio_event(tl_heap, 0, 2000000LL, TL_RADIO_TX);
    tl_radio_event(tl_heap, 0, 3000000LL, TL_RADIO_ON);

    /* Flush new events (simulating normal delta broadcasts) */
    tl_flush_new(tl_heap);

    /* Add more events */
    tl_radio_event(tl_heap, 0, 4000000LL, TL_RADIO_RX);
    tl_radio_event(tl_heap, 0, 5000000LL, TL_RADIO_ON);

    /* tl_events_to_json should only return 2 new events */
    char *new_json = tl_events_to_json(tl_heap);
    CHECK(new_json != NULL, "new events JSON exists");
    cJSON *new_arr = cJSON_Parse(new_json);
    CHECK(new_arr && cJSON_GetArraySize(new_arr) == 2, "2 new events");
    cJSON_Delete(new_arr);
    free(new_json);

    /* tl_all_events_to_json should return all 5 events */
    char *all_json = tl_all_events_to_json(tl_heap);
    CHECK(all_json != NULL, "all events JSON exists");
    cJSON *all_arr = cJSON_Parse(all_json);
    CHECK(all_arr && cJSON_GetArraySize(all_arr) == 5, "5 total events");

    /* Verify order: first event should be earliest */
    cJSON *first = cJSON_GetArrayItem(all_arr, 0);
    double t0 = cJSON_GetObjectItem(first, "t")->valuedouble;
    CHECK(fabs(t0 - 1.0) < 0.001, "first event at 1.0 ms");

    cJSON *last = cJSON_GetArrayItem(all_arr, 4);
    double t4 = cJSON_GetObjectItem(last, "t")->valuedouble;
    CHECK(fabs(t4 - 5.0) < 0.001, "last event at 5.0 ms");

    cJSON_Delete(all_arr);
    free(all_json);
    tl_flush_new(tl_heap);

    /* After flush, all_events still returns everything */
    char *still_all = tl_all_events_to_json(tl_heap);
    CHECK(still_all != NULL, "all events still available after flush");
    cJSON *still_arr = cJSON_Parse(still_all);
    CHECK(still_arr && cJSON_GetArraySize(still_arr) == 5, "still 5 total events");
    cJSON_Delete(still_arr);
    free(still_all);
}

/* --- Run all timeline tests --- */
int run_timeline_tests(int verbose) {
    (void)verbose;
    printf("\n=== Timeline Unit Tests ===\n");

    tl_heap = (timeline_t *)malloc(sizeof(timeline_t));
    if (!tl_heap) {
        printf("  FAIL: could not allocate timeline_t (%zu bytes)\n",
               sizeof(timeline_t));
        return 1;
    }

    test_radio_events();
    test_sub_ms_precision();
    test_value_field();
    test_all_event_types();
    test_ring_buffer();
    test_flush_semantics();
    test_disabled();
    test_interference();
    test_multinode_interleave();
    test_extra_truncation();
    test_all_events_json();

    free(tl_heap);
    tl_heap = NULL;

    printf("\n  %d/%d timeline tests passed\n", pass_count, test_count);
    return test_count - pass_count;
}
