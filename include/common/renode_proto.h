/*
 * renode_proto — Renode's co-simulation wire protocol (the message codec).
 *
 * Renode's CoSimulationPlugin talks to an external simulator over two TCP
 * sockets carrying one fixed 24-byte message:
 *
 *     int32 action; uint64 addr; uint64 value; int32 peripheralIndex
 *
 * little-endian, packed (Renode marshals the C# struct with Pack = 2, and
 * every field is naturally aligned at pack 2, so the layout is the plain
 * 4/8/8/4 sequence with no padding).  The main socket carries Renode's
 * requests and our replies; the async socket carries our unsolicited
 * events (interrupt, logMessage).
 *
 * This file is the codec only: no sockets, no simulation state, so it is
 * unit-testable on its own (test_renode_cosim.c).  The transport and the
 * protocol loop live in src/services/renode_cosim_service.c; the register
 * window behind the bus accesses lives in src/native/renode_dev.c.
 *
 * Design: docs/design/renode-cosim-plan.md.
 *
 * The action ids MUST stay in sync with Renode's
 * src/Plugins/CoSimulationPlugin/Connection/Protocols/ActionType.cs and its
 * C++ mirror renode_action_enumerators.svh.  New actions are appended at
 * the end there, so ours are safe to pin by value.
 */
#ifndef RENODE_PROTO_H
#define RENODE_PROTO_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* One protocol message on the wire. */
#define RENODE_MSG_SIZE 24

/* "not associated with any peripheral" (Renode's ProtocolMessage.NoPeripheralIndex) */
#define RENODE_NO_PERIPHERAL_INDEX (-1)

enum renode_action {
    RENODE_INVALID          = 0,
    RENODE_TICK_CLOCK       = 1,   /* master: advance `value` ticks; we echo  */
    RENODE_WRITE_REQUEST    = 2,   /* obsolete in Renode; never sent          */
    RENODE_READ_REQUEST     = 3,   /* obsolete as a REQUEST — still the reply
                                    * action for every read width             */
    RENODE_RESET_PERIPHERAL = 4,
    RENODE_LOG_MESSAGE      = 5,   /* async: addr = strlen, value = level,
                                    * followed by the raw text bytes          */
    RENODE_INTERRUPT        = 6,   /* async: addr = irq index, value = level  */
    RENODE_DISCONNECT       = 7,
    RENODE_ERROR            = 8,
    RENODE_OK               = 9,   /* write acknowledgement                   */
    RENODE_HANDSHAKE        = 10,
    /* 11..20 belong to co-simulated CPU cores (push/get word, isHalted,
     * registerGet/Set, singleStepMode).  A network peripheral never sees
     * them; we name them so a stray one can be logged by name. */
    RENODE_PUSH_DOUBLE_WORD = 11,
    RENODE_GET_DOUBLE_WORD  = 12,
    RENODE_PUSH_WORD        = 13,
    RENODE_GET_WORD         = 14,
    RENODE_PUSH_BYTE        = 15,
    RENODE_GET_BYTE         = 16,
    RENODE_IS_HALTED        = 17,
    RENODE_REGISTER_GET     = 18,
    RENODE_REGISTER_SET     = 19,
    RENODE_SINGLE_STEP_MODE = 20,
    RENODE_READ_BYTE        = 21,
    RENODE_READ_WORD        = 22,
    RENODE_READ_DWORD       = 23,
    RENODE_READ_QWORD       = 24,
    RENODE_WRITE_BYTE       = 25,
    RENODE_WRITE_WORD       = 26,
    RENODE_WRITE_DWORD      = 27,
    RENODE_WRITE_QWORD      = 28,
    RENODE_PUSH_QWORD       = 29,
    RENODE_GET_QWORD        = 30,
    RENODE_PUSH_CONFIRMATION = 31,
    RENODE_STEP             = 100,
};

/* Renode's LogLevel values (Antmicro.Renode.Logging.LogLevel). */
enum renode_log_level {
    RENODE_LOG_NOISY   = -1,
    RENODE_LOG_DEBUG   = 0,
    RENODE_LOG_INFO    = 1,
    RENODE_LOG_WARNING = 2,
    RENODE_LOG_ERROR   = 3,
};

typedef struct renode_msg {
    int32_t  action;
    uint64_t addr;
    uint64_t value;
    int32_t  peripheral_index;
} renode_msg_t;

/* Encode/decode by hand rather than memcpy'ing a packed struct: the wire is
 * little-endian regardless of the host, and a packed-struct memcpy would
 * silently produce a different byte order on a big-endian build. */
void renode_msg_encode(const renode_msg_t *m, uint8_t out[RENODE_MSG_SIZE]);
void renode_msg_decode(const uint8_t in[RENODE_MSG_SIZE], renode_msg_t *m);

/* Access width in bytes for a bus read/write action, else 0. */
int  renode_action_width(int32_t action);
bool renode_action_is_read(int32_t action);
bool renode_action_is_write(int32_t action);

/* Stable lowercase name, matching Renode's enumerator spelling; "unknown"
 * for a value we do not model. */
const char *renode_action_name(int32_t action);

#ifdef __cplusplus
}
#endif

#endif /* RENODE_PROTO_H */
