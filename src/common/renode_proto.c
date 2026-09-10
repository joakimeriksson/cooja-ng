/*
 * renode_proto — Renode co-simulation message codec.
 * See include/common/renode_proto.h.
 */
#include "renode_proto.h"

static void put_u32(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)(v);
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
}

static void put_u64(uint8_t *p, uint64_t v) {
    for (int i = 0; i < 8; i++)
        p[i] = (uint8_t)(v >> (8 * i));
}

static uint32_t get_u32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static uint64_t get_u64(const uint8_t *p) {
    uint64_t v = 0;
    for (int i = 0; i < 8; i++)
        v |= (uint64_t)p[i] << (8 * i);
    return v;
}

void renode_msg_encode(const renode_msg_t *m, uint8_t out[RENODE_MSG_SIZE]) {
    if (!m || !out) return;
    put_u32(out + 0,  (uint32_t)m->action);
    put_u64(out + 4,  m->addr);
    put_u64(out + 12, m->value);
    put_u32(out + 20, (uint32_t)m->peripheral_index);
}

void renode_msg_decode(const uint8_t in[RENODE_MSG_SIZE], renode_msg_t *m) {
    if (!m || !in) return;
    m->action           = (int32_t)get_u32(in + 0);
    m->addr             = get_u64(in + 4);
    m->value            = get_u64(in + 12);
    m->peripheral_index = (int32_t)get_u32(in + 20);
}

int renode_action_width(int32_t action) {
    switch (action) {
    case RENODE_READ_BYTE:
    case RENODE_WRITE_BYTE:  return 1;
    case RENODE_READ_WORD:
    case RENODE_WRITE_WORD:  return 2;
    /* The obsolete generic readRequest/writeRequest were doubleword-wide;
     * Renode no longer sends them, but treating them as 4 costs nothing and
     * keeps an old master working. */
    case RENODE_READ_REQUEST:
    case RENODE_WRITE_REQUEST:
    case RENODE_READ_DWORD:
    case RENODE_WRITE_DWORD: return 4;
    case RENODE_READ_QWORD:
    case RENODE_WRITE_QWORD: return 8;
    default:                 return 0;
    }
}

bool renode_action_is_read(int32_t action) {
    switch (action) {
    case RENODE_READ_REQUEST:
    case RENODE_READ_BYTE:
    case RENODE_READ_WORD:
    case RENODE_READ_DWORD:
    case RENODE_READ_QWORD:  return true;
    default:                 return false;
    }
}

bool renode_action_is_write(int32_t action) {
    switch (action) {
    case RENODE_WRITE_REQUEST:
    case RENODE_WRITE_BYTE:
    case RENODE_WRITE_WORD:
    case RENODE_WRITE_DWORD:
    case RENODE_WRITE_QWORD: return true;
    default:                 return false;
    }
}

const char *renode_action_name(int32_t action) {
    switch (action) {
    case RENODE_INVALID:           return "invalidAction";
    case RENODE_TICK_CLOCK:        return "tickClock";
    case RENODE_WRITE_REQUEST:     return "writeRequest";
    case RENODE_READ_REQUEST:      return "readRequest";
    case RENODE_RESET_PERIPHERAL:  return "resetPeripheral";
    case RENODE_LOG_MESSAGE:       return "logMessage";
    case RENODE_INTERRUPT:         return "interrupt";
    case RENODE_DISCONNECT:        return "disconnect";
    case RENODE_ERROR:             return "error";
    case RENODE_OK:                return "ok";
    case RENODE_HANDSHAKE:         return "handshake";
    case RENODE_PUSH_DOUBLE_WORD:  return "pushDoubleWord";
    case RENODE_GET_DOUBLE_WORD:   return "getDoubleWord";
    case RENODE_PUSH_WORD:         return "pushWord";
    case RENODE_GET_WORD:          return "getWord";
    case RENODE_PUSH_BYTE:         return "pushByte";
    case RENODE_GET_BYTE:          return "getByte";
    case RENODE_IS_HALTED:         return "isHalted";
    case RENODE_REGISTER_GET:      return "registerGet";
    case RENODE_REGISTER_SET:      return "registerSet";
    case RENODE_SINGLE_STEP_MODE:  return "singleStepMode";
    case RENODE_READ_BYTE:         return "readRequestByte";
    case RENODE_READ_WORD:         return "readRequestWord";
    case RENODE_READ_DWORD:        return "readRequestDoubleWord";
    case RENODE_READ_QWORD:        return "readRequestQuadWord";
    case RENODE_WRITE_BYTE:        return "writeRequestByte";
    case RENODE_WRITE_WORD:        return "writeRequestWord";
    case RENODE_WRITE_DWORD:       return "writeRequestDoubleWord";
    case RENODE_WRITE_QWORD:       return "writeRequestQuadWord";
    case RENODE_PUSH_QWORD:        return "pushQuadWord";
    case RENODE_GET_QWORD:         return "getQuadWord";
    case RENODE_PUSH_CONFIRMATION: return "pushConfirmation";
    case RENODE_STEP:              return "step";
    default:                       return "unknown";
    }
}
