/*
 * Native Cooja mote — frame/byte radio bridging
 *
 * Converts between the native Cooja frame-based radio interface
 * (simInDataBuffer/simOutDataBuffer) and the byte-stream 802.15.4
 * interface used by emulated CC2420 and CC2538 radios.
 *
 * Byte stream format (same as CC2420/CC2538):
 *   4x 0x00 (preamble) + 0x7A (SFD) + length + payload + CRC16
 *
 * CRC is CCITT-16 with bit reversal, matching CC2420 hardware.
 */
#include "native_node.h"
#include <string.h>

/* --- CRC-CCITT-16 with bit reversal (matches CC2420) --- */

static uint8_t bitrev(uint8_t data) {
    return (uint8_t)(
        ((data << 7) & 0x80) | ((data << 5) & 0x40) |
        ((data << 3) & 0x20) | ((data << 1) & 0x10) |
        ((data >> 7) & 0x01) | ((data >> 5) & 0x02) |
        ((data >> 3) & 0x04) | ((data >> 1) & 0x08)
    );
}

static uint16_t crc_add(uint16_t crc, uint8_t data) {
    uint16_t newcrc = ((crc >> 8) & 0xff) | ((crc << 8) & 0xffff);
    newcrc ^= data;
    newcrc ^= (newcrc & 0xff) >> 4;
    newcrc ^= (newcrc << 12) & 0xffff;
    newcrc ^= (newcrc & 0xff) << 5;
    return newcrc & 0xffff;
}

static uint16_t crc_add_bitrev(uint16_t crc, uint8_t data) {
    return crc_add(crc, bitrev(data));
}

uint16_t native_crc16(const uint8_t *data, int len) {
    uint16_t crc = 0;
    for (int i = 0; i < len; i++)
        crc = crc_add_bitrev(crc, data[i]);
    return crc;
}

/* --- Frame to byte-stream conversion --- */

int native_frame_to_bytes(const uint8_t *frame, int frame_len,
                          uint8_t *out, int out_max) {
    /* Output: 4x preamble + SFD + length + frame + CRC(2) */
    int total = 4 + 1 + 1 + frame_len + 2;
    if (total > out_max) return 0;

    int pos = 0;

    /* Preamble: 4 zero bytes */
    out[pos++] = 0x00;
    out[pos++] = 0x00;
    out[pos++] = 0x00;
    out[pos++] = 0x00;

    /* SFD */
    out[pos++] = 0x7A;

    /* PHY length byte: frame_len + 2 (for FCS) */
    out[pos++] = (uint8_t)(frame_len + 2);

    /* Frame payload */
    memcpy(out + pos, frame, (size_t)frame_len);
    pos += frame_len;

    /* CRC-16 over the frame payload (bit-reversed CCITT) */
    uint16_t crc = native_crc16(frame, frame_len);
    out[pos++] = bitrev((crc >> 8) & 0xFF);
    out[pos++] = bitrev(crc & 0xFF);

    return pos;
}

/* --- Byte-stream to frame reassembler --- */

void native_rx_assembler_reset(native_rx_assembler_t *a) {
    a->state = RX_ASM_PREAMBLE;
    a->count = 0;
    a->zero_count = 0;
    a->expected_len = 0;
}

void native_rx_assembler_feed(native_node_t *node, uint8_t byte) {
    native_rx_assembler_t *a = &node->rx_asm;

    switch (a->state) {
    case RX_ASM_PREAMBLE:
        if (byte == 0x00) {
            a->zero_count++;
        } else if (byte == 0x7A && a->zero_count >= 4) {
            /* Got SFD after preamble */
            a->state = RX_ASM_LENGTH;
            a->count = 0;
        } else {
            /* Reset on unexpected byte */
            a->zero_count = 0;
        }
        break;

    case RX_ASM_SFD:
        /* Not used — SFD detection is in PREAMBLE state */
        break;

    case RX_ASM_LENGTH:
        /* PHY length byte = payload + FCS(2) */
        a->expected_len = byte;
        if (a->expected_len < 3 || a->expected_len > 127) {
            /* Invalid length, reset */
            native_rx_assembler_reset(a);
            break;
        }
        a->state = RX_ASM_PAYLOAD;
        a->count = 0;
        break;

    case RX_ASM_PAYLOAD:
        if (a->count < (int)sizeof(a->buf))
            a->buf[a->count++] = byte;

        if (a->count >= a->expected_len) {
            /* Frame complete: strip FCS (last 2 bytes), deliver to native node */
            int frame_len = a->expected_len - 2;
            if (frame_len > 0 && frame_len <= 128) {
                native_deliver_frame(node, a->buf, frame_len);
            }
            native_rx_assembler_reset(a);
        }
        break;
    }
}
