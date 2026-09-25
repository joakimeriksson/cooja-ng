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
#include "ieee_802154.h"
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
    /* Output: PHY header (4x preamble + SFD + length) + frame + FCS */
    int total = IEEE802154_PHY_HEADER_BYTES + frame_len + IEEE802154_FCS_LEN;
    if (total > out_max) return 0;

    int pos = 0;

    /* Preamble */
    for (int i = 0; i < IEEE802154_PREAMBLE_LEN; i++)
        out[pos++] = IEEE802154_PREAMBLE_BYTE;

    /* SFD */
    out[pos++] = IEEE802154_SFD;

    /* PHY length byte: frame_len + FCS */
    out[pos++] = (uint8_t)(frame_len + IEEE802154_FCS_LEN);

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

bool native_rx_assembler_feed(native_node_t *node, uint8_t byte,
                              int64_t air_ns) {
    native_rx_assembler_t *a = &node->rx_asm;
    bool delivered = false;

    switch (a->state) {
    case RX_ASM_PREAMBLE:
        if (byte == IEEE802154_PREAMBLE_BYTE) {
            a->zero_count++;
        } else if (byte == IEEE802154_SFD &&
                   a->zero_count >= IEEE802154_PREAMBLE_LEN) {
            /* Got SFD after preamble: reception start (ContikiRadio.
             * signalReceptionStart) — mark receiving and stamp the frame
             * with the SFD's air time. */
            a->state = RX_ASM_LENGTH;
            a->count = 0;
            if (node->simReceiving) *node->simReceiving = 1;
            if (node->simLastPacketTimestamp)
                *node->simLastPacketTimestamp = (uint64_t)(air_ns / 1000LL);
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
            /* Frame complete: strip the FCS, deliver to the native node */
            int frame_len = a->expected_len - IEEE802154_FCS_LEN;
            if (frame_len > 0 && frame_len <= 128) {
                /* Byte-stream reassembly: the frame ends when its last
                 * byte leaves the air, on the bus's clock -- the same
                 * instant the bus's on-air window (this node's CCA) ends,
                 * so the frame is never read while the channel it came
                 * on still reads busy.  Queue it from its first preamble
                 * byte, which puts its end there; it is completed on the
                 * tick at that time (the caller wakes the node for it). */
                int64_t end_ns = air_ns + IEEE802154_BYTE_NS;
                native_deliver_frame(node, a->buf, frame_len,
                                     end_ns - IEEE802154_FRAME_AIR_NS(frame_len),
                                     -1);
                delivered = true;
            }
            native_rx_assembler_reset(a);
        }
        break;
    }
    return delivered;
}
