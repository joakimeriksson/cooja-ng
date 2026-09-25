/*
 * Shared IEEE 802.15.4 PHY helpers — used by every 2.4 GHz radio model
 * in this tree (cc2420, cc2538_rfcore, nrf52840 RADIO, nrf54l15 RADIO),
 * by the radio bus's air-time clock and by the native Cooja mote's
 * frame/byte bridge.
 *
 * Holds the on-air constants (preamble + SFD + PHR layout, byte
 * duration) and the CCITT-16 FCS, all of which were duplicated verbatim
 * across four driver files.  Per-SoC bits — DPPI publish, EVENT/SHORTS register
 * layout, BCMATCH semantics — stay in the SoC files.
 *
 * CC1200 (sub-GHz, 802.15.4g) uses a different PHY (0x55 preamble +
 * 4-byte sync word, no FCS computation in csim) and does NOT include
 * this header.
 */
#ifndef IEEE_802154_H
#define IEEE_802154_H

#include <stdint.h>

/* On-air framing — same for every 2.4 GHz O-QPSK PHY in this tree.
 * SHR (Synchronization Header) = 4 bytes of 0x00 preamble + 1 byte SFD. */
#define IEEE802154_PREAMBLE_BYTE  0x00
#define IEEE802154_PREAMBLE_LEN   4
#define IEEE802154_SFD            0x7A
#define IEEE802154_SFD_LEN        1
#define IEEE802154_PHR_LEN        1   /* the length byte */
#define IEEE802154_FCS_LEN        2
/* PHY header on the air: 4 preamble + SFD + length byte = 6 bytes. */
#define IEEE802154_PHY_HEADER_BYTES \
    (IEEE802154_PREAMBLE_LEN + IEEE802154_SFD_LEN + IEEE802154_PHR_LEN)

/* Byte duration at 250 kbit/s = 32 µs.  The one rule for how long a
 * 2.4 GHz frame occupies the air: the bus's byte clock, the on-air window
 * it announces, and the native mote's TX end and RX end all derive from
 * it, so they cannot drift apart. */
#define IEEE802154_BYTE_NS        32000LL

/* On-air bytes, and time, of a frame carrying mac_len MAC bytes: the PHY
 * header, the MAC frame and its FCS -- the bytes a chip receiver is fed
 * one by one, and how long any sender of that frame occupies the air.
 * The one rule for a frame-level (native / JS / external) sender: the
 * bus's busy window and collision marking, the native model's TX end and
 * RX end, and the PHY wrap it is carried in all use it.  A window that
 * counts the MAC bytes only reads clear while a chip receiver is still
 * taking the last bytes. */
#define IEEE802154_FRAME_AIR_BYTES(mac_len) \
    (IEEE802154_PHY_HEADER_BYTES + (mac_len) + IEEE802154_FCS_LEN)
#define IEEE802154_FRAME_AIR_NS(mac_len) \
    ((int64_t)IEEE802154_FRAME_AIR_BYTES(mac_len) * IEEE802154_BYTE_NS)

/* RX byte-stream parser phases.  Every radio that builds a frame from
 * incoming on-air bytes walks this same sequence. */
enum ieee802154_rx_phase {
    IEEE802154_RX_WAIT_PREAMBLE = 0,
    IEEE802154_RX_WAIT_SFD,
    IEEE802154_RX_READ_PHR,
    IEEE802154_RX_READ_PAYLOAD
};

/* Bit-reversal of one byte.  CC2420/CC2538 hardware ingests FCS bytes
 * in bit-reversed order before running the CRC; nRF radios don't.
 * Keep inline so callers don't pay a function-call hop per byte. */
static inline uint8_t ieee802154_bitrev(uint8_t data) {
    return (uint8_t)(
        ((data << 7) & 0x80) | ((data << 5) & 0x40) |
        ((data << 3) & 0x20) | ((data << 1) & 0x10) |
        ((data >> 7) & 0x01) | ((data >> 5) & 0x02) |
        ((data >> 3) & 0x04) | ((data >> 1) & 0x08));
}

/* CCITT-16 FCS step — IEEE 802.15.4 FCS computed over the MPDU
 * (FCF + seq + payload), one byte at a time.  Caller seeds `crc=0`. */
static inline uint16_t ieee802154_crc_add(uint16_t crc, uint8_t data) {
    uint16_t n = ((crc >> 8) & 0xff) | ((crc << 8) & 0xffff);
    n ^= data;
    n ^= (n & 0xff) >> 4;
    n ^= (n << 12) & 0xffff;
    n ^= (n & 0xff) << 5;
    return n & 0xffff;
}

/* Same CRC with bit-reversed input — CC2420 / CC2538 hardware idiom. */
static inline uint16_t ieee802154_crc_add_bitrev(uint16_t crc, uint8_t data) {
    return ieee802154_crc_add(crc, ieee802154_bitrev(data));
}

#endif /* IEEE_802154_H */
