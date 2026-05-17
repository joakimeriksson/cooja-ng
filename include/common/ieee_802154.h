/*
 * Shared IEEE 802.15.4 PHY helpers — used by every 2.4 GHz radio model
 * in this tree (cc2420, cc2538_rfcore, nrf52840 RADIO, nrf54l15 RADIO).
 *
 * Holds the on-air constants (preamble + SFD + PHR layout) and the
 * CCITT-16 FCS, all of which were duplicated verbatim across four
 * driver files.  Per-SoC bits — DPPI publish, EVENT/SHORTS register
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
