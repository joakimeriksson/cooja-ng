/*
 * 802.15.4 / 6LoWPAN / IPv6 / RPL packet analyzer
 *
 * Stateless decoder: takes raw 802.15.4 frame bytes (after SFD, starting
 * with PHY length byte) and produces a human-readable summary string.
 */
#ifndef PACKET_ANALYZER_H
#define PACKET_ANALYZER_H

#include <stdint.h>

/* Maximum length of formatted packet summary */
#define PKT_SUMMARY_LEN 512

/* Frame types */
#define PKT_802154_BEACON  0
#define PKT_802154_DATA    1
#define PKT_802154_ACK     2
#define PKT_802154_CMD     3

/* 6LoWPAN dispatch values */
#define LOWPAN_DISPATCH_NALP       0x00  /* Not a LoWPAN frame */
#define LOWPAN_DISPATCH_IPV6       0x41  /* Uncompressed IPv6 */
#define LOWPAN_DISPATCH_HC1        0x42  /* HC1 compressed IPv6 */
#define LOWPAN_DISPATCH_IPHC       0x60  /* IPHC compressed (0x6x) */
#define LOWPAN_DISPATCH_IPHC_MASK  0xE0  /* Top 3 bits = 011 */
#define LOWPAN_DISPATCH_MESH       0x80  /* Mesh header */
#define LOWPAN_DISPATCH_FRAG1      0xC0  /* First fragment */
#define LOWPAN_DISPATCH_FRAGN      0xE0  /* Subsequent fragment */
#define LOWPAN_DISPATCH_NHC_UDP    0xF0  /* NHC-UDP (0xF0-0xF3) */

/* IPv6 next header values */
#define IPV6_NH_HOP_BY_HOP  0
#define IPV6_NH_TCP          6
#define IPV6_NH_UDP         17
#define IPV6_NH_ICMPV6      58

/* ICMPv6 types */
#define ICMPV6_RPL         155
#define ICMPV6_NS          135
#define ICMPV6_NA          136
#define ICMPV6_ECHO_REQ    128
#define ICMPV6_ECHO_REPLY  129

/* RPL codes */
#define RPL_DIS   0x00
#define RPL_DIO   0x01
#define RPL_DAO   0x02
#define RPL_DAO_ACK 0x03

/* Decoded packet info for structured access */
typedef struct {
    /* 802.15.4 MAC */
    uint8_t  frame_type;       /* 0=beacon, 1=data, 2=ACK, 3=cmd */
    uint8_t  seq_num;
    uint16_t dst_pan;
    uint16_t src_pan;
    uint16_t dst_short;        /* 0xFFFF if broadcast or not present */
    uint16_t src_short;        /* 0xFFFF if not present */
    uint8_t  dst_ext[8];       /* extended address (if used) */
    uint8_t  src_ext[8];
    int      dst_mode;         /* 0=none, 2=short, 3=extended */
    int      src_mode;         /* 0=none, 2=short, 3=extended */
    int      mac_hdr_len;      /* bytes consumed by MAC header */

    /* 6LoWPAN */
    uint8_t  lowpan_dispatch;  /* first dispatch byte */
    int      has_lowpan;       /* 1 if 6LoWPAN header decoded */

    /* IPv6 */
    uint8_t  ipv6_src[16];
    uint8_t  ipv6_dst[16];
    uint8_t  next_header;      /* final next-header value */
    int      has_ipv6;

    /* Transport */
    uint16_t udp_src_port;
    uint16_t udp_dst_port;
    uint8_t  icmpv6_type;
    uint8_t  icmpv6_code;
    int      has_udp;
    int      has_icmpv6;

    /* Summary string */
    char     summary[PKT_SUMMARY_LEN];
} pkt_info_t;

/*
 * Analyze a raw 802.15.4 frame.
 *
 * @param data   Frame bytes starting with PHY length byte
 * @param len    Total bytes available (including length byte)
 * @param info   Output: decoded packet info (caller allocates)
 *
 * Returns 0 on success, -1 if frame too short or invalid.
 * Always fills info->summary with a human-readable string.
 */
int pkt_analyze(const uint8_t *data, int len, pkt_info_t *info);

/*
 * Format a short one-line summary of the packet.
 * Returns pointer to info->summary.
 */
const char *pkt_summary(const uint8_t *data, int len, pkt_info_t *info);

/*
 * Format an IPv6 address as a string (abbreviated).
 * buf must be at least 40 bytes.
 */
void pkt_fmt_ipv6(const uint8_t *addr, char *buf, int buflen);

#endif /* PACKET_ANALYZER_H */
