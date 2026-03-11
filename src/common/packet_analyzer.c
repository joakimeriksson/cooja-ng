/*
 * 802.15.4 / 6LoWPAN / IPv6 / RPL packet analyzer
 */
#include "packet_analyzer.h"
#include <stdio.h>
#include <string.h>

/* Read 16-bit little-endian */
static inline uint16_t rd16(const uint8_t *p) {
    return (uint16_t)(p[0] | (p[1] << 8));
}

void pkt_fmt_ipv6(const uint8_t *addr, char *buf, int buflen) {
    /* Abbreviated IPv6 format: show last 4 bytes for link-local,
     * or full for global addresses */
    if (addr[0] == 0xfe && addr[1] == 0x80) {
        /* Link-local: fe80::XX:XX:XX:XX */
        snprintf(buf, buflen, "fe80::%02x%02x:%02x%02x:%02x%02x:%02x%02x",
                 addr[8], addr[9], addr[10], addr[11],
                 addr[12], addr[13], addr[14], addr[15]);
    } else if (addr[0] == 0xff) {
        /* Multicast */
        snprintf(buf, buflen, "ff%02x::%02x%02x",
                 addr[1], addr[14], addr[15]);
    } else {
        /* Global: show prefix + IID */
        snprintf(buf, buflen, "%02x%02x:%02x%02x::%02x%02x:%02x%02x:%02x%02x:%02x%02x",
                 addr[0], addr[1], addr[2], addr[3],
                 addr[8], addr[9], addr[10], addr[11],
                 addr[12], addr[13], addr[14], addr[15]);
    }
}

static const char *frame_type_str(int type) {
    switch (type) {
    case PKT_802154_BEACON: return "Beacon";
    case PKT_802154_DATA:   return "Data";
    case PKT_802154_ACK:    return "ACK";
    case PKT_802154_CMD:    return "Cmd";
    default:                return "???";
    }
}

static const char *rpl_code_str(int code) {
    switch (code) {
    case RPL_DIS:     return "DIS";
    case RPL_DIO:     return "DIO";
    case RPL_DAO:     return "DAO";
    case RPL_DAO_ACK: return "DAO-ACK";
    default:          return "RPL-?";
    }
}

static const char *icmpv6_type_str(int type) {
    switch (type) {
    case ICMPV6_RPL:        return "RPL";
    case ICMPV6_NS:         return "NS";
    case ICMPV6_NA:         return "NA";
    case ICMPV6_ECHO_REQ:   return "EchoReq";
    case ICMPV6_ECHO_REPLY: return "EchoReply";
    default:                return "ICMPv6";
    }
}

/*
 * Decode 802.15.4 MAC header.
 * Returns number of bytes consumed, or -1 on error.
 */
static int decode_mac(const uint8_t *frame, int frame_len, pkt_info_t *info) {
    if (frame_len < 2) return -1;

    uint16_t fcf = rd16(frame);
    info->frame_type = fcf & 0x07;
    int security = (fcf >> 3) & 1;
    int pending = (fcf >> 4) & 1;
    int ack_req = (fcf >> 5) & 1;
    int panid_compress = (fcf >> 6) & 1;
    info->dst_mode = (fcf >> 10) & 0x03;
    int frame_version = (fcf >> 12) & 0x03;
    info->src_mode = (fcf >> 14) & 0x03;
    (void)security; (void)pending; (void)ack_req; (void)frame_version;

    int pos = 2;

    /* Sequence number */
    if (pos >= frame_len) return -1;
    info->seq_num = frame[pos++];

    /* ACK frames have no addresses */
    if (info->frame_type == PKT_802154_ACK) {
        info->dst_pan = 0;
        info->src_pan = 0;
        info->dst_short = 0xFFFF;
        info->src_short = 0xFFFF;
        info->mac_hdr_len = pos;
        return pos;
    }

    /* Destination PAN + address */
    info->dst_pan = 0;
    info->dst_short = 0xFFFF;
    memset(info->dst_ext, 0, 8);
    if (info->dst_mode != 0) {
        if (pos + 2 > frame_len) return -1;
        info->dst_pan = rd16(frame + pos);
        pos += 2;
        if (info->dst_mode == 2) {
            /* Short address */
            if (pos + 2 > frame_len) return -1;
            info->dst_short = rd16(frame + pos);
            pos += 2;
        } else if (info->dst_mode == 3) {
            /* Extended address */
            if (pos + 8 > frame_len) return -1;
            memcpy(info->dst_ext, frame + pos, 8);
            pos += 8;
        }
    }

    /* Source PAN + address */
    info->src_pan = 0;
    info->src_short = 0xFFFF;
    memset(info->src_ext, 0, 8);
    if (info->src_mode != 0) {
        if (!panid_compress) {
            if (pos + 2 > frame_len) return -1;
            info->src_pan = rd16(frame + pos);
            pos += 2;
        } else {
            info->src_pan = info->dst_pan;
        }
        if (info->src_mode == 2) {
            if (pos + 2 > frame_len) return -1;
            info->src_short = rd16(frame + pos);
            pos += 2;
        } else if (info->src_mode == 3) {
            if (pos + 8 > frame_len) return -1;
            memcpy(info->src_ext, frame + pos, 8);
            pos += 8;
        }
    }

    info->mac_hdr_len = pos;
    return pos;
}

/*
 * Decode IPHC-compressed IPv6 header.
 * Returns bytes consumed from payload, or -1 on error.
 */
static int decode_iphc(const uint8_t *data, int len, pkt_info_t *info,
                       const uint8_t *mac_src, int mac_src_mode,
                       const uint8_t *mac_dst, int mac_dst_mode) {
    if (len < 2) return -1;

    uint16_t iphc = ((uint16_t)data[0] << 8) | data[1];
    int pos = 2;

    /* TF: Traffic class / Flow label */
    int tf = (iphc >> 11) & 0x03;
    switch (tf) {
    case 0: pos += 4; break;  /* Inline ECN+DSCP+4B flow */
    case 1: pos += 3; break;  /* ECN+2B flow */
    case 2: pos += 1; break;  /* ECN+DSCP */
    case 3: break;            /* Elided */
    }

    /* NH: Next Header */
    int nh_compressed = (iphc >> 10) & 1;
    if (!nh_compressed) {
        if (pos >= len) return -1;
        info->next_header = data[pos++];
    }

    /* HLIM: Hop Limit */
    int hlim = (iphc >> 8) & 0x03;
    if (hlim == 0) pos += 1;  /* Inline */
    /* 1=1, 2=64, 3=255: no bytes consumed */

    /* CID: Context identifier extension */
    int cid = (iphc >> 7) & 1;
    if (cid) pos += 1;

    /* SAC + SAM: Source address */
    int sac = (iphc >> 6) & 1;
    int sam = (iphc >> 4) & 0x03;

    memset(info->ipv6_src, 0, 16);
    if (sac == 0) {
        switch (sam) {
        case 0: /* 128-bit inline */
            if (pos + 16 > len) return -1;
            memcpy(info->ipv6_src, data + pos, 16);
            pos += 16;
            break;
        case 1: /* 64-bit: link-local prefix + 64-bit inline */
            info->ipv6_src[0] = 0xfe;
            info->ipv6_src[1] = 0x80;
            if (pos + 8 > len) return -1;
            memcpy(info->ipv6_src + 8, data + pos, 8);
            pos += 8;
            break;
        case 2: /* 16-bit: fe80::ff:fe00:XXXX */
            info->ipv6_src[0] = 0xfe;
            info->ipv6_src[1] = 0x80;
            info->ipv6_src[11] = 0xff;
            info->ipv6_src[12] = 0xfe;
            if (pos + 2 > len) return -1;
            memcpy(info->ipv6_src + 14, data + pos, 2);
            pos += 2;
            break;
        case 3: /* Elided: derive from MAC */
            info->ipv6_src[0] = 0xfe;
            info->ipv6_src[1] = 0x80;
            if (mac_src_mode == 3) {
                memcpy(info->ipv6_src + 8, mac_src, 8);
                info->ipv6_src[8] ^= 0x02; /* flip U/L bit */
            } else if (mac_src_mode == 2) {
                info->ipv6_src[11] = 0xff;
                info->ipv6_src[12] = 0xfe;
                info->ipv6_src[14] = mac_src[0];
                info->ipv6_src[15] = mac_src[1];
            }
            break;
        }
    } else {
        /* Context-based: simplified — just read inline bytes */
        switch (sam) {
        case 1: if (pos + 8 <= len) { memcpy(info->ipv6_src + 8, data + pos, 8); pos += 8; } break;
        case 2: if (pos + 2 <= len) { memcpy(info->ipv6_src + 14, data + pos, 2); pos += 2; } break;
        case 3: /* Fully from context + MAC */ break;
        default: if (pos + 16 <= len) { memcpy(info->ipv6_src, data + pos, 16); pos += 16; } break;
        }
    }

    /* M + DAC + DAM: Destination address */
    int m = (iphc >> 3) & 1;
    int dac = (iphc >> 2) & 1;
    int dam = iphc & 0x03;

    memset(info->ipv6_dst, 0, 16);
    if (m == 0 && dac == 0) {
        switch (dam) {
        case 0:
            if (pos + 16 > len) return -1;
            memcpy(info->ipv6_dst, data + pos, 16);
            pos += 16;
            break;
        case 1:
            info->ipv6_dst[0] = 0xfe;
            info->ipv6_dst[1] = 0x80;
            if (pos + 8 > len) return -1;
            memcpy(info->ipv6_dst + 8, data + pos, 8);
            pos += 8;
            break;
        case 2:
            info->ipv6_dst[0] = 0xfe;
            info->ipv6_dst[1] = 0x80;
            info->ipv6_dst[11] = 0xff;
            info->ipv6_dst[12] = 0xfe;
            if (pos + 2 > len) return -1;
            memcpy(info->ipv6_dst + 14, data + pos, 2);
            pos += 2;
            break;
        case 3:
            info->ipv6_dst[0] = 0xfe;
            info->ipv6_dst[1] = 0x80;
            if (mac_dst_mode == 3) {
                memcpy(info->ipv6_dst + 8, mac_dst, 8);
                info->ipv6_dst[8] ^= 0x02;
            } else if (mac_dst_mode == 2) {
                info->ipv6_dst[11] = 0xff;
                info->ipv6_dst[12] = 0xfe;
                info->ipv6_dst[14] = mac_dst[0];
                info->ipv6_dst[15] = mac_dst[1];
            }
            break;
        }
    } else if (m == 1 && dac == 0) {
        /* Multicast */
        info->ipv6_dst[0] = 0xff;
        switch (dam) {
        case 0:
            if (pos + 16 > len) return -1;
            memcpy(info->ipv6_dst, data + pos, 16);
            pos += 16;
            break;
        case 1:
            if (pos + 6 > len) return -1;
            info->ipv6_dst[1] = data[pos];
            memcpy(info->ipv6_dst + 11, data + pos + 1, 5);
            pos += 6;
            break;
        case 2:
            if (pos + 4 > len) return -1;
            info->ipv6_dst[1] = data[pos];
            memcpy(info->ipv6_dst + 13, data + pos + 1, 3);
            pos += 4;
            break;
        case 3:
            if (pos + 1 > len) return -1;
            info->ipv6_dst[1] = 0x02;
            info->ipv6_dst[15] = data[pos];
            pos += 1;
            break;
        }
    } else {
        /* Context-based multicast or unicast — simplified */
        switch (dam) {
        case 1: if (pos + 8 <= len) { memcpy(info->ipv6_dst + 8, data + pos, 8); pos += 8; } break;
        case 2: if (pos + 2 <= len) { memcpy(info->ipv6_dst + 14, data + pos, 2); pos += 2; } break;
        case 3: break;
        default: if (pos + 16 <= len) { memcpy(info->ipv6_dst, data + pos, 16); pos += 16; } break;
        }
    }

    info->has_ipv6 = 1;

    /* Decode NHC headers (may chain: ext-hdr -> ext-hdr -> UDP) */
    int nhc_depth = 0;
    while (nh_compressed && pos < len && nhc_depth < 5) {
        nhc_depth++;
        uint8_t nhc = data[pos];
        if ((nhc & 0xF8) == 0xF0) {
            /* NHC-UDP */
            info->next_header = IPV6_NH_UDP;
            int port_mode = nhc & 0x03;
            pos++;
            info->has_udp = 1;
            switch (port_mode) {
            case 0: /* Both 16-bit inline */
                if (pos + 4 > len) return pos;
                info->udp_src_port = (uint16_t)(data[pos] << 8 | data[pos+1]);
                info->udp_dst_port = (uint16_t)(data[pos+2] << 8 | data[pos+3]);
                pos += 4;
                break;
            case 1: /* Src 16-bit, Dst 8-bit (0xF0 + val) */
                if (pos + 3 > len) return pos;
                info->udp_src_port = (uint16_t)(data[pos] << 8 | data[pos+1]);
                info->udp_dst_port = 0xF000 | data[pos+2];
                pos += 3;
                break;
            case 2: /* Src 8-bit, Dst 16-bit */
                if (pos + 3 > len) return pos;
                info->udp_src_port = 0xF000 | data[pos];
                info->udp_dst_port = (uint16_t)(data[pos+1] << 8 | data[pos+2]);
                pos += 3;
                break;
            case 3: /* Both 4-bit (0xF0B + nibble) */
                if (pos + 1 > len) return pos;
                info->udp_src_port = 0xF0B0 | ((data[pos] >> 4) & 0x0F);
                info->udp_dst_port = 0xF0B0 | (data[pos] & 0x0F);
                pos += 1;
                break;
            }
            /* Skip checksum (inline or elided) */
            if (!(nhc & 0x04)) pos += 2; /* checksum inline */
            nh_compressed = 0; /* Done */
        } else if ((nhc & 0xFE) == 0xE0) {
            /* NHC-IPv6-Ext: hop-by-hop (0xE0) or routing (0xE2) */
            int ext_nh_compressed = nhc & 0x01; /* bit 0: next NH also compressed */
            pos++; /* skip NHC byte */
            if (pos >= len) break;
            if (!ext_nh_compressed) {
                /* Next header field inline */
                info->next_header = data[pos];
                pos++;
            }
            if (pos >= len) break;
            int ext_len = data[pos]; /* length in bytes (excluding type+len) */
            pos++; /* skip length byte */
            /* Check for RPL hop-by-hop option (type=0x63) */
            if (pos < len && data[pos] == 0x63) {
                /* RPL option found — useful context but keep parsing */
            }
            pos += ext_len; /* skip extension body */
            nh_compressed = ext_nh_compressed;
        } else {
            /* Unknown NHC — try to decode what follows as transport */
            break;
        }
    }

    /* If we ended up with ICMPv6 next header (from ext header chain) */
    if (!nh_compressed && info->next_header == IPV6_NH_ICMPV6 &&
        pos + 2 <= len && !info->has_icmpv6) {
        info->has_icmpv6 = 1;
        info->icmpv6_type = data[pos];
        info->icmpv6_code = data[pos+1];
    }

    return pos;
}

int pkt_analyze(const uint8_t *data, int len, pkt_info_t *info) {
    memset(info, 0, sizeof(*info));
    info->dst_short = 0xFFFF;
    info->src_short = 0xFFFF;

    if (len < 3) {
        snprintf(info->summary, PKT_SUMMARY_LEN, "[too short: %d bytes]", len);
        return -1;
    }

    /* data[0] = PHY length byte; frame starts at data[1] */
    int phy_len = data[0];
    const uint8_t *frame = data + 1;
    int frame_len = len - 1;
    if (frame_len > phy_len) frame_len = phy_len;

    /* Decode MAC header */
    int mac_len = decode_mac(frame, frame_len, info);
    if (mac_len < 0) {
        snprintf(info->summary, PKT_SUMMARY_LEN, "[bad MAC header]");
        return -1;
    }

    /* For ACK frames, we're done */
    if (info->frame_type == PKT_802154_ACK) {
        snprintf(info->summary, PKT_SUMMARY_LEN, "ACK seq=%d", info->seq_num);
        return 0;
    }

    /* Payload after MAC header, before CRC (2 bytes) */
    int payload_start = mac_len;
    int payload_len = frame_len - mac_len - 2; /* subtract FCS */
    if (payload_len < 0) payload_len = 0;
    const uint8_t *payload = frame + payload_start;

    /* Check for 6LoWPAN */
    if (payload_len > 0) {
        info->lowpan_dispatch = payload[0];
        info->has_lowpan = 1;

        if ((payload[0] & 0xE0) == 0x60) {
            /* IPHC compressed IPv6 */
            const uint8_t *mac_src = (info->src_mode == 3) ? info->src_ext :
                                     (info->src_mode == 2) ? (const uint8_t *)&info->src_short : NULL;
            const uint8_t *mac_dst = (info->dst_mode == 3) ? info->dst_ext :
                                     (info->dst_mode == 2) ? (const uint8_t *)&info->dst_short : NULL;

            int iphc_len = decode_iphc(payload, payload_len, info,
                                       mac_src, info->src_mode,
                                       mac_dst, info->dst_mode);

            /* Decode transport layer if not already done by NHC */
            if (iphc_len > 0 && !info->has_udp && !info->has_icmpv6) {
                int tpos = iphc_len;
                if (info->next_header == IPV6_NH_UDP && tpos + 4 <= payload_len) {
                    info->has_udp = 1;
                    info->udp_src_port = (uint16_t)(payload[tpos] << 8 | payload[tpos+1]);
                    info->udp_dst_port = (uint16_t)(payload[tpos+2] << 8 | payload[tpos+3]);
                } else if (info->next_header == IPV6_NH_ICMPV6 && tpos + 2 <= payload_len) {
                    info->has_icmpv6 = 1;
                    info->icmpv6_type = payload[tpos];
                    info->icmpv6_code = payload[tpos+1];
                }
            }
        } else if (payload[0] == LOWPAN_DISPATCH_IPV6) {
            /* Uncompressed IPv6 */
            if (payload_len >= 41) { /* 1 dispatch + 40 IPv6 header */
                info->has_ipv6 = 1;
                info->next_header = payload[7]; /* IPv6 next header */
                memcpy(info->ipv6_src, payload + 9, 16);
                memcpy(info->ipv6_dst, payload + 25, 16);

                int tpos = 41;
                if (info->next_header == IPV6_NH_UDP && tpos + 4 <= payload_len) {
                    info->has_udp = 1;
                    info->udp_src_port = (uint16_t)(payload[tpos] << 8 | payload[tpos+1]);
                    info->udp_dst_port = (uint16_t)(payload[tpos+2] << 8 | payload[tpos+3]);
                } else if (info->next_header == IPV6_NH_ICMPV6 && tpos + 2 <= payload_len) {
                    info->has_icmpv6 = 1;
                    info->icmpv6_type = payload[tpos];
                    info->icmpv6_code = payload[tpos+1];
                }
            }
        }
    }

    /* Build summary string */
    pkt_summary(data, len, info);
    return 0;
}

const char *pkt_summary(const uint8_t *data, int len, pkt_info_t *info) {
    char *p = info->summary;
    int remain = PKT_SUMMARY_LEN;
    int n;

    /* Frame type + seq */
    n = snprintf(p, remain, "%s #%d %dB",
                 frame_type_str(info->frame_type), info->seq_num, len - 1);
    p += n; remain -= n;

    /* Addressing — show short addr as node ID (Contiki: 0x00NN or 0xNN00) */
    if (info->frame_type != PKT_802154_ACK) {
        if (info->src_mode == 2) {
            int sid = 0;
            if ((info->src_short & 0xFF00) == 0 && info->src_short > 0)
                sid = info->src_short & 0xFF;
            else if ((info->src_short & 0x00FF) == 0 && info->src_short > 0)
                sid = (info->src_short >> 8) & 0xFF;
            if (sid > 0)
                n = snprintf(p, remain, " N%d", sid);
            else
                n = snprintf(p, remain, " %04X", info->src_short);
            p += n; remain -= n;
        } else if (info->src_mode == 3) {
            n = snprintf(p, remain, " %02X%02X",
                         info->src_ext[6], info->src_ext[7]);
            p += n; remain -= n;
        }
        if (info->dst_mode == 2) {
            if (info->dst_short == 0xFFFF) {
                n = snprintf(p, remain, "->bcast");
            } else {
                int did = 0;
                if ((info->dst_short & 0xFF00) == 0 && info->dst_short > 0)
                    did = info->dst_short & 0xFF;
                else if ((info->dst_short & 0x00FF) == 0 && info->dst_short > 0)
                    did = (info->dst_short >> 8) & 0xFF;
                if (did > 0)
                    n = snprintf(p, remain, "->N%d", did);
                else
                    n = snprintf(p, remain, "->%04X", info->dst_short);
            }
            p += n; remain -= n;
        } else if (info->dst_mode == 3) {
            n = snprintf(p, remain, "->%02X%02X",
                         info->dst_ext[6], info->dst_ext[7]);
            p += n; remain -= n;
        }
    }

    /* Protocol layers */
    if (info->has_icmpv6) {
        if (info->icmpv6_type == ICMPV6_RPL) {
            n = snprintf(p, remain, " %s", rpl_code_str(info->icmpv6_code));
        } else {
            n = snprintf(p, remain, " %s", icmpv6_type_str(info->icmpv6_type));
        }
        p += n; remain -= n;
    } else if (info->has_udp) {
        n = snprintf(p, remain, " UDP %d->%d",
                     info->udp_src_port, info->udp_dst_port);
        p += n; remain -= n;
    } else if (info->has_lowpan) {
        if ((info->lowpan_dispatch & 0xE0) == 0x60)
            n = snprintf(p, remain, " 6LoWPAN");
        else if (info->lowpan_dispatch == LOWPAN_DISPATCH_IPV6)
            n = snprintf(p, remain, " IPv6");
        else
            n = snprintf(p, remain, " disp=0x%02X", info->lowpan_dispatch);
        p += n; remain -= n;
    }

    /* IPv6 addresses (abbreviated) if available */
    if (info->has_ipv6 && remain > 40) {
        char src_str[48], dst_str[48];
        pkt_fmt_ipv6(info->ipv6_src, src_str, sizeof(src_str));
        pkt_fmt_ipv6(info->ipv6_dst, dst_str, sizeof(dst_str));
        n = snprintf(p, remain, " [%s -> %s]", src_str, dst_str);
        p += n; remain -= n;
    }

    (void)remain;
    return info->summary;
}
