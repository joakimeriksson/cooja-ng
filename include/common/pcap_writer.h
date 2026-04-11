/*
 * Minimal libpcap writer for csim radio captures.
 *
 * Writes the classic libpcap file format with the nanosecond magic
 * (0xa1b23c4d) so Wireshark shows ns-precise timestamps that match
 * csim's internal clock exactly.
 *
 * Default link type is LINKTYPE_IEEE802_15_4_WITHFCS (195) — Wireshark
 * dissects this through 6LoWPAN, IPv6, ICMPv6, RPL, UDP/TCP, etc. with
 * no extra configuration.
 *
 * Single-file, single-writer.  Not thread-safe — the multinode driver
 * is sequential when running with --pcap, which is the case for any
 * non-trivial debugging session anyway.
 */
#ifndef PCAP_WRITER_H
#define PCAP_WRITER_H

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

/* Common DLT/link types we may need */
#define PCAP_LINKTYPE_IEEE802_15_4_WITHFCS  195
#define PCAP_LINKTYPE_IEEE802_15_4_NOFCS    230

typedef struct pcap_writer {
    FILE   *f;
    int     link_type;
    int64_t packet_count;
} pcap_writer_t;

/* Open `path` for writing and emit the global header.
 * Returns 0 on success, -1 on failure (errno set). */
int  pcap_writer_open(pcap_writer_t *pw, const char *path, int link_type);

/* Append a packet record. time_ns is the simulation time in
 * nanoseconds; data/len is the on-air frame (without preamble/SFD,
 * including FCS for link type 195). */
void pcap_writer_packet(pcap_writer_t *pw, int64_t time_ns,
                        const uint8_t *data, int len);

/* Flush and close. Safe to call on a never-opened writer. */
void pcap_writer_close(pcap_writer_t *pw);

/* True if the writer is open and ready. */
static inline bool pcap_writer_is_open(const pcap_writer_t *pw) {
    return pw && pw->f != NULL;
}

#endif /* PCAP_WRITER_H */
