/*
 * Minimal libpcap writer.  See include/common/pcap_writer.h.
 *
 * File layout (classic libpcap, nanosecond variant):
 *
 *   Global header (24 bytes, written once at open):
 *     uint32_t magic           = 0xa1b23c4d  (ns precision)
 *     uint16_t version_major   = 2
 *     uint16_t version_minor   = 4
 *     int32_t  thiszone        = 0           (UTC)
 *     uint32_t sigfigs         = 0
 *     uint32_t snaplen         = 256         (max captured bytes/packet)
 *     uint32_t network         = link_type
 *
 *   Per-packet record header (16 bytes), then `incl_len` payload bytes:
 *     uint32_t ts_sec
 *     uint32_t ts_nsec         (because of magic 0xa1b23c4d)
 *     uint32_t incl_len        (bytes captured in this record)
 *     uint32_t orig_len        (bytes that were on the wire)
 *
 * Everything is little-endian on disk; we just write native uint32_t
 * since both x86-64 and Apple Silicon are LE.
 */
#include "pcap_writer.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#define PCAP_MAGIC_NSEC   0xa1b23c4d
#define PCAP_SNAPLEN      256

int pcap_writer_open(pcap_writer_t *pw, const char *path, int link_type) {
    if (!pw || !path) { errno = EINVAL; return -1; }
    memset(pw, 0, sizeof(*pw));

    pw->f = fopen(path, "wb");
    if (!pw->f) return -1;

    pw->link_type = link_type;
    pw->packet_count = 0;

    /* Global header */
    uint32_t magic    = PCAP_MAGIC_NSEC;
    uint16_t v_major  = 2;
    uint16_t v_minor  = 4;
    int32_t  thiszone = 0;
    uint32_t sigfigs  = 0;
    uint32_t snaplen  = PCAP_SNAPLEN;
    uint32_t network  = (uint32_t)link_type;

    size_t ok = 0;
    ok += fwrite(&magic,    4, 1, pw->f);
    ok += fwrite(&v_major,  2, 1, pw->f);
    ok += fwrite(&v_minor,  2, 1, pw->f);
    ok += fwrite(&thiszone, 4, 1, pw->f);
    ok += fwrite(&sigfigs,  4, 1, pw->f);
    ok += fwrite(&snaplen,  4, 1, pw->f);
    ok += fwrite(&network,  4, 1, pw->f);
    if (ok != 7 || fflush(pw->f) != 0) {
        fprintf(stderr, "pcap: short write on header for %s\n", path);
        fclose(pw->f);
        pw->f = NULL;
        return -1;
    }

    return 0;
}

void pcap_writer_packet(pcap_writer_t *pw, int64_t time_ns,
                         const uint8_t *data, int len) {
    if (!pw || !pw->f || len <= 0 || !data) return;

    if (time_ns < 0) time_ns = 0;
    uint32_t ts_sec  = (uint32_t)(time_ns / 1000000000LL);
    uint32_t ts_nsec = (uint32_t)(time_ns % 1000000000LL);
    int incl = len > PCAP_SNAPLEN ? PCAP_SNAPLEN : len;
    uint32_t incl_len = (uint32_t)incl;
    uint32_t orig_len = (uint32_t)len;

    size_t ok = 0;
    ok += fwrite(&ts_sec,   4, 1, pw->f);
    ok += fwrite(&ts_nsec,  4, 1, pw->f);
    ok += fwrite(&incl_len, 4, 1, pw->f);
    ok += fwrite(&orig_len, 4, 1, pw->f);
    if (ok != 4 || fwrite(data, 1, (size_t)incl, pw->f) != (size_t)incl) {
        /* Disk full / write error: stop capturing rather than silently
         * producing a corrupt pcap (a partial record desyncs every
         * subsequent record boundary). */
        fprintf(stderr, "pcap: write error after %ld packets — capture stopped\n",
                (long)pw->packet_count);
        fclose(pw->f);
        pw->f = NULL;
        return;
    }

    pw->packet_count++;
}

void pcap_writer_close(pcap_writer_t *pw) {
    if (!pw || !pw->f) return;
    fflush(pw->f);
    fclose(pw->f);
    pw->f = NULL;
}
