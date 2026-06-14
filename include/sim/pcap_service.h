/*
 * pcap_service — 802.15.4 PCAP capture as a kernel service.
 *
 * Phase 6 milestone 33 (§3.19).  Owns the pcap_writer_t, the --pcap path,
 * and the open/write/close lifecycle that the runner used to keep as a
 * file-scope global.  Every completed TX frame is captured at the
 * sender's on-air timestamp; the runner's single radio-bus host hook
 * (frame_observed) feeds the MAC bytes here via pcap_service_write().
 *
 * The open ("PCAP: writing …") and close ("PCAP: wrote N frames …")
 * prints stay at their original call sites (right after node init, and
 * right after "Simulation complete") so the output is byte-identical; the
 * service is also attached to the host so its destroy is a teardown safety
 * net (closes the file if the explicit close was skipped).
 *
 * The packet analyzer (pkt_analyze) is intentionally NOT a service: it is
 * a stateless decoder whose verbose output is interleaved with the UI
 * frame-summary emit and even reaches MSP430 firmware symbols ([UIP]); it
 * moves with the UI frame-summary path in M39, not here.
 */
#ifndef PCAP_SERVICE_H
#define PCAP_SERVICE_H

#include <stdbool.h>
#include <stdint.h>

#include "sim_service.h"
#include "pcap_writer.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct pcap_service {
    pcap_writer_t writer;
    const char   *path;   /* --pcap PATH; NULL = capture disabled */
} pcap_service_t;

/* Open the capture file if `path` is non-NULL (802.15.4-with-FCS link
 * type) and print the status line.  Returns 0 on success or when disabled,
 * -1 on open failure (the writer stays closed, matching the prior
 * warn-and-continue behavior). */
int  pcap_service_open(pcap_service_t *svc, const char *path);

static inline bool pcap_service_is_open(const pcap_service_t *svc) {
    return pcap_writer_is_open(&svc->writer);
}

/* Append one captured MAC frame at `ts_ns`.  No-op when not open. */
void pcap_service_write(pcap_service_t *svc, int64_t ts_ns,
                        const uint8_t *mac, int mac_len);

/* Close the file and print "PCAP: wrote N frames to PATH" (if open). */
void pcap_service_close(pcap_service_t *svc);

/* Host vtable: init adopts an already-opened service, destroy closes the
 * file silently as a teardown safety net (the explicit pcap_service_close
 * normally runs first). */
extern const sim_service_ops_t pcap_service_ops;

#ifdef __cplusplus
}
#endif

#endif /* PCAP_SERVICE_H */
