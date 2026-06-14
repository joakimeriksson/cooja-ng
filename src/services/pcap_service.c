/*
 * Implementation of the PCAP capture service — see include/sim/pcap_service.h.
 *
 * Phase 6 milestone 33 (§3.19).  Verbatim move of the runner's pcap open/
 * write/close logic into a service-owned pcap_writer_t.
 */
#include "pcap_service.h"
#include "sim_runtime.h"

#include <stdio.h>
#include <stddef.h>

int pcap_service_open(pcap_service_t *svc, const char *path) {
    svc->path = path;
    if (!path) return 0;
    if (pcap_writer_open(&svc->writer, path,
                         PCAP_LINKTYPE_IEEE802_15_4_WITHFCS) == 0) {
        printf("  PCAP: writing 802.15.4 capture to %s\n", path);
        return 0;
    }
    fprintf(stderr, "  PCAP: failed to open %s for writing\n", path);
    svc->path = NULL;
    return -1;
}

void pcap_service_write(pcap_service_t *svc, int64_t ts_ns,
                        const uint8_t *mac, int mac_len) {
    if (pcap_writer_is_open(&svc->writer) && mac_len > 0)
        pcap_writer_packet(&svc->writer, ts_ns, mac, mac_len);
}

void pcap_service_close(pcap_service_t *svc) {
    if (pcap_writer_is_open(&svc->writer)) {
        printf("  PCAP: wrote %lld frames to %s\n",
               (long long)svc->writer.packet_count, svc->path);
        pcap_writer_close(&svc->writer);
    }
}

/* Host vtable.  init adopts the already-opened service (the runner calls
 * pcap_service_open at its original call site to preserve the "writing …"
 * print position); destroy closes the file silently if the explicit
 * pcap_service_close was somehow skipped. */
static int pcap_service_init(sim_runtime_t *sim, const void *cfg, void **state) {
    (void)sim;
    *state = (void *)cfg;
    return 0;
}

static void pcap_service_destroy(sim_runtime_t *sim, void *state) {
    (void)sim;
    pcap_service_t *svc = (pcap_service_t *)state;
    if (svc && pcap_writer_is_open(&svc->writer))
        pcap_writer_close(&svc->writer);  /* silent: explicit close prints */
}

const sim_service_ops_t pcap_service_ops = {
    .name    = "pcap",
    .init    = pcap_service_init,
    .destroy = pcap_service_destroy,
};
