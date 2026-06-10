/*
 * sim_radio_bus — RF routing helpers.  See include/sim/sim_radio_bus.h.
 */
#include "sim_radio_bus.h"

/* On-air formats:
 *   2.4 GHz: preamble(4) 0x00 + SFD 0x7A + length at data[5]; receiving
 *     chip pushes (length + 1) bytes into its RXFIFO.
 *   sub-GHz: preamble(4) 0x55 + sync(4) + PHR at data[8]; receiving
 *     chip pushes (PHR + payload + 2 status) = (length + 3) bytes. */
int frame_fifo_bytes(const uint8_t *data, int len, bool subghz) {
    if (subghz) {
        /* Need preamble(4) + sync(4) + PHR(1) at minimum to read length. */
        if (len < 9) return 9999;
        return (int)data[8] + 3;
    }
    if (len < 6) return 9999;
    return (int)data[5] + 1;
}
