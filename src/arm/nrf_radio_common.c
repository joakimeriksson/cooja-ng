/*
 * Implementation of nrf_radio_common.h — see header for rationale.
 */
#include "nrf_radio_common.h"
#include "ieee_802154.h"

#include <stdbool.h>

bool nrf_radio_emit_ieee802154_frame(arm_cpu_t *cpu, uint32_t packetptr,
                                      nrf_radio_tx_byte_cb cb, void *cb_user) {
    if (cb == NULL) return false;

    /* PACKETPTR must point into SRAM.  Real EasyDMA silently fails
     * otherwise; mirror that by dropping the frame. */
    if (packetptr < cpu->sram_base || packetptr + 128 > cpu->sram_end)
        return false;

    uint8_t phr = arm_read8(cpu, packetptr);
    if (phr < 2 || phr > 127) return false;   /* malformed; drop silently */

    /* Driver layout: PACKETPTR[0]=PHR; PACKETPTR[1..PHR-2]=payload
     * (PHR-2 bytes); PACKETPTR[PHR-1..PHR]=FCS slot the hardware fills
     * in.  Compute FCS over the payload and append. */
    uint8_t payload[125];
    int payload_len = (int)phr - 2;
    for (int i = 0; i < payload_len; i++)
        payload[i] = arm_read8(cpu, packetptr + 1 + (uint32_t)i);

    /* True IEEE 802.15.4 FCS (LSB-first CCITT-16, computed via the
     * bit-reversal idiom) — the same on-air convention CC2420/CC2538
     * and the native frame-to-bytes bridge use, so receivers can
     * verify one common FCS regardless of the sender's radio model. */
    uint16_t crc = 0;
    for (int i = 0; i < payload_len; i++)
        crc = ieee802154_crc_add_bitrev(crc, payload[i]);

    for (int i = 0; i < IEEE802154_PREAMBLE_LEN; i++)
        cb(cb_user, IEEE802154_PREAMBLE_BYTE);
    cb(cb_user, IEEE802154_SFD);
    cb(cb_user, phr);
    for (int i = 0; i < payload_len; i++)
        cb(cb_user, payload[i]);
    cb(cb_user, ieee802154_bitrev((uint8_t)((crc >> 8) & 0xFF)));
    cb(cb_user, ieee802154_bitrev((uint8_t)(crc & 0xFF)));
    return true;
}
