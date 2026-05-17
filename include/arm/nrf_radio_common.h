/*
 * Shared helpers for the nRF RADIO peripheral models (nrf52840,
 * nrf54l15).  Both fire the same TX byte stream — SHR (4-byte
 * preamble + SFD) + PHR + payload (PHR-2 bytes) + 2-byte FCS — from
 * the same PACKETPTR layout, with the same bounds-check.  Per-SoC
 * differences (DPPI publish, EVENT/SHORTS layout, BCMATCH semantics,
 * deferred PHYEND) stay in the SoC files.
 *
 * Not used by cc2420 / cc2538_rfcore — those drive their TX byte
 * stream from a per-byte state machine spread across multiple ticks,
 * so a one-shot emit helper would be a poor fit.
 */
#ifndef NRF_RADIO_COMMON_H
#define NRF_RADIO_COMMON_H

#include <stdint.h>

#include "arm_cpu.h"

/* TX byte listener — fires once per on-air byte (preamble, SFD, PHR,
 * payload, FCS).  Same signature both SoCs already use. */
typedef void (*nrf_radio_tx_byte_cb)(void *user, uint8_t byte);

/* Emit an IEEE 802.15.4 frame from PACKETPTR via `cb`.
 *
 * PACKETPTR layout (Nordic EasyDMA):
 *   [0]        PHR (length byte; counts payload + 2 FCS bytes)
 *   [1..PHR-2] payload (FCF + seq + …, no FCS — that's added here)
 *
 * Returns true if the frame was emitted, false if PACKETPTR or PHR
 * failed validation (matching real EasyDMA's silent-drop behavior).
 * Skips entirely if `cb` is NULL.
 */
bool nrf_radio_emit_ieee802154_frame(arm_cpu_t *cpu, uint32_t packetptr,
                                      nrf_radio_tx_byte_cb cb, void *cb_user);

#endif /* NRF_RADIO_COMMON_H */
