/*
 * nRF52840 SoC peripheral bundle (skeleton).
 *
 * Empty for now — peripherals (CLOCK, RTC, TIMER, GPIO/GPIOTE, UARTE,
 * RADIO, PPI, NVMC, RNG, POWER) will land here as the L0 → L6 ladder
 * progresses.
 *
 * Plugs into `arm_platform_t` through the `arm_soc_ops_t` vtable defined
 * in `arm_platform.h`. Consumers that know they're on an nRF52840
 * platform reach this state via `arm_platform_nrf52840()`.
 */
#ifndef NRF52840_SOC_H
#define NRF52840_SOC_H

#include "arm_platform.h"

typedef struct nrf52840_soc {
    /* Peripheral instances will live here as they're added. */
    int _placeholder;   /* C requires at least one member */
} nrf52840_soc_t;

extern const arm_soc_ops_t nrf52840_soc_ops;

static inline nrf52840_soc_t *arm_platform_nrf52840(arm_platform_t *plat) {
    if (!plat || !plat->config || plat->config->soc_ops != &nrf52840_soc_ops)
        return NULL;
    return (nrf52840_soc_t *)plat->soc;
}

#endif /* NRF52840_SOC_H */
