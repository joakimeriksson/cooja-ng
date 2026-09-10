/*
 * ARM SoC configuration definitions
 */
#ifndef ARM_CONFIG_H
#define ARM_CONFIG_H

#include <stdint.h>
#include <stdbool.h>

typedef struct arm_config {
    const char *name;

    uint32_t rom_size;
    uint32_t flash_size;
    uint32_t flash_base;
    uint32_t sram_size;
    uint32_t sram_base;

    uint32_t default_cpu_freq;   /* Hz */

    /* Number of external IRQs */
    int      num_irqs;

    /* SoC-side default for the application vector table address.
     * Usually 0 (use SoC-specific discovery: e.g. CC2538 reads CCA at
     * flash_end - 0x2C). Boards with a bootloader region overlay this
     * via `arm_platform_config_t::vtor_default` (e.g. nrf52840 dongle
     * has the Open Bootloader at 0x0..0xfff and the application
     * vector table at 0x1000). */
    uint32_t vtor_default;

    /* ARMv8-M Security Extension (TrustZone-M) present on this SoC.
     * Defaults to false via designated-initializer omission; only the
     * nRF54L15 (Cortex-M33 + Nordic SPU) sets it once the extension is
     * implemented. See docs/design/trustzone-m-plan.md. */
    bool has_trustzone;
    /* Peripherals are reachable through two aliases: 0x5xxx_xxxx (Secure)
     * and 0x4xxx_xxxx (Non-secure), as on the Nordic nRF54L/nRF53 IDAU
     * scheme. The SoC registers each peripheral once at its 0x5 base; the
     * IO dispatch folds a 0x4 access onto it and records the transaction
     * security for the SPU/attribution checks. */
    bool periph_ns_alias;
    /* SCB CPUID value. 0 = the historical Cortex-M3 r2p1 id (0x412FC231),
     * kept as the default so existing SoC output is unchanged. */
    uint32_t cpuid;
} arm_config_t;

/* Pre-defined configurations */
extern const arm_config_t cc2538_config;
extern const arm_config_t nrf52840_config;
extern const arm_config_t nrf54l15_config;

#endif /* ARM_CONFIG_H */
