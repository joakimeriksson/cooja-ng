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
} arm_config_t;

/* Pre-defined configurations */
extern const arm_config_t cc2538_config;
extern const arm_config_t nrf52840_config;

#endif /* ARM_CONFIG_H */
