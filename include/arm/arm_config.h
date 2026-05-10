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

    /* Default vector table address used at reset. Per Cortex-M, VTOR
     * defaults to 0 on power-up; firmware then reprograms it. csim has
     * no bootloader stage, so the SoC must declare where the application
     * vector table actually lives in flash:
     *   - 0 (or omitted): use SoC-specific discovery if present
     *     (e.g. CC2538 reads CCA at flash_end - 0x2C); otherwise fall
     *     back to flash_base.
     *   - non-zero: absolute address; reset reads MSP from [vtor],
     *     PC from [vtor + 4]. */
    uint32_t vtor_default;
} arm_config_t;

/* Pre-defined configurations */
extern const arm_config_t cc2538_config;
extern const arm_config_t nrf52840_config;

#endif /* ARM_CONFIG_H */
