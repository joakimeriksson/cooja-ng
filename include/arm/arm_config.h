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
} arm_config_t;

/* Pre-defined configurations */
extern const arm_config_t cc2538_config;

#endif /* ARM_CONFIG_H */
