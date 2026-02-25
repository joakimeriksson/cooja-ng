/*
 * ARM SoC configuration definitions
 */
#include "arm_config.h"

const arm_config_t cc2538_config = {
    .name            = "CC2538",
    .rom_size        = 128 * 1024,
    .flash_size      = 512 * 1024,
    .flash_base      = 0x00200000,
    .sram_size       = 32 * 1024,
    .sram_base       = 0x20000000,
    .default_cpu_freq = 32000000,   /* 32 MHz system clock */
    .num_irqs        = 179,
};
