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

/* Nordic nRF52840.
 *
 * Per nRF52840 Product Specification v1.7 (Nordic doc 4397_734 v1.7):
 *   - Cortex-M4F @ 64 MHz (HFXO 32 MHz × 2 PLL).
 *   - 1 MiB flash @ 0x00000000.
 *   - 256 KiB SRAM @ 0x20000000.
 *   - 48 IRQ lines in the vector table (NVIC ISER[0] + ISER[1] cover IDs 0–47).
 *
 * No `rom_size` — the chip's "ROM" (FICR/UICR factory data) lives in the
 * flash region's high pages and is read through the regular memory map,
 * not a separate ROM window.
 *
 * The current ARM interpreter is Cortex-M3. The Thumb-2 ISA M4 uses is a
 * superset; networking firmware (Contiki RPL/UDP/TSCH) does not exercise
 * the FPU or DSP/SIMD extensions, so the M3 interpreter is expected to
 * cover the common path. M4-only opcodes are added on demand once
 * firmware traps on them.
 */
const arm_config_t nrf52840_config = {
    .name            = "nRF52840",
    .rom_size        = 0,
    .flash_size      = 1024 * 1024,
    .flash_base      = 0x00000000,
    .sram_size       = 256 * 1024,
    .sram_base       = 0x20000000,
    .default_cpu_freq = 64000000,   /* 64 MHz HFCLK */
    .num_irqs        = 48,
    /* VTOR is board-specific (DK = 0x0, Dongle = 0x1000 because of
     * the Open Bootloader). Set per-platform via
     * `arm_platform_config_t::vtor_override`. */
    .vtor_default    = 0,
};
