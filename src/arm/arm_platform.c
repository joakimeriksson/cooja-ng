/*
 * ARM platform — SoC-agnostic dispatch.
 *
 * Holds the platform registry (cc2538dk, openmote, zoul-firefly, …) and
 * wires the common Cortex-M state (CPU, NVIC, SysTick) before delegating
 * SoC-specific peripheral lifecycle to `arm_soc_ops_t`. The actual
 * peripheral models live next to the SoC (e.g. `cc2538_soc.c`).
 */
#include "arm_platform.h"
#include "arm_elf.h"
#include "cc2538_soc.h"
#include "nrf52840_soc.h"
#include "nrf54l15_soc.h"
#include <ctype.h>
#include <string.h>

/* --- Platform definitions --- */

static const arm_platform_config_t platform_cc2538dk = {
    .name          = "cc2538dk",
    .soc           = &cc2538_config,
    .soc_ops       = &cc2538_soc_ops,
    .console_uart  = 0,
};

static const arm_platform_config_t platform_openmote = {
    .name          = "openmote",
    .soc           = &cc2538_config,
    .soc_ops       = &cc2538_soc_ops,
    .console_uart  = 0,
};

/* Zolertia Firefly (TARGET=zoul, BOARD=firefly).
 * Same CC2538 SoC as cc2538dk/openmote — only the board wiring differs.
 * LED1 Red   = PD5, LED2 Green = PD4, LED3 Blue  = PD3 (all active-high).
 * USER button = PA3 (active-low, internal pull-up; shared with bootloader).
 * Console = UART0 (PA0=RX, PA1=TX) → CP2104 USB-serial bridge.
 *
 * Off-SoC CC1200 sub-GHz radio over SSI0:
 *   CSn  = PB5 (active low, driven by firmware as a GPIO, not by SSI's FSS)
 *   RESET= PC7 (active low, pulsed during cc1200_arch_init())
 *   GDO0 = PB4 (input to MCU; PKT_SYNC_RXTX edge interrupt)
 *   GDO2 = PB0 (input to MCU; optional, only if CC1200_USE_GPIO2 set) */
static const arm_platform_config_t platform_zoul_firefly = {
    .name          = "zoul-firefly",
    .soc           = &cc2538_config,
    .soc_ops       = &cc2538_soc_ops,
    .console_uart  = 0,
    .leds = {
        { .port = 3, .pin = 5, .active_low = false },  /* LED1 Red   PD5 */
        { .port = 3, .pin = 4, .active_low = false },  /* LED2 Green PD4 */
        { .port = 3, .pin = 3, .active_low = false },  /* LED3 Blue  PD3 */
    },
    .button = { .port = 0, .pin = 3, .active_low = true },  /* USER PA3 */
    .has_cc1200    = true,
    .cc1200_ssi    = 0,                                       /* SSI0 */
    .cc1200_csn    = { .port = 1, .pin = 5, .active_low = true },  /* PB5 */
    .cc1200_reset  = { .port = 2, .pin = 7, .active_low = true },  /* PC7 */
    .cc1200_gdo0   = { .port = 1, .pin = 4, .active_low = false }, /* PB4 */
    .cc1200_gdo2   = { .port = 1, .pin = 0, .active_low = false }, /* PB0 */
};

/* Nordic nRF52840 USB Dongle (PCA10059, TARGET=nrf52840 BOARD=dongle).
 * Reserves 0x0..0xfff for the Open Bootloader so the application
 * vector table sits at 0x1000.
 *   LED1 (mono green) = P0.6                    active-low
 *   LED2 R / G / B    = P0.8 / P1.9 / P0.12     active-low
 *   SW1 USER button   = P1.6                    active-low pull-up */
static const arm_platform_config_t platform_nrf52840_dongle = {
    .name          = "nrf52840-dongle",
    .soc           = &nrf52840_config,
    .soc_ops       = &nrf52840_soc_ops,
    .console_uart  = 0,
    .leds = {
        { .port = 0, .pin = 6,  .active_low = true },   /* LD1 green   P0.6  */
        { .port = 0, .pin = 8,  .active_low = true },   /* LD2 red     P0.8  */
        { .port = 0, .pin = 12, .active_low = true },   /* LD2 blue    P0.12 */
    },
    .button        = { .port = 1, .pin = 6, .active_low = true },  /* SW1 P1.6 */
    .vtor_override = 0x00001000,
};

/* Nordic nRF52840 Development Kit (PCA10056, TARGET=nrf52840 BOARD=dk).
 * No bootloader region — firmware is flashed via SEGGER J-Link directly
 * to flash base, so the vector table is at 0x0.
 *   LED1..4   = P0.13 / P0.14 / P0.15 / P0.16   active-low
 *   BUTTON1..4 = P0.11 / P0.12 / P0.24 / P0.25  active-low pull-up
 *   Console  = UARTE0 (legacy register window) to SEGGER USB-CDC */
static const arm_platform_config_t platform_nrf52840_dk = {
    .name          = "nrf52840-dk",
    .soc           = &nrf52840_config,
    .soc_ops       = &nrf52840_soc_ops,
    .console_uart  = 0,
    .leds = {
        { .port = 0, .pin = 13, .active_low = true },   /* LED1 P0.13 */
        { .port = 0, .pin = 14, .active_low = true },   /* LED2 P0.14 */
        { .port = 0, .pin = 15, .active_low = true },   /* LED3 P0.15 */
    },
    .button        = { .port = 0, .pin = 11, .active_low = true },  /* BUTTON1 P0.11 */
    .vtor_override = 0,                                              /* VTOR = flash_base = 0x0 */
};

/* Nordic nRF54L15 Development Kit (PCA10156, TARGET=nrf TARGET_BOARD=dk
 * with the nrf54l15 SoC in the Contiki-NG port). Cortex-M33, no
 * bootloader region → vector table at flash base (VTOR = 0).
 *   LED1..4   = P2.9 / P1.10 / P2.7 / P1.14
 *   BUTTON1..4 = P1.13 / P1.9 / P1.8 / P0.4
 *   Console  = UARTE20 (P1.4 TX / P1.5 RX) via SEGGER JLink VCP
 *
 * The peripheral set here is intentionally minimal — GLOBAL_CLOCK
 * only — and grows as L0–L6 progress. LED/button wiring is recorded
 * for future GPIO-output-callback fan-out; the underlying GPIO
 * peripheral isn't modelled yet. */
static const arm_platform_config_t platform_nrf54l15_dk = {
    .name          = "nrf54l15-dk",
    .soc           = &nrf54l15_config,
    .soc_ops       = &nrf54l15_soc_ops,
    .console_uart  = 0,
    .leds = {
        { .port = 2, .pin = 9,  .active_low = true },   /* LED1 P2.9  */
        { .port = 1, .pin = 10, .active_low = true },   /* LED2 P1.10 */
        { .port = 2, .pin = 7,  .active_low = true },   /* LED3 P2.7  */
    },
    .button        = { .port = 1, .pin = 13, .active_low = true },  /* BUTTON1 P1.13 */
    /* On-board MX25R6435F flash (DK schematic: SPIM00, CS P2.05) and the
     * ENC28J60 module on the expansion header (SPIM22, CS P1.12) — the
     * placements the Contiki-NG spi-flash / enc28j60-test examples use
     * and that were validated on a real PCA10156 on 2026-09-06. */
    .spi_chips = {
        { "mx25r6435f", 0,  2, 5  },
        { "enc28j60",   22, 1, 12 },
    },
    .vtor_override = 0,
};

static const arm_platform_config_t *all_arm_platforms[] = {
    &platform_cc2538dk,
    &platform_openmote,
    &platform_zoul_firefly,
    &platform_nrf52840_dongle,
    &platform_nrf52840_dk,
    &platform_nrf54l15_dk,
    NULL
};

const arm_platform_config_t *arm_platform_find(const char *name) {
    if (!name) return NULL;
    for (int i = 0; all_arm_platforms[i]; i++) {
        const char *a = name;
        const char *b = all_arm_platforms[i]->name;
        while (*a && *b && tolower((unsigned char)*a) == tolower((unsigned char)*b)) {
            a++;
            b++;
        }
        if (*a == '\0' && *b == '\0') return all_arm_platforms[i];
    }
    return NULL;
}

/* --- SoC-agnostic dispatch --- */

void arm_platform_init(arm_platform_t *plat, const arm_platform_config_t *config) {
    memset(plat, 0, sizeof(*plat));
    plat->config = config;

    /* Common Cortex-M state */
    arm_cpu_init(&plat->cpu, config->soc);
    arm_nvic_init(&plat->nvic, &plat->cpu);
    arm_systick_init(&plat->systick, &plat->cpu, &plat->nvic);

    /* SoC-specific peripherals + host vtable population */
    config->soc_ops->init(plat);
}

void arm_platform_destroy(arm_platform_t *plat) {
    if (plat->config && plat->config->soc_ops)
        plat->config->soc_ops->destroy(plat);
    arm_cpu_destroy(&plat->cpu);
}

void arm_platform_set_console(arm_platform_t *plat,
                              arm_uart_tx_callback cb, void *user_data) {
    plat->config->soc_ops->set_console(plat, cb, user_data);
}
