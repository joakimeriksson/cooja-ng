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

static const arm_platform_config_t *all_arm_platforms[] = {
    &platform_cc2538dk,
    &platform_openmote,
    &platform_zoul_firefly,
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
