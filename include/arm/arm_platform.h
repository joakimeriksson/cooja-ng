/*
 * ARM platform — bundles SoC config with all peripheral instances.
 */
#ifndef ARM_PLATFORM_H
#define ARM_PLATFORM_H

#include "arm_cpu.h"
#include "arm_config.h"
#include "arm_nvic.h"
#include "arm_systick.h"
#include "cc2538_uart.h"
#include "cc2538_gpio.h"
#include "cc2538_gptimer.h"
#include "cc2538_sys_ctrl.h"
#include "cc2538_ioc.h"
#include "cc2538_rfcore.h"

/* UART TX callback */
typedef void (*arm_uart_tx_callback)(void *user_data, uint8_t byte);

/* Static platform configuration */
typedef struct arm_platform_config {
    const char          *name;
    const arm_config_t  *soc;
    int                  console_uart;   /* 0 or 1 */
} arm_platform_config_t;

/* Platform runtime state */
typedef struct arm_platform {
    arm_cpu_t         cpu;
    arm_nvic_t        nvic;
    arm_systick_t     systick;
    cc2538_uart_t     uart0;
    cc2538_uart_t     uart1;
    cc2538_gpio_t     gpio;
    cc2538_gptimer_t  gptimer[4];
    cc2538_sys_ctrl_t sys_ctrl;
    cc2538_ioc_t      ioc;
    cc2538_rfcore_t   rfcore;
    const arm_platform_config_t *config;
} arm_platform_t;

/* Initialize all peripherals from platform config */
void arm_platform_init(arm_platform_t *plat, const arm_platform_config_t *config);

/* Clean up */
void arm_platform_destroy(arm_platform_t *plat);

/* Connect a TX callback to the console UART */
void arm_platform_set_console(arm_platform_t *plat,
                              arm_uart_tx_callback cb, void *user_data);

/* Lookup platform by name */
const arm_platform_config_t *arm_platform_find(const char *name);

#endif /* ARM_PLATFORM_H */
