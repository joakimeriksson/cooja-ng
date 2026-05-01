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
#include "cc2538_sleeptimer.h"
#include "cc2538_ssi.h"
#include "sim_host.h"

/* UART TX callback */
typedef void (*arm_uart_tx_callback)(void *user_data, uint8_t byte);

/* Optional GPIO pin descriptor (port 0..3 = A..D, pin 0..7).
 * Used by platform configs to document board wiring (LEDs, buttons,
 * off-SoC chip control pins). port < 0 means "unused/not present". */
typedef struct arm_gpio_pin {
    int8_t  port;         /* 0=A, 1=B, 2=C, 3=D, -1=unused */
    int8_t  pin;          /* 0..7 */
    bool    active_low;   /* true if asserted = drive low */
} arm_gpio_pin_t;

/* Static platform configuration */
typedef struct arm_platform_config {
    const char          *name;
    const arm_config_t  *soc;
    int                  console_uart;   /* 0 or 1 */
    /* Board wiring metadata. Optional — zero-initialized entries are
     * treated as "not described". Currently informational only; the
     * underlying CC2538 GPIO peripheral is wired identically for all
     * boards. Fields populated here become useful once a board needs
     * platform-managed LED state or output-pin callback fan-out
     * (e.g. for off-SoC chip control pins). */
    arm_gpio_pin_t       leds[3];        /* up to 3 status LEDs */
    arm_gpio_pin_t       button;         /* user button */
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
    cc2538_sleeptimer_t sleeptimer;
    cc2538_ssi_t      ssi0;
    cc2538_ssi_t      ssi1;
    /* uDMA state (opaque, allocated by platform init) */
    void *udma;
    /* USB state (opaque, allocated by platform init) */
    void *usb;
    /* CPU-agnostic vtable for off-SoC chip drivers (CC1200, external CC2420, etc.) */
    sim_host_t      host;
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
