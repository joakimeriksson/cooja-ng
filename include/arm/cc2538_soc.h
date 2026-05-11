/*
 * CC2538 SoC peripheral bundle.
 *
 * Owns every peripheral instance specific to TI's CC2538 (UART, GPIO,
 * GPTimer, IOC, sys-ctrl, SSI, RF Core, sleep timer, off-SoC CC1200
 * wiring, plus the uDMA/USB stubs). Plugs into `arm_platform_t` through
 * the `arm_soc_ops_t` vtable defined in `arm_platform.h`.
 *
 * Consumers that know they're running on a CC2538-class platform (e.g.
 * the multinode test runner installing radio callbacks) reach this state
 * via the `arm_platform_cc2538()` accessor.
 */
#ifndef CC2538_SOC_H
#define CC2538_SOC_H

#include "arm_platform.h"
#include "cc2538_uart.h"
#include "cc2538_gpio.h"
#include "cc2538_gptimer.h"
#include "cc2538_sys_ctrl.h"
#include "cc2538_ioc.h"
#include "cc2538_rfcore.h"
#include "cc2538_sleeptimer.h"
#include "cc2538_ssi.h"
#include "cc1200.h"

typedef struct cc2538_soc {
    cc2538_uart_t       uart0;
    cc2538_uart_t       uart1;
    cc2538_gpio_t       gpio;
    cc2538_gptimer_t    gptimer[4];
    cc2538_sys_ctrl_t   sys_ctrl;
    cc2538_ioc_t        ioc;
    cc2538_rfcore_t     rfcore;
    cc2538_sleeptimer_t sleeptimer;
    cc2538_ssi_t        ssi0;
    cc2538_ssi_t        ssi1;
    cc1200_t            cc1200;          /* used only when has_cc1200 */
    void               *udma;            /* opaque uDMA state */
    void               *usb;             /* opaque USB stub state */
} cc2538_soc_t;

extern const arm_soc_ops_t cc2538_soc_ops;

/* Returns the CC2538 SoC bundle if `plat` is a CC2538-class platform,
 * else NULL. Callers that already know the platform type may dereference
 * the result without a NULL check. */
static inline cc2538_soc_t *arm_platform_cc2538(arm_platform_t *plat) {
    if (!plat || !plat->config || plat->config->soc_ops != &cc2538_soc_ops)
        return NULL;
    return (cc2538_soc_t *)plat->soc;
}

#endif /* CC2538_SOC_H */
