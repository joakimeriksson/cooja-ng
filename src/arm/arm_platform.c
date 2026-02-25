/*
 * ARM platform — bundles SoC config with all peripheral instances.
 */
#include "arm_platform.h"
#include "arm_elf.h"
#include <string.h>
#include <ctype.h>
#include <stdio.h>

/* --- Platform definitions --- */

static const arm_platform_config_t platform_cc2538dk = {
    .name          = "cc2538dk",
    .soc           = &cc2538_config,
    .console_uart  = 0,
};

static const arm_platform_config_t platform_openmote = {
    .name          = "openmote",
    .soc           = &cc2538_config,
    .console_uart  = 0,
};

static const arm_platform_config_t *all_arm_platforms[] = {
    &platform_cc2538dk,
    &platform_openmote,
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

/* Flash controller stub (0x400D3000) */
#define FLASH_CTRL_BASE  0x400D3000
#define FLASH_CTRL_SIZE  0x1000

static int flash_ctrl_read(void *user_data, uint32_t addr) {
    (void)user_data;
    uint32_t offset = addr - FLASH_CTRL_BASE;
    if (offset == 0x08) return 0; /* FCTL: flash ready */
    return 0;
}

static void flash_ctrl_write(void *user_data, uint32_t addr, uint32_t value) {
    (void)user_data; (void)addr; (void)value;
}

/* Watchdog stub (0x400D5000) */
#define WDT_BASE  0x400D5000
#define WDT_SIZE  0x1000

static int wdt_read(void *user_data, uint32_t addr) {
    (void)user_data; (void)addr;
    return 0;
}

static void wdt_write(void *user_data, uint32_t addr, uint32_t value) {
    (void)user_data; (void)addr; (void)value;
}

/* AES / PKA / crypto stubs */
#define ANA_REGS_BASE 0x400D6000
#define ANA_REGS_SIZE 0x1000

static int ana_read(void *user_data, uint32_t addr) {
    (void)user_data; (void)addr;
    return 0;
}

static void ana_write(void *user_data, uint32_t addr, uint32_t value) {
    (void)user_data; (void)addr; (void)value;
}

/* SOC ADC stub */
#define SOC_ADC_BASE 0x400D7000
#define SOC_ADC_SIZE 0x1000

static int soc_adc_read(void *user_data, uint32_t addr) {
    (void)user_data;
    uint32_t offset = addr - SOC_ADC_BASE;
    if (offset == 0x00) return 0; /* ADCCON1: end of conversion */
    return 0;
}

static void soc_adc_write(void *user_data, uint32_t addr, uint32_t value) {
    (void)user_data; (void)addr; (void)value;
}

/* SSI (SPI) stub */
#define SSI0_BASE 0x40008000
#define SSI1_BASE 0x40009000
#define SSI_SIZE  0x1000

static int ssi_read(void *user_data, uint32_t addr) {
    (void)user_data; (void)addr;
    return 0x2; /* SR: TNF=1 (TX not full) */
}

static void ssi_write(void *user_data, uint32_t addr, uint32_t value) {
    (void)user_data; (void)addr; (void)value;
}

/* I2C stub */
#define I2CM_BASE 0x40020000
#define I2CM_SIZE 0x1000

static int i2c_read(void *user_data, uint32_t addr) {
    (void)user_data; (void)addr;
    return 0;
}

static void i2c_write(void *user_data, uint32_t addr, uint32_t value) {
    (void)user_data; (void)addr; (void)value;
}

/* uDMA stub */
#define UDMA_BASE 0x400FF000
#define UDMA_SIZE 0x1000

static int udma_read(void *user_data, uint32_t addr) {
    (void)user_data; (void)addr;
    return 0;
}

static void udma_write(void *user_data, uint32_t addr, uint32_t value) {
    (void)user_data; (void)addr; (void)value;
}

void arm_platform_init(arm_platform_t *plat, const arm_platform_config_t *config) {
    memset(plat, 0, sizeof(*plat));
    plat->config = config;

    /* CPU */
    arm_cpu_init(&plat->cpu, config->soc);

    /* NVIC */
    arm_nvic_init(&plat->nvic, &plat->cpu);

    /* SysTick */
    arm_systick_init(&plat->systick, &plat->cpu, &plat->nvic);

    /* System Control */
    cc2538_sys_ctrl_init(&plat->sys_ctrl, &plat->cpu);

    /* UARTs */
    cc2538_uart_init(&plat->uart0, &plat->cpu, 0x4000C000, 5);
    cc2538_uart_init(&plat->uart1, &plat->cpu, 0x4000D000, 6);

    /* GPIO */
    cc2538_gpio_init(&plat->gpio, &plat->cpu);

    /* IOC */
    cc2538_ioc_init(&plat->ioc, &plat->cpu);

    /* GPTimers */
    cc2538_gptimer_init(&plat->gptimer[0], &plat->cpu, GPTIMER0_BASE, 19, 20, 0);
    cc2538_gptimer_init(&plat->gptimer[1], &plat->cpu, GPTIMER1_BASE, 21, 22, 1);
    cc2538_gptimer_init(&plat->gptimer[2], &plat->cpu, GPTIMER2_BASE, 23, 24, 2);
    cc2538_gptimer_init(&plat->gptimer[3], &plat->cpu, GPTIMER3_BASE, 35, 36, 3);

    /* RF Core */
    cc2538_rfcore_init(&plat->rfcore, &plat->cpu);

    /* Peripheral stubs */
    arm_register_io(&plat->cpu, FLASH_CTRL_BASE, FLASH_CTRL_SIZE,
                    flash_ctrl_read, flash_ctrl_write, plat);
    arm_register_io(&plat->cpu, WDT_BASE, WDT_SIZE, wdt_read, wdt_write, plat);
    arm_register_io(&plat->cpu, ANA_REGS_BASE, ANA_REGS_SIZE, ana_read, ana_write, plat);
    arm_register_io(&plat->cpu, SOC_ADC_BASE, SOC_ADC_SIZE, soc_adc_read, soc_adc_write, plat);
    arm_register_io(&plat->cpu, SSI0_BASE, SSI_SIZE, ssi_read, ssi_write, plat);
    arm_register_io(&plat->cpu, SSI1_BASE, SSI_SIZE, ssi_read, ssi_write, plat);
    arm_register_io(&plat->cpu, I2CM_BASE, I2CM_SIZE, i2c_read, i2c_write, plat);
    arm_register_io(&plat->cpu, UDMA_BASE, UDMA_SIZE, udma_read, udma_write, plat);
}

void arm_platform_destroy(arm_platform_t *plat) {
    arm_cpu_destroy(&plat->cpu);
}

void arm_platform_set_console(arm_platform_t *plat,
                              arm_uart_tx_callback cb, void *user_data) {
    if (plat->config->console_uart == 0) {
        cc2538_uart_set_callback(&plat->uart0, cb, user_data);
    } else {
        cc2538_uart_set_callback(&plat->uart1, cb, user_data);
    }
}
