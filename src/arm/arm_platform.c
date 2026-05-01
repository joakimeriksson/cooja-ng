/*
 * ARM platform — bundles SoC config with all peripheral instances.
 */
#include "arm_platform.h"
#include "arm_elf.h"
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <stdio.h>

/* ============================================================
 * sim_host_t shims — bind the ARM CPU/GPIO into the
 * CPU-agnostic vtable used by off-SoC chip drivers (CC1200,
 * external CC2420, etc.). Mirrors the MSP430 host shims in
 * src/msp430/msp430_platform.c.
 * ============================================================ */

static int64_t arm_host_now_ns(void *cpu) {
    return ((arm_cpu_t *)cpu)->sim_time_ns;
}

static void arm_host_schedule_ns(void *cpu, cpu_event_t *ev, int64_t fire_ns) {
    arm_schedule_event_ns((arm_cpu_t *)cpu, ev, fire_ns);
}

static void arm_host_cancel(void *cpu, cpu_event_t *ev) {
    arm_cancel_event((arm_cpu_t *)cpu, ev);
}

static void arm_host_set_input_pin(void *gpio, int port, int pin, bool value) {
    cc2538_gpio_set_input((cc2538_gpio_t *)gpio, port, pin, value);
}

static void arm_host_force_irq_edge(void *gpio, int port, int pin, bool rising) {
    cc2538_gpio_force_irq_edge((cc2538_gpio_t *)gpio, port, pin, rising);
}

/* ============================================================
 * CC1200 wiring shims (Firefly only — other platforms set
 * has_cc1200 = false and these are not installed).
 * ============================================================ */

/* SSI exchange callback: forward every byte to the CC1200 driver. The
 * driver itself ignores bytes when CSn is high, so even if firmware
 * accidentally drives the bus without selecting the chip, nothing
 * goes wrong. */
static uint8_t arm_cc1200_ssi_exchange(void *user_data, uint8_t mosi) {
    arm_platform_t *plat = (arm_platform_t *)user_data;
    return cc1200_spi_exchange(&plat->cc1200, mosi);
}

/* GPIO output-callback dispatcher: watches the CC1200 control pins
 * (CSn on PB5, RESET on PC7 by default) and forwards level changes
 * to the chip driver. Pattern mirrors msp430_platform.c's CC2420
 * wiring — a single callback fans out to whichever pins matter. */
static void arm_cc1200_gpio_output(void *user_data, int port,
                                    uint8_t old_val, uint8_t new_val) {
    arm_platform_t *plat = (arm_platform_t *)user_data;
    const arm_platform_config_t *cfg = plat->config;
    if (!cfg || !cfg->has_cc1200) return;

    /* CSn — active low. Only react if this port owns the CSn pin. */
    if (cfg->cc1200_csn.port == port) {
        uint8_t mask = (uint8_t)(1u << cfg->cc1200_csn.pin);
        bool old_low = (old_val & mask) == 0;
        bool new_low = (new_val & mask) == 0;
        if (old_low != new_low) {
            cc1200_set_csn(&plat->cc1200, new_low);
        }
    }

    /* RESET — active low. */
    if (cfg->cc1200_reset.port == port) {
        uint8_t mask = (uint8_t)(1u << cfg->cc1200_reset.pin);
        bool old_low = (old_val & mask) == 0;
        bool new_low = (new_val & mask) == 0;
        if (old_low != new_low) {
            cc1200_set_reset(&plat->cc1200, new_low);
        }
    }
}

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
    switch (offset) {
        case 0x00: return 0x80;  /* ADCCON1: EOC=1 (conversion complete) */
        case 0x0C: return 0x80;  /* ADCL: ADC result low (mid-range) */
        case 0x10: return 0x20;  /* ADCH: ADC result high (mid-range) */
        default:   return 0;
    }
}

static void soc_adc_write(void *user_data, uint32_t addr, uint32_t value) {
    (void)user_data; (void)addr; (void)value;
}

/* SSI base addresses (peripheral implementation lives in cc2538_ssi.c) */
#define SSI0_BASE 0x40008000
#define SSI1_BASE 0x40009000

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

/* USB controller stub (0x40089000)
 * Firmware polls USB_CTRL (offset 0x3C) bit 7 during usb_arch_setup.
 * We store writes and return them on reads, with bit 7 always set
 * so the firmware doesn't spin forever. */
#define USB_BASE  0x40089000
#define USB_SIZE  0x1000

typedef struct {
    uint32_t regs[USB_SIZE / 4];
} usb_state_t;

static int usb_read(void *user_data, uint32_t addr) {
    usb_state_t *usb = (usb_state_t *)user_data;
    uint32_t offset = addr - USB_BASE;
    uint32_t idx = offset / 4;
    if (idx < USB_SIZE / 4) {
        uint32_t val = usb->regs[idx];
        /* USB_CTRL at offset 0x3C: bit 7 = USB PLL locked */
        if (offset == 0x3C)
            val |= (1u << 7);
        return (int)val;
    }
    return 0;
}

static void usb_write(void *user_data, uint32_t addr, uint32_t value) {
    usb_state_t *usb = (usb_state_t *)user_data;
    uint32_t offset = addr - USB_BASE;
    uint32_t idx = offset / 4;
    if (idx < USB_SIZE / 4)
        usb->regs[idx] = value;
}

/* Minimal uDMA — supports byte transfers for RF TX FIFO */
#define UDMA_BASE         0x400FF000
#define UDMA_SIZE         0x1000
#define UDMA_DMASTAT      0x000
#define UDMA_DMACFG       0x004
#define UDMA_DMACTLBASE   0x008   /* Channel control base pointer */
#define UDMA_DMAALTBASE   0x00C   /* Alternate channel control base (read-only) */
#define UDMA_DMASWREQ     0x014   /* Software request */
#define UDMA_DMAREQMASKSET 0x020  /* Set request mask */
#define UDMA_DMAREQMASKCLR 0x024  /* Clear request mask */
#define UDMA_DMAENASET    0x028   /* Set channel enable */
#define UDMA_DMAENACLR    0x02C   /* Clear channel enable */
#define UDMA_DMAALTSET    0x030   /* Set alternate control */
#define UDMA_DMAALTCLR    0x034   /* Clear alternate control */

typedef struct {
    arm_cpu_t *cpu;
    uint32_t ctrl_base;   /* Channel control structure base address */
    uint32_t ena;         /* Channel enable bits */
    uint32_t cfg;         /* Master enable */
    uint32_t altset;      /* Alternate control structure select */
    uint32_t reqmask;     /* Request mask bits */
} udma_state_t;

static void udma_execute_channel(udma_state_t *dma, int ch) {
    if (!dma->ctrl_base) return;
    uint32_t desc_addr = dma->ctrl_base + (uint32_t)ch * 16;
    uint32_t src_end  = arm_read32(dma->cpu, desc_addr + 0);
    uint32_t dst_end  = arm_read32(dma->cpu, desc_addr + 4);
    uint32_t ctrl     = arm_read32(dma->cpu, desc_addr + 8);

    int cycle = ctrl & 7;
    if (cycle == 0) return; /* STOP — nothing to do */

    int n = ((ctrl >> 4) & 0x3FF) + 1;  /* n_minus_1 + 1 */
    int src_inc_code = (ctrl >> 26) & 3;
    int dst_inc_code = (ctrl >> 30) & 3;
    int src_inc = (src_inc_code == 3) ? 0 : (1 << src_inc_code);
    int dst_inc = (dst_inc_code == 3) ? 0 : (1 << dst_inc_code);

    /* Compute start addresses from end pointers.
     * For incrementing channels: start = end - (n-1) * inc.
     * For no-increment (inc=0): all accesses go to the end pointer address. */
    uint32_t src = (src_inc > 0) ? src_end - (uint32_t)(n - 1) * (uint32_t)src_inc : src_end;
    uint32_t dst = (dst_inc > 0) ? dst_end - (uint32_t)(n - 1) * (uint32_t)dst_inc : dst_end;

    /* Perform the transfer */
    for (int i = 0; i < n; i++) {
        uint8_t byte = arm_read8(dma->cpu, src);
        arm_write8(dma->cpu, dst, byte);
        src += (uint32_t)src_inc;
        dst += (uint32_t)dst_inc;
    }

    /* Mark channel as STOP (transfer complete) */
    ctrl &= ~7u;
    arm_write32(dma->cpu, desc_addr + 8, ctrl);
}

static int udma_read(void *user_data, uint32_t addr) {
    udma_state_t *dma = (udma_state_t *)user_data;
    uint32_t off = addr - UDMA_BASE;
    switch (off) {
        case UDMA_DMASTAT:    return (int)(dma->cfg ? (1u << 28) : 0);
        case UDMA_DMACFG:     return (int)dma->cfg;
        case UDMA_DMACTLBASE: return (int)dma->ctrl_base;
        case UDMA_DMAALTBASE: return (int)(dma->ctrl_base + 32 * 16);
        case UDMA_DMAENASET:  return (int)dma->ena;
        case UDMA_DMAALTSET:  return (int)dma->altset;
        default: return 0;
    }
}

static void udma_write(void *user_data, uint32_t addr, uint32_t value) {
    udma_state_t *dma = (udma_state_t *)user_data;
    uint32_t off = addr - UDMA_BASE;
    switch (off) {
        case UDMA_DMACFG:       dma->cfg = value & 1; break;
        case UDMA_DMACTLBASE:   dma->ctrl_base = value; break;
        case UDMA_DMAENASET:    dma->ena |= value; break;
        case UDMA_DMAENACLR:    dma->ena &= ~value; break;
        case UDMA_DMAREQMASKSET: dma->reqmask |= value; break;
        case UDMA_DMAREQMASKCLR: dma->reqmask &= ~value; break;
        case UDMA_DMAALTSET:    dma->altset |= value; break;
        case UDMA_DMAALTCLR:    dma->altset &= ~value; break;
        case UDMA_DMASWREQ:
            for (int ch = 0; ch < 32; ch++) {
                if ((value & (1u << ch)) && (dma->ena & (1u << ch)))
                    udma_execute_channel(dma, ch);
            }
            break;
        default: break;
    }
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
    cc2538_uart_set_nvic(&plat->uart0, &plat->nvic);
    cc2538_uart_set_nvic(&plat->uart1, &plat->nvic);

    /* GPIO */
    cc2538_gpio_init(&plat->gpio, &plat->cpu);

    /* Build the CPU-agnostic host vtable used by off-SoC chip drivers. */
    plat->host.cpu            = &plat->cpu;
    plat->host.gpio           = &plat->gpio;
    plat->host.now_ns         = arm_host_now_ns;
    plat->host.schedule_ns    = arm_host_schedule_ns;
    plat->host.cancel         = arm_host_cancel;
    plat->host.set_input_pin  = arm_host_set_input_pin;
    plat->host.force_irq_edge = arm_host_force_irq_edge;

    /* IOC */
    cc2538_ioc_init(&plat->ioc, &plat->cpu);

    /* GPTimers */
    cc2538_gptimer_init(&plat->gptimer[0], &plat->cpu, GPTIMER0_BASE, 19, 20, 0);
    cc2538_gptimer_init(&plat->gptimer[1], &plat->cpu, GPTIMER1_BASE, 21, 22, 1);
    cc2538_gptimer_init(&plat->gptimer[2], &plat->cpu, GPTIMER2_BASE, 23, 24, 2);
    cc2538_gptimer_init(&plat->gptimer[3], &plat->cpu, GPTIMER3_BASE, 35, 36, 3);

    /* RF Core */
    cc2538_rfcore_init(&plat->rfcore, &plat->cpu, &plat->nvic);

    /* Peripheral stubs */
    arm_register_io(&plat->cpu, FLASH_CTRL_BASE, FLASH_CTRL_SIZE,
                    flash_ctrl_read, flash_ctrl_write, plat);
    /* Sleep Timer + Watchdog (SMWDTHROSC module) */
    cc2538_sleeptimer_init(&plat->sleeptimer, &plat->cpu, &plat->nvic, 145);
    arm_register_io(&plat->cpu, ANA_REGS_BASE, ANA_REGS_SIZE, ana_read, ana_write, plat);
    arm_register_io(&plat->cpu, SOC_ADC_BASE, SOC_ADC_SIZE, soc_adc_read, soc_adc_write, plat);
    /* SSI controllers: real peripheral with exchange callback hook used
     * by platforms that wire an off-SoC SPI chip (Firefly → CC1200). */
    cc2538_ssi_init(&plat->ssi0, &plat->cpu, SSI0_BASE, CC2538_SSI_BUS_0);
    cc2538_ssi_init(&plat->ssi1, &plat->cpu, SSI1_BASE, CC2538_SSI_BUS_1);
    arm_register_io(&plat->cpu, I2CM_BASE, I2CM_SIZE, i2c_read, i2c_write, plat);
    /* uDMA */
    {
        udma_state_t *dma = (udma_state_t *)calloc(1, sizeof(udma_state_t));
        dma->cpu = &plat->cpu;
        plat->udma = dma;
        arm_register_io(&plat->cpu, UDMA_BASE, UDMA_SIZE, udma_read, udma_write, dma);
    }

    /* USB stub */
    {
        usb_state_t *usb = (usb_state_t *)calloc(1, sizeof(usb_state_t));
        plat->usb = usb;
        arm_register_io(&plat->cpu, USB_BASE, USB_SIZE, usb_read, usb_write, usb);
    }

    /* Off-SoC CC1200 sub-GHz radio (Firefly only).
     * Wires the chip driver into the SSI bus + GPIO control pins. The
     * driver itself takes only the sim_host vtable — no ARM types leak
     * across the boundary. */
    if (config->has_cc1200) {
        cc1200_init(&plat->cc1200, &plat->host);
        cc1200_set_gdo0_pin(&plat->cc1200,
                             config->cc1200_gdo0.port,
                             config->cc1200_gdo0.pin);
        if (config->cc1200_gdo2.port >= 0) {
            cc1200_set_gdo2_pin(&plat->cc1200,
                                 config->cc1200_gdo2.port,
                                 config->cc1200_gdo2.pin);
        }
        /* Route the chosen SSI bus to the CC1200 driver. */
        cc2538_ssi_t *bus = (config->cc1200_ssi == 1) ? &plat->ssi1 : &plat->ssi0;
        cc2538_ssi_set_exchange_callback(bus, arm_cc1200_ssi_exchange, plat);
        /* Tap GPIO output transitions for CSn + RESET. */
        cc2538_gpio_set_output_callback(&plat->gpio, arm_cc1200_gpio_output, plat);
    }
}

void arm_platform_destroy(arm_platform_t *plat) {
    free(plat->udma);
    free(plat->usb);
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
