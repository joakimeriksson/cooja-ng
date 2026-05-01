/*
 * MSP430 platform — bundles MCU config with all peripheral instances.
 */
#include "msp430_platform.h"
#include <string.h>
#include <ctype.h>
#include <stdio.h>

/* ============================================================
 * sim_host_t shims — bind the MSP430 CPU/GPIO into the
 * CPU-agnostic vtable used by off-SoC chip drivers (CC2420 etc.).
 * ============================================================ */

static int64_t msp430_host_now_ns(void *cpu) {
    return ((msp430_cpu_t *)cpu)->sim_time_ns;
}

static void msp430_host_schedule_ns(void *cpu, cpu_event_t *ev, int64_t fire_ns) {
    msp430_schedule_event_ns((msp430_cpu_t *)cpu, ev, fire_ns);
}

static void msp430_host_cancel(void *cpu, cpu_event_t *ev) {
    msp430_cancel_event((msp430_cpu_t *)cpu, ev);
}

static void msp430_host_set_input_pin(void *gpio, int port, int pin, bool value) {
    msp430_gpio_set_input_pin((msp430_gpio_t *)gpio, port, pin, value);
}

static void msp430_host_force_irq_edge(void *gpio, int port, int pin, bool rising) {
    msp430_gpio_force_irq_edge((msp430_gpio_t *)gpio, port, pin, rising);
}

/* SFR IO callbacks (IE1/IE2/IFG1/IFG2 at 0x00-0x07) */

static int sfr_read(void *user_data, uint32_t addr, bool word, int64_t cycles) {
    msp430_platform_t *plat = (msp430_platform_t *)user_data;
    (void)word; (void)cycles;
    if (addr < 8) return plat->sfr_regs[addr];
    return 0;
}

static void sfr_write(void *user_data, uint32_t addr, int value, bool word, int64_t cycles) {
    msp430_platform_t *plat = (msp430_platform_t *)user_data;
    (void)word; (void)cycles;
    if (addr < 8) {
        plat->sfr_regs[addr] = (uint8_t)value;
        /* Re-assert IFG flags for SPI USARTs only.  SPI exchange completes
         * instantly, so TXIFG/RXIFG should always be set for SPI polling.
         * For UART-mode USARTs (those using TX interrupts), do NOT re-assert —
         * the ISR clears TXIFG and expects it to stay cleared. */
        if (plat->usart0.ifg_ptr && plat->usart0.spi_exchange)
            *plat->usart0.ifg_ptr |= plat->usart0.ifg_tx_mask | plat->usart0.ifg_rx_mask;
        if (plat->usart1.ifg_ptr && plat->usart1.spi_exchange)
            *plat->usart1.ifg_ptr |= plat->usart1.ifg_tx_mask | plat->usart1.ifg_rx_mask;
        /* Re-check USCI interrupts when IE or IFG registers change */
        msp430_usart_update_interrupts(&plat->usart0);
        msp430_usart_update_interrupts(&plat->usart1);
    }
}

/* Hardware multiplier registers (MSP430F1xx/F2xx at 0x0130-0x013E) */
#define HW_MPY_BASE  0x0130
#define HW_MPY       0x0130  /* Multiply unsigned */
#define HW_MPYS      0x0132  /* Multiply signed */
#define HW_MAC       0x0134  /* Multiply unsigned and accumulate */
#define HW_MACS      0x0136  /* Multiply signed and accumulate */
#define HW_OP2       0x0138  /* Second operand — triggers multiplication */
#define HW_RESLO     0x013A  /* Result low word */
#define HW_RESHI     0x013C  /* Result high word */
#define HW_SUMEXT    0x013E  /* Sum extension register */

static int hwmul_read(void *user_data, uint32_t addr, bool word, int64_t cycles) {
    msp430_platform_t *plat = (msp430_platform_t *)user_data;
    (void)word; (void)cycles;
    switch (addr) {
        case HW_MPY:
        case HW_MPYS:
        case HW_MAC:
        case HW_MACS:  return plat->mpy_op1;
        case HW_OP2:    return plat->mpy_op2;
        case HW_RESLO:  return plat->mpy_reslo;
        case HW_RESHI:  return plat->mpy_reshi;
        case HW_SUMEXT: return plat->mpy_sumext;
        default:        return 0;
    }
}

static void hwmul_write(void *user_data, uint32_t addr, int value, bool word, int64_t cycles) {
    msp430_platform_t *plat = (msp430_platform_t *)user_data;
    (void)cycles;
    uint16_t val = word ? (uint16_t)value : (uint8_t)value;

    switch (addr) {
        case HW_MPY:   plat->mpy_op1 = val; plat->mpy_mode = 0; break;
        case HW_MPYS:  plat->mpy_op1 = val; plat->mpy_mode = 1; break;
        case HW_MAC:   plat->mpy_op1 = val; plat->mpy_mode = 2; break;
        case HW_MACS:  plat->mpy_op1 = val; plat->mpy_mode = 3; break;
        case HW_OP2: {
            /* Writing OP2 triggers the multiplication */
            plat->mpy_op2 = val;  /* Store for readback (SXT pattern) */
            uint16_t op1 = plat->mpy_op1;
            uint16_t op2 = val;
            uint32_t result;

            switch (plat->mpy_mode) {
                case 0: /* MPY: unsigned × unsigned */
                    result = (uint32_t)op1 * (uint32_t)op2;
                    plat->mpy_reslo = (uint16_t)(result & 0xFFFF);
                    plat->mpy_reshi = (uint16_t)(result >> 16);
                    plat->mpy_sumext = 0;
                    break;
                case 1: /* MPYS: signed × signed */
                    result = (uint32_t)((int32_t)(int16_t)op1 * (int32_t)(int16_t)op2);
                    plat->mpy_reslo = (uint16_t)(result & 0xFFFF);
                    plat->mpy_reshi = (uint16_t)(result >> 16);
                    plat->mpy_sumext = (result & 0x80000000) ? 0xFFFF : 0;
                    break;
                case 2: /* MAC: unsigned multiply-accumulate */
                    result = (uint32_t)op1 * (uint32_t)op2;
                    result += ((uint32_t)plat->mpy_reshi << 16) | plat->mpy_reslo;
                    plat->mpy_reslo = (uint16_t)(result & 0xFFFF);
                    plat->mpy_reshi = (uint16_t)(result >> 16);
                    plat->mpy_sumext = 0; /* carry not tracked for simplicity */
                    break;
                case 3: /* MACS: signed multiply-accumulate */
                    result = (uint32_t)((int32_t)(int16_t)op1 * (int32_t)(int16_t)op2);
                    result += ((uint32_t)plat->mpy_reshi << 16) | plat->mpy_reslo;
                    plat->mpy_reslo = (uint16_t)(result & 0xFFFF);
                    plat->mpy_reshi = (uint16_t)(result >> 16);
                    plat->mpy_sumext = (result & 0x80000000) ? 0xFFFF : 0;
                    break;
            }
            break;
        }
        case HW_RESLO:  plat->mpy_reslo = val; break;
        case HW_RESHI:  plat->mpy_reshi = val; break;
        case HW_SUMEXT: plat->mpy_sumext = val; break;
    }
}

/* --- DMA controller (minimal: DMA0 only) ---
 *
 * Only supports DMA0 triggered by USART1 RXIFG (DMA0TSEL=9).
 * This is used by firmware that configures DMA to copy bytes from
 * RXBUF1 to a ring buffer in RAM (repeated single byte transfer).
 *
 * DMA0CTL bit layout:
 *   15:12  DMADT     (transfer mode: 0=single, 4=repeated single, etc.)
 *    11:10 (reserved)
 *     9:8  DMADSTINCR (00=unchanged, 10=decrement, 11=increment)
 *     7:6  DMASRCINCR (00=unchanged, 10=decrement, 11=increment)
 *      5   DMADSTBYTE (0=word, 1=byte) — we always treat as byte
 *      4   DMAEN      (enable)
 *      3   DMALEVEL   (level/edge)
 *      2   DMAABORT
 *      1   DMAIE      (interrupt enable)
 *      0   DMAIFG     (interrupt flag)
 */
#define DMACTL0_ADDR  0x0122
#define DMA0CTL_ADDR  0x01E0
#define DMA0SA_ADDR   0x01E2
#define DMA0DA_ADDR   0x01E4
#define DMA0SZ_ADDR   0x01E6

#define DMA0TSEL_MASK 0x000F  /* bits 3:0 of DMACTL0 */
#define DMA0TSEL_URXIFG1  9   /* USART1 receive */

#define DMAEN         0x0010  /* bit 4: enable */
#define DMADT_MASK    0xF000  /* bits 15:12 */
#define DMADT_SHIFT   12
#define DMADSTINCR_MASK  0x0C00  /* bits 11:10 */
#define DMADSTINCR_SHIFT 10

static int dma_ctl_read(void *user_data, uint32_t addr, bool word, int64_t cycles) {
    msp430_platform_t *plat = (msp430_platform_t *)user_data;
    (void)word; (void)cycles;
    switch (addr) {
        case DMACTL0_ADDR: return plat->dma0.dmactl0;
        case DMA0CTL_ADDR: return plat->dma0.ctl;
        case DMA0SA_ADDR:  return plat->dma0.sa;
        case DMA0DA_ADDR:  return plat->dma0.da;
        case DMA0SZ_ADDR:  return plat->dma0.sz;
        default:           return 0;
    }
}

static void dma_ctl_write(void *user_data, uint32_t addr, int value, bool word, int64_t cycles) {
    msp430_platform_t *plat = (msp430_platform_t *)user_data;
    (void)word; (void)cycles;
    uint16_t val = (uint16_t)value;
    switch (addr) {
        case DMACTL0_ADDR:
            plat->dma0.dmactl0 = val;
            break;
        case DMA0CTL_ADDR:
            /* When DMAEN is set, snapshot DA and SZ for repeated mode reload */
            if ((val & DMAEN) && !(plat->dma0.ctl & DMAEN)) {
                plat->dma0.da_saved = plat->dma0.da;
                plat->dma0.sz_saved = plat->dma0.sz;
            }
            plat->dma0.ctl = val;
            break;
        case DMA0SA_ADDR:
            plat->dma0.sa = val;
            break;
        case DMA0DA_ADDR:
            plat->dma0.da = val;
            /* Update saved DA if DMA is enabled (firmware may reconfigure) */
            if (plat->dma0.ctl & DMAEN)
                plat->dma0.da_saved = val;
            break;
        case DMA0SZ_ADDR:
            plat->dma0.sz = val;
            /* Update saved SZ if DMA is enabled */
            if (plat->dma0.ctl & DMAEN)
                plat->dma0.sz_saved = val;
            break;
    }
}

/* DMACTL0 at 0x0122 (2 bytes) */
static int dmactl0_read(void *user_data, uint32_t addr, bool word, int64_t cycles) {
    return dma_ctl_read(user_data, DMACTL0_ADDR, word, cycles);
}

static void dmactl0_write(void *user_data, uint32_t addr, int value, bool word, int64_t cycles) {
    dma_ctl_write(user_data, DMACTL0_ADDR, value, word, cycles);
}

void msp430_platform_dma0_trigger(msp430_platform_t *plat, int trigger_source) {
    /* Check if DMA0 is enabled and configured for this trigger */
    if (!(plat->dma0.ctl & DMAEN)) {
        return;
    }
    if ((plat->dma0.dmactl0 & DMA0TSEL_MASK) != (uint16_t)trigger_source) {
        return;
    }

    /* Perform single byte transfer: memory[SA] -> memory[DA] */
    uint16_t sa = plat->dma0.sa;
    uint16_t da = plat->dma0.da;
    uint8_t byte;

    /* Read source byte — use IO read if source has IO callbacks,
     * otherwise read raw memory.  RXBUF1 (0x7E) has IO registered. */
    if (sa < plat->cpu.max_mem_io && plat->cpu.io_read[sa]) {
        byte = (uint8_t)plat->cpu.io_read[sa](plat->cpu.io_user_data[sa],
                                                sa, false, plat->cpu.cycles);
    } else {
        byte = plat->cpu.memory[sa];
    }

    /* Write destination byte — direct memory write (destination is RAM) */
    if (da < plat->cpu.max_mem) {
        plat->cpu.memory[da] = byte;
    }

    /* Destination increment (bits 9:8 of DMA0CTL) */
    uint8_t dstincr = (plat->dma0.ctl & DMADSTINCR_MASK) >> DMADSTINCR_SHIFT;
    if (dstincr == 3)       /* increment */
        plat->dma0.da++;
    else if (dstincr == 2)  /* decrement */
        plat->dma0.da--;
    /* 0 = unchanged */

    /* Decrement transfer count */
    plat->dma0.sz--;

    if (plat->dma0.sz == 0) {
        uint8_t dmadt = (plat->dma0.ctl & DMADT_MASK) >> DMADT_SHIFT;
        if (dmadt >= 4) {
            /* Repeated mode: reload DA and SZ from saved values */
            plat->dma0.da = plat->dma0.da_saved;
            plat->dma0.sz = plat->dma0.sz_saved;
        } else {
            /* Single mode: clear DMAEN */
            plat->dma0.ctl &= ~DMAEN;
        }
    }
}

/* Dummy IO stub — stores bytes in platform's stub_io[] buffer, reads return 0.
 * Used for unmodeled peripherals (USCI A1/B1, ADC12) so firmware accesses
 * don't corrupt raw memory and reads return safe default values. */
#define STUB_IO_SIZE 256

static int stub_io_read(void *user_data, uint32_t addr, bool word, int64_t cycles) {
    msp430_platform_t *plat = (msp430_platform_t *)user_data;
    (void)word; (void)cycles;
    uint32_t off = addr & (STUB_IO_SIZE - 1);
    return plat->stub_io[off];
}

static void stub_io_write(void *user_data, uint32_t addr, int value, bool word, int64_t cycles) {
    msp430_platform_t *plat = (msp430_platform_t *)user_data;
    (void)word; (void)cycles;
    uint32_t off = addr & (STUB_IO_SIZE - 1);
    plat->stub_io[off] = (uint8_t)value;
}

/* --- Platform definitions --- */

static const msp430_platform_config_t platform_sky = {
    .name           = "sky",
    .mcu            = &msp430f1611_config,
    .console_usart  = 1,
    .num_leds       = 3,
    .leds           = { {5, 4}, {5, 5}, {5, 6} },
    .cc2420         = {
        .has_cc2420  = true,
        .cs_port = 4,    .cs_pin = 2,
        .vreg_port = 4,  .vreg_pin = 5,
        .fifop_port = 1, .fifop_pin = 0,
        .fifo_port = 1,  .fifo_pin = 3,
        .cca_port = 1,   .cca_pin = 4,
        .sfd_port = 4,   .sfd_pin = 1,
        .spi_usart = 0,
    },
};

static const msp430_platform_config_t platform_esb = {
    .name           = "esb",
    .mcu            = &msp430f149_config,
    .console_usart  = 1,
    .num_leds       = 3,
    .leds           = { {2, 0}, {2, 1}, {2, 2} },
};

static const msp430_platform_config_t platform_z1 = {
    .name           = "z1",
    .mcu            = &msp430f2617_config,
    .console_usart  = 0,
    .num_leds       = 3,
    .leds           = { {5, 4}, {5, 5}, {5, 6} },
    .cc2420         = {
        .has_cc2420   = true,
        .fifop_port   = 1, .fifop_pin = 2,
        .fifo_port    = 1, .fifo_pin  = 3,
        .cca_port     = 1, .cca_pin   = 4,
        .sfd_port     = 4, .sfd_pin   = 1,
        .vreg_port    = 4, .vreg_pin  = 5,
        .cs_port      = 3, .cs_pin    = 0,
        .spi_usart    = 1,  /* USCI B0 = usart1 */
    },
};

static const msp430_platform_config_t platform_wismote = {
    .name           = "wismote",
    .mcu            = &msp430f5437_config,
    .console_usart  = 1,
    .num_leds       = 3,
    .leds           = { {2, 4}, {5, 2}, {8, 6} },
};

static const msp430_platform_config_t platform_exp5438 = {
    .name           = "exp5438",
    .mcu            = &msp430f5437_config,
    .console_usart  = 1,
    .num_leds       = 2,
    .leds           = { {1, 0}, {1, 1} },
};

static const msp430_platform_config_t platform_cc430 = {
    .name           = "cc430",
    .mcu            = &cc430f5137_config,
    .console_usart  = 0,
    .num_leds       = 0,
};

static const msp430_platform_config_t *all_platforms[] = {
    &platform_sky,
    &platform_esb,
    &platform_z1,
    &platform_wismote,
    &platform_exp5438,
    &platform_cc430,
    NULL
};

const msp430_platform_config_t *msp430_platform_find(const char *name) {
    if (!name) return NULL;
    for (int i = 0; all_platforms[i]; i++) {
        const char *a = name;
        const char *b = all_platforms[i]->name;
        while (*a && *b && tolower((unsigned char)*a) == tolower((unsigned char)*b)) {
            a++;
            b++;
        }
        if (*a == '\0' && *b == '\0') return all_platforms[i];
    }
    return NULL;
}

/* ---- M25P16 SPI flash (Zolertia Z1 external memory) ----
 * Minimal emulation matching MSPSim's M25P80.java.  Responds to
 * READ_STATUS, READ_IDENT, WRITE_ENABLE/DISABLE.  Data reads return 0xFF
 * (erased).  Writes are accepted but not persisted. */

#define FLASH_READ_STATUS   0x05
#define FLASH_READ_IDENT    0x9f
#define FLASH_READ_DATA     0x03
#define FLASH_WRITE_ENABLE  0x06
#define FLASH_WRITE_DISABLE 0x04
#define FLASH_PAGE_PROGRAM  0x02
#define FLASH_SECTOR_ERASE  0xd8
#define FLASH_BULK_ERASE    0xc7
#define FLASH_DEEP_POWER_DOWN 0xb9
#define FLASH_WAKE_UP       0xab

static const uint8_t flash_identity[] = {
    0x20, 0x20, 0x14, 0x10,  /* M25P16: Manufacturer=0x20, Type=0x20, Cap=0x14 */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static uint8_t flash_spi_exchange(msp430_platform_t *plat, uint8_t byte) {
    if (!plat->flash.chip_select) return 0;

    switch (plat->flash.state) {
    case FLASH_READ_STATUS:
        /* Return status: bit 1=WEL, bit 0=WIP (write in progress) */
        return plat->flash.status | (plat->flash.write_enable ? 0x02 : 0);
    case FLASH_READ_IDENT:
        if (plat->flash.pos < (int)sizeof(flash_identity))
            return flash_identity[plat->flash.pos++];
        return 0;
    case FLASH_READ_DATA:
        if (plat->flash.pos < 3) {
            plat->flash.address = (plat->flash.address << 8) | byte;
            plat->flash.pos++;
            return 0;
        }
        plat->flash.address++;
        return 0xFF;  /* erased flash */
    case FLASH_PAGE_PROGRAM:
    case FLASH_SECTOR_ERASE:
        if (plat->flash.pos < 3) {
            plat->flash.address = (plat->flash.address << 8) | byte;
            plat->flash.pos++;
            return 0;
        }
        return 0;  /* accept but don't persist */
    }

    /* First byte = command */
    switch (byte) {
    case FLASH_WRITE_ENABLE:  plat->flash.write_enable = true;  break;
    case FLASH_WRITE_DISABLE: plat->flash.write_enable = false; break;
    case FLASH_READ_STATUS:
        plat->flash.state = FLASH_READ_STATUS;
        return 0;
    case FLASH_READ_IDENT:
        plat->flash.state = FLASH_READ_IDENT;
        plat->flash.pos = 0;
        return 0;
    case FLASH_READ_DATA:
        plat->flash.state = FLASH_READ_DATA;
        plat->flash.pos = plat->flash.address = 0;
        break;
    case FLASH_PAGE_PROGRAM:
        plat->flash.state = FLASH_PAGE_PROGRAM;
        plat->flash.pos = plat->flash.address = 0;
        break;
    case FLASH_SECTOR_ERASE:
        plat->flash.state = FLASH_SECTOR_ERASE;
        plat->flash.pos = 0;
        break;
    case FLASH_BULK_ERASE:
    case FLASH_DEEP_POWER_DOWN:
    case FLASH_WAKE_UP:
        break;
    }
    return 0;
}

/* Z1 flash chip select: P4.4 (active low) */
#define Z1_FLASH_CS_PORT 4
#define Z1_FLASH_CS_PIN  4

/* SFD pin change callback: forward to Timer B CCR1 capture input.
 * On Z1/Sky, the CC2420 SFD pin is connected to Timer B CCR1 via P4SEL.
 * The firmware uses TBCCR1 to timestamp received frames for TSCH. */
static void platform_sfd_changed(void *data, bool value) {
    msp430_platform_t *plat = (msp430_platform_t *)data;
    if (!plat) return;
    msp430_timer_capture_input(&plat->timer_b, 1, value);
}

/* GPIO output callback — detects CC2420 CS, VREG, and flash CS changes */
static void platform_gpio_changed(void *data, int port, uint8_t old_out, uint8_t new_out) {
    msp430_platform_t *plat = (msp430_platform_t *)data;
    const msp430_cc2420_config_t *rc = &plat->config->cc2420;
    uint8_t changed = old_out ^ new_out;

    if (rc->has_cc2420) {
        if (port == rc->cs_port && (changed & (1 << rc->cs_pin))) {
            bool cs_active = !(new_out & (1 << rc->cs_pin));
            cc2420_set_chip_select(&plat->cc2420, cs_active);
        }
        if (port == rc->vreg_port && (changed & (1 << rc->vreg_pin))) {
            bool vreg_on = (new_out & (1 << rc->vreg_pin)) != 0;
            cc2420_set_vreg(&plat->cc2420, vreg_on);
        }
    }

    /* M25P16 flash CS on Z1: P4.4 active low */
    if (port == Z1_FLASH_CS_PORT && (changed & (1 << Z1_FLASH_CS_PIN))) {
        bool was_selected = plat->flash.chip_select;
        plat->flash.chip_select = !(new_out & (1 << Z1_FLASH_CS_PIN));
        if (was_selected && !plat->flash.chip_select) {
            /* CS deasserted — reset state (like M25P80.portWrite) */
            plat->flash.state = 0;
        }
    }
}

/* SPI exchange callback — routes to CC2420 or M25P16 flash based on CS.
 * Returns -1 if no device responds (matching MSPSim: RXIFG not set).
 * On Z1 (shared SPI bus), both CC2420 and flash get every byte; if
 * neither CS is active or CC2420 VREG is off, no device responds.
 * On Sky (single device), CC2420 handles its own CS check internally. */
static int platform_spi_exchange(void *data, uint8_t byte) {
    msp430_platform_t *plat = (msp430_platform_t *)data;
    const msp430_cc2420_config_t *rc = &plat->config->cc2420;
    /* Z1/MSP430X: shared SPI bus with flash — route by CS.
     * Flash gets priority when its CS is active. Otherwise always
     * route to CC2420 (like Sky). The CC2420 handles chip_select
     * internally, returning a status byte even when CS is inactive.
     * Returning -1 would prevent RXIFG from being set, causing the
     * firmware's SPI_WAITFOREORx() to hang indefinitely. */
    if (plat->config->mcu->is_msp430x) {
        if (plat->flash.chip_select)
            return flash_spi_exchange(plat, byte);
        if (rc->has_cc2420)
            return cc2420_spi_exchange(&plat->cc2420, byte);
        return 0;
    }

    /* Sky/classic: CC2420 is the only SPI device, handles CS internally */
    if (rc->has_cc2420)
        return cc2420_spi_exchange(&plat->cc2420, byte);
    return 0;
}

void msp430_platform_init(msp430_platform_t *plat,
                            const msp430_platform_config_t *config) {
    memset(plat, 0, sizeof(*plat));
    plat->config = config;
    const msp430_config_t *mcu = config->mcu;

    /* CPU */
    msp430_cpu_init(&plat->cpu, mcu);

    /* SFR registers (IE1/IE2/IFG1/IFG2) */
    msp430_register_io(&plat->cpu, 0x00, 8, sfr_read, sfr_write, plat);

    /* Pre-set SPI RX flags so firmware doesn't hang polling for SPI
     * devices we don't emulate (e.g., Z1's M25P16 flash via USCI_B0).
     * IFG2 bit 1 = UCA0TXIFG, bit 3 = UCB0RXIFG. */
    if (mcu->is_msp430x) {
        plat->sfr_regs[3] |= 0x0E;  /* UCB0TXIFG(3) | UCB0RXIFG(2) | UCA0TXIFG(1) */
    }

    /* Hardware multiplier (16-bit: 0x0130-0x013E) */
    msp430_register_io(&plat->cpu, HW_MPY_BASE, 16, hwmul_read, hwmul_write, plat);

    /* DMA controller — DMACTL0 at 0x0122 (2 bytes) + DMA0 regs at 0x01E0 (8 bytes) */
    msp430_register_io(&plat->cpu, DMACTL0_ADDR, 2, dmactl0_read, dmactl0_write, plat);
    msp430_register_io(&plat->cpu, DMA0CTL_ADDR, 8, dma_ctl_read, dma_ctl_write, plat);

    /* Clock */
    msp430_clock_init(&plat->clock, &plat->cpu, mcu->bcs_base, mcu->max_dco_freq);

    /* Timer A */
    msp430_timer_init(&plat->timer_a, &plat->cpu, &plat->clock, "Timer_A",
                       mcu->timer_a_base, mcu->timer_a_iv,
                       mcu->timer_a_num_ccr, mcu->timer_a_ccr0_vec,
                       mcu->timer_a_ccr1_vec);
    msp430_clock_add_timer(&plat->clock, &plat->timer_a);

    /* Timer B */
    if (mcu->timer_b_base != 0) {
        msp430_timer_init(&plat->timer_b, &plat->cpu, &plat->clock, "Timer_B",
                           mcu->timer_b_base, mcu->timer_b_iv,
                           mcu->timer_b_num_ccr, mcu->timer_b_ccr0_vec,
                           mcu->timer_b_ccr1_vec);
        msp430_clock_add_timer(&plat->clock, &plat->timer_b);
    }

    /* GPIO */
    msp430_gpio_init_from_config(&plat->gpio, &plat->cpu, mcu);

    /* USARTs */
    if (mcu->usart0_base != 0) {
        msp430_usart_init(&plat->usart0, &plat->cpu,
                           mcu->usart0_base, mcu->usart_tx_offset);
        plat->usart0.is_usci = mcu->is_msp430x;
        if (mcu->is_msp430x) {
            /* USCI_A0: IFG2 (0x03) bit 1 = TX, bit 0 = RX
             * IE2 (0x01) bit 1 = TXIE, bit 0 = RXIE
             * Vector 23 = USCIAB0TX, Vector 22 = USCIAB0RX */
            msp430_usart_set_ifg(&plat->usart0, &plat->sfr_regs[3], 0x02);
            msp430_usart_set_ie(&plat->usart0, &plat->sfr_regs[1], 0x02, 23, 22);
        } else {
            /* Classic USART0: IFG1 (0x02) bit 7 = TX, bit 6 = RX
             * IE1 (0x00) bit 7 = UTXIE0, bit 6 = URXIE0
             * Vector 9 = USART0TX, Vector 8 = USART0RX */
            msp430_usart_set_ifg(&plat->usart0, &plat->sfr_regs[2], 0x80);
            msp430_usart_set_ie(&plat->usart0, &plat->sfr_regs[0], 0x80, 9, 8);
        }
    }
    if (mcu->usart1_base != 0) {
        msp430_usart_init(&plat->usart1, &plat->cpu,
                           mcu->usart1_base, mcu->usart_tx_offset);
        plat->usart1.is_usci = mcu->is_msp430x;
        if (mcu->is_msp430x) {
            /* USCI_B0: IFG2 (0x03) bit 3 = TX, bit 2 = RX
             * IE2 (0x01) bit 3 = TXIE, bit 2 = RXIE
             * Vector 23 = USCIAB0TX, Vector 22 = USCIAB0RX (shared with USCI_A0) */
            msp430_usart_set_ifg(&plat->usart1, &plat->sfr_regs[3], 0x08);
            msp430_usart_set_ie(&plat->usart1, &plat->sfr_regs[1], 0x08, 23, 22);
        } else {
            /* Classic USART1: IFG2 (0x03) bit 5 = TX, bit 4 = RX
             * IE2 (0x01) bit 5 = UTXIE1, bit 4 = URXIE1
             * Vector 7 = USART1TX, Vector 6 = USART1RX */
            msp430_usart_set_ifg(&plat->usart1, &plat->sfr_regs[3], 0x20);
            msp430_usart_set_ie(&plat->usart1, &plat->sfr_regs[1], 0x20, 7, 6);
        }
    }

    /* Connect DMA0 trigger to USART1 RX (trigger source 9).
     * Only for classic USART (F1611/F149) — USCI platforms use different DMA. */
    if (mcu->usart1_base != 0 && !mcu->is_msp430x) {
        plat->usart1.dma_trigger = (void (*)(void *, int))msp430_platform_dma0_trigger;
        plat->usart1.dma_data = plat;
        plat->usart1.dma_trigger_source = DMA0TSEL_URXIFG1;
    }

    /* USCI A1/B1 stubs (0xD0-0xDF) for MSP430X platforms like F2617.
     * These are not fully modeled but firmware may access them. */
    if (mcu->is_msp430x && mcu->usart0_base == 0x60) {
        msp430_register_io(&plat->cpu, 0xD0, 16, stub_io_read, stub_io_write, plat);
    }

    /* ADC12 stubs — firmware may poll ADC12CTL1.BUSY (returns 0 = not busy).
     * F2617 ADC12 registers: 0x080-0x08F (control), 0x140-0x15E (memory),
     * 0x1A0-0x1AF (memory control). Register stubs for all ranges. */
    if (mcu->is_msp430x && mcu->usart0_base == 0x60) {
        msp430_register_io(&plat->cpu, 0x080, 16, stub_io_read, stub_io_write, plat);
        msp430_register_io(&plat->cpu, 0x140, 32, stub_io_read, stub_io_write, plat);
        msp430_register_io(&plat->cpu, 0x1A0, 16, stub_io_read, stub_io_write, plat);
    }

    /* Build the CPU-agnostic host vtable used by off-SoC chip drivers. */
    plat->host.cpu            = &plat->cpu;
    plat->host.gpio           = &plat->gpio;
    plat->host.now_ns         = msp430_host_now_ns;
    plat->host.schedule_ns    = msp430_host_schedule_ns;
    plat->host.cancel         = msp430_host_cancel;
    plat->host.set_input_pin  = msp430_host_set_input_pin;
    plat->host.force_irq_edge = msp430_host_force_irq_edge;

    /* CC2420 radio */
    if (config->cc2420.has_cc2420) {
        const msp430_cc2420_config_t *rc = &config->cc2420;

        cc2420_init(&plat->cc2420, &plat->host);
        cc2420_set_pins(&plat->cc2420,
                         rc->fifop_port, rc->fifop_pin,
                         rc->fifo_port, rc->fifo_pin,
                         rc->cca_port, rc->cca_pin,
                         rc->sfd_port, rc->sfd_pin);

        /* Connect SFD pin to Timer B CCR1 capture input.
         * The CC2420 SFD pin on Z1/Sky is captured by Timer B CCR1
         * for TSCH packet timestamping (cc2420_sfd_start_time). */
        /* Connect SFD pin to Timer B CCR1 capture input.
         * The CC2420 SFD pin on Z1/Sky is captured by Timer B CCR1
         * for TSCH packet timestamping (cc2420_sfd_start_time). */
        plat->cc2420.sfd_callback = platform_sfd_changed;
        plat->cc2420.sfd_callback_data = plat;

        /* Register GPIO output callback for CS/VREG detection */
        msp430_gpio_set_output_callback(&plat->gpio, platform_gpio_changed, plat);

        /* Register SPI exchange on the designated USART */
        msp430_usart_t *spi_usart = (rc->spi_usart == 0) ?
                                     &plat->usart0 : &plat->usart1;
        /* RX buffer offset from USART/USCI base address:
         *   Classic USART (Sky): URXBUF at base+6
         *   USCI A0 (UART): UCA0RXBUF at base+6 (0x60+6=0x66)
         *   USCI B0 (SPI):  UCB0RXBUF at base+6 (0x68+6=0x6E) */
        msp430_usart_set_spi_exchange(spi_usart, platform_spi_exchange,
                                       plat, 6);
    }
}

void msp430_platform_destroy(msp430_platform_t *plat) {
    msp430_cpu_destroy(&plat->cpu);
}

void msp430_platform_set_console(msp430_platform_t *plat,
                                   usart_tx_callback cb, void *user_data) {
    if (plat->config->console_usart == 0) {
        msp430_usart_set_callback(&plat->usart0, cb, user_data);
    } else {
        msp430_usart_set_callback(&plat->usart1, cb, user_data);
    }
}
