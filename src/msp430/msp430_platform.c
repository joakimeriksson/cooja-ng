/*
 * MSP430 platform — bundles MCU config with all peripheral instances.
 */
#include "msp430_platform.h"
#include <string.h>
#include <ctype.h>
#include <stdio.h>

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
    if (addr < 8) plat->sfr_regs[addr] = (uint8_t)value;
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
        case HW_OP2:    return 0;
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

/* GPIO output callback — detects CC2420 chip select and VREG changes */
static void platform_gpio_changed(void *data, int port, uint8_t old_out, uint8_t new_out) {
    msp430_platform_t *plat = (msp430_platform_t *)data;
    const msp430_cc2420_config_t *rc = &plat->config->cc2420;
    if (!rc->has_cc2420) return;

    uint8_t changed = old_out ^ new_out;

    if (port == rc->cs_port && (changed & (1 << rc->cs_pin))) {
        /* CS is active LOW */
        bool cs_active = !(new_out & (1 << rc->cs_pin));
        cc2420_set_chip_select(&plat->cc2420, cs_active);
    }
    if (port == rc->vreg_port && (changed & (1 << rc->vreg_pin))) {
        bool vreg_on = (new_out & (1 << rc->vreg_pin)) != 0;
        cc2420_set_vreg(&plat->cc2420, vreg_on);
    }
}

/* SPI exchange callback — bridges USART TX to CC2420 */
static uint8_t platform_spi_exchange(void *data, uint8_t byte) {
    cc2420_t *radio = (cc2420_t *)data;
    return cc2420_spi_exchange(radio, byte);
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

    /* Hardware multiplier (16-bit: 0x0130-0x013E) */
    msp430_register_io(&plat->cpu, HW_MPY_BASE, 16, hwmul_read, hwmul_write, plat);

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
        /* Connect to IFG: USART0 TX flag = IFG1 bit 7 (classic) or IFG2 bit 1 (USCI) */
        if (!mcu->is_msp430x) {
            msp430_usart_set_ifg(&plat->usart0, &plat->sfr_regs[2], 0x80);
        }
    }
    if (mcu->usart1_base != 0) {
        msp430_usart_init(&plat->usart1, &plat->cpu,
                           mcu->usart1_base, mcu->usart_tx_offset);
        /* Connect to IFG: USART1 TX flag = IFG2 bit 5 (classic) */
        if (!mcu->is_msp430x) {
            msp430_usart_set_ifg(&plat->usart1, &plat->sfr_regs[3], 0x20);
        }
    }

    /* CC2420 radio */
    if (config->cc2420.has_cc2420) {
        const msp430_cc2420_config_t *rc = &config->cc2420;

        cc2420_init(&plat->cc2420, &plat->cpu, &plat->gpio);
        cc2420_set_pins(&plat->cc2420,
                         rc->fifop_port, rc->fifop_pin,
                         rc->fifo_port, rc->fifo_pin,
                         rc->cca_port, rc->cca_pin,
                         rc->sfd_port, rc->sfd_pin);

        /* Register GPIO output callback for CS/VREG detection */
        msp430_gpio_set_output_callback(&plat->gpio, platform_gpio_changed, plat);

        /* Register SPI exchange on the designated USART */
        msp430_usart_t *spi_usart = (rc->spi_usart == 0) ?
                                     &plat->usart0 : &plat->usart1;
        /* RX offset: 6 for classic USART (URXBUF0 = base+6) */
        msp430_usart_set_spi_exchange(spi_usart, platform_spi_exchange,
                                       &plat->cc2420, 6);
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
