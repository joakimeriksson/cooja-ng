/*
 * MSP430 MCU configuration definitions
 */
#include "msp430_config.h"
#include <string.h>
#include <ctype.h>

/* MSP430f149: 16-bit classic, 64KB address space */
const msp430_config_t msp430f149_config = {
    .name             = "MSP430f149",
    .is_msp430x       = false,
    .max_mem          = 0x10000,     /* 64KB */
    .max_mem_io       = 0x200,       /* IO space 0x0000-0x01FF */
    .max_interrupt    = 15,

    .ram_start        = 0x0200,
    .ram_size         = 2 * 1024,    /* 2KB */
    .ram_mirror_start = 0,
    .ram_mirror_size  = 0,

    .main_flash_start = 0x1100,
    .main_flash_size  = 60 * 1024,   /* 60KB */
    .info_mem_start   = 0x1000,
    .info_mem_size    = 256,

    .usart0_base      = 0x70,
    .usart1_base      = 0x78,
    .usart_tx_offset  = 7,           /* TX buffer at base + 7 */

    .timer_a_base     = 0x160,
    .timer_a_iv       = 0x12E,
    .timer_a_num_ccr  = 3,
    .timer_a_ccr0_vec = 6,
    .timer_a_ccr1_vec = 5,

    .timer_b_base     = 0x180,
    .timer_b_iv       = 0x11E,
    .timer_b_num_ccr  = 7,
    .timer_b_ccr0_vec = 13,
    .timer_b_ccr1_vec = 12,

    .bcs_base         = 0x56,
    .max_dco_freq     = 4915200,

    .gpio_num_ports   = 6,
    .gpio_ports       = {
        { .base_addr = 0x20, .interrupt_vector = 4 },   /* P1 */
        { .base_addr = 0x28, .interrupt_vector = 1 },   /* P2 */
        { .base_addr = 0x18, .interrupt_vector = -1 },  /* P3 */
        { .base_addr = 0x1C, .interrupt_vector = -1 },  /* P4 */
        { .base_addr = 0x30, .interrupt_vector = -1 },  /* P5 */
        { .base_addr = 0x34, .interrupt_vector = -1 },  /* P6 */
    },
};

/* MSP430f1611: 16-bit classic, 64KB address space */
const msp430_config_t msp430f1611_config = {
    .name             = "MSP430f1611",
    .is_msp430x       = false,
    .max_mem          = 0x10000,     /* 64KB */
    .max_mem_io       = 0x200,       /* IO space 0x0000-0x01FF */
    .max_interrupt    = 15,

    .ram_start        = 0x1100,
    .ram_size         = 10 * 1024,   /* 10KB */
    .ram_mirror_start = 0x0200,
    .ram_mirror_size  = 2 * 1024,    /* 2KB mirror */

    .main_flash_start = 0x4000,
    .main_flash_size  = 48 * 1024,   /* 48KB */
    .info_mem_start   = 0x1000,
    .info_mem_size    = 256,

    .usart0_base      = 0x70,
    .usart1_base      = 0x78,
    .usart_tx_offset  = 7,           /* TX buffer at base + 7 */

    .timer_a_base     = 0x160,
    .timer_a_iv       = 0x12E,
    .timer_a_num_ccr  = 3,
    .timer_a_ccr0_vec = 6,
    .timer_a_ccr1_vec = 5,

    .timer_b_base     = 0x180,
    .timer_b_iv       = 0x11E,
    .timer_b_num_ccr  = 7,
    .timer_b_ccr0_vec = 13,
    .timer_b_ccr1_vec = 12,

    .bcs_base         = 0x56,
    .max_dco_freq     = 4915200,

    .gpio_num_ports   = 6,
    .gpio_ports       = {
        { .base_addr = 0x20, .interrupt_vector = 4 },   /* P1 */
        { .base_addr = 0x28, .interrupt_vector = 1 },   /* P2 */
        { .base_addr = 0x18, .interrupt_vector = -1 },  /* P3 */
        { .base_addr = 0x1C, .interrupt_vector = -1 },  /* P4 */
        { .base_addr = 0x30, .interrupt_vector = -1 },  /* P5 */
        { .base_addr = 0x34, .interrupt_vector = -1 },  /* P6 */
    },
};

/* MSP430f2617: MSP430X, 20-bit address space */
const msp430_config_t msp430f2617_config = {
    .name             = "MSP430f2617",
    .is_msp430x       = true,
    .max_mem          = 0x100000,    /* 1MB */
    .max_mem_io       = 0x200,       /* IO space 0x0000-0x01FF */
    .max_interrupt    = 31,          /* MSP430X: 32 vectors */

    .ram_start        = 0x1100,
    .ram_size         = 8 * 1024,    /* 8KB */
    .ram_mirror_start = 0x0200,
    .ram_mirror_size  = 2 * 1024,    /* 2KB mirror of 0x1100-0x18FF */

    .main_flash_start = 0x3100,
    .main_flash_size  = 92 * 1024,   /* 92KB */
    .info_mem_start   = 0x1000,
    .info_mem_size    = 256,

    .usart0_base      = 0x60,       /* USCI_A0 */
    .usart1_base      = 0x68,       /* USCI_A1 (actually USCI_B0 area, simplified) */
    .usart_tx_offset  = 7,           /* TXBUF offset */

    .timer_a_base     = 0x160,
    .timer_a_iv       = 0x12E,
    .timer_a_num_ccr  = 3,
    .timer_a_ccr0_vec = 25,          /* __isr_25: timera0 (CCR0) */
    .timer_a_ccr1_vec = 24,          /* __isr_24: timera1 (CCR1+, overflow) */

    .timer_b_base     = 0x180,
    .timer_b_iv       = 0x11E,
    .timer_b_num_ccr  = 7,
    .timer_b_ccr0_vec = 29,          /* __isr_29: Timer_B CCR0 */
    .timer_b_ccr1_vec = 28,          /* __isr_28: cc2420_timerb1 (CCR1+) */

    .bcs_base         = 0x56,
    .max_dco_freq     = 9000000,       /* matches MSPSim getMaxClockSpeed() */

    .gpio_num_ports   = 8,
    .gpio_ports       = {
        { .base_addr = 0x20, .interrupt_vector = 18 },  /* P1: __isr_18 (port1_isr) */
        { .base_addr = 0x28, .interrupt_vector = 19 },  /* P2: __isr_19 (irq_p2) */
        { .base_addr = 0x18, .interrupt_vector = -1 },  /* P3 */
        { .base_addr = 0x1C, .interrupt_vector = -1 },  /* P4 */
        { .base_addr = 0x30, .interrupt_vector = -1 },  /* P5 */
        { .base_addr = 0x34, .interrupt_vector = -1 },  /* P6 */
        { .base_addr = 0x38, .interrupt_vector = -1 },  /* P7 */
        { .base_addr = 0x3C, .interrupt_vector = -1 },  /* P8 */
    },
};

/* MSP430f5437: 20-bit MSP430X, 1MB address space */
const msp430_config_t msp430f5437_config = {
    .name             = "MSP430f5437",
    .is_msp430x       = true,
    .max_mem          = 0x100000,    /* 1MB */
    .max_mem_io       = 0x800,       /* IO space 0x0000-0x07FF */
    .max_interrupt    = 63,

    .ram_start        = 0x1C00,
    .ram_size         = 16 * 1024,   /* 16KB */
    .ram_mirror_start = 0,           /* no mirror */
    .ram_mirror_size  = 0,

    .main_flash_start = 0x5C00,
    .main_flash_size  = 256 * 1024,  /* 256KB */
    .info_mem_start   = 0x1800,
    .info_mem_size    = 512,

    .usart0_base      = 0x5C0,      /* USCI_A0 */
    .usart1_base      = 0x600,      /* USCI_A1 */
    .usart_tx_offset  = 14,          /* TXBUF offset for USCI */

    .timer_a_base     = 0x340,       /* Timer A0 (TA0) */
    .timer_a_iv       = 0x36E,       /* TA0IV */
    .timer_a_num_ccr  = 5,
    .timer_a_ccr0_vec = 53,          /* TA0CCR0 */
    .timer_a_ccr1_vec = 52,          /* TA0CCR1-4, overflow */

    .timer_b_base     = 0x3C0,       /* Timer B0 (TB0) */
    .timer_b_iv       = 0x3EE,       /* TB0IV */
    .timer_b_num_ccr  = 7,
    .timer_b_ccr0_vec = 45,          /* TB0CCR0 */
    .timer_b_ccr1_vec = 44,          /* TB0CCR1-6, overflow */

    .bcs_base         = 0x160,       /* UCS base for 5xxx */
    .max_dco_freq     = 25000000,

    .gpio_num_ports   = 10,
    .gpio_ports       = {
        { .base_addr = 0x200, .interrupt_vector = -1 },  /* P1 (TODO: 47 for MSP430X) */
        { .base_addr = 0x200, .interrupt_vector = -1 },  /* P2 (paired with P1) */
        { .base_addr = 0x220, .interrupt_vector = -1 },  /* P3 */
        { .base_addr = 0x220, .interrupt_vector = -1 },  /* P4 (paired with P3) */
        { .base_addr = 0x240, .interrupt_vector = -1 },  /* P5 */
        { .base_addr = 0x240, .interrupt_vector = -1 },  /* P6 (paired with P5) */
        { .base_addr = 0x260, .interrupt_vector = -1 },  /* P7 */
        { .base_addr = 0x260, .interrupt_vector = -1 },  /* P8 (paired with P7) */
        { .base_addr = 0x280, .interrupt_vector = -1 },  /* P9 */
        { .base_addr = 0x280, .interrupt_vector = -1 },  /* P10 (paired with P9) */
    },
};

/* CC430f5137: MSP430X with integrated radio */
const msp430_config_t cc430f5137_config = {
    .name             = "CC430f5137",
    .is_msp430x       = true,
    .max_mem          = 0x100000,    /* 1MB */
    .max_mem_io       = 0x800,       /* IO space 0x0000-0x07FF */
    .max_interrupt    = 63,

    .ram_start        = 0x1C00,
    .ram_size         = 4 * 1024,    /* 4KB */
    .ram_mirror_start = 0,
    .ram_mirror_size  = 0,

    .main_flash_start = 0x8000,
    .main_flash_size  = 32 * 1024,   /* 32KB */
    .info_mem_start   = 0x1800,
    .info_mem_size    = 512,

    .usart0_base      = 0x5C0,      /* USCI_A0 */
    .usart1_base      = 0,          /* No USCI_A1 */
    .usart_tx_offset  = 14,

    .timer_a_base     = 0x340,       /* Timer A0 (TA0) */
    .timer_a_iv       = 0x36E,
    .timer_a_num_ccr  = 5,
    .timer_a_ccr0_vec = 53,
    .timer_a_ccr1_vec = 52,

    .timer_b_base     = 0x380,       /* Timer A1 (TA1) */
    .timer_b_iv       = 0x3AE,
    .timer_b_num_ccr  = 3,
    .timer_b_ccr0_vec = 49,
    .timer_b_ccr1_vec = 48,

    .bcs_base         = 0x160,
    .max_dco_freq     = 25000000,

    .gpio_num_ports   = 6,
    .gpio_ports       = {
        { .base_addr = 0x200, .interrupt_vector = -1 },  /* P1 (TODO: for MSP430X) */
        { .base_addr = 0x200, .interrupt_vector = -1 },  /* P2 (paired with P1) */
        { .base_addr = 0x220, .interrupt_vector = -1 },  /* P3 */
        { .base_addr = 0x220, .interrupt_vector = -1 },  /* P4 (paired with P3) */
        { .base_addr = 0x240, .interrupt_vector = -1 },  /* P5 */
        { .base_addr = 0x240, .interrupt_vector = -1 },  /* P6 (paired with P5) */
    },
};

/* All known MCU configs */
static const msp430_config_t *all_configs[] = {
    &msp430f149_config,
    &msp430f1611_config,
    &msp430f2617_config,
    &msp430f5437_config,
    &cc430f5137_config,
    NULL
};

const msp430_config_t *msp430_config_find(const char *name) {
    if (!name) return NULL;
    for (int i = 0; all_configs[i]; i++) {
        /* Case-insensitive comparison */
        const char *a = name;
        const char *b = all_configs[i]->name;
        while (*a && *b && tolower((unsigned char)*a) == tolower((unsigned char)*b)) {
            a++;
            b++;
        }
        if (*a == '\0' && *b == '\0') return all_configs[i];
    }
    return NULL;
}
