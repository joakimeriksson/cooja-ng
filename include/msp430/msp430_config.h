/*
 * MSP430 MCU configuration definitions
 */
#ifndef MSP430_CONFIG_H
#define MSP430_CONFIG_H

#include <stdint.h>
#include <stdbool.h>

#define MSP430_MAX_GPIO_PORTS 10

typedef struct {
    uint32_t base_addr;   /* 0 = port not present */
    int      interrupt_vector;  /* -1 = no interrupt capability */
} gpio_port_config_t;

typedef struct msp430_config {
    const char *name;
    bool     is_msp430x;
    uint32_t max_mem;
    uint32_t max_mem_io;
    int      max_interrupt;

    uint32_t ram_start;
    uint32_t ram_size;
    uint32_t ram_mirror_start;  /* 0 if no mirror */
    uint32_t ram_mirror_size;

    uint32_t main_flash_start;
    uint32_t main_flash_size;
    uint32_t info_mem_start;
    uint32_t info_mem_size;

    /* Peripherals */
    uint32_t usart0_base;       /* USART0 base address */
    uint32_t usart1_base;       /* USART1 base address */
    uint32_t usart_tx_offset;   /* TX buffer register offset within USART */

    /* Timer A */
    uint32_t timer_a_base;      /* 0x160 for f1611, 0x340 for f5437 */
    uint32_t timer_a_iv;        /* 0x12E for f1611, 0x36E for f5437 */
    int      timer_a_num_ccr;   /* 3 or 5 */
    int      timer_a_ccr0_vec;  /* interrupt vector for CCR0 */
    int      timer_a_ccr1_vec;  /* shared vector for CCR1+ and overflow */

    /* Timer B */
    uint32_t timer_b_base;      /* 0x180 for f1611, 0x3C0 for f5437 */
    uint32_t timer_b_iv;        /* 0x11E for f1611, 0x3EE for f5437 */
    int      timer_b_num_ccr;   /* 3 or 7 */
    int      timer_b_ccr0_vec;  /* interrupt vector for CCR0 */
    int      timer_b_ccr1_vec;  /* shared vector for CCR1+ and overflow */

    /* Clock */
    uint32_t bcs_base;          /* 0x56 for f1611, 0x160 for fr5969 */
    uint32_t max_dco_freq;      /* 4915200 for f1611 */
    int      clock_type;        /* 0=BCS (classic), 1=CS (FR5xxx) */

    /* Hardware multiplier */
    uint32_t hwmul_base;        /* 0x130 for classic, 0x4C0 for FR5xxx */

    /* USCI variant */
    bool     is_eusci;          /* true for FR5xxx eUSCI; false for classic USART or USCI */

    /* GPIO */
    int              gpio_num_ports;
    gpio_port_config_t gpio_ports[MSP430_MAX_GPIO_PORTS];
} msp430_config_t;

/* Pre-defined configurations */
extern const msp430_config_t msp430f149_config;
extern const msp430_config_t msp430f1611_config;
extern const msp430_config_t msp430f2617_config;
extern const msp430_config_t msp430f5437_config;
extern const msp430_config_t cc430f5137_config;
extern const msp430_config_t msp430fr5969_config;

/* Lookup by name (case-insensitive), returns NULL if not found */
const msp430_config_t *msp430_config_find(const char *name);

#endif /* MSP430_CONFIG_H */
