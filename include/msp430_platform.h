/*
 * MSP430 platform — bundles MCU config with all peripheral instances.
 */
#ifndef MSP430_PLATFORM_H
#define MSP430_PLATFORM_H

#include "msp430_cpu.h"
#include "msp430_config.h"
#include "msp430_clock.h"
#include "msp430_timer.h"
#include "msp430_gpio.h"
#include "msp430_usart.h"
#include "cc2420.h"

/* LED definition: port number (1-based) and pin (0-7) */
#define MSP430_PLATFORM_MAX_LEDS 3

typedef struct {
    int port;
    int pin;
} msp430_led_config_t;

/* CC2420 radio pin/SPI configuration */
typedef struct {
    bool has_cc2420;
    int  cs_port, cs_pin;       /* chip select (active low) */
    int  vreg_port, vreg_pin;   /* voltage regulator enable */
    int  fifop_port, fifop_pin; /* FIFOP output to MCU */
    int  fifo_port, fifo_pin;   /* FIFO output to MCU */
    int  cca_port, cca_pin;     /* CCA output to MCU */
    int  sfd_port, sfd_pin;     /* SFD output to MCU */
    int  spi_usart;             /* which USART for SPI (0 or 1) */
} msp430_cc2420_config_t;

/* Static platform configuration */
typedef struct msp430_platform_config {
    const char            *name;
    const msp430_config_t *mcu;
    int                    console_usart;  /* 0 or 1 */
    int                    num_leds;
    msp430_led_config_t    leds[MSP430_PLATFORM_MAX_LEDS];
    msp430_cc2420_config_t cc2420;
} msp430_platform_config_t;

/* Platform runtime state */
typedef struct msp430_platform {
    msp430_cpu_t    cpu;
    msp430_clock_t  clock;
    msp430_timer_t  timer_a;
    msp430_timer_t  timer_b;
    msp430_gpio_t   gpio;
    msp430_usart_t  usart0;
    msp430_usart_t  usart1;
    cc2420_t        cc2420;
    uint8_t         sfr_regs[8];
    /* Hardware multiplier state */
    uint16_t        mpy_op1;      /* MPY/MPYS/MAC/MACS operand 1 */
    uint8_t         mpy_mode;     /* 0=MPY, 1=MPYS, 2=MAC, 3=MACS */
    uint16_t        mpy_reslo;    /* RESLO */
    uint16_t        mpy_reshi;    /* RESHI */
    uint16_t        mpy_sumext;   /* SUMEXT */
    const msp430_platform_config_t *config;
} msp430_platform_t;

/* Initialize all peripherals from platform config */
void msp430_platform_init(msp430_platform_t *plat,
                            const msp430_platform_config_t *config);

/* Clean up */
void msp430_platform_destroy(msp430_platform_t *plat);

/* Connect a TX callback to the console USART */
void msp430_platform_set_console(msp430_platform_t *plat,
                                   usart_tx_callback cb, void *user_data);

/* Lookup platform by name (case-insensitive), returns NULL if not found */
const msp430_platform_config_t *msp430_platform_find(const char *name);

#endif /* MSP430_PLATFORM_H */
