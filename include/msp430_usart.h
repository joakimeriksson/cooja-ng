/*
 * Minimal USART stub for capturing TX output from firmware tests
 */
#ifndef MSP430_USART_H
#define MSP430_USART_H

#include "msp430_cpu.h"

typedef void (*usart_tx_callback)(void *user_data, uint8_t byte);

/* SPI exchange: firmware sends tx_byte, device returns response byte */
typedef uint8_t (*usart_spi_exchange_fn)(void *user_data, uint8_t tx_byte);

typedef struct msp430_usart {
    msp430_cpu_t      *cpu;
    uint32_t           base_addr;
    uint32_t           tx_offset;
    uint32_t           rx_offset;      /* RX register offset (for SPI reads) */
    usart_tx_callback  tx_callback;
    void              *tx_user_data;
    usart_spi_exchange_fn spi_exchange;
    void              *spi_exchange_data;
    uint8_t            rx_buf;         /* Last received SPI byte */
    uint8_t           *ifg_ptr;       /* Pointer to IFG register variable */
    uint8_t            ifg_tx_mask;   /* Bit mask for UTXIFG in IFG register */
    uint8_t            ifg_rx_mask;   /* Bit mask for URXIFG in IFG register */
} msp430_usart_t;

/* Initialize and register a USART at the given base address */
void msp430_usart_init(msp430_usart_t *usart, msp430_cpu_t *cpu,
                        uint32_t base_addr, uint32_t tx_offset);

/* Set the callback for TX byte output */
void msp430_usart_set_callback(msp430_usart_t *usart,
                                usart_tx_callback cb, void *user_data);

/* Connect USART to IFG register for TX ready flag management.
 * After calling this, UTXIFG is set (buffer ready). The USART will
 * re-set UTXIFG after each TX completes. */
void msp430_usart_set_ifg(msp430_usart_t *usart, uint8_t *ifg_ptr, uint8_t tx_mask);

/* Register SPI exchange callback and set RX register offset */
void msp430_usart_set_spi_exchange(msp430_usart_t *usart,
                                    usart_spi_exchange_fn cb, void *user_data,
                                    uint32_t rx_offset);

#endif /* MSP430_USART_H */
