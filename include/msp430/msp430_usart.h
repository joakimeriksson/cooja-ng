/*
 * Minimal USART stub for capturing TX output from firmware tests
 */
#ifndef MSP430_USART_H
#define MSP430_USART_H

#include "msp430_cpu.h"

typedef void (*usart_tx_callback)(void *user_data, uint8_t byte);

/* SPI exchange: firmware sends tx_byte, device returns response byte.
 * Return -1 if no device responds (don't set RXIFG). */
typedef int (*usart_spi_exchange_fn)(void *user_data, uint8_t tx_byte);

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
    uint8_t           *ie_ptr;        /* Pointer to IE register variable */
    uint8_t            ie_tx_mask;    /* Bit mask for UTXIE in IE register */
    uint8_t            ie_rx_mask;    /* Bit mask for URXIE in IE register */
    int                tx_vector;     /* Interrupt vector for TX (USCI: shared TX vector) */
    int                rx_vector;     /* Interrupt vector for RX */
    bool               is_usci;       /* true for USCI (MSP430X), false for classic USART */
    bool               ucswrst;       /* USCI software reset (TX disabled while set) */
    uint8_t            baud_lo;       /* USCI BR0 (baud rate low byte) */
    uint8_t            baud_hi;       /* USCI BR1 (baud rate high byte) */
    /* DMA trigger callback — called when RXIFG is set (for DMA-driven RX) */
    void             (*dma_trigger)(void *dma_data, int trigger_source);
    void              *dma_data;
    int                dma_trigger_source;  /* e.g., 9 for USART1 RX */
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

/* Connect USART to IE register for interrupt generation.
 * When IFG & IE bits are both set, the USART fires the interrupt.
 * tx_vector/rx_vector are the interrupt vector numbers. */
void msp430_usart_set_ie(msp430_usart_t *usart, uint8_t *ie_ptr,
                          uint8_t tx_mask, int tx_vector, int rx_vector);

/* Inject a received byte into the USART RX buffer.
 * Sets RXIFG and fires RX interrupt if enabled. */
void msp430_usart_receive_byte(msp430_usart_t *usart, uint8_t byte);

/* Re-check IFG & IE bits and fire/clear interrupts as needed.
 * Called after IE or IFG register writes. */
void msp430_usart_update_interrupts(msp430_usart_t *usart);

#endif /* MSP430_USART_H */
