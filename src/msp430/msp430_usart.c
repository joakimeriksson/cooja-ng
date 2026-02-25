/*
 * Minimal USART stub for capturing TX output from firmware tests
 */
#include "msp430_usart.h"
#include <string.h>

/*
 * Classic USART register offsets:
 *   +0: UCTL  (control)
 *   +1: UTCTL (transmit control) — TXEPT is bit 0
 *   +2: URCTL (receive control)
 *   +3: UMCTL (modulation control)
 *   +4: UBR0  (baud rate low)
 *   +5: UBR1  (baud rate high)
 *   +6: URXBUF (receive buffer)
 *   +7: UTXBUF (transmit buffer)
 */
#define USART_UTCTL_OFFSET  1
#define USART_TXEPT         0x01  /* Transmitter empty (shift register empty) */

/* IO callback for USART reads */
static int usart_read(void *user_data, uint32_t addr, bool word, int64_t cycles) {
    msp430_usart_t *usart = (msp430_usart_t *)user_data;
    (void)word; (void)cycles;

    uint32_t offset = addr - usart->base_addr;

    /* UTCTL: return TXEPT=1 (transmitter always ready, we process instantly) */
    if (offset == USART_UTCTL_OFFSET) {
        return USART_TXEPT;
    }

    /* Return SPI RX buffer when RX register is read */
    if (usart->rx_offset != 0 && offset == usart->rx_offset) {
        return usart->rx_buf;
    }
    return 0;
}

/* IO callback for USART writes */
static void usart_write(void *user_data, uint32_t addr, int value, bool word, int64_t cycles) {
    msp430_usart_t *usart = (msp430_usart_t *)user_data;
    (void)word; (void)cycles;

    /* Check if this is the TX register */
    if (addr == usart->base_addr + usart->tx_offset) {
        if (usart->spi_exchange) {
            /* SPI mode: exchange byte and store response */
            usart->rx_buf = usart->spi_exchange(usart->spi_exchange_data,
                                                 (uint8_t)(value & 0xff));
            /* Set URXIFG — RX data available */
            if (usart->ifg_ptr) {
                *usart->ifg_ptr |= usart->ifg_rx_mask;
            }
        }
        if (usart->tx_callback) {
            usart->tx_callback(usart->tx_user_data, (uint8_t)(value & 0xff));
        }
        /* Set UTXIFG — TX buffer is ready for next byte */
        if (usart->ifg_ptr) {
            *usart->ifg_ptr |= usart->ifg_tx_mask;
        }
    }
}

void msp430_usart_init(msp430_usart_t *usart, msp430_cpu_t *cpu,
                        uint32_t base_addr, uint32_t tx_offset) {
    memset(usart, 0, sizeof(*usart));
    usart->cpu = cpu;
    usart->base_addr = base_addr;
    usart->tx_offset = tx_offset;

    /* Register IO range — 8 bytes for classic USART, 16 for USCI */
    uint32_t size = 16;
    msp430_register_io(cpu, base_addr, size, usart_read, usart_write, usart);
}

void msp430_usart_set_callback(msp430_usart_t *usart,
                                usart_tx_callback cb, void *user_data) {
    usart->tx_callback = cb;
    usart->tx_user_data = user_data;
}

void msp430_usart_set_ifg(msp430_usart_t *usart, uint8_t *ifg_ptr, uint8_t tx_mask) {
    usart->ifg_ptr = ifg_ptr;
    usart->ifg_tx_mask = tx_mask;
    /* TX buffer is ready after init */
    *ifg_ptr |= tx_mask;
}

void msp430_usart_set_spi_exchange(msp430_usart_t *usart,
                                    usart_spi_exchange_fn cb, void *user_data,
                                    uint32_t rx_offset) {
    usart->spi_exchange = cb;
    usart->spi_exchange_data = user_data;
    usart->rx_offset = rx_offset;
    /* URXIFG: for classic USART0, IFG1 bit 6; set via ifg_rx_mask */
    usart->ifg_rx_mask = usart->ifg_tx_mask >> 1;  /* URXIFG is one bit below UTXIFG */
}
