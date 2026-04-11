/*
 * Minimal USART stub for capturing TX output from firmware tests
 */
#include "msp430_usart.h"
#include <string.h>
#include <stdio.h>

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

    /* UTCTL: return TXEPT=1 (transmitter always ready, we process instantly).
     * Only for classic USART — on USCI, offset 1 is UCxCTL1 (not UTCTL),
     * and TX readiness is signaled via IFG, not a status register bit. */
    if (offset == USART_UTCTL_OFFSET && !usart->is_usci) {
        return USART_TXEPT;
    }

    /* Return SPI RX buffer when RX register is read.
     * On USCI, reading RXBUF (offset 6) clears RXIFG.
     * On classic USART, rx_offset is set via set_spi(). */
    uint32_t rx_off = usart->rx_offset ? usart->rx_offset : 6;
    if (offset == rx_off) {
        if (usart->ifg_ptr)
            *usart->ifg_ptr &= ~usart->ifg_rx_mask;
        msp430_usart_update_interrupts(usart);
        return usart->rx_buf;
    }

    return 0;
}

/* IO callback for USART writes */
static void usart_write(void *user_data, uint32_t addr, int value, bool word, int64_t cycles) {
    msp430_usart_t *usart = (msp430_usart_t *)user_data;
    (void)word; (void)cycles;

    /* USCI: track UCSWRST and baud rate.  TX is disabled while
     * UCSWRST=1 or baud rate is 0 (not configured). */
    if (usart->is_usci) {
        uint32_t off = addr - usart->base_addr;
        if (off == 1) {
            bool was_reset = usart->ucswrst;
            usart->ucswrst = (value & 0x01) != 0;
            /* MSPSim: clearing UCSWRST resets USCI — sets TXIFG, clears RXIFG.
             * This is needed for SPI: after spi_init() clears IFG and then
             * clears UCSWRST, TXIFG must be re-asserted so SPI_WAITFORTxREADY
             * doesn't hang. */
            if (was_reset && !usart->ucswrst && usart->ifg_ptr) {
                *usart->ifg_ptr |= usart->ifg_tx_mask;
                *usart->ifg_ptr &= ~usart->ifg_rx_mask;
                msp430_usart_update_interrupts(usart);
            }
        }
        if (off == 2) usart->baud_lo = (uint8_t)value;  /* BR0 */
        if (off == 3) usart->baud_hi = (uint8_t)value;  /* BR1 */
    }

    /* Check if this is the TX register */
    if (addr == usart->base_addr + usart->tx_offset) {
        /* Don't fire console TX callback while USCI is in reset or
         * baud rate not set.  SPI exchange always proceeds (it uses
         * the SPI clock, not the baud rate generator). */
        bool tx_blocked = usart->is_usci && !usart->spi_exchange &&
                          (usart->ucswrst || (usart->baud_lo == 0 && usart->baud_hi == 0));
        bool spi_responded = false;
        if (usart->spi_exchange) {
            int resp = usart->spi_exchange(usart->spi_exchange_data,
                                            (uint8_t)(value & 0xff));
            if (resp >= 0) {
                usart->rx_buf = (uint8_t)resp;
                spi_responded = true;
            }
            /* resp == -1: no device responded (e.g., CC2420 VREG off).
             * Don't set rx_buf or RXIFG — firmware SPI poll will hang
             * until the device is ready, matching MSPSim behavior. */
        }
        if (usart->tx_callback && !tx_blocked) {
            usart->tx_callback(usart->tx_user_data, (uint8_t)(value & 0xff));
        }
        /* Set IFG flags: TXIFG always (TX buffer ready for next byte).
         * RXIFG only if a device responded (SPI completed). */
        if (usart->ifg_ptr) {
            *usart->ifg_ptr |= usart->ifg_tx_mask;
            if (spi_responded || !usart->spi_exchange)
                *usart->ifg_ptr |= usart->ifg_rx_mask;
        }
        /* Check if TX/RX interrupts should fire (IFG & IE both set) */
        msp430_usart_update_interrupts(usart);
    }
}

void msp430_usart_init(msp430_usart_t *usart, msp430_cpu_t *cpu,
                        uint32_t base_addr, uint32_t tx_offset) {
    memset(usart, 0, sizeof(*usart));
    usart->cpu = cpu;
    usart->base_addr = base_addr;
    usart->tx_offset = tx_offset;
    /* USCI defaults to UCSWRST=1 after POR — TX blocked until firmware
     * clears it.  This prevents garbage output during USCI configuration.
     * is_usci is set later by platform_init; ucswrst is checked in
     * usart_write only when is_usci is true. */
    usart->ucswrst = true;

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
    usart->ifg_rx_mask = tx_mask >> 1;  /* RXIFG is one bit below TXIFG */
    /* TX+RX buffer ready after init */
    *ifg_ptr |= tx_mask | (tx_mask >> 1);
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

void msp430_usart_set_ie(msp430_usart_t *usart, uint8_t *ie_ptr,
                          uint8_t tx_mask, int tx_vector, int rx_vector) {
    usart->ie_ptr = ie_ptr;
    usart->ie_tx_mask = tx_mask;
    usart->ie_rx_mask = tx_mask >> 1;
    usart->tx_vector = tx_vector;
    usart->rx_vector = rx_vector;
}

void msp430_usart_receive_byte(msp430_usart_t *usart, uint8_t byte) {
    usart->rx_buf = byte;
    if (usart->ifg_ptr)
        *usart->ifg_ptr |= usart->ifg_rx_mask;

    /* If DMA is configured for this USART's RX trigger, fire it.
     * DMA transfer happens before the CPU interrupt — the DMA controller
     * has higher priority and will copy the byte from RXBUF to RAM.
     * After DMA fires, RXIFG is cleared automatically (DMA acknowledges it). */
    if (usart->dma_trigger && usart->dma_data) {
        usart->dma_trigger(usart->dma_data, usart->dma_trigger_source);
        /* DMA acknowledges the trigger by clearing RXIFG */
        if (usart->ifg_ptr)
            *usart->ifg_ptr &= ~usart->ifg_rx_mask;
    }

    msp430_usart_update_interrupts(usart);
}

void msp430_usart_update_interrupts(msp430_usart_t *usart) {
    if (!usart->ie_ptr || !usart->ifg_ptr || !usart->cpu) return;

    uint8_t ie = *usart->ie_ptr;
    uint8_t ifg = *usart->ifg_ptr;

    /* TX interrupt: fire if both TXIFG and TXIE are set.
     * Only FLAG (set=true), never explicitly clear shared vectors.
     * Multiple USARTs may share the same vector (e.g., USCI_A0/B0 both
     * use vector 23). Clearing here would cancel another USART's pending
     * interrupt. The interrupt source is cleared by service_interrupt
     * when the CPU services it. */
    if ((ifg & usart->ifg_tx_mask) && (ie & usart->ie_tx_mask)) {
        msp430_flag_interrupt(usart->cpu, usart->tx_vector, usart, NULL, true);
    }

    /* RX interrupt: same approach */
    if ((ifg & usart->ifg_rx_mask) && (ie & usart->ie_rx_mask)) {
        msp430_flag_interrupt(usart->cpu, usart->rx_vector, usart, NULL, true);
    }
}
