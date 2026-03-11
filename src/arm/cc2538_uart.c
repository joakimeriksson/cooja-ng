/*
 * CC2538 UART peripheral
 */
#include "cc2538_uart.h"
#include "arm_nvic.h"
#include <string.h>
#include <stdio.h>

#define UART_REG_SIZE  0x1000

static int uart_read(void *user_data, uint32_t addr) {
    cc2538_uart_t *uart = (cc2538_uart_t *)user_data;
    uint32_t offset = addr - uart->base_addr;

    switch (offset) {
        case UART_DR:
            if (uart->rx_fifo_count > 0) {
                uint8_t byte = uart->rx_fifo[uart->rx_fifo_head];
                uart->rx_fifo_head = (uart->rx_fifo_head + 1) % 16;
                uart->rx_fifo_count--;
                if (uart->rx_fifo_count == 0)
                    uart->fr |= UART_FR_RXFE;   /* RX FIFO empty */
                uart->fr &= ~UART_FR_RXFF;      /* RX FIFO not full */
                return (int)byte;
            }
            return (int)uart->dr;
        case UART_RSR:  return (int)uart->rsr;
        case UART_FR:   return (int)uart->fr;
        case UART_IBRD: return (int)uart->ibrd;
        case UART_FBRD: return (int)uart->fbrd;
        case UART_LCRH: return (int)uart->lcrh;
        case UART_CTL:  return (int)uart->ctl;
        case UART_IFLS: return (int)uart->ifls;
        case UART_IM:   return (int)uart->im;
        case UART_RIS:  return (int)uart->ris;
        case UART_MIS:  return (int)(uart->ris & uart->im);
        case UART_ICR:  return 0;
        default:        return 0;
    }
}

static void uart_write(void *user_data, uint32_t addr, uint32_t value) {
    cc2538_uart_t *uart = (cc2538_uart_t *)user_data;
    uint32_t offset = addr - uart->base_addr;

    switch (offset) {
        case UART_DR:
            uart->dr = value & 0xFF;
            if ((uart->ctl & UART_CTL_UARTEN) && (uart->ctl & UART_CTL_TXE)) {
                if (uart->tx_callback)
                    uart->tx_callback(uart->tx_user_data, (uint8_t)(value & 0xFF));
            }
            break;
        case UART_RSR:
            uart->rsr = 0; /* Write clears errors */
            break;
        case UART_IBRD:
            uart->ibrd = value & 0xFFFF;
            break;
        case UART_FBRD:
            uart->fbrd = value & 0x3F;
            break;
        case UART_LCRH:
            uart->lcrh = value;
            break;
        case UART_CTL:
            uart->ctl = value;
            break;
        case UART_IFLS:
            uart->ifls = value;
            break;
        case UART_IM:
            uart->im = value;
            break;
        case UART_ICR:
            uart->ris &= ~value;
            break;
    }
}

void cc2538_uart_init(cc2538_uart_t *uart, arm_cpu_t *cpu,
                      uint32_t base_addr, int irq_num) {
    memset(uart, 0, sizeof(*uart));
    uart->cpu = cpu;
    uart->base_addr = base_addr;
    uart->irq_num = irq_num;

    /* TX FIFO empty, RX FIFO empty */
    uart->fr = UART_FR_TXFE | UART_FR_RXFE;
    uart->ctl = UART_CTL_TXE | UART_CTL_RXE; /* TX and RX enabled by default */

    arm_register_io(cpu, base_addr, UART_REG_SIZE, uart_read, uart_write, uart);
}

void cc2538_uart_set_callback(cc2538_uart_t *uart,
                              cc2538_uart_tx_fn cb, void *user_data) {
    uart->tx_callback = cb;
    uart->tx_user_data = user_data;
}

void cc2538_uart_set_nvic(cc2538_uart_t *uart, struct arm_nvic *nvic) {
    uart->nvic = nvic;
}

void cc2538_uart_receive_byte(cc2538_uart_t *uart, uint8_t byte) {
    if (uart->rx_fifo_count >= 16) {
        /* FIFO full, set overrun */
        uart->fr |= UART_FR_RXFF;
        return;
    }
    int tail = (uart->rx_fifo_head + uart->rx_fifo_count) % 16;
    uart->rx_fifo[tail] = byte;
    uart->rx_fifo_count++;

    /* Clear RX FIFO empty flag */
    uart->fr &= ~UART_FR_RXFE;
    if (uart->rx_fifo_count >= 16)
        uart->fr |= UART_FR_RXFF;

    /* Set RX interrupt (RXRIS = bit 4) */
    uart->ris |= (1 << 4);

    /* Fire NVIC interrupt if RX interrupt is masked in */
    if (uart->nvic && (uart->im & (1 << 4))) {
        arm_nvic_set_pending(uart->nvic, uart->irq_num);
    }
}
