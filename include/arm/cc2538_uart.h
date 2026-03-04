/*
 * CC2538 UART peripheral (UART0 and UART1)
 */
#ifndef CC2538_UART_H
#define CC2538_UART_H

#include "arm_cpu.h"

/* UART register offsets */
#define UART_DR     0x000  /* Data Register */
#define UART_RSR    0x004  /* Receive Status Register */
#define UART_FR     0x018  /* Flag Register */
#define UART_ILPR   0x020  /* IrDA Low-Power Register */
#define UART_IBRD   0x024  /* Integer Baud-Rate Divisor */
#define UART_FBRD   0x028  /* Fractional Baud-Rate Divisor */
#define UART_LCRH   0x02C  /* Line Control Register */
#define UART_CTL    0x030  /* Control Register */
#define UART_IFLS   0x034  /* Interrupt FIFO Level Select */
#define UART_IM     0x038  /* Interrupt Mask */
#define UART_RIS    0x03C  /* Raw Interrupt Status */
#define UART_MIS    0x040  /* Masked Interrupt Status */
#define UART_ICR    0x044  /* Interrupt Clear */
#define UART_DMACTL 0x048  /* DMA Control */

/* FR bits */
#define UART_FR_TXFE  (1 << 7)  /* TX FIFO empty */
#define UART_FR_RXFF  (1 << 6)  /* RX FIFO full */
#define UART_FR_TXFF  (1 << 5)  /* TX FIFO full */
#define UART_FR_RXFE  (1 << 4)  /* RX FIFO empty */
#define UART_FR_BUSY  (1 << 3)  /* UART busy */

/* CTL bits */
#define UART_CTL_UARTEN  (1 << 0)
#define UART_CTL_TXE     (1 << 8)
#define UART_CTL_RXE     (1 << 9)

typedef void (*cc2538_uart_tx_fn)(void *user_data, uint8_t byte);

struct arm_nvic;  /* forward declaration */

typedef struct cc2538_uart {
    arm_cpu_t          *cpu;
    uint32_t            base_addr;
    int                 irq_num;       /* IRQ number (5 for UART0, 6 for UART1) */

    /* Registers */
    uint32_t            dr;
    uint32_t            rsr;
    uint32_t            fr;
    uint32_t            ibrd;
    uint32_t            fbrd;
    uint32_t            lcrh;
    uint32_t            ctl;
    uint32_t            ifls;
    uint32_t            im;
    uint32_t            ris;
    uint32_t            icr;

    /* TX callback */
    cc2538_uart_tx_fn   tx_callback;
    void               *tx_user_data;

    /* RX FIFO for serial input */
    uint8_t             rx_fifo[16];
    int                 rx_fifo_head;
    int                 rx_fifo_count;
    struct arm_nvic    *nvic;
} cc2538_uart_t;

/* Initialize UART and register IO region */
void cc2538_uart_init(cc2538_uart_t *uart, arm_cpu_t *cpu,
                      uint32_t base_addr, int irq_num);

/* Set TX callback */
void cc2538_uart_set_callback(cc2538_uart_t *uart,
                              cc2538_uart_tx_fn cb, void *user_data);

/* Receive a byte into the RX FIFO (serial input to mote) */
void cc2538_uart_receive_byte(cc2538_uart_t *uart, uint8_t byte);

/* Set NVIC pointer for interrupt generation on RX */
void cc2538_uart_set_nvic(cc2538_uart_t *uart, struct arm_nvic *nvic);

#endif /* CC2538_UART_H */
