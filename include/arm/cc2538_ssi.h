/*
 * CC2538 SSI (Synchronous Serial Interface / SPI) peripheral
 *
 * Two memory-mapped instances:
 *   SSI0 @ 0x40008000  (Firefly: routes to off-SoC CC1200)
 *   SSI1 @ 0x40009000
 *
 * The peripheral models the registers the Contiki-NG SSI driver actually
 * touches at boot/runtime: CR0, CR1, SR, DR, CPSR, IM/RIS/MIS/ICR, CC.
 * Bytes written to SSI_DR are immediately exchanged with whatever chip
 * the platform has wired in via cc2538_ssi_set_exchange_callback().
 * The byte returned from the callback is pushed onto the (single-entry)
 * RX FIFO and read back via SSI_DR. Status bits (TFE/TNF/RNE/RFF/BSY)
 * are kept consistent with that FIFO state.
 *
 * The callback design mirrors msp430_usart_set_spi_exchange() in the
 * MSP430 USART driver — keeping the on-SoC SPI master and the off-SoC
 * chip driver loosely coupled.
 */
#ifndef CC2538_SSI_H
#define CC2538_SSI_H

#include "arm_cpu.h"

/* SSI register offsets */
#define SSI_CR0     0x000   /* Control 0 */
#define SSI_CR1     0x004   /* Control 1 */
#define SSI_DR      0x008   /* Data */
#define SSI_SR      0x00C   /* Status */
#define SSI_CPSR    0x010   /* Clock Prescaler */
#define SSI_IM      0x014   /* Interrupt Mask */
#define SSI_RIS     0x018   /* Raw Interrupt Status */
#define SSI_MIS     0x01C   /* Masked Interrupt Status */
#define SSI_ICR     0x020   /* Interrupt Clear */
#define SSI_DMACTL  0x024   /* DMA Control */
#define SSI_CC      0xFC8   /* Clock Configuration */

/* SR bits */
#define SSI_SR_TFE  (1 << 0)   /* TX FIFO empty */
#define SSI_SR_TNF  (1 << 1)   /* TX FIFO not full */
#define SSI_SR_RNE  (1 << 2)   /* RX FIFO not empty */
#define SSI_SR_RFF  (1 << 3)   /* RX FIFO full */
#define SSI_SR_BSY  (1 << 4)   /* Busy */

/* Per-instance bus identity (passed to the exchange callback so a single
 * bus router can multiplex multiple chips on the same SSI). */
typedef enum {
    CC2538_SSI_BUS_0 = 0,
    CC2538_SSI_BUS_1 = 1,
} cc2538_ssi_bus_t;

/* SPI exchange callback: invoked when the firmware writes a byte to
 * SSI_DR. Returns the byte the slave device clocks back out (which
 * becomes the next SSI_DR read value). */
typedef uint8_t (*cc2538_ssi_exchange_fn)(void *user_data, uint8_t tx_byte);

typedef struct cc2538_ssi {
    arm_cpu_t   *cpu;
    uint32_t     base_addr;
    cc2538_ssi_bus_t bus;

    /* Registers */
    uint32_t     cr0;
    uint32_t     cr1;
    uint32_t     cpsr;
    uint32_t     im;
    uint32_t     ris;
    uint32_t     icr;
    uint32_t     dmactl;
    uint32_t     cc;

    /* Single-entry RX FIFO. The CC2538 SSI has 8-deep FIFOs in real
     * hardware, but Contiki's CC1200 driver always reads back each
     * exchanged byte before issuing the next, so depth-1 is enough for
     * functional correctness. Depth >1 would just need a small ring. */
    uint16_t     rx_byte;
    bool         rx_has_data;

    /* Exchange callback installed by the platform glue. NULL if no chip
     * is wired in — writes still complete, reads return 0. */
    cc2538_ssi_exchange_fn exchange_cb;
    void                  *exchange_user_data;
} cc2538_ssi_t;

/* Initialize an SSI instance and register its memory-mapped IO region.
 * `base_addr` selects SSI0 (0x40008000) or SSI1 (0x40009000). */
void cc2538_ssi_init(cc2538_ssi_t *ssi, arm_cpu_t *cpu,
                     uint32_t base_addr, cc2538_ssi_bus_t bus);

/* Install an exchange callback. The callback receives the byte the CPU
 * wrote into SSI_DR and returns the byte the slave clocks back. */
void cc2538_ssi_set_exchange_callback(cc2538_ssi_t *ssi,
                                       cc2538_ssi_exchange_fn cb,
                                       void *user_data);

#endif /* CC2538_SSI_H */
