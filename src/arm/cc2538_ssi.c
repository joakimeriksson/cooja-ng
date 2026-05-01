/*
 * CC2538 SSI peripheral implementation. See include/arm/cc2538_ssi.h.
 */
#include "cc2538_ssi.h"
#include <string.h>

#define SSI_SIZE  0x1000

static uint32_t status_register(const cc2538_ssi_t *ssi) {
    /* TX FIFO is treated as always-empty (writes complete synchronously
     * via the exchange callback). RX FIFO depth = 1 byte. */
    uint32_t sr = SSI_SR_TFE | SSI_SR_TNF;
    if (ssi->rx_has_data) sr |= SSI_SR_RNE;
    /* RFF only matters once depth > 1 — leave clear. */
    /* BSY: Contiki polls BSY between byte writes for some chips. We
     * complete the exchange synchronously so always-clear is fine. */
    return sr;
}

static int ssi_read(void *user_data, uint32_t addr) {
    cc2538_ssi_t *ssi = (cc2538_ssi_t *)user_data;
    uint32_t off = addr - ssi->base_addr;

    switch (off) {
    case SSI_CR0:    return (int)ssi->cr0;
    case SSI_CR1:    return (int)ssi->cr1;
    case SSI_CPSR:   return (int)ssi->cpsr;
    case SSI_IM:     return (int)ssi->im;
    case SSI_RIS:    return (int)ssi->ris;
    case SSI_MIS:    return (int)(ssi->ris & ssi->im);
    case SSI_DMACTL: return (int)ssi->dmactl;
    case SSI_CC:     return (int)ssi->cc;
    case SSI_SR:     return (int)status_register(ssi);
    case SSI_DR: {
        uint8_t v = 0;
        if (ssi->rx_has_data) {
            v = (uint8_t)ssi->rx_byte;
            ssi->rx_has_data = false;
        }
        return (int)v;
    }
    default:
        return 0;
    }
}

static void ssi_write(void *user_data, uint32_t addr, uint32_t value) {
    cc2538_ssi_t *ssi = (cc2538_ssi_t *)user_data;
    uint32_t off = addr - ssi->base_addr;

    switch (off) {
    case SSI_CR0:    ssi->cr0  = value; break;
    case SSI_CR1:    ssi->cr1  = value; break;
    case SSI_CPSR:   ssi->cpsr = value & 0xFF; break;
    case SSI_IM:     ssi->im   = value & 0x0F; break;
    case SSI_ICR:    ssi->ris &= ~(value & 0x0F); break;
    case SSI_DMACTL: ssi->dmactl = value & 0x03; break;
    case SSI_CC:     ssi->cc   = value & 0x0F; break;
    case SSI_DR: {
        uint8_t tx = (uint8_t)(value & 0xFF);
        uint8_t rx = 0;
        if (ssi->exchange_cb) {
            rx = ssi->exchange_cb(ssi->exchange_user_data, tx);
        }
        ssi->rx_byte = rx;
        ssi->rx_has_data = true;
        break;
    }
    default:
        break;
    }
}

void cc2538_ssi_init(cc2538_ssi_t *ssi, arm_cpu_t *cpu,
                     uint32_t base_addr, cc2538_ssi_bus_t bus) {
    memset(ssi, 0, sizeof(*ssi));
    ssi->cpu = cpu;
    ssi->base_addr = base_addr;
    ssi->bus = bus;
    arm_register_io(cpu, base_addr, SSI_SIZE, ssi_read, ssi_write, ssi);
}

void cc2538_ssi_set_exchange_callback(cc2538_ssi_t *ssi,
                                       cc2538_ssi_exchange_fn cb,
                                       void *user_data) {
    ssi->exchange_cb = cb;
    ssi->exchange_user_data = user_data;
}
