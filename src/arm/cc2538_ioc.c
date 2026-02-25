/*
 * CC2538 I/O Controller (pin muxing stub)
 */
#include "cc2538_ioc.h"
#include <string.h>

/* IOC has two register blocks:
 * 0x400D4000 - PA0_SEL through PD7_SEL (32 x 4 bytes = 0x80)
 * 0x400D4080 - PA0_OVER through PD7_OVER (32 x 4 bytes = 0x80)
 */
#define IOC_REG_SIZE  0x1000

static int ioc_read(void *user_data, uint32_t addr) {
    cc2538_ioc_t *ioc = (cc2538_ioc_t *)user_data;
    uint32_t offset = addr - IOC_BASE;

    if (offset < IOC_NUM_PINS * 4) {
        int idx = offset / 4;
        return (int)ioc->pin_sel[idx];
    }
    if (offset >= 0x80 && offset < 0x80 + IOC_NUM_PINS * 4) {
        int idx = (offset - 0x80) / 4;
        return (int)ioc->pin_over[idx];
    }
    return 0;
}

static void ioc_write(void *user_data, uint32_t addr, uint32_t value) {
    cc2538_ioc_t *ioc = (cc2538_ioc_t *)user_data;
    uint32_t offset = addr - IOC_BASE;

    if (offset < IOC_NUM_PINS * 4) {
        int idx = offset / 4;
        ioc->pin_sel[idx] = value;
        return;
    }
    if (offset >= 0x80 && offset < 0x80 + IOC_NUM_PINS * 4) {
        int idx = (offset - 0x80) / 4;
        ioc->pin_over[idx] = value;
        return;
    }
}

void cc2538_ioc_init(cc2538_ioc_t *ioc, arm_cpu_t *cpu) {
    memset(ioc, 0, sizeof(*ioc));
    ioc->cpu = cpu;

    arm_register_io(cpu, IOC_BASE, IOC_REG_SIZE, ioc_read, ioc_write, ioc);
}
