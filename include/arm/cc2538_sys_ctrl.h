/*
 * CC2538 System Control (clock config, peripheral clock gates, reset control)
 */
#ifndef CC2538_SYS_CTRL_H
#define CC2538_SYS_CTRL_H

#include "arm_cpu.h"

/* System Control base address */
#define SYS_CTRL_BASE  0x400D2000

/* Key register offsets */
#define SYS_CTRL_CLOCK_CTRL    0x000
#define SYS_CTRL_CLOCK_STA     0x004
#define SYS_CTRL_RCGCGPT      0x008
#define SYS_CTRL_SCGCGPT      0x00C
#define SYS_CTRL_DCGCGPT      0x010
#define SYS_CTRL_SRGPT        0x014
#define SYS_CTRL_RCGCSSI      0x018
#define SYS_CTRL_SCGCSSI      0x01C
#define SYS_CTRL_DCGCSSI      0x020
#define SYS_CTRL_SRSSI        0x024
#define SYS_CTRL_RCGCUART     0x028
#define SYS_CTRL_SCGCUART     0x02C
#define SYS_CTRL_DCGCUART     0x030
#define SYS_CTRL_SRUART       0x034
#define SYS_CTRL_RCGCI2C      0x038
#define SYS_CTRL_RCGCSEC      0x05C
#define SYS_CTRL_SCGCSEC      0x060
#define SYS_CTRL_DCGCSEC      0x064
#define SYS_CTRL_SRSEC        0x068
#define SYS_CTRL_PMCTL        0x070
#define SYS_CTRL_SRCRC        0x074
#define SYS_CTRL_PWRDBG       0x07C
#define SYS_CTRL_CLD          0x080
#define SYS_CTRL_IWE          0x094
#define SYS_CTRL_I_MAP        0x098
#define SYS_CTRL_RCGCRFC      0x0A8
#define SYS_CTRL_SCGCRFC      0x0AC
#define SYS_CTRL_DCGCRFC      0x0B0
#define SYS_CTRL_EMUOVR       0x0B4

typedef struct cc2538_sys_ctrl {
    arm_cpu_t  *cpu;

    uint32_t    clock_ctrl;
    uint32_t    clock_sta;
    uint32_t    rcgcgpt;
    uint32_t    rcgcuart;
    uint32_t    rcgcssi;
    uint32_t    rcgci2c;
    uint32_t    rcgcsec;
    uint32_t    rcgcrfc;
    uint32_t    scgcrfc;
    uint32_t    dcgcrfc;
    uint32_t    pmctl;
    uint32_t    i_map;
    uint32_t    emuovr;

    /* Pending OSC32K transition: reads remaining before OSC32K status bit sets */
    int         osc32k_pending;

    /* Generic storage for other registers */
    uint32_t    regs[64];
} cc2538_sys_ctrl_t;

/* Initialize and register IO region */
void cc2538_sys_ctrl_init(cc2538_sys_ctrl_t *sc, arm_cpu_t *cpu);

/* Get current system clock frequency in Hz */
uint32_t cc2538_sys_ctrl_get_sys_clock(cc2538_sys_ctrl_t *sc);

#endif /* CC2538_SYS_CTRL_H */
