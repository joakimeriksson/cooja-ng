/*
 * CC2538 System Control
 */
#include "cc2538_sys_ctrl.h"
#include <string.h>
#include <stdio.h>

#define SYS_CTRL_REG_SIZE  0x1000

static int sys_ctrl_read(void *user_data, uint32_t addr) {
    cc2538_sys_ctrl_t *sc = (cc2538_sys_ctrl_t *)user_data;
    uint32_t offset = addr - SYS_CTRL_BASE;

    switch (offset) {
        case SYS_CTRL_CLOCK_CTRL: return (int)sc->clock_ctrl;
        case SYS_CTRL_CLOCK_STA: {
            uint32_t sta = sc->clock_sta;
            /* OSC32K (32kHz osc, CLOCK_STA bit 26) self-stabilizes on read: a
             * CLOCK_CTRL write (e.g. PM1/2 entry via select_16_mhz_rcosc) clears
             * it; a couple of subsequent reads bring it back stable. Without this,
             * lpm_exit()'s "wait for bit26 to clear, then set" poll on wake from
             * PM1/2 spins forever (inside the rtimer ISR), wedging the node. The
             * read returns the pre-update value so the firmware first observes it
             * clear (its clear-wait passes) then set (its set-wait terminates). */
            if (!(sc->clock_sta & (1u << 26))) {
                if (++sc->osc32k_pending >= 2) {
                    sc->clock_sta |= (1u << 26);
                    sc->osc32k_pending = 0;
                }
            }
            return (int)sta;
        }
        case SYS_CTRL_RCGCGPT:   return (int)sc->rcgcgpt;
        case SYS_CTRL_RCGCUART:  return (int)sc->rcgcuart;
        case SYS_CTRL_RCGCSSI:   return (int)sc->rcgcssi;
        case SYS_CTRL_RCGCI2C:   return (int)sc->rcgci2c;
        case SYS_CTRL_RCGCSEC:   return (int)sc->rcgcsec;
        case SYS_CTRL_RCGCRFC:   return (int)sc->rcgcrfc;
        case SYS_CTRL_SCGCRFC:   return (int)sc->scgcrfc;
        case SYS_CTRL_DCGCRFC:   return (int)sc->dcgcrfc;
        case SYS_CTRL_PMCTL:     return (int)sc->pmctl;
        case SYS_CTRL_I_MAP:     return (int)sc->i_map;
        case SYS_CTRL_EMUOVR:    return (int)sc->emuovr;
        case SYS_CTRL_PWRDBG:    return 0xB4; /* Power debug: all powered */
        default: {
            int idx = offset / 4;
            if (idx < 64) return (int)sc->regs[idx];
            return 0;
        }
    }
}

static void sys_ctrl_write(void *user_data, uint32_t addr, uint32_t value) {
    cc2538_sys_ctrl_t *sc = (cc2538_sys_ctrl_t *)user_data;
    uint32_t offset = addr - SYS_CTRL_BASE;

    switch (offset) {
        case SYS_CTRL_CLOCK_CTRL:
            sc->clock_ctrl = value;
            /* Update clock status to reflect new config.
             * CC2538 CLOCK_STA register layout:
             *   [2:0]  SYS_DIV, [10:8] IO_DIV — mirror CLOCK_CTRL
             *   [16]   SOURCE_CHANGE = 0 (switch complete)
             *   [19]   XOSC_STB   = 1 when XOSC selected (OSC bit=0)
             *   [18]   HSOSC_STB  = 1 when RCOSC selected (OSC bit=1)
             *   [26]   OSC32K     = set after transition delay */
            {
                /* CLOCK_CTRL.OSC = bit 16 (SYS_CTRL_CLOCK_CTRL_OSC=0x10000):
                 * 0=32MHz XOSC, 1=16MHz RCOSC. CLOCK_STA.OSC (also bit 16) must
                 * mirror it so the firmware's select_16_mhz_rcosc()/
                 * select_32_mhz_xosc() "wait for the switch" poll
                 * (while(CLOCK_STA.OSC != selected)) terminates — otherwise a
                 * cc2538 dropping to PM1/2 spins forever. The OSC switch is
                 * modelled as instantaneous, so SOURCE_CHANGE (bit 20) stays 0. */
                int osc = (value >> 16) & 1;
                sc->clock_sta = (value & 0x707) |           /* SYS_DIV, IO_DIV */
                                (osc ? ((1u << 18) | (1u << 16)) /* HSOSC_STB + OSC */
                                     : (1u << 19));              /* XOSC_STB, OSC=0 */
                /* clock_sta recompute above cleared OSC32K (bit 26); restart the
                 * read-stabilization counter so it re-settles (see CLOCK_STA read). */
                sc->osc32k_pending = 0;
            }
            /* Update CPU frequency */
            arm_cpu_set_frequency(sc->cpu, cc2538_sys_ctrl_get_sys_clock(sc));
            break;
        case SYS_CTRL_RCGCGPT:   sc->rcgcgpt = value; break;
        case SYS_CTRL_RCGCUART:  sc->rcgcuart = value; break;
        case SYS_CTRL_RCGCSSI:   sc->rcgcssi = value; break;
        case SYS_CTRL_RCGCI2C:   sc->rcgci2c = value; break;
        case SYS_CTRL_RCGCSEC:   sc->rcgcsec = value; break;
        case SYS_CTRL_RCGCRFC:   sc->rcgcrfc = value; break;
        case SYS_CTRL_SCGCRFC:   sc->scgcrfc = value; break;
        case SYS_CTRL_DCGCRFC:   sc->dcgcrfc = value; break;
        case SYS_CTRL_PMCTL:     sc->pmctl = value; break;
        case SYS_CTRL_I_MAP:     sc->i_map = value; break;
        case SYS_CTRL_EMUOVR:    sc->emuovr = value; break;
        default: {
            int idx = offset / 4;
            if (idx < 64) sc->regs[idx] = value;
            break;
        }
    }
}

void cc2538_sys_ctrl_init(cc2538_sys_ctrl_t *sc, arm_cpu_t *cpu) {
    memset(sc, 0, sizeof(*sc));
    sc->cpu = cpu;

    /* Default clock: 32 MHz from 32 MHz XOSC */
    sc->clock_ctrl = 0;
    /* CLOCK_STA: XOSC_STB=bit19, HSOSC_STB=bit18, SYNC_32K=bit26 */
    sc->clock_sta = (1u << 19) | (1u << 18) | (1u << 26);

    arm_register_io(cpu, SYS_CTRL_BASE, SYS_CTRL_REG_SIZE,
                    sys_ctrl_read, sys_ctrl_write, sc);
}

uint32_t cc2538_sys_ctrl_get_sys_clock(cc2538_sys_ctrl_t *sc) {
    /* CC2538 system clock:
     * CLOCK_CTRL.SYS_DIV[2:0] = divisor (0=32MHz, 1=16MHz, 2=8MHz, etc.)
     * OSC bit: 0=32MHz XOSC, 1=16MHz RCOSC */
    int sys_div = sc->clock_ctrl & 7;
    /* CPU execution frequency is derived from the 32 MHz XOSC base, NOT the
     * CLOCK_CTRL.OSC (bit 16) source-select. On silicon OSC=1 selects the 16 MHz
     * HF-RCOSC, but firmware only does so transiently around PM1/2 entry, where
     * the core is HALTED in deep sleep — it never executes a real workload at the
     * RCOSC rate. This emulator does not halt the core in PM1/2 (it keeps stepping
     * instructions), so honoring the RCOSC downshift made a node that dipped into
     * lpm keep running its TSCH slot code at 8 MHz (16 MHz RCOSC >> SYS_DIV(1)),
     * consuming 2x the sim-time and overshooting the (frequency-independent) rtimer
     * slot deadlines every slot — the cc2538 TSCH "!dl-miss" storm that desynced the
     * leaf. rtimer/SysTick time bases are frequency-independent, so pinning the CPU
     * to the XOSC base is safe. The OSC bit is still mirrored into CLOCK_STA on a
     * CLOCK_CTRL write so the firmware's switch-poll terminates (see sys_ctrl_write). */
    return 32000000u >> sys_div;
}
