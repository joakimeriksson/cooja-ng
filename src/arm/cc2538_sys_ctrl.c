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
            /* SYNC_32K (CLOCK_STA bit 26, 0x04000000 — NOT OSC32K, which is bit
             * 24) self-stabilizes on read. Contiki's cc2538 clock setup polls it
             * with a clear-then-set sequence (sys-ctrl.c:
             *   while(REG(CLOCK_STA) & SYNC_32K);
             *   while(!(REG(CLOCK_STA) & SYNC_32K)); )
             * waiting for the 32-kHz domain to re-sync after a CLOCK_CTRL write
             * (e.g. PM1/2 entry). A CLOCK_CTRL write clears the bit; a couple of
             * subsequent reads bring it back set. Without this the poll spins
             * forever (inside the rtimer ISR on PM1/2 wake), wedging the node.
             * The read returns the pre-update value so the firmware first
             * observes it clear (clear-wait passes) then set (set-wait ends). */
            if (!(sc->clock_sta & (1u << 26))) {
                if (++sc->sync32k_pending >= 2) {
                    sc->clock_sta |= (1u << 26);
                    sc->sync32k_pending = 0;
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
             * CC2538 CLOCK_STA register layout (see Contiki sys-ctrl.h):
             *   [2:0]  SYS_DIV, [10:8] IO_DIV — mirror CLOCK_CTRL
             *   [16]   OSC          = selected source (0=XOSC, 1=RCOSC)
             *   [18]   HSOSC_STB     = 1 when RCOSC selected (OSC bit=1)
             *   [19]   XOSC_STB      = 1 when XOSC selected (OSC bit=0)
             *   [20]   SOURCE_CHANGE = 0 (switch complete)
             *   [26]   SYNC_32K      = set after 32-kHz sync (0x04000000) */
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
                /* clock_sta recompute above cleared SYNC_32K (bit 26); restart the
                 * read-stabilization counter so it re-settles (see CLOCK_STA read). */
                sc->sync32k_pending = 0;
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
    /* Derive the CPU execution frequency the way the firmware itself does.
     * Contiki's sys_ctrl_get_sys_clock() is
     *     return SYS_CTRL_32MHZ >> (CLOCK_STA & SYS_DIV);
     * (arch/cpu/cc2538/dev/sys-ctrl.c): a fixed 32 MHz XOSC base shifted by
     * SYS_DIV, ignoring the CLOCK_CTRL.OSC (bit 16) source-select entirely.
     * Pinning the emulated core to that same 32 MHz base keeps the emulator's
     * clock model in agreement with the firmware's, so the rtimer/SysTick math
     * lines up. CLOCK_STA mirrors CLOCK_CTRL's SYS_DIV, so reading either gives
     * the same divisor.
     *
     * (Honoring OSC=1 here was actively wrong. On silicon OSC=1 selects the
     * 16 MHz HF-RCOSC only transiently around PM1/2 entry, where the core is
     * HALTED — it never runs a real workload at the RCOSC rate. This emulator
     * does not halt the core in PM1/2 (it keeps stepping instructions), so the
     * RCOSC downshift made a node that dipped into lpm keep running its TSCH
     * slot code at 8 MHz (16 MHz >> SYS_DIV(1)) instead of 16 MHz — burning 2x
     * the sim-time and overshooting the frequency-independent rtimer slot
     * deadline every slot: the cc2538 TSCH "!dl-miss" storm that desynced the
     * leaf.) The OSC bit is still mirrored into CLOCK_STA on a CLOCK_CTRL write
     * so the firmware's switch-poll terminates (see sys_ctrl_write). */
    int sys_div = sc->clock_ctrl & 7;
    return 32000000u >> sys_div;
}
