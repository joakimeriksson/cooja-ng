/*
 * ARM Cortex-M3 SysTick 24-bit countdown timer
 */
#include "arm_systick.h"
#include <string.h>
#include <stdio.h>

#define SYSTICK_IO_BASE  0xE000E010
#define SYSTICK_IO_SIZE  0x10

static int systick_fire_count = 0;

static void systick_fire(void *user_data, arm_event_t *ev) {
    arm_systick_t *st = (arm_systick_t *)user_data;
    (void)ev;

    systick_fire_count++;

    /* Reload counter */
    st->cvr = st->rvr & 0x00FFFFFF;
    st->last_cycle = st->cpu->cycles;

    /* Set COUNTFLAG */
    st->csr |= SYST_CSR_COUNTFLAG;

    /* Generate interrupt if TICKINT is set */
    if (st->csr & SYST_CSR_TICKINT) {
        st->nvic->pending_exception = EXC_SYSTICK;
        st->nvic->scan_valid = false;
        st->nvic->has_pending = true;
        arm_nvic_check_pending(st->nvic);
    }

    /* Schedule next tick */
    if ((st->csr & SYST_CSR_ENABLE) && st->rvr > 0) {
        arm_schedule_event(st->cpu, &st->tick_event,
                          st->cpu->cycles + st->rvr + 1);
    }
}

static uint32_t systick_get_current(arm_systick_t *st) {
    if (!(st->csr & SYST_CSR_ENABLE) || st->rvr == 0)
        return st->cvr;

    int64_t elapsed = st->cpu->cycles - st->last_cycle;
    if (elapsed < 0) elapsed = 0;
    uint32_t reload = st->rvr & 0x00FFFFFF;
    uint32_t ticks = (uint32_t)(elapsed % (reload + 1));
    if (ticks > st->cvr)
        return reload - (ticks - st->cvr) + 1;
    return st->cvr - ticks;
}

static int systick_read(void *user_data, uint32_t addr) {
    arm_systick_t *st = (arm_systick_t *)user_data;
    uint32_t offset = addr - SYSTICK_IO_BASE;

    switch (offset) {
        case SYST_CSR: {
            uint32_t val = st->csr;
            st->csr &= ~SYST_CSR_COUNTFLAG; /* Read clears COUNTFLAG */
            return (int)val;
        }
        case SYST_RVR:
            return (int)(st->rvr & 0x00FFFFFF);
        case SYST_CVR:
            return (int)systick_get_current(st);
        case SYST_CALIB:
            return 0x00010000; /* No calibration value, NOREF=1 */
    }
    return 0;
}

static void systick_write(void *user_data, uint32_t addr, uint32_t value) {
    arm_systick_t *st = (arm_systick_t *)user_data;
    uint32_t offset = addr - SYSTICK_IO_BASE;

    switch (offset) {
        case SYST_CSR:
            st->csr = value & (SYST_CSR_ENABLE | SYST_CSR_TICKINT | SYST_CSR_CLKSOURCE);
            if ((st->csr & SYST_CSR_ENABLE) && st->rvr > 0) {
                st->last_cycle = st->cpu->cycles;
                st->cvr = st->rvr & 0x00FFFFFF;
                arm_schedule_event(st->cpu, &st->tick_event,
                                  st->cpu->cycles + st->rvr + 1);
            } else {
                arm_cancel_event(st->cpu, &st->tick_event);
            }
            break;
        case SYST_RVR:
            st->rvr = value & 0x00FFFFFF;
            break;
        case SYST_CVR:
            /* Writing any value clears CVR and COUNTFLAG */
            st->cvr = 0;
            st->csr &= ~SYST_CSR_COUNTFLAG;
            st->last_cycle = st->cpu->cycles;
            break;
    }
}

void arm_systick_init(arm_systick_t *st, arm_cpu_t *cpu, arm_nvic_t *nvic) {
    memset(st, 0, sizeof(*st));
    st->cpu = cpu;
    st->nvic = nvic;

    st->tick_event.callback = systick_fire;
    st->tick_event.user_data = st;

    arm_register_io(cpu, SYSTICK_IO_BASE, SYSTICK_IO_SIZE,
                    systick_read, systick_write, st);
}

void arm_systick_update(arm_systick_t *st) {
    if ((st->csr & SYST_CSR_ENABLE) && st->rvr > 0) {
        arm_schedule_event(st->cpu, &st->tick_event,
                          st->cpu->cycles + st->rvr + 1);
    }
}

int arm_systick_get_fire_count(void) {
    return systick_fire_count;
}

void arm_systick_reset_fire_count(void) {
    systick_fire_count = 0;
}
