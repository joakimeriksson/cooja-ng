/*
 * Minimal Cortex-M startup for the csim sniffer demo.
 * Vector table, .data copy, .bss zero, call main.  No libc.
 */
#include <stdint.h>

extern uint32_t _sidata, _sdata, _edata, _sbss, _ebss, _estack;
extern int main(void);

void Reset_Handler(void) {
    uint32_t *src = &_sidata, *dst = &_sdata;
    while (dst < &_edata) *dst++ = *src++;
    for (dst = &_sbss; dst < &_ebss; ) *dst++ = 0;
    main();
    for (;;) { }
}

static void Default_Handler(void) { for (;;) { } }

void csim_irq_handler(void);
void systick_handler(void);

/* The csim device's line is IRQ 10 in csim-host.repl, so its handler sits at
 * vector 16 + 10.  SysTick is vector 15. */
__attribute__((section(".isr_vector"), used))
void (* const vector_table[])(void) = {
    (void (*)(void))&_estack,
    Reset_Handler,
    Default_Handler,   /* NMI            */
    Default_Handler,   /* HardFault      */
    Default_Handler,   /* MemManage      */
    Default_Handler,   /* BusFault       */
    Default_Handler,   /* UsageFault     */
    0, 0, 0, 0,
    Default_Handler,   /* SVCall         */
    Default_Handler,   /* DebugMonitor   */
    0,
    Default_Handler,   /* PendSV         */
    systick_handler,   /* SysTick        */
    /* External interrupts 0..10; only 10 (the csim device) is used. */
    Default_Handler, Default_Handler, Default_Handler, Default_Handler,
    Default_Handler, Default_Handler, Default_Handler, Default_Handler,
    Default_Handler, Default_Handler,
    csim_irq_handler,  /* IRQ 10 — csim co-simulation device */
};
