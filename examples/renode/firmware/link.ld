/* Flat Cortex-M layout matching csim-host.repl: flash at 0, SRAM at 0x20000000. */
MEMORY {
    FLASH (rx)  : ORIGIN = 0x00000000, LENGTH = 1024K
    SRAM  (rwx) : ORIGIN = 0x20000000, LENGTH = 128K
}

_estack = ORIGIN(SRAM) + LENGTH(SRAM);

SECTIONS {
    .isr_vector : { KEEP(*(.isr_vector)) } > FLASH
    .text   : { *(.text*) *(.rodata*) } > FLASH
    _sidata = LOADADDR(.data);
    .data   : { _sdata = .; *(.data*) . = ALIGN(4); _edata = .; } > SRAM AT > FLASH
    .bss    : { _sbss = .; *(.bss*) *(COMMON) . = ALIGN(4); _ebss = .; } > SRAM
}
