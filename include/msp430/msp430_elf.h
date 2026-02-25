/*
 * Minimal ELF loader for MSP430 firmware
 */
#ifndef MSP430_ELF_H
#define MSP430_ELF_H

#include "msp430_cpu.h"

/* Load an ELF file into CPU memory. Returns 0 on success, -1 on error. */
int msp430_load_elf(msp430_cpu_t *cpu, const char *path);

/* Look up a symbol by name in an ELF file. Returns address, or 0 on failure. */
uint32_t msp430_elf_find_symbol(const char *path, const char *symbol_name);

#endif /* MSP430_ELF_H */
