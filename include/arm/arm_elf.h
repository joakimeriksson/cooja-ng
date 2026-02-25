/*
 * ARM ELF loader — loads sections into CPU memory regions
 */
#ifndef ARM_ELF_H
#define ARM_ELF_H

#include "arm_cpu.h"

/* Load an ARM ELF file into CPU memory. Returns 0 on success, -1 on error. */
int arm_load_elf(arm_cpu_t *cpu, const char *path);

/* Look up a symbol by name in an ARM ELF file. Returns address, or 0 on failure. */
uint32_t arm_elf_find_symbol(const char *path, const char *symbol_name);

/* Get the entry point from an ARM ELF file. Returns 0 on failure. */
uint32_t arm_elf_get_entry(const char *path);

#endif /* ARM_ELF_H */
