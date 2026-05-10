/*
 * ARM ELF loader — thin wrapper over shared ELF loader
 */
#include "arm_elf.h"
#include "elf_loader.h"
#include <stdio.h>

static elf_segment_route_t arm_route(void *ctx, uint32_t paddr,
                                      uint32_t vaddr, uint32_t size) {
    arm_cpu_t *cpu = ctx;

    /* Try paddr first, then vaddr */
    uint32_t addr = paddr;
    for (int attempt = 0; attempt < 2; attempt++) {
        if (addr >= cpu->flash_base && addr + size <= cpu->flash_end)
            return (elf_segment_route_t){
                cpu->flash + (addr - cpu->flash_base),
                cpu->flash_end - addr
            };
        if (addr >= cpu->sram_base && addr + size <= cpu->sram_end)
            return (elf_segment_route_t){
                cpu->sram + (addr - cpu->sram_base),
                cpu->sram_end - addr
            };
        if (cpu->rom && addr < cpu->rom_size)
            return (elf_segment_route_t){
                cpu->rom + addr,
                cpu->rom_size - addr
            };
        addr = vaddr;  /* retry with vaddr */
    }

    return (elf_segment_route_t){NULL, 0};
}

int arm_load_elf(arm_cpu_t *cpu, const char *path) {
    /* Verify ARM architecture */
    uint16_t machine = elf_get_machine(path);
    if (machine != 40) { /* EM_ARM = 40 */
        fprintf(stderr, "Not an ARM ELF file (machine=%d): %s\n", machine, path);
        return -1;
    }

    int rc = elf_load_segments(path, arm_route, cpu);
    if (rc != 0) return rc;

    /* Resolve firmware helper symbols (optional).
       Strip Thumb bit (bit 0) so addresses match the even-aligned PC. */
    cpu->fw_udivmoddi4 = elf_find_symbol(path, "__udivmoddi4") & ~1u;
    cpu->fw_aeabi_uldivmod = elf_find_symbol(path, "__aeabi_uldivmod") & ~1u;
    return 0;
}

uint32_t arm_elf_find_symbol(const char *path, const char *symbol_name) {
    return elf_find_symbol(path, symbol_name);
}

uint32_t arm_elf_get_entry(const char *path) {
    return elf_get_entry(path);
}
