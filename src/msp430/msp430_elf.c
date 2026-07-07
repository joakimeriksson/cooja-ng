/*
 * MSP430 ELF loader — thin wrapper over shared ELF loader
 */
#include "msp430_elf.h"
#include "elf_loader.h"

static elf_segment_route_t msp430_route(void *ctx, uint32_t paddr,
                                         uint32_t vaddr, uint32_t size) {
    (void)vaddr;
    msp430_cpu_t *cpu = ctx;
    /* paddr/size come straight from the ELF's p_paddr/p_filesz — check
     * without adding, so a crafted paddr near UINT32_MAX can't wrap the
     * sum past the bound and return a wild pointer. */
    if (paddr >= cpu->max_mem || size > cpu->max_mem - paddr)
        return (elf_segment_route_t){NULL, 0};
    return (elf_segment_route_t){cpu->memory + paddr, cpu->max_mem - paddr};
}

int msp430_load_elf(msp430_cpu_t *cpu, const char *path) {
    return elf_load_segments(path, msp430_route, cpu);
}

uint32_t msp430_elf_find_symbol(const char *path, const char *symbol_name) {
    return elf_find_symbol(path, symbol_name);
}
