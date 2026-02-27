/*
 * ARM ELF loader — loads firmware into CPU flash/SRAM regions
 */
#include "arm_elf.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ELF32 header structures */
#pragma pack(push, 1)

typedef struct {
    uint8_t  e_ident[16];
    uint16_t e_type;
    uint16_t e_machine;
    uint32_t e_version;
    uint32_t e_entry;
    uint32_t e_phoff;
    uint32_t e_shoff;
    uint32_t e_flags;
    uint16_t e_ehsize;
    uint16_t e_phentsize;
    uint16_t e_phnum;
    uint16_t e_shentsize;
    uint16_t e_shnum;
    uint16_t e_shstrndx;
} Elf32_Ehdr;

typedef struct {
    uint32_t p_type;
    uint32_t p_offset;
    uint32_t p_vaddr;
    uint32_t p_paddr;
    uint32_t p_filesz;
    uint32_t p_memsz;
    uint32_t p_flags;
    uint32_t p_align;
} Elf32_Phdr;

typedef struct {
    uint32_t sh_name;
    uint32_t sh_type;
    uint32_t sh_flags;
    uint32_t sh_addr;
    uint32_t sh_offset;
    uint32_t sh_size;
    uint32_t sh_link;
    uint32_t sh_info;
    uint32_t sh_addralign;
    uint32_t sh_entsize;
} Elf32_Shdr;

typedef struct {
    uint32_t st_name;
    uint32_t st_value;
    uint32_t st_size;
    uint8_t  st_info;
    uint8_t  st_other;
    uint16_t st_shndx;
} Elf32_Sym;

#pragma pack(pop)

#define PT_LOAD   1
#define SHT_SYMTAB 2

#define ELFMAG0    0x7f

/* Route an address to the correct memory region and return pointer + max writable */
static uint8_t *route_address(arm_cpu_t *cpu, uint32_t addr, uint32_t size, uint32_t *avail) {
    if (addr >= ARM_FLASH_BASE && addr + size <= ARM_FLASH_BASE + ARM_FLASH_SIZE) {
        *avail = ARM_FLASH_BASE + ARM_FLASH_SIZE - addr;
        return cpu->flash + (addr - ARM_FLASH_BASE);
    }
    if (addr >= ARM_SRAM_BASE && addr + size <= ARM_SRAM_BASE + ARM_SRAM_SIZE) {
        *avail = ARM_SRAM_BASE + ARM_SRAM_SIZE - addr;
        return cpu->sram + (addr - ARM_SRAM_BASE);
    }
    if (addr < ARM_ROM_SIZE) {
        *avail = ARM_ROM_SIZE - addr;
        return cpu->rom + addr;
    }
    *avail = 0;
    return NULL;
}

int arm_load_elf(arm_cpu_t *cpu, const char *path) {
    FILE *f = fopen(path, "rb");
    if (!f) {
        fprintf(stderr, "Cannot open ELF file: %s\n", path);
        return -1;
    }

    Elf32_Ehdr ehdr;
    if (fread(&ehdr, sizeof(ehdr), 1, f) != 1) {
        fprintf(stderr, "Failed to read ELF header\n");
        fclose(f);
        return -1;
    }

    if (ehdr.e_ident[0] != ELFMAG0 || ehdr.e_ident[1] != 'E' ||
        ehdr.e_ident[2] != 'L' || ehdr.e_ident[3] != 'F') {
        fprintf(stderr, "Not an ELF file: %s\n", path);
        fclose(f);
        return -1;
    }

    /* Verify ARM architecture */
    if (ehdr.e_machine != 40) { /* EM_ARM = 40 */
        fprintf(stderr, "Not an ARM ELF file (machine=%d): %s\n", ehdr.e_machine, path);
        fclose(f);
        return -1;
    }

    for (int i = 0; i < ehdr.e_phnum; i++) {
        Elf32_Phdr phdr;
        fseek(f, ehdr.e_phoff + i * ehdr.e_phentsize, SEEK_SET);
        if (fread(&phdr, sizeof(phdr), 1, f) != 1) {
            fprintf(stderr, "Failed to read program header %d\n", i);
            fclose(f);
            return -1;
        }

        if (phdr.p_type != PT_LOAD) continue;
        if (phdr.p_filesz == 0 && phdr.p_memsz == 0) continue;

        uint32_t avail;
        uint8_t *dest = route_address(cpu, phdr.p_paddr, phdr.p_filesz, &avail);
        if (!dest) {
            /* Try vaddr if paddr didn't work */
            dest = route_address(cpu, phdr.p_vaddr, phdr.p_filesz, &avail);
        }
        if (!dest) {
            fprintf(stderr, "ARM ELF: segment at 0x%08x (size 0x%x) doesn't map to memory\n",
                    phdr.p_paddr, phdr.p_filesz);
            continue;
        }

        if (phdr.p_filesz > 0) {
            fseek(f, phdr.p_offset, SEEK_SET);
            uint32_t read_size = phdr.p_filesz;
            if (read_size > avail) read_size = avail;
            if (fread(dest, 1, read_size, f) != read_size) {
                fprintf(stderr, "Failed to read segment data\n");
                fclose(f);
                return -1;
            }
        }

        /* Zero BSS */
        if (phdr.p_memsz > phdr.p_filesz) {
            uint32_t bss_offset = phdr.p_filesz;
            uint32_t bss_size = phdr.p_memsz - phdr.p_filesz;
            if (bss_offset + bss_size <= avail) {
                memset(dest + bss_offset, 0, bss_size);
            }
        }
    }

    fclose(f);

    /* Resolve firmware helper symbols (optional).
       Strip Thumb bit (bit 0) so addresses match the even-aligned PC. */
    cpu->fw_udivmoddi4 = arm_elf_find_symbol(path, "__udivmoddi4") & ~1u;
    cpu->fw_aeabi_uldivmod = arm_elf_find_symbol(path, "__aeabi_uldivmod") & ~1u;
    return 0;
}

uint32_t arm_elf_find_symbol(const char *path, const char *symbol_name) {
    FILE *f = fopen(path, "rb");
    if (!f) return 0;

    Elf32_Ehdr ehdr;
    if (fread(&ehdr, sizeof(ehdr), 1, f) != 1) { fclose(f); return 0; }

    Elf32_Shdr symtab_hdr = {0};
    bool found_symtab = false;

    for (int i = 0; i < ehdr.e_shnum; i++) {
        Elf32_Shdr shdr;
        fseek(f, ehdr.e_shoff + i * ehdr.e_shentsize, SEEK_SET);
        if (fread(&shdr, sizeof(shdr), 1, f) != 1) { fclose(f); return 0; }
        if (shdr.sh_type == SHT_SYMTAB) {
            symtab_hdr = shdr;
            found_symtab = true;
            break;
        }
    }
    if (!found_symtab) { fclose(f); return 0; }

    Elf32_Shdr strtab_hdr;
    fseek(f, ehdr.e_shoff + symtab_hdr.sh_link * ehdr.e_shentsize, SEEK_SET);
    if (fread(&strtab_hdr, sizeof(strtab_hdr), 1, f) != 1) { fclose(f); return 0; }

    char *strtab = (char *)malloc(strtab_hdr.sh_size);
    if (!strtab) { fclose(f); return 0; }
    fseek(f, strtab_hdr.sh_offset, SEEK_SET);
    if (fread(strtab, 1, strtab_hdr.sh_size, f) != strtab_hdr.sh_size) {
        free(strtab); fclose(f); return 0;
    }

    int num_syms = symtab_hdr.sh_size / sizeof(Elf32_Sym);
    uint32_t result = 0;
    for (int i = 0; i < num_syms; i++) {
        Elf32_Sym sym;
        fseek(f, symtab_hdr.sh_offset + i * sizeof(Elf32_Sym), SEEK_SET);
        if (fread(&sym, sizeof(sym), 1, f) != 1) break;
        if (sym.st_name < strtab_hdr.sh_size &&
            strcmp(strtab + sym.st_name, symbol_name) == 0) {
            result = sym.st_value;
            break;
        }
    }

    free(strtab);
    fclose(f);
    return result;
}

uint32_t arm_elf_get_entry(const char *path) {
    FILE *f = fopen(path, "rb");
    if (!f) return 0;

    Elf32_Ehdr ehdr;
    if (fread(&ehdr, sizeof(ehdr), 1, f) != 1) { fclose(f); return 0; }
    fclose(f);
    return ehdr.e_entry;
}
