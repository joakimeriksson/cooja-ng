/*
 * Minimal ELF loader for MSP430 firmware
 */
#include "msp430_elf.h"
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
#define SHT_STRTAB 3

#define EI_MAG0    0
#define ELFMAG0    0x7f
#define ELFMAG1    'E'
#define ELFMAG2    'L'
#define ELFMAG3    'F'

int msp430_load_elf(msp430_cpu_t *cpu, const char *path) {
    FILE *f = fopen(path, "rb");
    if (!f) {
        fprintf(stderr, "Cannot open ELF file: %s\n", path);
        return -1;
    }

    /* Read ELF header */
    Elf32_Ehdr ehdr;
    if (fread(&ehdr, sizeof(ehdr), 1, f) != 1) {
        fprintf(stderr, "Failed to read ELF header\n");
        fclose(f);
        return -1;
    }

    /* Verify ELF magic */
    if (ehdr.e_ident[EI_MAG0] != ELFMAG0 ||
        ehdr.e_ident[1] != ELFMAG1 ||
        ehdr.e_ident[2] != ELFMAG2 ||
        ehdr.e_ident[3] != ELFMAG3) {
        fprintf(stderr, "Not an ELF file: %s\n", path);
        fclose(f);
        return -1;
    }

    /* Process program headers */
    for (int i = 0; i < ehdr.e_phnum; i++) {
        Elf32_Phdr phdr;
        fseek(f, ehdr.e_phoff + i * ehdr.e_phentsize, SEEK_SET);
        if (fread(&phdr, sizeof(phdr), 1, f) != 1) {
            fprintf(stderr, "Failed to read program header %d\n", i);
            fclose(f);
            return -1;
        }

        if (phdr.p_type != PT_LOAD) continue;
        if (phdr.p_filesz == 0) continue;

        /* Check bounds */
        if (phdr.p_paddr + phdr.p_filesz > cpu->max_mem) {
            fprintf(stderr, "ELF segment exceeds memory: addr=0x%x size=0x%x\n",
                    phdr.p_paddr, phdr.p_filesz);
            fclose(f);
            return -1;
        }

        /* Load segment */
        fseek(f, phdr.p_offset, SEEK_SET);
        if (fread(cpu->memory + phdr.p_paddr, 1, phdr.p_filesz, f) != phdr.p_filesz) {
            fprintf(stderr, "Failed to read segment data\n");
            fclose(f);
            return -1;
        }

        /* Zero BSS (memsz > filesz) */
        if (phdr.p_memsz > phdr.p_filesz) {
            uint32_t bss_start = phdr.p_paddr + phdr.p_filesz;
            uint32_t bss_size = phdr.p_memsz - phdr.p_filesz;
            if (bss_start + bss_size <= cpu->max_mem) {
                memset(cpu->memory + bss_start, 0, bss_size);
            }
        }
    }

    fclose(f);
    return 0;
}

uint32_t msp430_elf_find_symbol(const char *path, const char *symbol_name) {
    FILE *f = fopen(path, "rb");
    if (!f) return 0;

    Elf32_Ehdr ehdr;
    if (fread(&ehdr, sizeof(ehdr), 1, f) != 1) { fclose(f); return 0; }

    /* Find symtab and its linked strtab */
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

    /* Read strtab */
    Elf32_Shdr strtab_hdr;
    fseek(f, ehdr.e_shoff + symtab_hdr.sh_link * ehdr.e_shentsize, SEEK_SET);
    if (fread(&strtab_hdr, sizeof(strtab_hdr), 1, f) != 1) { fclose(f); return 0; }

    char *strtab = (char *)malloc(strtab_hdr.sh_size);
    if (!strtab) { fclose(f); return 0; }
    fseek(f, strtab_hdr.sh_offset, SEEK_SET);
    if (fread(strtab, 1, strtab_hdr.sh_size, f) != strtab_hdr.sh_size) {
        free(strtab); fclose(f); return 0;
    }

    /* Iterate symbols */
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
