/*
 * Shared ELF32 loader implementation
 */
#include "elf_loader.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

int elf_check_magic(const Elf32_Ehdr *ehdr) {
    if (ehdr->e_ident[0] != ELFMAG0 ||
        ehdr->e_ident[1] != ELFMAG1 ||
        ehdr->e_ident[2] != ELFMAG2 ||
        ehdr->e_ident[3] != ELFMAG3) {
        return -1;
    }
    return 0;
}

int elf_load_segments(const char *path, elf_route_fn route, void *ctx) {
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

    if (elf_check_magic(&ehdr) != 0) {
        fprintf(stderr, "Not an ELF file: %s\n", path);
        fclose(f);
        return -1;
    }

    /* e_phentsize drives the header offsets below: anything but the ELF32
     * size makes them read the wrong bytes (0 re-reads one header e_phnum
     * times). */
    if (ehdr.e_phnum > 0 && ehdr.e_phentsize != sizeof(Elf32_Phdr)) {
        fprintf(stderr, "Unexpected program header size %u in %s\n",
                ehdr.e_phentsize, path);
        fclose(f);
        return -1;
    }

    /* Bytes actually placed in memory: an image that places none would
     * boot a zero-filled address space and look like it ran. */
    uint64_t loaded = 0;
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

        elf_segment_route_t seg = route(ctx, phdr.p_paddr, phdr.p_vaddr, phdr.p_filesz);
        if (!seg.dest) {
            if (phdr.p_filesz > 0) {
                fprintf(stderr, "ELF segment at 0x%x (size 0x%x) doesn't map to memory\n",
                        phdr.p_paddr, phdr.p_filesz);
            }
            continue;
        }

        if (phdr.p_filesz > 0) {
            fseek(f, phdr.p_offset, SEEK_SET);
            uint32_t read_size = phdr.p_filesz;
            if (read_size > seg.capacity) read_size = seg.capacity;
            if (fread(seg.dest, 1, read_size, f) != read_size) {
                fprintf(stderr, "Failed to read segment data\n");
                fclose(f);
                return -1;
            }
            loaded += read_size;
        }

        /* Zero BSS (memsz > filesz) */
        if (phdr.p_memsz > phdr.p_filesz) {
            uint32_t bss_offset = phdr.p_filesz;
            uint32_t bss_size = phdr.p_memsz - phdr.p_filesz;
            if (bss_offset + bss_size <= seg.capacity) {
                memset(seg.dest + bss_offset, 0, bss_size);
            }
        }
    }

    fclose(f);
    if (loaded == 0) {
        fprintf(stderr, "No loadable segment maps to memory: %s\n", path);
        return -1;
    }
    return 0;
}

uint32_t elf_find_symbol(const char *path, const char *symbol_name) {
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

    /* Read strtab.  sh_link/sh_size come from the file; validate the section
     * index against e_shnum and cap the allocation so a malformed symtab
     * can't drive an out-of-range fseek or a multi-GB malloc (DoS). */
    if (symtab_hdr.sh_link >= ehdr.e_shnum) { fclose(f); return 0; }
    Elf32_Shdr strtab_hdr;
    fseek(f, ehdr.e_shoff + symtab_hdr.sh_link * ehdr.e_shentsize, SEEK_SET);
    if (fread(&strtab_hdr, sizeof(strtab_hdr), 1, f) != 1) { fclose(f); return 0; }

    #define ELF_STRTAB_MAX (16u * 1024u * 1024u)   /* 16 MB — far above any real firmware */
    if (strtab_hdr.sh_size == 0 || strtab_hdr.sh_size > ELF_STRTAB_MAX) {
        fclose(f); return 0;
    }
    /* One extra byte for a terminator: a real strtab ends in NUL, a crafted
     * one need not, and strcmp below would then run off the allocation. */
    char *strtab = (char *)malloc(strtab_hdr.sh_size + 1);
    if (!strtab) { fclose(f); return 0; }
    fseek(f, strtab_hdr.sh_offset, SEEK_SET);
    if (fread(strtab, 1, strtab_hdr.sh_size, f) != strtab_hdr.sh_size) {
        free(strtab); fclose(f); return 0;
    }
    strtab[strtab_hdr.sh_size] = '\0';

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

uint32_t elf_get_entry(const char *path) {
    FILE *f = fopen(path, "rb");
    if (!f) return 0;

    Elf32_Ehdr ehdr;
    if (fread(&ehdr, sizeof(ehdr), 1, f) != 1) { fclose(f); return 0; }
    fclose(f);
    return ehdr.e_entry;
}

uint16_t elf_get_machine(const char *path) {
    FILE *f = fopen(path, "rb");
    if (!f) return 0;

    Elf32_Ehdr ehdr;
    if (fread(&ehdr, sizeof(ehdr), 1, f) != 1) { fclose(f); return 0; }
    fclose(f);
    return ehdr.e_machine;
}
