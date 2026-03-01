/*
 * Shared ELF32 loader — structures, constants, and common operations
 */
#ifndef ELF_LOADER_H
#define ELF_LOADER_H

#include <stdint.h>

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

/* ELF constants */
#define PT_LOAD    1
#define SHT_SYMTAB 2
#define SHT_STRTAB 3

#define ELFMAG0    0x7f
#define ELFMAG1    'E'
#define ELFMAG2    'L'
#define ELFMAG3    'F'

/* Segment routing callback — architecture provides address routing */
typedef struct {
    uint8_t *dest;      /* destination pointer (NULL = skip segment) */
    uint32_t capacity;  /* max bytes writable at dest */
} elf_segment_route_t;

typedef elf_segment_route_t (*elf_route_fn)(void *ctx, uint32_t paddr,
                                             uint32_t vaddr, uint32_t size);

/* Check ELF magic bytes. Returns 0 on success, -1 on failure. */
int elf_check_magic(const Elf32_Ehdr *ehdr);

/* Load PT_LOAD segments from an ELF file using the given routing callback.
 * The callback maps physical/virtual addresses to destination pointers.
 * Returns 0 on success, -1 on error. */
int elf_load_segments(const char *path, elf_route_fn route, void *ctx);

/* Look up a symbol by name in an ELF file. Returns address, or 0 on failure. */
uint32_t elf_find_symbol(const char *path, const char *symbol_name);

/* Get the entry point from an ELF file. Returns 0 on failure. */
uint32_t elf_get_entry(const char *path);

/* Get the machine type from an ELF file. Returns 0 on failure. */
uint16_t elf_get_machine(const char *path);

#endif /* ELF_LOADER_H */
