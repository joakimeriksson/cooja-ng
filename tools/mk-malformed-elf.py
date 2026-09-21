#!/usr/bin/env python3
"""Build the malformed MSP430 ELF inputs for tools/check-elf-malformed.sh.

Each image is malformed in one way: a symbol value far outside memory,
straddling its end or wrapping a naive bound; a string table with no
terminator; a diagnostic table at the top of memory; a header that places
nothing in memory.  The symbol cases carry a real, minimal program -- a
`jmp $` at 0x4000 and a reset vector pointing at it -- so they get past the
loader and reach the symbol-patching code.

Usage: tools/mk-malformed-elf.py OUTDIR
Prints one line per image: <file> <expectation>.
"""
import os
import struct
import sys

EHDR_SIZE, PHDR_SIZE, SHDR_SIZE, SYM_SIZE = 52, 32, 40, 16
STB_GLOBAL_OBJECT = 0x11


def ehdr(phoff=0, phnum=0, phentsize=PHDR_SIZE, shoff=0, shnum=0):
    return (b'\x7fELF' + bytes([1, 1, 1]) + b'\x00' * 9 +
            struct.pack('<HHIIIIIHHHHHH', 2, 105, 1, 0x4000,
                        phoff, shoff, 0, EHDR_SIZE, phentsize, phnum,
                        SHDR_SIZE, shnum, 0))


def phdr(paddr, off, size, memsz=None):
    memsz = size if memsz is None else memsz
    return struct.pack('<IIIIIIII', 1, off, paddr, paddr, size, memsz, 7, 2)


def shdr(typ=0, off=0, size=0, link=0, entsize=0):
    return struct.pack('<IIIIIIIIII', 0, typ, 0, 0, off, size,
                       link, 0, 1, entsize)


def image(symbols, strtab=None, segments=None):
    """An MSP430 ELF with the given PT_LOAD segments and symbol table.

    symbols:  [(name, value)], or raw (strtab, symtab) via `strtab`.
    segments: [(paddr, bytes)]; default is the minimal program.
    """
    if segments is None:
        segments = [(0x4000, b'\xff\x3f'),      # jmp $
                    (0xfffe, b'\x00\x40')]      # reset vector -> 0x4000
    if strtab is None:
        names, syms = b'\x00', b''
        for name, value in symbols:
            syms += struct.pack('<IIIBBH', len(names), value, 2,
                                STB_GLOBAL_OBJECT, 0, 1)
            names += name.encode() + b'\x00'
    else:
        names, syms = strtab

    phoff = EHDR_SIZE
    data_off = phoff + PHDR_SIZE * len(segments)
    body, phdrs = b'', b''
    for paddr, data in segments:
        phdrs += phdr(paddr, data_off + len(body), len(data))
        body += data
    symtab_off = data_off + len(body)
    strtab_off = symtab_off + len(syms)
    shoff = strtab_off + len(names)
    shdrs = (shdr() +
             shdr(typ=2, off=symtab_off, size=len(syms), link=2,
                  entsize=SYM_SIZE) +
             shdr(typ=3, off=strtab_off, size=len(names)))
    return (ehdr(phoff=phoff, phnum=len(segments), shoff=shoff, shnum=3) +
            phdrs + body + syms + names + shdrs)


WILD = 0x40000000          # 1 GiB: far outside any MSP430 address space
TOP16 = 0xfffc             # an 8-byte write here straddles the end of 64 KB
TOP20 = 0xffffc            # the same for a 1 MB MSP430X part
WRAP = 0xffffffff          # addr + len wraps a naive bound

# A table-sized block at the top of 64 KB: 14 nonzero "used" bytes, then the
# reset vector.  With the table base at 0xffff every used[] entry points past
# the end of memory.
TOP_OF_MEMORY = [(0x4000, b'\xff\x3f'),
                 (0xfff0, b'\x01' * 14 + b'\x00\x40')]

# (file, expectation, bytes).  Expectations:
#   patch-skipped  runs to completion and says a symbol was not patched
#   no-crash       runs to completion (exit < 128, no sanitizer report)
#   rejected       fails to boot: non-zero exit and "Failed to initialize node"
CASES = [
    ('ds2411-id-wild.sky', 'patch-skipped',
     image([('ds2411_id', WILD)])),
    ('ds2411-id-straddle.sky', 'patch-skipped',
     image([('ds2411_id', TOP16)])),
    ('ds2411-init-wild.sky', 'patch-skipped',
     image([('ds2411_init', WILD)])),
    ('node-id-wrap.sky', 'patch-skipped',
     image([('node_id', WRAP)])),
    ('msp430x-linkaddr-wild.z1', 'patch-skipped',
     image([('linkaddr_node_addr', WILD)])),
    ('msp430x-linkaddr-straddle.z1', 'patch-skipped',
     image([('linkaddr_node_addr', TOP20)])),
    ('msp430x-xmem-init-wild.z1', 'patch-skipped',
     image([('xmem_init', WILD)])),
    ('msp430x-node-id-wild.z1', 'patch-skipped',
     image([('node_id', WILD)])),
    # strtab ending in "main" with no NUL: strcmp would read past the buffer.
    ('unterminated-strtab.sky', 'no-crash',
     image(None, strtab=(b'A' * 4092 + b'main',
                         struct.pack('<IIIBBH', 4092, 0x4242, 0,
                                     STB_GLOBAL_OBJECT, 0, 1)))),
    ('neighbor-table-top.sky', 'no-crash',
     image([('neighbor_addr_mem_memb_mem', 0xffff),
            ('neighbor_addr_mem_memb_used', 0xfff0)],
           segments=TOP_OF_MEMORY)),
    ('sr-table-top.sky', 'no-crash',
     image([('nodememb_memb_mem', 0xffff),
            ('nodememb_memb_used', 0xfff0)],
           segments=TOP_OF_MEMORY)),
    ('tsch-state-wild.sky', 'no-crash',
     image([('tsch_is_coordinator', WILD),
            ('tsch_is_initialized', WILD),
            ('tsch_is_started', WILD),
            ('tsch_current_asn', WRAP),
            ('count', TOP16 + 2)])),
    # A valid header and nothing placed in memory.
    ('no-program-headers.sky', 'rejected', ehdr()),
    # e_phentsize 0 re-reads one (valid) header 0xffff times.
    ('phentsize-zero.sky', 'rejected',
     (ehdr(phoff=EHDR_SIZE, phnum=0xffff, phentsize=0) +
      phdr(0x4000, 0x100, 4)).ljust(0x200, b'\xcc')),
    # The only segment is at an ARM address: nothing routes to MSP430 memory.
    ('unmapped-segment.sky', 'rejected',
     image([], segments=[(0x20000000, b'\xff\x3f')])),
    # The only segment is BSS: memory is zeroed, no code is placed.
    ('bss-only.sky', 'rejected',
     ehdr(phoff=EHDR_SIZE, phnum=1) + phdr(0x4000, 0, 0, memsz=0x100)),
    # Partially loaded: one good segment, one that routes nowhere.  Counting
    # only the bytes placed would boot this -- with the hole where the second
    # segment belonged, here the reset vector, so it starts at PC=0.
    ('partial-load-unroutable.sky', 'rejected',
     image([], segments=[(0x4000, b'\xff\x3f'), (0xffffffff, b'\x00\x40')])),
    # The same, with the unroutable segment straddling the end of memory.
    ('partial-load-straddle.sky', 'rejected',
     image([], segments=[(0x4000, b'\xff\x3f'), (0xfffe, b'\x00\x40\x00\x40')])),
]


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    out = sys.argv[1]
    os.makedirs(out, exist_ok=True)
    for name, expect, data in CASES:
        with open(os.path.join(out, name), 'wb') as f:
            f.write(data)
        print(name, expect)


if __name__ == '__main__':
    main()
