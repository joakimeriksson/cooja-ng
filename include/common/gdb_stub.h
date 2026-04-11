/*
 * GDB Remote Serial Protocol stub
 *
 * Implements enough of the GDB RSP for arm-none-eabi-gdb / msp430-gdb to
 * connect via `target remote :PORT` and:
 *   - read/write CPU registers
 *   - read/write memory
 *   - set/clear software breakpoints
 *   - single-step and continue
 *   - interrupt a running CPU with Ctrl+C
 *
 * Architecture-neutral: per-arch glue provides a small vtable
 * (gdb_arch_ops_t) that bridges register layout, memory access, and
 * the breakpoint check hook into the CPU's hot path.
 *
 * Single-connection at a time, blocking-on-halt, non-blocking-on-run.
 */
#ifndef GDB_STUB_H
#define GDB_STUB_H

#include <stdint.h>
#include <stdbool.h>

#define GDB_MAX_BREAKPOINTS  64
#define GDB_PACKET_BUF_SIZE  4096

/* Per-architecture vtable. Each function takes the CPU pointer that was
 * passed to gdb_stub_attach(). */
typedef struct gdb_arch_ops {
    /* Pack all GDB-visible registers into buf in GDB's expected order.
     * Returns the number of bytes written. */
    int  (*read_regs)(void *cpu, uint8_t *buf, int buf_size);

    /* Unpack registers from buf (length matches read_regs output). */
    void (*write_regs)(void *cpu, const uint8_t *buf, int len);

    /* Read N bytes from CPU memory at addr into buf. Returns bytes read
     * (may be less than len on partial / unmapped). */
    int  (*read_mem)(void *cpu, uint32_t addr, uint8_t *buf, int len);

    /* Write N bytes from buf to CPU memory at addr. Returns bytes written. */
    int  (*write_mem)(void *cpu, uint32_t addr, const uint8_t *buf, int len);

    /* Get/set the program counter. */
    uint32_t (*get_pc)(void *cpu);
    void     (*set_pc)(void *cpu, uint32_t pc);
} gdb_arch_ops_t;

/* GDB stub state — opaque to callers, but defined here so callers can
 * embed it in larger structs without heap allocation. */
typedef struct gdb_stub {
    int  listen_fd;
    int  client_fd;
    int  port;
    bool connected;

    /* True when the CPU should be paused waiting for GDB commands.
     * Set on breakpoint hit, single-step completion, or Ctrl+C from GDB.
     * Cleared by `c` (continue) and `s` (step). */
    bool halted;

    /* Stop signal to report on next ?/T packet (5 = SIGTRAP). */
    int  stop_signal;

    /* Software breakpoints — addresses the CPU should halt at. */
    uint32_t breakpoints[GDB_MAX_BREAKPOINTS];
    int      num_breakpoints;

    /* Per-arch glue */
    void                 *cpu;
    const gdb_arch_ops_t *ops;

    /* Packet I/O scratch buffers (avoid heap allocation in hot path). */
    char rx_buf[GDB_PACKET_BUF_SIZE];
    int  rx_len;
    char tx_buf[GDB_PACKET_BUF_SIZE];
} gdb_stub_t;

/* Initialize the stub and bind a TCP listener on the given port.
 * Returns 0 on success, -1 on bind/listen failure (errno set). */
int  gdb_stub_init(gdb_stub_t *stub, int port);

/* Attach the per-arch CPU + vtable. Must be called before the first
 * gdb_stub_poll() that the simulator runs. */
void gdb_stub_attach(gdb_stub_t *stub, void *cpu, const gdb_arch_ops_t *ops);

/* Wait (blocking) until a GDB client connects. Useful for --gdb-wait. */
int  gdb_stub_wait_for_client(gdb_stub_t *stub);

/* Non-blocking poll: accept new connections, drain incoming packets,
 * dispatch commands. Safe to call from the sim main loop every tick.
 * Returns true if the stub is currently halted (CPU should not advance). */
bool gdb_stub_poll(gdb_stub_t *stub);

/* Check if PC matches a breakpoint; if so, halt and notify GDB.
 * Call this in the per-arch step loop just before each instruction.
 * Returns true if execution should pause now. */
bool gdb_stub_check_breakpoint(gdb_stub_t *stub, uint32_t pc);

/* Notify GDB of a stop and enter halted state. signal is one of:
 *   5 = SIGTRAP (breakpoint, single-step)
 *   2 = SIGINT  (Ctrl+C from GDB) */
void gdb_stub_notify_halt(gdb_stub_t *stub, int signal);

/* Tear down the stub, close all sockets. */
void gdb_stub_destroy(gdb_stub_t *stub);

#endif /* GDB_STUB_H */
