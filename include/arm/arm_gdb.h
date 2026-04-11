/*
 * GDB stub glue for ARM Cortex-M3.
 *
 * Provides the gdb_arch_ops_t vtable that the generic stub uses to
 * read/write registers, read/write memory, and get/set the PC for
 * an arm_cpu_t instance.
 */
#ifndef ARM_GDB_H
#define ARM_GDB_H

#include "gdb_stub.h"

extern const gdb_arch_ops_t arm_gdb_ops;

#endif /* ARM_GDB_H */
