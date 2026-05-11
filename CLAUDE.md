# CLAUDE.md — Cooja-NG C Emulator

## Build

```sh
make              # O3, LTO, auto-detects GNU Lightning for JIT
make debug        # O0, -g, DEBUG flag
make pgo          # Profile-guided optimization (~40% faster)
make clean        # Remove build/
```

GNU Lightning is optional (auto-detected via pkg-config). Without it, the interpreter is used for all execution.

## Testing

```sh
# MSP430 tests
./build/test_runner correctness -v    # 68 instruction-level tests
./build/test_runner bench             # 7 micro-benchmarks + 2 firmware benchmarks
./build/test_runner firmware          # Firmware integration tests (cputest.sky, timertest.sky)
./build/test_runner multinode         # 2-node nullnet-broadcast (default 20s)
./build/test_runner multinode firmware/sky/udp-server.sky firmware/sky/udp-client.sky -t 60000

# ARM Cortex-M3/M4 tests
./build/test_runner arm-correctness -v   # 74 instruction-level tests (Thumb-2 + M4 DSP + M4 VFP)
./build/test_runner arm-firmware -v      # Firmware boot test (hello-world.cc2538dk)
./build/test_runner arm-multinode firmware/cc2538dk/nullnet-broadcast.cc2538dk -t 20000
./build/test_runner arm-multinode firmware/cc2538dk/udp-server.cc2538dk firmware/cc2538dk/udp-client.cc2538dk -t 60000

# nRF52840 USB Dongle (PCA10059, Cortex-M4F + on-chip 802.15.4 radio)
./build/test_runner nrf52840-dongle-multinode firmware/nrf52840-dongle/udp-server.nrf52840-dongle firmware/nrf52840-dongle/udp-client.nrf52840-dongle -t 60000

# nRF52840 Development Kit (PCA10056, same SoC + SEGGER UART)
./build/test_runner nrf52840-dk-multinode firmware/nrf52840-dk/udp-server.nrf52840-dk firmware/nrf52840-dk/udp-client.nrf52840-dk -t 60000
```

Multinode options: `-t ms` (sim duration), `-n nodes` (node count), `-q` (quiet), `-v` (verbose).

## Project Structure

```
src/
  msp430/             MSP430 emulator source files
  arm/                ARM Cortex-M3 emulator source files
include/
  msp430/             MSP430 header files
  arm/                ARM header files
test/                 Test runner, correctness, benchmarks, firmware, multinode
firmware/sky/         Pre-compiled Contiki-NG firmware for Tmote Sky
firmware/cc2538dk/    Pre-compiled Contiki-NG firmware for CC2538DK
```

### MSP430 Source Files (src/msp430/)

| File | Purpose |
|------|---------|
| `msp430_cpu.c` | Core CPU: computed-goto interpreter, event queue, interrupt service, step/step_until |
| `msp430_config.c` | MCU configurations: F149, F1611, F2617, F5437, CC430F5137 |
| `msp430_clock.c` | Clock module: DCO frequency from DCOCTL/BCSCTL1, ACLK/SMCLK dividers |
| `msp430_timer.c` | Timer A/B: on-demand counter, CCR compare events, capture mode |
| `msp430_usart.c` | USART: TX callback, SPI exchange (bridges to CC2420) |
| `msp430_gpio.c` | GPIO P1-P10: IN/OUT/DIR/SEL/IFG/IES/IE, interrupt generation, output callbacks |
| `msp430_platform.c` | Platform bundles: MCU + all peripherals, platform lookup by name |
| `msp430_decode.c` | Stateless instruction decoder: memory -> decoded_insn_t, basic block decoder |
| `msp430_jit.c` | GNU Lightning JIT: compiles hot basic blocks to native ARM64/x86 code |
| `msp430_elf.c` | ELF loader: loads sections into memory, symbol lookup |
| `cc2420.c` | CC2420 radio: SPI protocol, TX/RX state machine, CRC, auto-ACK, GPIO pins |

### ARM Source Files (src/arm/)

| File | Purpose |
|------|---------|
| `arm_cpu.c` | Core Cortex-M3 CPU: Thumb/Thumb-2 interpreter, IT blocks, exception handling, step/step_until |
| `arm_config.c` | MCU configurations: CC2538 (512KB flash, 32KB SRAM, 32MHz) |
| `arm_elf.c` | ELF loader: loads sections into flash/SRAM, symbol lookup |
| `arm_nvic.c` | NVIC: interrupt priority, pending/enable registers, exception entry/return |
| `arm_systick.c` | SysTick timer: periodic tick generation via event queue |
| `arm_platform.c` | Platform bundles: CPU + all CC2538 peripherals |
| `cc2538_uart.c` | UART: TX callback, status registers (TXFF/TXFE) |
| `cc2538_gpio.c` | GPIO: port A-D, interrupt support |
| `cc2538_gptimer.c` | General Purpose Timers: one-shot, periodic, prescaler |
| `cc2538_sys_ctrl.c` | System control: clock config, OSC32K selection |
| `cc2538_ioc.c` | IO Controller: pin mux, pad configuration |
| `cc2538_rfcore.c` | RF Core: 802.15.4 TX/RX, FFSM address registers, RFRND |
| `cc2538_sleeptimer.c` | Sleep Timer: 32kHz counter, compare match interrupts |

## Architecture

### CPU Execution Engine

Computed-goto dispatch with flat memory array and per-address IO callbacks.

**Hot path** (no overhead from ns timing):
1. Fetch instruction at PC
2. Dispatch via computed goto (or JIT compiled block if hot)
3. Execute, update registers/flags/cycles
4. Check `cycles >= next_event_cycle` -> fire events if needed
5. Check pending interrupts if GIE=1

**Execution modes:**
- `msp430_step(cpu, n)` — Step n instructions
- `msp430_step_until(cpu, target_cycle)` — Run until cycle target reached

### Nanosecond Simulation Time

Dual-time architecture: CPU cycles (hot path) + nanosecond wall-clock time (for peripherals).

**Key fields in `msp430_cpu_t`:**
- `int64_t sim_time_ns` — current simulation time in nanoseconds
- `uint32_t cpu_freq_hz` — current CPU frequency (set by clock module)

**Event scheduling:**
- `msp430_schedule_event(cpu, ev, cycle)` — fire at specific CPU cycle (fire_ns=0)
- `msp430_schedule_event_ns(cpu, ev, ns)` — fire at wall-clock ns time (shadow fire_cycle computed)
- `msp430_cancel_event(cpu, ev)` — remove from queue

**Frequency changes** (`msp430_cpu_set_frequency()`):
- Called from `recalculate_dco()` when firmware writes DCOCTL/BCSCTL1/BCSCTL2
- Syncs `sim_time_ns` from cycles using OLD frequency
- Recomputes `fire_cycle` for all ns-based events using NEW frequency
- Re-sorts the event queue

**Conversion helpers** (inline):
```c
msp430_ns_to_cycles(ns, freq_hz)     // ns -> cycles
msp430_cycles_to_ns(cycles, freq_hz) // cycles -> ns
```

**sim_time_ns sync points:**
- `execute_events()` — before firing each event callback
- `msp430_step_until()` — at end, after reaching target cycle
- `msp430_cpu_set_frequency()` — before changing frequency

### JIT Compiler (GNU Lightning)

Compiles hot basic blocks to native code. Auto-detected via pkg-config.

**What gets JIT-compiled:**
- Two-operand ALU: ADD, ADDC, SUB, CMP, AND, OR, XOR, BIT, BIC, BIS, MOV (reg/CG/imm -> reg only)
- All 8 jump conditions: JNE, JEQ, JNC, JC, JN, JGE, JL, JMP
- Only blocks where ALL instructions are inlineable (no mixed inline/fallback)

**NOT compiled:** SUBC, DADD, memory-addressed operands, SR/PC writes, PUSH/CALL/RETI

**Configuration:**
- `MSPSIM_JIT_THRESHOLD` env var (default 100): executions before compiling
- `MSPSIM_JIT_INBLOCK_CHECKS` env var (default 0): interrupt checks inside JIT blocks
- `block_exec_count[pc>>1]` tracks execution count per block
- Cache invalidation on memory writes (clears compiled_cache + block_exec_count)

**Register allocation:**
- JIT_V0 = cpu pointer, JIT_V1 = reg[] base, JIT_V2 = cycles accumulator
- JIT_R0/R1/R2 = temporaries

### CC2420 Radio

Full state machine matching Java MSPSim's CC2420.java.

**Radio states:** VREG_OFF -> POWER_DOWN -> IDLE -> RX_CALIBRATE -> RX_SFD_SEARCH -> RX_FRAME (and TX chain)

**Timing (ns-based, CPU-clock independent):**
- Symbol period: 16,000 ns (62.5 ksym/s)
- Byte period: 32,000 ns (2 symbols/byte)
- Oscillator startup: 1,000,000 ns (1 ms)
- RX calibration: 12 symbols (192,000 ns)
- TX calibration: 12 symbols

**SPI protocol:** First byte determines operation (strobe, register read/write, RAM access, RXFIFO read, TXFIFO write). Status byte returned on every exchange.

**RX incoming buffer:** Bytes arriving during RX_CALIBRATE or RX_WAIT (post-TX turnaround) are buffered in `rx_incoming[256]` and batch-delivered when transitioning to RX_SFD_SEARCH.

**Auto-ACK:** If enabled (MDMCTRL0.AUTOACK) and frame has ACK_REQUEST bit and passes CRC + address filter, sends 5-byte ACK frame automatically.

**CRC:** CCITT-16 with bit reversal, matching CC2420 hardware behavior.

### Timer A/B

On-demand counter: not incremented every cycle. Instead, counter value is computed from elapsed cycles when read.

**Clock sources:** TCLK (external), ACLK (32768 Hz), SMCLK (DCO-derived), INCLK
**Modes:** Stop, Up (count to CCR0), Continuous (count to 0xFFFF), Up/Down
**Events:** CCR compare events scheduled in CPU event queue (cycle-based)

### Clock Module

DCO frequency formula (matches Java MSPSim):
```
dco_factor = (max_dco_freq - 1000) / 2048
freq = ((dco_freq << 5) + dco_mod + (rsel << 8)) * dco_factor + 1000
```

Default: DCOCTL=0x60, BCSCTL1=0x84 -> ~2.69 MHz (F1611). After firmware calibration: ~3.9 MHz.

ACLK is fixed at 32,768 Hz (crystal). SMCLK = DCO / divider.

## Platforms

| Platform | MCU | CC2420 | Console | Notes |
|----------|-----|--------|---------|-------|
| **sky** | MSP430F1611 | Yes | USART1 | Tmote Sky, primary test target |
| **esb** | MSP430F149 | No | USART1 | ETH ESB |
| **z1** | MSP430F2617 | No | USART0 | Zolertia Z1 |
| **wismote** | MSP430F5437 | No | USART1 | WisMote |
| **exp5438** | MSP430F5437 | No | USART1 | MSP-EXP5438 |
| **cc430** | CC430F5137 | No | USART0 | CC430 eval board |
| **cc2538dk** | CC2538 (ARM Cortex-M3) | Yes (on-chip) | UART0 | TI SmartRF06 + CC2538EM |
| **openmote** | CC2538 (ARM Cortex-M3) | Yes (on-chip) | UART0 | OpenMote board (same SoC as cc2538dk) |
| **zoul-firefly** | CC2538 (ARM Cortex-M3) | Yes (on-chip) + CC1200 (off-SoC sub-GHz) | UART0 | Zolertia Firefly — dual-band |
| **nrf52840-dongle** | nRF52840 (ARM Cortex-M4F) | Yes (on-chip 2.4 GHz) | UART0 (legacy window) | Nordic PCA10059 USB Dongle, M4F + FPv4-SP-D16, VTOR=0x1000 (Open Bootloader region at 0x0..0xfff) |
| **nrf52840-dk** | nRF52840 (ARM Cortex-M4F) | Yes (on-chip 2.4 GHz) | UART0 (legacy window) | Nordic PCA10056 Development Kit, same SoC as Dongle, VTOR=0x0, SEGGER VCP console |

## MCU Configurations

| MCU | Address Space | RAM | Flash | Max DCO | MSP430X |
|-----|---------------|-----|-------|---------|---------|
| MSP430F149 | 64 KB | 2 KB @ 0x200 | 60 KB @ 0x1100 | 4.9 MHz | No |
| MSP430F1611 | 64 KB | 10 KB @ 0x1100 | 48 KB @ 0x4000 | 4.9 MHz | No |
| MSP430F2617 | 1 MB | 8 KB @ 0x1100 | 92 KB @ 0x3100 | 16 MHz | Yes |
| MSP430F5437 | 1 MB | 16 KB @ 0x1C00 | 256 KB @ 0x5C00 | 25 MHz | Yes |
| CC430F5137 | 1 MB | 4 KB @ 0x1C00 | 32 KB @ 0x8000 | 25 MHz | Yes |

## Multi-Node Simulation

Single-threaded, round-robin time-stepped execution with ns-based coordination.

**Architecture:**
1. Initialize nodes: load ELF, patch ds2411_init -> RET, run crt0 to main
2. Patch ds2411_id per node (unique MAC addresses)
3. Main loop: advance sim_ns by 1ms, step each node to corresponding cycle target
4. RF: TX callback buffers bytes, deliver_rf_bytes() delivers between time steps

**ns-based stepping:**
```c
while (sim_ns < end_ns) {
    sim_ns += 1000000;  // 1ms
    for (each node) {
        delta_ns = sim_ns - cpu->sim_time_ns;
        target_cycle = cpu->cycles + ns_to_cycles(delta_ns, cpu->cpu_freq_hz);
        msp430_step_until(cpu, target_cycle);
    }
}
```

This correctly handles nodes running at different CPU frequencies after DCO calibration.

### ARM Multi-Node (CC2538)

Same time-stepped architecture as MSP430, with CC2538 RF Core for 802.15.4 radio:

1. Initialize nodes: load ELF, run crt0 to main, patch `linkaddr_node_addr` per node
2. Each node gets unique IEEE address (00:12:74:node_id:00:00:00:node_id)
3. RF Core TX callback buffers frames, `arm_deliver_rf_bytes()` delivers between time steps
4. Auto-ACK at link layer, CSMA retransmissions, 6LoWPAN/RPL networking all functional

**Supported firmware:**
- `nullnet-broadcast.cc2538dk` — simple broadcast, 2-node
- `udp-server.cc2538dk` + `udp-client.cc2538dk` — RPL-UDP with DAG formation

## MSP430 Instruction Encoding Notes

- Double-op format: `opcode(4)|src_reg(4)|Ad(1)|BW(1)|As(2)|dst_reg(4)`
- CG1 (R2): As=10->4, As=11->8. CG2 (R3): As=00->0, As=01->1, As=10->2, As=11->0xFFFF
- @Rn+ (autoincrement) = As=11, NOT As=01 (which is indexed)
- JMP offset: raw 10-bit value * 2 added to PC
- Reset interrupt adds 6 cycles

## Performance (Apple Silicon, PGO build)

| Benchmark | MIPS |
|-----------|------|
| Micro-benchmarks (avg) | ~430 |
| Firmware blink.sky | ~194 |
| Firmware energest-demo.sky | ~196 |
| 2-node nullnet (20s sim) | ~800x real-time |
| 2-node RPL-UDP (60s sim) | ~2400x real-time |

### ARM (CC2538, interpreter only)

| Benchmark | Speed |
|-----------|-------|
| 2-node RPL-UDP (60s sim) | ~4x real-time |
