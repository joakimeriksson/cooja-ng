# Cooja-NG

A fast multi-architecture emulator and network simulator written in C, designed for large-scale IoT simulation with Contiki-NG firmware.

## Features

### MSP430
- **MSP430/MSP430X CPU emulator** with threaded interpreter (computed goto dispatch)
- **JIT compiler** using GNU Lightning for hot basic blocks (~430 MIPS on Apple Silicon)
- **CC2420 802.15.4 radio** with full state machine, SPI, TX/RX FIFO, CRC, and auto-ACK
- **Peripherals**: Timer A/B, USART, GPIO, clock system (DCO/MCLK/SMCLK/ACLK)
- **Platform support**: Tmote Sky (MSP430F1611 + CC2420)

### ARM Cortex-M3
- **ARM Cortex-M3 CPU emulator** with Thumb/Thumb-2 interpreter
- **CC2538 SoC** with on-chip 802.15.4 radio (RF Core), NVIC, SysTick
- **Peripherals**: UART, GPIO, General Purpose Timers, Sleep Timer, System Control, IOC
- **Platform support**: CC2538DK (SmartRF06 + CC2538EM)
- **RPL-UDP networking**: full 6LoWPAN/RPL stack convergence in multi-node simulation

### Common
- **Multi-node simulation** with time-stepped RF byte delivery
- **Mixed-platform simulation**: MSP430 and ARM nodes in the same network
- **ELF loader** for Contiki-NG firmware binaries
- **Nanosecond simulation time** for CPU-clock-independent scheduling

## Building

```sh
make        # standard build
make pgo    # profile-guided optimization (~40% faster)
```

GNU Lightning is optional (auto-detected via pkg-config). Without it, the interpreter is used for all execution.

## Running tests

### MSP430

```sh
./build/test_runner correctness      # 68 instruction-level tests
./build/test_runner bench            # micro-benchmarks + firmware benchmarks
./build/test_runner firmware         # firmware integration tests (cputest, timertest)
./build/test_runner multinode        # 2-node nullnet-broadcast simulation
```

### ARM Cortex-M3

```sh
./build/test_runner arm-correctness      # 33 instruction-level tests
./build/test_runner arm-firmware         # firmware boot test (hello-world)
./build/test_runner arm-multinode firmware/cc2538dk/nullnet-broadcast.cc2538dk -t 20000
```

### Mixed-platform (MSP430 + ARM)

```sh
# Nullnet broadcast: Sky + CC2538DK in the same network
./build/test_runner mixed-multinode firmware/sky/nullnet-broadcast.sky firmware/cc2538dk/nullnet-broadcast.cc2538dk -t 20000

# RPL-UDP: Sky server + CC2538DK client
./build/test_runner mixed-multinode firmware/sky/udp-server.sky firmware/cc2538dk/udp-client.cc2538dk -t 60000

# RPL-UDP: CC2538DK server + Sky client
./build/test_runner mixed-multinode firmware/cc2538dk/udp-server.cc2538dk firmware/sky/udp-client.sky -t 60000
```

Node type is auto-detected from firmware file extension (`.sky` → MSP430, `.cc2538dk` → ARM).

### Multi-node options

```sh
# MSP430: RPL-UDP server + client, 60 seconds simulated time
./build/test_runner multinode firmware/sky/udp-server.sky firmware/sky/udp-client.sky -t 60000

# ARM: RPL-UDP server + client, 60 seconds simulated time
./build/test_runner arm-multinode firmware/cc2538dk/udp-server.cc2538dk firmware/cc2538dk/udp-client.cc2538dk -t 60000

# Quiet mode (no per-node UART output)
./build/test_runner multinode -q -t 60000
```

## Performance

On Apple Silicon (M-series, PGO build):

### MSP430 (with JIT)

| Benchmark | Speed |
|-----------|-------|
| Micro-benchmarks (avg) | ~430 MIPS |
| Firmware blink.sky | ~195 MIPS |
| 2-node nullnet (60s sim) | ~500x real-time |
| 2-node RPL-UDP (60s sim) | ~1600x real-time |

### ARM Cortex-M3 (interpreter)

| Benchmark | Speed |
|-----------|-------|
| 2-node RPL-UDP (60s sim) | ~4x real-time |

## JIT configuration

Environment variables:

| Variable | Default | Description |
|----------|---------|-------------|
| `MSPSIM_JIT_THRESHOLD` | 100 | Executions before a block is compiled |
| `MSPSIM_JIT_INBLOCK_CHECKS` | 0 | Emit interrupt checks inside JIT blocks (tighter latency) |

## Project structure

```
src/
  msp430/             MSP430 CPU, peripherals, JIT compiler
  arm/                ARM Cortex-M3 CPU, CC2538 peripherals
include/
  msp430/             MSP430 header files
  arm/                ARM header files
test/                 Test runner, correctness tests, benchmarks, multi-node sim
firmware/sky/         Pre-compiled Contiki-NG firmware for Tmote Sky
firmware/cc2538dk/    Pre-compiled Contiki-NG firmware for CC2538DK
```

## License

TBD
