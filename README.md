# Cooja-NG

A fast MSP430 emulator and network simulator written in C, designed for large-scale IoT simulation with Contiki-NG firmware.

## Features

- **MSP430/MSP430X CPU emulator** with threaded interpreter (computed goto dispatch)
- **JIT compiler** using GNU Lightning for hot basic blocks (~430 MIPS on Apple Silicon)
- **CC2420 802.15.4 radio** with full state machine, SPI, TX/RX FIFO, CRC, and auto-ACK
- **Multi-node simulation** with time-stepped RF byte delivery
- **Peripherals**: Timer A/B, USART, GPIO, clock system (DCO/MCLK/SMCLK/ACLK)
- **Platform support**: Tmote Sky (MSP430F1611 + CC2420)
- **ELF loader** for Contiki-NG firmware binaries

## Building

```sh
make        # standard build
make pgo    # profile-guided optimization (~40% faster)
```

GNU Lightning is optional (auto-detected via pkg-config). Without it, the interpreter is used for all execution.

## Running tests

```sh
./build/test_runner correctness      # 68 instruction-level tests
./build/test_runner bench            # micro-benchmarks + firmware benchmarks
./build/test_runner firmware         # firmware integration tests (cputest, timertest)
./build/test_runner multinode        # 2-node nullnet-broadcast simulation
```

Multi-node options:

```sh
# RPL-UDP server + client, 60 seconds simulated time
./build/test_runner multinode firmware/sky/udp-server.sky firmware/sky/udp-client.sky -t 60000

# Quiet mode (no per-node UART output)
./build/test_runner multinode -q -t 60000
```

## Performance

On Apple Silicon (M-series, PGO build):

| Benchmark | MIPS |
|-----------|------|
| Micro-benchmarks (avg) | ~430 |
| Firmware blink.sky | ~195 |
| Firmware energest-demo.sky | ~196 |
| 2-node nullnet (60s sim) | 500x real-time |
| 2-node RPL-UDP (60s sim) | 1600x real-time |

## JIT configuration

Environment variables:

| Variable | Default | Description |
|----------|---------|-------------|
| `MSPSIM_JIT_THRESHOLD` | 100 | Executions before a block is compiled |
| `MSPSIM_JIT_INBLOCK_CHECKS` | 0 | Emit interrupt checks inside JIT blocks (tighter latency) |

## Project structure

```
src/            MSP430 CPU, peripherals, JIT compiler
include/        Header files
test/           Test runner, correctness tests, benchmarks, multi-node sim
firmware/sky/   Pre-compiled Contiki-NG firmware for Tmote Sky
```

## License

TBD
