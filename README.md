# Cooja-NG — a C-based Cooja / MSPSim re-implementation

A fast, multi-architecture emulator and network simulator for Contiki-NG, written in portable C. Cooja-NG (codename `csim`) is a clean-room re-implementation of the parts of Cooja and MSPSim needed to run the upstream Contiki-NG test suite headlessly, with a strong focus on simulation speed, deterministic timing, and faithful peripheral behaviour.

**Status:** the full Contiki-NG Cooja test suite passes — **89 / 89** including the TUN/border-router cases.  Supported SoCs cover MSP430 (F149/F1611/F2617/F5437/CC430F5137/FR5969), ARM Cortex-M3 (TI CC2538, Zolertia Firefly with off-SoC CC1200 sub-GHz), Cortex-M4F (Nordic nRF52840), and Cortex-M33 (Nordic nRF54L15) — plus native Cooja motes (`dlopen`) for full mixed-platform networks.  See [Test results](#test-results) below.

```
                          ┌──────────────────────────────────────────────┐
                          │             Cooja-NG test_runner             │
                          │                                              │
firmware/*.sky / .z1 ────►│  MSP430 F149/F1611/F2617/F5437/CC430/FR5969  │
firmware/*.cc2538dk    ──►│  ARM Cortex-M3 + CC2538 RF Core (on-chip)    │──► UART, packets,
firmware/*.zoul-firefly►──│  ARM Cortex-M3 + CC2538 + CC1200 sub-GHz     │    timeline,
firmware/*.nrf52840-dk ──►│  ARM Cortex-M4F + Nordic 802.15.4 radio      │    web UI,
firmware/*.nrf54l15-dk ──►│  ARM Cortex-M33 + Nordic 802.15.4 + DPPI/GRTC│    COOJA.testlog
firmware/*.cooja       ──►│  Native Cooja motes (dlopen)                 │
                          │                                              │
                          │  shared event queue • per-radio medium       │
                          │  multi-channel • ns-precise time             │
                          │  JS-driven assertions                        │
                          └──────────────────────────────────────────────┘
```

---

## Table of Contents

1. [What is Cooja-NG?](#what-is-cooja-ng)
2. [Features](#features)
3. [Quick start](#quick-start)
4. [Building](#building)
5. [Running tests](#running-tests)
6. [Multi-node simulation](#multi-node-simulation)
7. [JSON simulation configs](#json-simulation-configs)
8. [The Cooja test suite](#the-cooja-test-suite)
9. [Web UI](#web-ui)
10. [Performance](#performance)
11. [Architecture](#architecture)
12. [Tuning and environment variables](#tuning-and-environment-variables)
13. [Project layout](#project-layout)
14. [Test results](#test-results)
15. [Known issues](#known-issues)
16. [License](#license)

---

## What is Cooja-NG?

Cooja-NG runs Contiki-NG firmware binaries inside an emulator process and lets you connect multiple emulated nodes through a shared 802.15.4 radio medium with per-radio multi-channel support (so 2.4 GHz CC2538 and sub-GHz CC1200 can coexist on the same network without cross-band interference). It is designed for three use cases:

1. **Headless CI / regression testing** — replace `Cooja --no-gui` for the upstream Contiki-NG test suite. Cooja-NG consumes the same `.csc` files (via `tools/csc2json.py`) and the same JS test scripts (via the embedded QuickJS engine).
2. **Network research** — run hundreds of emulated nodes on a single machine, mix MSP430 / ARM / native motes (and 2.4 GHz / sub-GHz radios) in the same network, script topology changes, and capture full packet timelines.
3. **Firmware debugging** — boot a single firmware image, watch UART, inspect register state on a wedge, and re-run with deterministic seeds.

Compared to upstream Cooja + MSPSim, Cooja-NG is roughly an order of magnitude faster, has no JVM dependency, and builds cleanly with `make` on Linux and macOS. It is *not* a full Cooja replacement: there is no GTK GUI, no Java plugin ecosystem, and no support for closed-source mote types.

---

## Features

### MSP430 emulator (`src/msp430/`)

- **Full MSP430 + MSP430X instruction set** — computed-goto interpreter, all double-op / single-op / jump instructions, extension words for `.A`/`.W`/`.B` modes, repeat counts (immediate and register), zero-carry, 20-bit PC and addressing.
- **Cycle-accurate timing** — operand-mode cycle tables matching MSPSim, 6-cycle interrupt service, deterministic event scheduling.
- **Optional JIT** — GNU Lightning compiles hot basic blocks to native code (~430 MIPS for ALU-bound micro-benchmarks on Apple Silicon). Auto-detected via `pkg-config`; without it the interpreter is used everywhere. Disabled for MSP430X by design (extension-word ambiguity).
- **CC2420 radio** — full state machine matching `CC2420.java`: VREG_OFF → POWER_DOWN → IDLE → calibrate → SFD search → frame reception, plus the TX chain. CCITT-16 with bit reversal, address filtering, auto-ACK, RXFIFO circular buffer, RX "incoming buffer" for bytes that arrive during calibration. ns-based byte timing (16 µs symbol / 32 µs byte / 192 µs cal / 1 ms VREG startup).
- **Peripherals** — Timer A/B (on-demand counter, CCR compare, capture mode, all clock sources), USART / USCI / eUSCI in UART and SPI mode, GPIO ports P1–P10 with edge-triggered interrupts, BCS (classic) and CS (FR5xxx) clock modules with separate MCLK/SMCLK/ACLK sources and dividers, hardware multiplier (16-bit MPY/MPYS/MAC/MACS, plus MPY32 32×32→64 on FR5xxx).
- **Platforms** — Tmote Sky (F1611), ETH ESB (F149), Zolertia Z1 (F2617), WisMote / EXP5438 (F5437), CC430F5137 eval board, MSP-EXP430FR5969 LaunchPad (FR5969 with FRAM and eUSCI).
- **Debug helpers** — ELF loader with symbol lookup, instruction-level tracing hooks, JIT cache inspection.

### ARM emulator (`src/arm/`)

- **Thumb / Thumb-2 interpreter** — full Cortex-M3 / M4F / M33 user instruction set including IT blocks, hi-reg ADD/MOV/CMP, ADR T2/T3, exclusive load/store stubs, bit-band region.  Optional FPv4-SP-D16 VFP for M4F (`src/arm/arm_vfp.c`).
- **NVIC** — interrupt priority, pending/enable, exception entry/exit, tail-chaining.  Per-instruction pending check so peripherals raising IRQs mid-instruction don't sit behind a never-cleared PRIMASK.
- **SysTick** — periodic tick generation through the shared event queue.
- **CC2538 SoC peripherals** — UART (TX callback + status flags), GPIO ports A–D with interrupts, General Purpose Timers (one-shot / periodic / prescaler), Sleep Timer (32 kHz, compare-match interrupt), System Control (clock config, OSC32K), IO Controller (pin mux), SSI (SPI master, both SSI0 and SSI1), and the on-chip RF Core (802.15.4 TX/RX, FFSM address filter, RFRND, frame interrupts).
- **Off-chip CC1200 sub-GHz radio** — TI SimpleLink CC1200 emulated as an event-driven SPI peripheral. Full software auto-ACK path, register-level fidelity, IOCFG-driven GPIO events, and 73-test mock-host suite for chip-driver compliance. Runs the upstream Contiki-NG `cc1200_802154g_863_870_fsk_50kbps` configuration.
- **Nordic nRF52840** — CLOCK/HFCLK/LFCLK, RTC0/1/2, TIMER0–4, GPIO, GPIOTE, PPI, RNG, NVMC, FICR (per-node DEVICEID), UARTE EasyDMA, and a full 802.15.4 RADIO model (PACKETPTR EasyDMA, SHORTS, INTENSET, BCMATCH, hardware-style auto-ACK).  Boards: PCA10059 USB Dongle (`nrf52840-dongle`), PCA10056 DK (`nrf52840-dk`).
- **Nordic nRF54L15** — newer Cortex-M33 family with GRTC (1 MHz syscounter, RELATIVE_COMPARE + RELATIVE_SYSCOUNTER modes), DPPI (32-channel publish/subscribe fabric), EGU (software-event source bridging to NVIC), TIMER10/20/21–24 (live-cycles-derived counter), per-node FICR.DEVICEID, UARTE20 EasyDMA, and an 802.15.4 RADIO with deferred PHYEND so the driver's NVIC-disabling critical section exits before the IRQ fires.  Board: PCA10156 DK (`nrf54l15-dk`).  2-node RPL-UDP exchanges request/response over RPL/6LoWPAN/CSMA end-to-end.
- **Shared helpers** — `include/common/ieee_802154.h` (PHY constants + CCITT-16 FCS used by all four 802.15.4 radios) and `arm/nrf_radio_common.h` (one shared `nrf_radio_emit_ieee802154_frame()` for both nRF radios).
- **Platforms** — TI SmartRF06 + CC2538EM (`cc2538dk`), Zolertia Firefly (`zoul-firefly`, CC2538 + CC1200), Nordic PCA10059/PCA10056 (`nrf52840-{dongle,dk}`), Nordic PCA10156 (`nrf54l15-dk`).

### Native Cooja motes (`src/native/`)

- **`.cooja` shared libraries** loaded with `dlopen`, exposing the same `simInSize`/`simOutSize`/`simRtimerNextExpirationTime` interface that real Cooja uses.
- **Cross-platform networking** — native, MSP430, and ARM nodes can be mixed in a single simulation network.
- **Dynamic transmission state** — explicit `radio_is_transmitting` / `radio_tx_finished` interval matching Cooja's `ContikiRadio.doActionsAfterTick()`.

### Multi-node simulation (`test/test_mixed_multinode.c`, `src/common/`)

- **Time-stepped event loop** — all nodes share a single ns-precise simulation clock; each step advances every node to the same target.
- **Per-node CPU frequencies** handled correctly across DCO calibration.
- **Per-radio radio medium** — each radio chip registers its own slot in a multi-channel medium. Frames are routed by `(channel, band)` so 2.4 GHz CC2538 and sub-GHz CC1200 networks coexist without cross-band leakage. UDGM (Unit Disk Graph with separate transmission and interference ranges, configurable TX/RX success ratios, deterministic seed) is the default propagation model. 235-test safety-net suite makes future refactors bisectable. See [`docs/radio-medium.md`](docs/radio-medium.md).
- **Per-byte RF delivery** matching Cooja's wire-level model — bytes are streamed into receivers as they arrive on the medium, not as full frames.
- **Packet analyzer** that decodes 802.15.4 / 6LoWPAN / IPv6 / RPL / UDP frames at runtime for human-readable logs.
- **Timeline recorder** — every radio TX/RX event is timestamped and can be exported to JSON for offline analysis.
- **Threading** — single-threaded by default (deterministic, fastest for ≤100 nodes), with an optional `--threads N` mode for very large topologies.

### Test scripting

- **JSON config files** describing nodes, topology, radio medium, timed actions (move, send, remove, add), assertion steps, fail-on patterns, and aggregate validators. See [`docs/test-format.md`](docs/test-format.md) for the full schema.
- **Embedded QuickJS engine** that runs Cooja-style JavaScript test scripts inline (`TIMEOUT`, `WAIT_UNTIL`, `log.testOK`, `log.testFailed`).
- **`csc2json.py`** auto-converts Cooja `.csc` files (and most of their JS scripts) to Cooja-NG JSON.
- **`run-cooja-tests.sh`** drives the entire upstream Contiki-NG test suite headlessly, with PASS / FAIL / SKIP reporting and auto-build of missing firmware.

### Web UI (optional)

- **Embedded WebSocket server** (`--ui [port]`) serves a single-page topology view at `http://localhost:8080/` showing live node positions, RPL parent links, packet animations, and per-node UART output.
- Pure C, no Node.js, no build dependencies.

---

## Quick start

```sh
# 1. Clone with the Contiki-NG sibling for firmware builds
git clone https://github.com/joakimeriksson/cooja-ng.git
git clone https://github.com/contiki-ng/contiki-ng.git
cd cooja-ng

# 2. Build (auto-detects GNU Lightning for JIT)
make

# 3. Run the unit tests (~5 seconds)
./build/test_runner correctness         # 68 MSP430 instruction tests
./build/test_runner arm-correctness     # 81 ARM Cortex-M3/M4F/M33 tests (Thumb-2 + DSP + VFP)
./build/test_runner cc1200-mock-host    # 73 CC1200 chip-driver tests
./build/test_runner radio-medium        # 235 radio-medium routing tests

# 4. Run a real RPL-UDP simulation: 60 simulated seconds in ~250 ms wall-clock
./build/test_runner mixed-multinode \
    firmware/sky/udp-server.sky firmware/sky/udp-client.sky -t 60000

# 5. Run the full Contiki-NG Cooja test suite (89 tests, ~30–45 min cold, ~10–15 min warm)
make configure CONTIKI_DIR=$(pwd)/../contiki-ng
make cooja-tests VERBOSE=1
```

---

## Building

| Target | Flags | Notes |
|---|---|---|
| `make` | `-O3 -flto -march=native` | Default release build, JIT auto-detected |
| `make debug` | `-O0 -g -DDEBUG` | Debuggable, no LTO |
| `make pgo` | Two-stage profile-guided optimization | Apple Clang only, ~40 % faster on hot loops |
| `make clean` | — | Remove `build/` |

**Optional dependencies**

| Dependency | Purpose | How it's detected |
|---|---|---|
| **GNU Lightning** | MSP430 JIT compiler | `pkg-config --libs lightning` (silent fallback to interpreter if missing) |
| **QuickJS** | JS test scripts | Bundled under `lib/quickjs/` — built automatically |
| **cJSON, cbor** | Config / serialization | Bundled under `lib/` |
| **Contiki-NG** | Test firmware sources | `csim.conf` (legacy filename), `CONTIKI_DIR` env var, or `../contiki-ng` |

There are no other runtime dependencies besides libc, libm, libpthread, and (on Linux for the TUN tests) `iproute2` + `tunslip6`.

**Setting CONTIKI_DIR**

```sh
make configure CONTIKI_DIR=/absolute/path/to/contiki-ng
# or
export CONTIKI_DIR=/absolute/path/to/contiki-ng
```

This is only needed if you want to (re)build firmware or run `make cooja-tests`. Pre-built firmware ships under `firmware/` for direct use.

---

## Running tests

### Unit and instruction-level tests

| Command | What it does |
|---|---|
| `./build/test_runner correctness [-v]` | 68 MSP430 instruction-level tests (MOV, ADD, jumps, MSP430X MOVA/ADDA/SUBA, cycle counts, byte mode, CALL/RET, CMP flags…) |
| `./build/test_runner arm-correctness [-v]` | 81 ARM tests — Cortex-M3 Thumb-2 + M4 DSP halfword multiply + M4 VFP (FPv4-SP-D16), ADR T2/T3 alignment, hi-reg ADD/MOV/CMP, ARMv8-M LDAEX/STLEX stubs |
| `./build/test_runner timeline [-v]` | 76 unit tests for the radio event timeline serializer |
| `./build/test_runner cc1200-mock-host` | 73 CC1200 chip-driver compliance tests (register-level, IOCFG, auto-ACK) |
| `./build/test_runner radio-medium` | 235 radio-medium tests (per-radio multi-channel routing, UDGM, band coexistence) |
| `./build/test_runner firmware [-v]` | Boots `cputest.sky` and `timertest.sky` to completion |
| `./build/test_runner arm-firmware [-v]` | Boots `hello-world.cc2538dk` (full Contiki-NG init + UART output) |
| `./build/test_runner bench` | Micro-benchmarks + firmware benchmarks, prints MIPS |
| `./build/test_runner all [-v]` | All of the above except multi-node |

### Multi-node simulations

```sh
# MSP430 RPL-UDP, 2 Sky nodes, 60 simulated seconds
./build/test_runner mixed-multinode \
    firmware/sky/udp-server.sky firmware/sky/udp-client.sky -t 60000

# ARM RPL-UDP, 2 CC2538DK nodes, 60 simulated seconds
./build/test_runner mixed-multinode \
    firmware/cc2538dk/udp-server.cc2538dk \
    firmware/cc2538dk/udp-client.cc2538dk -t 60000

# Mixed network: Sky server + CC2538 client
./build/test_runner mixed-multinode \
    firmware/sky/udp-server.sky \
    firmware/cc2538dk/udp-client.cc2538dk -t 60000

# Native Cooja mote (cooja-side compiled to a shared library)
./build/test_runner mixed-multinode \
    firmware/cooja/udp-server.cooja \
    firmware/cooja/udp-client.cooja -t 60000

# 100-node grid from a JSON config
./build/test_runner mixed-multinode configs/udgm-100node-grid-arm.json
```

Node platform is auto-detected from the firmware extension:

| Extension | Platform |
|---|---|
| `.sky` | MSP430 (Tmote Sky + CC2420) |
| `.esb` | MSP430 (ETH ESB) |
| `.z1` | MSP430 (Zolertia Z1 + CC2420) |
| `.wismote` / `.exp5438` | MSP430 (MSP-EXP430F5438 LaunchPad / WisMote, F5437) |
| `.cc430` | MSP430 (CC430F5137 eval board) |
| `.msp430fr5969` | MSP430 (MSP-EXP430FR5969 LaunchPad, FRAM, no radio) |
| `.cc2538dk` | ARM Cortex-M3 (TI CC2538 + on-chip 802.15.4) |
| `.zoul-firefly` | ARM Cortex-M3 (Zolertia Firefly: CC2538 + CC1200 sub-GHz) |
| `.nrf52840-dongle` | ARM Cortex-M4F (Nordic nRF52840 USB Dongle PCA10059) |
| `.nrf52840-dk` | ARM Cortex-M4F (Nordic nRF52840 Development Kit PCA10056) |
| `.nrf54l15-dk` | ARM Cortex-M33 (Nordic nRF54L15 Development Kit PCA10156) |
| `.cooja` | Native Cooja shared library |

### Multi-node options

| Option | Default | Description |
|---|---|---|
| `-t ms` | 20 000 | Simulated duration (overrides JSON `timeout_ms` if both given) |
| `-n nodes` | from firmware count | Override node count |
| `-v` | off | Verbose: print every UART line, RF event, packet decode |
| `-q` | off | Quiet: suppress per-node UART, only summary stats |
| `--threads N` | 0 (single-threaded) | Use N worker threads for very large simulations |
| `--ui [port]` | off | Start the WebSocket UI server (default port 8080) |

### The Cooja test suite

This is the most thorough validation Cooja-NG has against real Contiki-NG behaviour. It runs every test in `contiki-ng/tests/` headlessly using Cooja-NG instead of `Cooja --no-gui`.

```sh
# One-time setup
make configure CONTIKI_DIR=/path/to/contiki-ng

# Run the 81 non-TUN tests (no sudo needed) — ~10–15 min warm, longer if firmware must be rebuilt
make cooja-tests
make cooja-tests VERBOSE=1                  # show per-test output

# Run a single test or a subset
make cooja-tests PATTERN='07-simulation-base/26-tsch-drift-z1'
make cooja-tests PATTERN='14-rpl-lite*'
make cooja-tests PATTERN='19-cooja-rpl-tsch'

# Force a fresh build from current Contiki sources (wipes firmware/{cooja,sky,z1}/* first)
./tools/run-cooja-tests.sh --clean

# Include the 8 TUN/border-router tests
# Recommended: setcap once so tunslip6 doesn't need sudo per-run:
sudo setcap cap_net_admin+eip ../contiki-ng/tools/serial-io/tunslip6
./tools/run-cooja-tests.sh --with-tun -v 2>&1 | tee cooja-tests-tun.log

# Alternative: cache sudo + keep alive for the run
sudo -v
( while true; do sudo -nv 2>/dev/null; sleep 60; done ) &
./tools/run-cooja-tests.sh --with-tun -v 2>&1 | tee cooja-tests-tun.log
kill %1

# Rebuild the test firmware (only needed if Contiki-NG sources changed)
make build-firmware
make build-firmware PATTERN='07-*'
```

`run-cooja-tests.sh` automatically:

1. Globs `tests/*` in your Contiki-NG checkout.
2. Converts each `.csc` to JSON via `tools/csc2json.py`.
3. Builds any missing firmware (unless `--no-build` is passed).
4. Runs `build/test_runner mixed-multinode <generated.json>` with the JS test script attached.
5. Aggregates PASS / FAIL / SKIP / ERROR counts and exits non-zero if anything fails.

---

## JSON simulation configs

A complete schema is documented in [`docs/test-format.md`](docs/test-format.md). The short version:

```jsonc
{
    "title": "RPL-UDP 3-node linear chain",
    "timeout_ms": 60000,
    "seed": 42,
    "startup_delay_ms": 1000,
    "radiomedium": {
        "type": "udgm",
        "tx_range": 50.0,
        "interference_range": 100.0,
        "success_ratio_tx": 1.0,
        "success_ratio_rx": 1.0
    },
    "nodes": [
        { "firmware": "firmware/sky/udp-server.sky", "id": 1, "x":  0.0, "y": 0.0 },
        { "firmware": "firmware/sky/udp-client.sky", "id": 2, "x": 30.0, "y": 0.0 },
        { "firmware": "firmware/sky/udp-client.sky", "id": 3, "x": 60.0, "y": 0.0 }
    ],
    "test": {
        "timeout_is_success": true,
        "fail_on": ["packet loss", "parent switch: -> (NULL"],
        "validators": [
            { "pattern": "Received response", "min_count": 6 }
        ],
        "actions": [
            { "at_ms": 30000, "type": "move", "node": 3, "x": 200.0, "y": 0.0 },
            { "at_ms": 45000, "type": "move", "node": 3, "x":  60.0, "y": 0.0 }
        ]
    }
}
```

The bundled `configs/` directory has working examples for many scenarios:

| File | What it demonstrates |
|---|---|
| `rpl-udp-sky.json` | Minimal MSP430 server + client |
| `rpl-udp-cc2538dk.json` | ARM CC2538 RPL-UDP |
| `rpl-udp-native.json` | Native Cooja motes |
| `mixed-sky-native.json` | Mixed MSP430 + native in one network |
| `udgm-3node.json` | UDGM topology with explicit positions |
| `udgm-in-range.json` / `out-of-range.json` | Connectivity boundary tests |
| `udgm-100node-grid.json` | 100-node grid (sky / arm / mixed variants) |
| `test-4node-chain.json` | 4-node multi-hop chain with delivery assertions |
| `test-js-rpl-udp.json` | Inline QuickJS test script |
| `ui-rpl-udp-grid.json` | 16-node 4×4 grid for the web UI |

Run any of them with:

```sh
./build/test_runner mixed-multinode configs/udgm-100node-grid-arm.json -v
```

---

## Web UI

```sh
./build/test_runner mixed-multinode configs/ui-rpl-udp-grid.json --ui
# then open http://localhost:8080/ in a browser
```

What you get:

- Live 2D node positions (auto-laid out from `x`/`y`).
- Animated TX/RX packet flashes when frames are exchanged.
- Per-node UART output streamed to the browser.
- RPL parent links rendered as edges (when the packet analyzer detects DIO/DAO).
- Pause / step / speed controls.

The UI is a single embedded HTML page (`ui/index.html`) served by a tiny non-blocking WebSocket server (`src/ui/ws_server.c`). No Node.js, no build step. Up to 8 concurrent browser clients.

---

## Performance

Measured on Apple Silicon (M-series, PGO build):

### MSP430 (with JIT)

| Benchmark | Speed |
|---|---|
| Micro-benchmarks (avg of 7) | ~430 MIPS |
| Firmware `blink.sky` | ~195 MIPS |
| Firmware `energest-demo.sky` | ~196 MIPS |
| 2-node nullnet (60 s sim) | ~500× real-time |
| 2-node RPL-UDP (60 s sim) | ~1600× real-time |

On Linux x86-64 (release build, no PGO) you can reproducibly expect ~250× real-time on the 2-node RPL-UDP test — your mileage will depend heavily on the host CPU and whether GNU Lightning is available.

### ARM (interpreter only — no JIT yet)

| Platform | Benchmark | Speed |
|---|---|---|
| CC2538DK | 2-node RPL-UDP (60 s sim) | ~4× real-time |
| nRF52840 (PCA10059/PCA10056) | 2-node RPL-UDP (60 s sim) | ~9× real-time |
| nRF54L15 (PCA10156) | 2-node RPL-UDP (60 s sim, deferred PHYEND) | ~0.1× real-time |

The nrf54l15 number is slow because the GRTC is modeled at full 1 MHz resolution and every TX defers PHYEND through the event queue — both are correctness-over-speed choices.  Speed is the next optimization target once the RPL-UDP regression has stabilised.

### Reproducing the numbers

```sh
make pgo                                    # or `make` for non-PGO
./build/test_runner bench                   # micro + firmware MIPS
./build/test_runner mixed-multinode \
    firmware/sky/udp-server.sky firmware/sky/udp-client.sky -t 60000 -q
./build/test_runner mixed-multinode \
    firmware/cc2538dk/udp-server.cc2538dk \
    firmware/cc2538dk/udp-client.cc2538dk -t 60000 -q
```

The tail of every multinode run prints a phase-timing breakdown (CPU step / radio deliver / output flush / channel sync / overhead) and a `Speed ratio` line.

---

## Architecture

### Dual time domains

Every CPU tracks two clocks in parallel:

- **`cycles`** — monotonically increasing CPU-cycle count, the unit the interpreter advances each instruction.
- **`sim_time_ns`** — monotonic ns-precise wall clock, the unit the radio, peripherals, and inter-node coordination use.

These two are reconciled at three sync points: before each event callback fires, at the end of `*_step_until()`, and inside `*_set_frequency()` whenever DCO calibration changes the CPU frequency. This dual model is what lets the simulator stay correct when one node speeds up after DCO calibration while another is still running at boot frequency, or when an MSP430 and a CC2538 (different MHz, different clocks) share the same radio medium.

### Hot path

```
fetch -> dispatch via computed goto (or JIT block)
      -> execute, update regs/flags/cycles
      -> if (cycles >= next_event_cycle) execute_events()
      -> if (interrupts pending && GIE) service_interrupt()
```

There is **no** ns-conversion or floating-point math on the hot path. Events are scheduled in cycle units; ns-based events are converted to cycles once at scheduling time and re-converted only on frequency change.

### Shared event queue

`include/common/event_queue.h` provides a header-only intrusive linked-list event queue, instantiated once per architecture (`EVENT_QUEUE_IMPL(msp430, ...)` / `EVENT_QUEUE_IMPL(arm, ...)`). The same data structure is used for timer compares, radio state transitions, GPIO debounce, and the multi-node sim's per-node wakeups.

### Multi-node loop

```c
while (sim_ns < end_ns) {
    sim_ns += time_step_ns;            // 1 ms by default
    for (each node) {
        delta_ns     = sim_ns - cpu->sim_time_ns;
        target_cycle = cpu->cycles + ns_to_cycles(delta_ns, cpu->cpu_freq_hz);
        cpu_step_until(cpu, target_cycle);
    }
    deliver_rf_bytes();                // per-byte radio medium delivery
    flush_uart();
    process_actions();                 // timed move/send/remove/add
    check_test_engine();               // step matching, validators, fail-on
}
```

Per-byte RF delivery is the key fidelity choice: instead of dropping a whole frame at the receiver in one go (which is wrong for byte-level firmware that polls SPI / FIFO between bytes), Cooja-NG hands receivers one symbol at a time at the exact ns when it would arrive on a real link. This is what makes TSCH and tight CSMA timing actually work.

### Per-radio multi-channel medium

Every radio chip (CC2420, CC2538 RF Core, CC1200) registers its own slot in the radio medium and pushes channel changes synchronously. The medium routes frames by `(band, channel)` so that:

- 2.4 GHz CC2538 and sub-GHz CC1200 networks coexist on the same simulation without cross-band leakage.
- TSCH channel hops correctly affect what each receiver hears.
- New radio drivers register a slot rather than touching the medium itself.

A 235-test safety-net suite (`./build/test_runner radio-medium`) makes future refactors bisectable. See [`docs/radio-medium.md`](docs/radio-medium.md) for the routing model and [`docs/porting-a-device.md`](docs/porting-a-device.md) §8 for the chip-driver event model.

### Cooja test wrapper

`tools/run-cooja-tests.sh` is the bridge between Cooja-NG and the upstream Contiki-NG test infrastructure. It treats Cooja-NG as a drop-in replacement for `java -jar Cooja.jar --no-gui`, runs the same `.csc` topologies, parses the same Cooja JS scripts via `csc2json.py`, and reports compatible PASS/FAIL output. This is how Cooja-NG claims compatibility — every fix is validated against the upstream test suite, not a private regression set.

---

## Tuning and environment variables

| Variable | Default | Effect |
|---|---|---|
| `MSPSIM_JIT_THRESHOLD` | `100` | Number of times a basic block must execute before the JIT compiles it |
| `MSPSIM_JIT_INBLOCK_CHECKS` | `1` | Emit interrupt / event-fire checks inside JIT blocks (needed for tight timer loops; set to `0` for max throughput on pure compute loops) |
| `CSIM_TRACE_TSCH_ACK` | unset | Verbose CC2420 ACK timing trace for TSCH debugging |
| `CSIM_TRACE_EVENT_SPIN` | unset | Trace multi-node event spin-loop iterations (useful when investigating wakeup hangs) |
| `CONTIKI_DIR` | `../contiki-ng` | Where `make cooja-tests` and `tools/build-test-firmware.sh` look for the Contiki-NG checkout |

---

## Project layout

```
cooja-ng/
├── src/
│   ├── msp430/                MSP430 CPU, peripherals, JIT, CC2420
│   ├── arm/                   ARM CPU (M3/M4F/M33), NVIC, SoC peripherals:
│   │                            cc2538_soc + cc2538_{uart,gpio,gptimer,...}
│   │                            cc1200 (off-SoC sub-GHz)
│   │                            nrf52840_soc (CLOCK/RTC/TIMER/RADIO/…)
│   │                            nrf54l15_soc (GRTC/DPPI/EGU/TIMER/RADIO)
│   │                            nrf_radio_common (shared emit_frame helper)
│   ├── native/                Native Cooja mote loader
│   ├── common/                Shared ELF loader, radio medium, event queue,
│   │                          timeline, packet analyzer, JS test engine,
│   │                          ieee_802154.h (PHY constants + CCITT-16 FCS)
│   └── ui/                    WebSocket server + sim state serializer
├── include/
│   ├── msp430/ arm/ native/ common/ ui/   Public headers per subsystem
├── lib/
│   ├── cJSON.{c,h}            JSON parser
│   ├── cbor.{c,h}             CBOR encoder
│   └── quickjs/               Embedded JavaScript engine
├── test/
│   ├── test_main.c            Test runner entry point
│   ├── test_correctness.c     MSP430 instruction tests
│   ├── test_arm_correctness.c ARM Cortex-M3/M4F/M33 instruction tests
│   ├── test_firmware.c        MSP430 firmware integration
│   ├── test_arm_firmware.c    ARM firmware integration
│   ├── test_benchmark.c       Performance benchmarks
│   ├── test_mixed_multinode.c Multi-node sim driver (MSP430 + ARM + native)
│   ├── test_cc1200.c          CC1200 chip-driver mock-host suite (73 tests)
│   ├── test_radio_medium.c    Radio-medium routing/multi-channel suite (235 tests)
│   ├── test_mock_host.c       sim_host_t vtable tests for chip drivers
│   └── test_timeline.c        Timeline serializer unit tests
├── configs/                   Example JSON simulation configs
├── docs/                      test-format.md, radio-medium.md, architecture.md,
│                              porting-a-device.md
├── firmware/                  Pre-built Contiki-NG firmware
│   ├── sky/  z1/              MSP430 (Tmote Sky F1611, Zolertia Z1 F2617)
│   ├── cc2538dk/              CC2538DK (ARM Cortex-M3)
│   ├── zoul-firefly/          Zolertia Firefly (CC2538 + CC1200 sub-GHz)
│   ├── nrf52840-dongle/       Nordic PCA10059 USB Dongle (Cortex-M4F)
│   ├── nrf52840-dk/           Nordic PCA10056 DK (Cortex-M4F)
│   ├── nrf54l15-dk/           Nordic PCA10156 DK (Cortex-M33)
│   └── cooja/                 Native Cooja mote .so libraries
├── devices/                   Per-device port docs (SPEC, STATUS, hw test plan)
│   └── zoul-firefly/          Zolertia Firefly port narrative + L6 resolution
├── tools/
│   ├── csc2json.py            Cooja .csc → Cooja-NG JSON converter
│   ├── run-cooja-tests.sh     Run upstream Contiki-NG test suite
│   ├── build-test-firmware.sh Auto-build firmware needed by tests
│   ├── build-device-firmware.sh  Docker-or-host firmware builder for new ports
│   └── ...                    Debug/timing utilities
├── ui/index.html              Embedded UI page
├── csim.conf                  CONTIKI_DIR config (created by `make configure`)
├── Makefile
├── CLAUDE.md                  Detailed architecture notes
├── PLAN.md                    Active development log / roadmap
└── STATUS.txt                 Latest verified test counts
```

For deeper technical notes on each subsystem (CPU dispatch loop, JIT code generation, CC2420 state machine, ns-time event scheduling) see [`CLAUDE.md`](CLAUDE.md).

---

## Test results

Last verified run on Linux x86-64 with `make` (release, JIT auto-detected):

### Unit and integration tests

| Suite | Result |
|---|---|
| `correctness` (MSP430 instructions) | **68 / 68 PASS** |
| `arm-correctness` (ARM Cortex-M3/M4F/M33 instructions, incl. DSP + VFP) | **81 / 81 PASS** |
| `timeline` (event serializer) | **76 / 76 PASS** |
| `cc1200-mock-host` (CC1200 chip driver) | **73 / 73 PASS** |
| `radio-medium` (per-radio multi-channel routing) | **235 / 235 PASS** |
| `firmware` (`cputest.sky`, `timertest.sky`) | **2 / 2 PASS** |
| `arm-firmware` (`hello-world.cc2538dk` + nRF bring-up) | **PASS** (boots Contiki-NG cleanly) |
| `zoul-firefly-multinode` RPL-UDP | **6 / 6 hello cycles** in 60 s, ~9× real-time |
| `nrf52840-dongle-multinode` RPL-UDP | UDP request/response round-trip @ ~9× real-time |
| `nrf54l15-dk-multinode` RPL-UDP | UDP request/response round-trip end-to-end (slow, ~0.1× real-time) |
| `bench` | runs cleanly, ~430 MIPS micro avg |

### Cooja test suite

```
=== Cooja Test Suite (csim) ===
Total:  89
Passed: 89
Failed:  0
Skipped: 0
Errors:  0
```

That includes:

- All 27 `07-simulation-base/*` cases (RPL-Lite, TSCH, Orchestra variants, multicast, IPv6, stack guard, data structures), including the previously stubborn `26-tsch-drift-z1` (16 s).
- All 12 `09-ipv6/*` ping permutations (CSMA / TSCH × LLA / ULA × with / without RPL).
- All 9 `13-ieee802154/*` 6top tests.
- All 14 `14-rpl-lite/*` and 19 `15-rpl-classic/*` cases, including the long-running 28-hour simulated DAG stability tests.
- All 8 `17-tun-rpl-br/*` border-router tests with real `tun0` + `tunslip6`, including `01-border-router-cooja` (94 s) and `10-native-nat64-cooja` (214 s, UDP+TCP echo through the NAT64 gateway).

The exact log is in `cooja-tests-tun.log` after running:

```sh
sudo setcap cap_net_admin+eip ../contiki-ng/tools/serial-io/tunslip6     # one-time
./tools/run-cooja-tests.sh --with-tun -v 2>&1 | tee cooja-tests-tun.log
```

---

## Known issues

These are real, currently reproducible quirks in the *standalone CLI shortcuts* — they do not affect the Cooja test wrapper path, the JSON-config flow, or any production use.

1. **`./build/test_runner multinode` (no firmware) hangs.** The default-firmware shortcut routes to `firmware/sky/nullnet-broadcast.sky` and gets stuck after init. Workaround: use a JSON config or explicit firmware pair, e.g. `./build/test_runner mixed-multinode firmware/sky/udp-server.sky firmware/sky/udp-client.sky -t 60000`.

2. **nRF54L15 RPL-UDP convergence is slow (~30 s sim).** End-to-end works (DIO → DAO → DAO-ACK → UDP request/response), but RPL takes longer to settle than on nrf52840 because of the deferred-PHYEND model and the 1 MHz GRTC fidelity.  CSMA retransmits visible in the packet log; cosmetic.

3. **`test_firmware.c` reports `timertest.sky` as PASS** even when the firmware itself prints `FW: FAIL: count > 10 failed at timertest.c:166`. The runner only matches `EXIT`. Cosmetic, but worth tightening.

4. **The same defer-PHYEND fix that unblocked nRF54L15 has not been applied to nrf52840** — that platform works today, but the same critical-section-during-TX scenario would surface if a faster RPL config exercises it.

The active development state and any new regressions are tracked in [`PLAN.md`](PLAN.md). The most recently verified totals live in [`STATUS.txt`](STATUS.txt).

---

## License

3-clause BSD. See [LICENSE](LICENSE).

Copyright © 2026 Joakim Eriksson, RISE Research Institutes of Sweden.

The bundled `lib/quickjs/` is © Fabrice Bellard and Charlie Gordon, MIT license.
