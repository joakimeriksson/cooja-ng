# Cooja-NG — a C-based Cooja / MSPSim re-implementation

A fast, multi-architecture emulator and network simulator for Contiki-NG, written in portable C.  Cooja-NG (codename `csim`) is a clean-room re-implementation of the parts of Cooja and MSPSim needed to run the upstream Contiki-NG test suite headlessly, with a strong focus on simulation speed, deterministic timing, and faithful peripheral behaviour.

**Status:** the full Contiki-NG Cooja test suite passes — **89 / 89** including the TUN/border-router cases.  See [Test results](#test-results).

Architecture and refactor direction are tracked in
[`docs/design/refactor-plan.md`](docs/design/refactor-plan.md), including the
internal simulation kernel boundary, static plugin registry model, and staged
runner extraction plan.

```
                          ┌──────────────────────────────────────────────┐
                          │             Cooja-NG test_runner             │
                          │                                              │
firmware/*.sky / .z1 ────►│  MSP430 F149/F1611/F2617/F5437/CC430/FR5969  │
firmware/*.cc2538dk    ──►│  ARM Cortex-M3 + CC2538 RF Core (on-chip)    │──► UART, packets,
firmware/*.zoul-firefly►──│  ARM Cortex-M3 + CC2538 + CC1200 sub-GHz     │    timeline,
firmware/*.nrf52840-dk ──►│  ARM Cortex-M4F + Nordic 802.15.4 radio      │    web UI,
firmware/*.nrf54l15-dk ──►│  ARM Cortex-M33 + Nordic 802.15.4 + DPPI/GRTC│    COOJA.testlog
   └─ + RV32E FLPR  ──────►│  RISC-V coprocessor, dual-core, shared SRAM  │
firmware/*.cooja       ──►│  Native Cooja motes (dlopen)                 │
                          │                                              │
                          │  shared event queue • per-radio medium       │
                          │  multi-channel • ns-precise time             │
                          │  JS-driven assertions                        │
                          └──────────────────────────────────────────────┘
```

**Multi-ISA:** MSP430, ARM Cortex-M3/M4F/M33, and **RISC-V** — the latter via the nRF54L15's FLPR (an RV32E coprocessor) running unmodified Contiki-NG, dual-core alongside the M33 over shared SRAM. (RV32E is `rv32e_zicsr_zifencei` — base integer + CSR + fence.i, in the coprocessor role; not yet a standalone RISC-V networking node.)

Designed for: headless CI for the Contiki-NG test suite, network research with hundreds of mixed-architecture nodes, and single-firmware debugging with deterministic seeds.  Roughly an order of magnitude faster than Cooja + MSPSim, no JVM dependency, builds with `make` on Linux and macOS.  Not a full Cooja replacement — no GTK GUI, no Java plugin ecosystem, no closed-source motes.

## Quick start

```sh
# Clone with the Contiki-NG sibling for firmware builds
git clone https://github.com/joakimeriksson/cooja-ng.git
git clone https://github.com/contiki-ng/contiki-ng.git
cd cooja-ng

# Build (auto-detects GNU Lightning for JIT)
make

# Unit tests (~5 seconds)
./build/test_runner correctness         # 68 MSP430 instruction tests
./build/test_runner arm-correctness     # 81 ARM Cortex-M3/M4F/M33 tests
./build/test_runner cc1200-mock-host    # 73 CC1200 chip-driver tests
./build/test_runner radio-medium        # 235 radio-medium routing tests

# 2-node RPL-UDP simulation — 60 simulated seconds in ~250 ms wall-clock
./build/test_runner mixed-multinode \
    firmware/sky/udp-server.sky firmware/sky/udp-client.sky -t 60000

# The full Contiki-NG Cooja test suite (89 tests, ~10–15 min warm)
make configure CONTIKI_DIR=$(pwd)/../contiki-ng
make cooja-tests VERBOSE=1
```

## Building

| Target | Notes |
|---|---|
| `make` | Default release build (`-O3 -flto`, native CPU tuning: `-march=native` on x86 / `-mcpu=native` on arm64), JIT auto-detected via pkg-config |
| `make debug` | `-O0 -g -DDEBUG`, no LTO |
| `make pgo` | Two-stage profile-guided optimization, ~40 % faster on hot loops |
| `make clean` | Remove `build/` |

Optional dependencies: **GNU Lightning** (MSP430 JIT, silent fallback if missing), **QuickJS / cJSON / cbor** (bundled under `lib/`), **Contiki-NG** (`csim.conf`, `CONTIKI_DIR` env, or `../contiki-ng`).  Runtime needs only libc/libm/libpthread, plus `iproute2` + `tunslip6` on Linux for the TUN tests.

`make configure CONTIKI_DIR=/abs/path` sets the persistent path used by `make cooja-tests` / `tools/build-test-firmware.sh`.  Pre-built firmware ships under `firmware/` for direct use without Contiki.

## Supported platforms

| Extension | Platform |
|---|---|
| `.sky` | MSP430F1611 (Tmote Sky) + CC2420 |
| `.esb` | MSP430F149 (ETH ESB) |
| `.z1` | MSP430F2617 (Zolertia Z1) + CC2420 |
| `.wismote` / `.exp5438` | MSP430F5437 |
| `.cc430` | CC430F5137 eval board |
| `.msp430fr5969` | MSP-EXP430FR5969 LaunchPad (FRAM, no radio) |
| `.cc2538dk` | ARM Cortex-M3 — TI CC2538 + on-chip 802.15.4 |
| `.zoul-firefly` | ARM Cortex-M3 — Zolertia Firefly (CC2538 + CC1200 sub-GHz) |
| `.nrf52840-dongle` | ARM Cortex-M4F — Nordic PCA10059 USB Dongle |
| `.nrf52840-dk` | ARM Cortex-M4F — Nordic PCA10056 DK |
| `.nrf54l15-dk` | ARM Cortex-M33 — Nordic PCA10156 DK |
| `.cooja` | Native Cooja shared library |

Per-board details live in [`devices/`](devices/) and the SoC source files under `src/{msp430,arm}/`.

## Features

**MSP430 emulator** — full MSP430 + MSP430X instruction set (computed-goto interpreter, cycle-accurate operand-mode tables, MSPSim-compatible 6-cycle interrupt service); optional GNU Lightning JIT (~430 MIPS, ALU only); full CC2420 model (ns-based byte timing, CCITT-16 + bit reversal, auto-ACK, RXFIFO); Timer A/B + USART/USCI/eUSCI + GPIO + BCS/CS clock modules + 16-bit / 32-bit hardware multiplier.

**ARM emulator** — Thumb/Thumb-2 for Cortex-M3/M4F/M33 with IT blocks, ADR T2/T3 alignment, hi-reg ADD/MOV/CMP, ARMv8-M LDAEX/STLEX stubs; optional FPv4-SP-D16 VFP for M4F; NVIC with per-instruction pending check (peripherals raising IRQs mid-instruction don't sit behind a never-cleared PRIMASK); SysTick.  SoC peripherals:

- **TI CC2538** — UART, GPIO, GPTimer, Sleep Timer, IOC, SSI, on-chip RF Core (802.15.4 with FFSM filter and frame IRQs).
- **TI CC1200** (off-SoC) — event-driven SPI peripheral, register-level fidelity, IOCFG-driven GPIO events, full software auto-ACK, 73-test mock-host suite.
- **Nordic nRF52840** — CLOCK/HFCLK/LFCLK (+ STAT/SRC status regs), RTC0/RTC1, TIMER0–4, GPIO/GPIOTE, PPI, RNG, NVMC, FICR (per-node DEVICEID), UART (legacy + UARTE EasyDMA), and a full 802.15.4 RADIO (PACKETPTR EasyDMA, SHORTS, INTENSET, BCMATCH, hardware-style auto-ACK).  **Also boots stock Zephyr OS and RIOT OS** (experimental / best-effort; Contiki-NG is the primary, fully-validated target) — an unmodified `nrf52840dk` `hello_world` prints over the default UARTE console, stock Zephyr `echo_server`/`echo_client` exchange UDP over 802.15.4, and two RIOT `gnrc_networking` nodes form an RPL DODAG; see [`docs/zephyr.md`](docs/zephyr.md) and [`docs/riot.md`](docs/riot.md).
- **Nordic nRF54L15** — Cortex-M33 family with GRTC (1 MHz syscounter, RELATIVE_COMPARE + RELATIVE_SYSCOUNTER), DPPI (32-channel publish/subscribe), EGU (software-event bridge to NVIC), TIMER10/20–24, per-node FICR.DEVICEID, UARTE20 EasyDMA, GPIO P0/P1/P2, and an 802.15.4 RADIO with deferred PHYEND so the driver's NVIC-disabling critical section exits before the IRQ fires.  2-node RPL-UDP exchanges request/response end-to-end.
- **Nordic nRF54L15 FLPR (RISC-V / RV32E)** — the on-die **VPR coprocessor**, modelled as a second core (`src/riscv/`) that shares the M33's bus. The M33 (`flpr-host`) loads the FLPR blob into shared SRAM and releases it via the VPR `CPURUN` register (SPU SECATTR-gated, as on hardware); the RV32E core then runs **unmodified Contiki-NG** (`hello-vpr`) dual-core with the M33. Verified end-to-end: the M33 prints `[FLPR] tick N` (advancing ~2/sec) and LED0 (P2.9, RISC-V) / LED1 (P1.10, M33) blink at 1 Hz / 2 Hz — visible live in the web UI. ISA `rv32e_zicsr_zifencei` (base integer + CSR + `fence.i`; no M/C). Plan + register map: [`docs/design/riscv-vpr-plan.md`](docs/design/riscv-vpr-plan.md).

Shared 802.15.4 helpers live in [`include/common/ieee_802154.h`](include/common/ieee_802154.h) (PHY constants + CCITT-16 FCS used by all four radios) and [`include/arm/nrf_radio_common.h`](include/arm/nrf_radio_common.h) (one `nrf_radio_emit_ieee802154_frame()` for both nRF radios).

**Native Cooja motes** — `.cooja` shared libraries via `dlopen`, exposing the `simInSize` / `simOutSize` / `simRtimerNextExpirationTime` interface; mixable in a single network with MSP430 and ARM motes; dynamic `radio_is_transmitting` interval matching Cooja's `ContikiRadio.doActionsAfterTick()`.

**Multi-node** — time-stepped event loop, all nodes on one ns-precise clock; per-node CPU frequencies handled across DCO calibration; per-radio multi-channel medium routes frames by `(band, channel)` so 2.4 GHz and sub-GHz networks coexist; per-byte RF delivery (one symbol at a time, not whole frames) which is what makes TSCH and tight CSMA timing work; packet analyzer for 802.15.4 / 6LoWPAN / IPv6 / RPL / UDP; timeline recorder exportable to JSON; single-threaded default with optional `--threads N`.

**Test scripting** — JSON config files (see [`docs/test-format.md`](docs/test-format.md)), embedded QuickJS engine for Cooja-style JS scripts (`TIMEOUT`, `WAIT_UNTIL`, `log.testOK`, `log.testFailed`), `tools/csc2json.py` to convert `.csc` files, `tools/run-cooja-tests.sh` to drive the whole upstream test suite.

**Web UI** — `--ui [port]` starts a WebSocket server, single embedded HTML page at `http://localhost:8080/`: live node positions, packet animations, RPL parent edges, per-node UART, pause/step/speed.  No Node.js, no build step.

## Running tests

### Unit and integration

```sh
./build/test_runner correctness         # 68 MSP430 instruction tests
./build/test_runner arm-correctness     # 81 ARM (Thumb-2 + M4 DSP + M4 VFP)
./build/test_runner timeline            # 76 radio-event serializer tests
./build/test_runner cc1200-mock-host    # 73 CC1200 chip-driver tests
./build/test_runner radio-medium        # 235 multi-channel routing tests
./build/test_runner firmware            # boots cputest.sky, timertest.sky
./build/test_runner arm-firmware        # boots hello-world.cc2538dk + nRF
./build/test_runner bench               # MIPS micro + firmware benchmarks
./build/test_runner all                 # everything except multi-node
```

### Multi-node simulations

```sh
# MSP430 RPL-UDP, 2 Sky nodes, 60 simulated seconds
./build/test_runner mixed-multinode \
    firmware/sky/udp-server.sky firmware/sky/udp-client.sky -t 60000

# ARM CC2538DK RPL-UDP
./build/test_runner mixed-multinode \
    firmware/cc2538dk/udp-server.cc2538dk \
    firmware/cc2538dk/udp-client.cc2538dk -t 60000

# Mixed network — Sky server + CC2538 client
./build/test_runner mixed-multinode \
    firmware/sky/udp-server.sky \
    firmware/cc2538dk/udp-client.cc2538dk -t 60000

# 100-node grid from a JSON config
./build/test_runner mixed-multinode configs/udgm-100node-grid-arm.json
```

Platform is auto-detected from the firmware extension (see table above).  Options: `-t ms` (sim duration), `-n nodes`, `-v` (verbose), `-q` (quiet), `--threads N`, `--ui [port]`.

### The Cooja test suite

The most thorough validation Cooja-NG has — runs every test in `contiki-ng/tests/` headlessly using Cooja-NG instead of `Cooja --no-gui`.

```sh
make configure CONTIKI_DIR=/path/to/contiki-ng     # one-time

# All 81 non-TUN tests (no sudo) — ~10–15 min warm
make cooja-tests
make cooja-tests VERBOSE=1                         # per-test output
make cooja-tests PATTERN='14-rpl-lite*'            # subset

# Force a rebuild against current Contiki sources
./tools/run-cooja-tests.sh --clean

# All 89 tests including TUN border-router cases.
# One-time: setcap so tunslip6 doesn't need sudo per run.
sudo setcap cap_net_admin+eip ../contiki-ng/tools/serial-io/tunslip6
./tools/run-cooja-tests.sh --with-tun -v 2>&1 | tee cooja-tests-tun.log

# Rebuild test firmware (only if Contiki sources changed)
make build-firmware
```

`run-cooja-tests.sh` globs `tests/*/*.csc` in your Contiki checkout, converts each to JSON via `csc2json.py`, builds any missing firmware (`--no-build` to skip), runs `test_runner mixed-multinode` with the JS test script attached, and reports compatible PASS / FAIL / SKIP totals.

## JSON simulation configs

Full schema in [`docs/test-format.md`](docs/test-format.md).  Short version:

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
        "validators": [ { "pattern": "Received response", "min_count": 6 } ],
        "actions": [
            { "at_ms": 30000, "type": "move", "node": 3, "x": 200.0, "y": 0.0 },
            { "at_ms": 45000, "type": "move", "node": 3, "x":  60.0, "y": 0.0 }
        ]
    }
}
```

Working examples in [`configs/`](configs/): `rpl-udp-{sky,cc2538dk,native}.json`, `mixed-sky-native.json`, `udgm-{3node,in-range,out-of-range,100node-grid}.json`, `test-4node-chain.json`, `test-js-rpl-udp.json`, `ui-rpl-udp-grid.json`.  Run any with `./build/test_runner mixed-multinode configs/<file>.json [-v]`.

## Performance

Measured on Apple Silicon (PGO build) and Linux x86-64 (release).  See `bench` for reproducible numbers.

| Platform / scenario | Speed |
|---|---|
| MSP430 micro-benchmarks (JIT, Apple Silicon, avg of 7) | ~430 MIPS |
| MSP430 `blink.sky` / `energest-demo.sky` (JIT) | ~195 MIPS |
| MSP430 2-node nullnet (60 s sim) | ~500× real-time |
| MSP430 2-node RPL-UDP (60 s sim, Apple Silicon PGO) | ~1600× real-time |
| MSP430 2-node RPL-UDP (Linux x86-64 release) | ~250× real-time |
| CC2538DK 2-node RPL-UDP (interpreter) | ~300× real-time |
| nRF52840 2-node RPL-UDP (interpreter) | ~360× real-time |
| nRF54L15 2-node RPL-UDP (interpreter) | ~100× real-time |
| nRF54L15 FLPR dual-core demo (M33 + RV32E, `flpr-host`) | ~0.1× real-time |

nRF54L15 RPL-UDP runs ~100× real-time (it sleeps in WFI when the radio is idle, which the emulator fast-forwards).  The **FLPR dual-core demo** is the slow one (~0.1×): the `hello-vpr` firmware uses *polled* etimers, so the RV32E core busy-spins on the GRTC clock between ticks — ~360 M instructions for a 4 s sim, almost all of it polling.  That's a faithful emulation of the firmware, not an emulator inefficiency; the fix is interrupt-driven etimers (GRTC CC3) in Contiki's `nrf-vpr` port, plus modelling RV32E `WFI` as idle in csim.  See the FLPR row in [Platforms](#platforms) and `docs/design/riscv-vpr-plan.md`.

## Architecture

**Ports and adapters.**  At the structural level Cooja-NG is a ports-and-adapters
(hexagonal) design.  A small simulation **kernel** — `sim_runtime_t` (clock,
unified event queue, radio bus, mote slots) plus the service host and the radio
medium — is the core, and it depends on nothing concrete: it talks only to a
handful of narrow vtable **ports**, and everything platform- or
feature-specific is an **adapter** behind one of them.

| Port (interface) | Adapters (implementations) |
|---|---|
| `sim_mote_ops_t` — "a node" | MSP430-ELF, ARM-ELF (CC2538/Firefly/nRF52840/nRF54L15), native Cooja mote, JS app mote |
| `sim_service_ops_t` — "an observer/feature" | timeline, pcap, progress, gdb, websocket-UI, energest, JSON/JS test engines |
| `sim_medium_ops_t` — "a propagation policy" | UDGM, NONE, and plugin media |
| `sim_host_t` — chip→host callbacks | the runner's radio channel/power/byte hooks |
| `csim_api` (`csim_plugin.h`) — the dlopen boundary | external `.so` plugins (service / medium / UI panel) |
| `sim_observer_event_t` — the outbound event stream | anything that subscribes |

The enforced rule is **dependency inversion**: the kernel never branches on a
node's type — *"add an op, don't `switch` on `NODE_MSP430`"* — so the runner
includes no chip headers and a new SoC, radio policy, feature, or plugin slots
in by implementing a port, not by editing the core.  The plugin ABI is the
strictest port: a plugin reaches the host *only* through the passed vtable,
never a kernel struct.  (Pragmatic C, not dogma: the kernel exposes a concrete
`sim_runtime_t` rather than an opaque handle, and the `test_runner` frontend is
still a fat driving adapter — but the dependency arrows point inward.)  The
staged refactor that established this boundary is tracked in
[`docs/design/refactor-plan.md`](docs/design/refactor-plan.md).

**Dual time domains.**  Every CPU tracks `cycles` (CPU-cycle count, the unit the interpreter advances) and `sim_time_ns` (ns-precise wall clock, the unit peripherals and inter-node coordination use).  Reconciled at three sync points: before each event callback, at the end of `*_step_until()`, and inside `*_set_frequency()` on DCO calibration.  This is what lets one node speed up after DCO cal while another is still booting, or an MSP430 and a CC2538 (different MHz) share the same medium.

**Hot path.**

```
fetch -> dispatch via computed goto (or JIT block)
      -> execute, update regs/flags/cycles
      -> if (cycles >= next_event_cycle) execute_events()
      -> if (interrupts pending && GIE) service_interrupt()
```

No ns-conversion or floating-point on the hot path.  Events scheduled in cycle units; ns-based events are converted once at scheduling time and re-converted only on frequency change.

**Multi-node loop.**

```c
while (sim_ns < end_ns) {
    sim_ns += time_step_ns;                       // 1 ms by default
    for (each node) {
        delta_ns     = sim_ns - cpu->sim_time_ns;
        target_cycle = cpu->cycles + ns_to_cycles(delta_ns, cpu->cpu_freq_hz);
        cpu_step_until(cpu, target_cycle);
    }
    deliver_rf_bytes();                           // per-byte radio medium delivery
    flush_uart();
    process_actions();                            // timed move/send/remove/add
    check_test_engine();                          // step matching, validators, fail_on
}
```

Per-byte RF delivery is the key fidelity choice — receivers see one symbol at a time at the exact ns it arrives on the link, instead of whole frames dropped in one go.

**Per-radio multi-channel medium.**  Every radio chip (CC2420, CC2538 RF Core, CC1200, nRF52840/nRF54L15) registers its own slot.  Frames route by `(band, channel)` so 2.4 GHz and sub-GHz networks coexist without cross-band leakage; TSCH channel hops correctly affect what each receiver hears.  235-test safety net — see [`docs/radio-medium.md`](docs/radio-medium.md) and [`docs/porting-a-device.md`](docs/porting-a-device.md) §8 for the chip-driver event model.

**Cooja test wrapper.**  `tools/run-cooja-tests.sh` is the bridge to upstream Contiki-NG's test infrastructure — treats Cooja-NG as a drop-in for `java -jar Cooja.jar --no-gui`, runs the same `.csc` topologies, parses the same JS scripts via `csc2json.py`, reports compatible PASS/FAIL.  Every fix is validated against the upstream test suite, not a private regression set.

Deeper notes on each subsystem in [`CLAUDE.md`](CLAUDE.md) and [`docs/architecture.md`](docs/architecture.md).

## Tuning

| Variable | Default | Effect |
|---|---|---|
| `MSPSIM_JIT_THRESHOLD` | `100` | Number of times a basic block must execute before JIT compiles it |
| `MSPSIM_JIT_INBLOCK_CHECKS` | `1` | Emit IRQ/event-fire checks inside JIT blocks (needed for tight timer loops; `0` for max throughput) |
| `CSIM_TRACE_TSCH_ACK` | unset | Verbose CC2420 ACK timing trace for TSCH debugging |
| `CSIM_TRACE_EVENT_SPIN` | unset | Trace multi-node event spin-loop iterations |
| `NRF54L_RADIO_TRACE` | unset | nrf54l15 RADIO state/event/INTENSET trace; pair with `NRF54L_RADIO_NODE=<cpu-tag>` to filter |
| `CONTIKI_DIR` | `../contiki-ng` | Where `make cooja-tests` and `tools/build-test-firmware.sh` look for the Contiki checkout |

## Test results

| Suite | Result |
|---|---|
| `correctness` (MSP430 instructions) | **68 / 68 PASS** |
| `arm-correctness` (Cortex-M3/M4F/M33, incl. DSP + VFP) | **81 / 81 PASS** |
| `timeline` (event serializer) | **76 / 76 PASS** |
| `cc1200-mock-host` (CC1200 chip driver) | **73 / 73 PASS** |
| `radio-medium` (per-radio multi-channel) | **235 / 235 PASS** |
| `firmware` (cputest.sky, timertest.sky) | **2 / 2 PASS** |
| `arm-firmware` (cc2538dk + nRF bring-up) | **PASS** |
| `zoul-firefly-multinode` RPL-UDP | **6 / 6 hello cycles** in 60 s, ~9× real-time |
| `nrf52840-dongle-multinode` RPL-UDP | UDP request/response round-trip, ~9× real-time |
| `nrf54l15-dk-multinode` RPL-UDP | UDP request/response end-to-end (~100× real-time) |
| **Cooja test suite (89 tests, with `--with-tun`)** | **89 / 89 PASS** |

Cooja-suite coverage: all 27 `07-simulation-base/*` (RPL-Lite, TSCH, Orchestra, multicast, IPv6, stack guard, data structures) including the once-stubborn `26-tsch-drift-z1` (16 s); all 12 `09-ipv6/*`; all 9 `13-ieee802154/*`; all 14 `14-rpl-lite/*` and 19 `15-rpl-classic/*` (including 28-h simulated DAG stability tests); all 8 `17-tun-rpl-br/*` with real `tun0` + `tunslip6`, including `10-native-nat64-cooja` (214 s, UDP+TCP echo through the NAT64 gateway).

## Known issues

Real, currently reproducible quirks in the *standalone CLI shortcuts* — none affect the Cooja test wrapper, JSON-config flow, or production use.

1. **`./build/test_runner multinode` (no firmware) hangs.**  The default-firmware shortcut routes to `firmware/sky/nullnet-broadcast.sky` and gets stuck after init.  Workaround: use a JSON config or explicit firmware pair.
2. **nRF54L15 RPL-UDP takes ~30 s of *simulated* time to converge.**  End-to-end works (DIO → DAO → DAO-ACK → UDP request/response); RPL settles a bit later than nrf52840 (deferred-PHYEND model + 1 MHz GRTC timing fidelity), with CSMA retransmits visible in the packet log.  This is simulated-time-to-converge, not wall-clock — the run itself executes at ~100× real-time.  Cosmetic.
3. **`test_firmware.c` reports `timertest.sky` as PASS** even when the firmware itself prints `FW: FAIL: count > 10 failed at timertest.c:166`.  The runner only matches `EXIT`.  Cosmetic.
4. *(resolved — was: "the deferred-PHYEND fix has not been ported to nrf52840")* nrf52840 is **not** exposed to the critical-section-during-TX race: it fires PHYEND/END at full frame air-time (`(6+len)·32 µs`, ≥~350 µs after `TASKS_START`; `nrf52840_soc.c`), which already lands the IRQ during the driver's busy-wait, well after its NVIC-disabling critical section. nRF54L15 uses a *different* 100 µs defer only because it delivers the frame at TX-start (synchronous model) and a full-air-time defer would leave it stuck in TX when the peer's ACK arrives — not because nrf52840 lacks protection. The timing is per-frame air-time, independent of RPL rate.

Release history and per-version notes in [`CHANGELOG.md`](CHANGELOG.md).

## License

3-clause BSD.  See [LICENSE](LICENSE).  Copyright © 2026 Joakim Eriksson, RISE Research Institutes of Sweden.

Bundled `lib/quickjs/` is © Fabrice Bellard and Charlie Gordon, MIT license.
