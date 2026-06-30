# CLAUDE.md — Cooja-NG C Emulator

## Design Documents

Before kernel, platform, plugin, or simulation-runner refactors, read
[`docs/design/refactor-plan.md`](docs/design/refactor-plan.md). It defines the
internal simulation kernel direction, static plugin registry model, and staged
runtime extraction plan.

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
./build/test_runner correctness -v    # 72 instruction-level tests
./build/test_runner bench             # 7 micro-benchmarks + 2 firmware benchmarks
./build/test_runner firmware          # Firmware integration tests (cputest.sky, timertest.sky)
./build/test_runner multinode         # 2-node nullnet-broadcast (default 20s)
./build/test_runner multinode firmware/sky/udp-server.sky firmware/sky/udp-client.sky -t 60000

# ARM Cortex-M3/M4 tests
./build/test_runner arm-correctness -v   # 146 instruction-level tests (Thumb-2 + M4 DSP/VFP + M33)
./build/test_runner arm-firmware -v      # Firmware boot test (hello-world.cc2538dk)
./build/test_runner arm-multinode firmware/cc2538dk/nullnet-broadcast.cc2538dk -t 20000
./build/test_runner arm-multinode firmware/cc2538dk/udp-server.cc2538dk firmware/cc2538dk/udp-client.cc2538dk -t 60000

# Chip-driver + radio-medium unit suites
./build/test_runner cc1200-mock-host        # 73 CC1200 chip tests (mock host, no CPU)
./build/test_runner radio-medium            # 235 radio-medium routing tests

# nRF52840 USB Dongle (PCA10059, Cortex-M4F + on-chip 802.15.4 radio)
./build/test_runner nrf52840-dongle-multinode firmware/nrf52840-dongle/udp-server.nrf52840-dongle firmware/nrf52840-dongle/udp-client.nrf52840-dongle -t 60000

# nRF52840 Development Kit (PCA10056, same SoC + SEGGER UART)
./build/test_runner nrf52840-dk-multinode firmware/nrf52840-dk/udp-server.nrf52840-dk firmware/nrf52840-dk/udp-client.nrf52840-dk -t 60000

# TSCH (802.15.4e time-slotted channel hopping) — 2-node association + HELD
# sync (>=3 periodic drift reports prove the enhanced-ACK/EB sync loop is
# alive, not just association). Contiki-NG 6tisch/simple-node; node 1
# self-selects coordinator (node_id==1).
./build/test_runner test configs/test-tsch-cc2538dk.json      # ~240s sim
./build/test_runner test configs/test-tsch-nrf52840-dk.json   # ~25s sim

# Stock Zephyr 802.15.4 (echo_server/echo_client, only the sample's overlay-802154.conf).
# Two-node UDP echo over 802.15.4/6LoWPAN/IPv6/RPL; server logs "Received and replied", 0 timeouts.
./build/test_runner nrf52840-dk-multinode firmware/nrf52840-dk/zephyr-echo-server.nrf52840-dk firmware/nrf52840-dk/zephyr-echo-client.nrf52840-dk -t 40000

# nRF54L15 FLPR dual-core / RISC-V (Contiki-NG nrf-vpr). One M33 image launches
# the RV32E FLPR; the M33 prints "[FLPR] tick N" (advances ~2/sec). Add --ui 8080
# to watch LED0 (P2.9, 1 Hz, RISC-V) + LED1 (P1.10, 2 Hz, M33) blink in the browser.
# CSIM_GPIO_TRACE=1 dumps timestamped GPIO OUT changes.
./build/test_runner mixed-multinode firmware/nrf54l15-dk/flpr-host.nrf54l15-dk -t 4000
```

Multinode options: `-t ms` (sim duration), `-n nodes` (node count), `-q` (quiet), `-v` (verbose).

```sh
# Dynamic plugins (Phase 9) — dlopen a .so that registers a service or medium
make plugins                                    # packet_sink.so + lossy_medium.so
./build/test_runner test configs/test-rpl-udp-sky.json --plugin build/plugins/packet_sink.so
./build/test_runner test configs/plugin-demo-sky-v2.json   # config v2 "plugins": [...]
# Energy estimation — a COMPILED-IN plugin (built-in service) selected by config
# name; per-mote radio duty cycle + CPU/LPM energy (PowerTracker/Energest); also
# publishes a live web-UI panel (v3 ui ABI). No .so to ship.
./build/test_runner test configs/plugin-energest-builtin-v2.json   # "plugins": ["energest"]
# Pluggable radio medium (Phase 11) — config medium.type names a plugin medium
./build/test_runner test configs/medium-plugin-sky-v2.json # "medium": {"type": "lossy"}
# Gilbert-Elliott burst-loss medium — 2-state Markov; knobs via CSIM_GE_* env
make test-ge                                    # statistical model validation (drop rate, burst length)
CSIM_GE_AVG_DROP=0.2 CSIM_GE_BURST_LEN=8 ./build/test_runner test configs/medium-plugin-gilbert-elliott.json
tools/check-plugin.sh                           # plugin smoke check (service + medium + energy)
```

## Project Structure

```
src/
  sim/                Simulation kernel: runtime, event pump, radio bus, board registry, service host
  services/           Extracted services (Phase 6): timeline, pcap, progress, json-test, js-test, gdb, websocket-ui
  motes/              Per-kind mote modules: boot policy, full mote vtable, kind registry
  common/             Shared infrastructure: event queue, radio medium, ELF loader, GDB stub
  msp430/             MSP430 emulator source files
  arm/                ARM Cortex-M3/M4/M33 emulator source files
  riscv/              RV32E emulator (nRF54L15 FLPR coprocessor) + SoC bridge
  native/             Native Cooja motes (dlopen) + JS app motes (QuickJS)
  ui/                 WebSocket/state bridge (observation only)
include/
  sim/                Kernel headers (sim_runtime.h, sim_mote.h, sim_radio_bus.h, sim_board.h, sim_registry.h, csim_plugin.h)
  common/, msp430/, arm/, riscv/, native/, ui/
test/                 Test runner, correctness, benchmarks, firmware, multinode
firmware/sky/         Pre-compiled Contiki-NG firmware for Tmote Sky
firmware/cc2538dk/    Pre-compiled Contiki-NG firmware for CC2538DK
```

### Simulation Kernel (src/sim/)

| File | Purpose |
|------|---------|
| `sim_runtime.c` | `sim_runtime_t` container: now_ns, unified event queue, radio medium, mote slots + generations, observer fan-out, `sim_runtime_run_until()` event pump |
| `sim_radio_bus.c` | Full RF delivery path (Phase 5): per-sender byte clock, frame assembler (802.15.4 + 802.15.4g), medium-filtered per-receiver dispatch (SYNC/PER_BYTE/BATCH), RX-stall timer, emulated RX core (deliver/queue/drain), frame-complete policy (air-time + collision windows, RXFIFO backpressure, dual 192 µs auto-ACK windows), native/JS frame path, channel push/pull, channel-busy query, bus-owned RX/frame stats |
| `sim_board.c` | Board registry: firmware extension → {mote kind, platform name, label} |
| `sim_registry.c` | Static built-in registry (Phase 8): one `sim_registry_t` lookup surface for boards, mote kinds, services, and radio media; `csim_register_builtin_{platforms,mote_types,services,media}` populate it; services + media (Phase 11: "udgm"/"none" + plugins) resolve by name via owned name→ops catalogs |
| `sim_plugin.c` | Dynamic plugin loader (Phase 9): `sim_plugin_load` dlopens a `.so` (RTLD_NOW\|RTLD_LOCAL), resolves `csim_plugin_init`, and hands it a `csim_api_t` so the plugin registers a service. ABI is additive/version-gated (`include/sim/csim_plugin.h`): v1 `register_service`, v2 `+register_radio_medium`, v3 `+ui->publish_panel` (a plugin draws a live web-UI panel — see [`docs/design/ui-plugins.md`](docs/design/ui-plugins.md)). Dynamic `.so` examples: `plugins/packet_sink.c` (service), `plugins/lossy_medium.c` (medium). A plugin can also be **compiled in** as a built-in service (registered in `sim_registry.c`) and selected by config name (`"plugins": ["energest"]`, Cooja's built-in-plugin style) — example: the energy estimator `src/services/energest_{engine,service}.c` |
| `sim_config.c` | JSON config loader (Phase 7): `sim_config_load` dispatches on `version` to `parse_v1`/`parse_v2`, both populating one `sim_normalized_config_t` the runtime consumes |
| `sim_service.c` | Service host (Phase 6 M31): `sim_service_ops_t` vtable table + one fan-out observer + ordered poll/teardown + error policy |
| `sim_serial_bridge.c` | TCP serial socket service (Cooja serial-socket protocol) |
| `sim_external_command.c` | External command service (border-router etc. helper processes) |

### Services (src/services/)

Phase 6 extracted the runner's optional/observation features into services
behind the `sim_service_ops_t` host: `timeline_service.c` (activity
timeline), `pcap_service.c` (802.15.4 capture), `progress_service.c`
(per-tick progress report), `json_test_service.c` (JSON step/validator
runner), `js_test_service.c` (JS test-engine line feed), `gdb_service.c`
(per-mote GDB stub; sets `cpu->gdb_stub`), `websocket_ui_service.c` (live
UI: ws_server + console + serialization). The end-of-run statistics stay
runner-side (type-specific diagnostics that read chip memory + firmware
symbols).

A later addition is the energy estimator, shipped as a **compiled-in plugin**:
`energest_engine.c` (host-agnostic core — per-mote radio duty cycle + Energest
CPU/LPM/TX/LISTEN energy from the `SIM_OBS_RADIO_STATE`/`SIM_OBS_CPU_STATE`
observer stream, driven by a `publish`+`log` sink) and `energest_service.c`
(the built-in wrapper, registered in `sim_registry.c`, selectable by config
name `"plugins": ["energest"]`). It also publishes a live web-UI panel via the
v3 plugin ABI. See [`docs/design/ui-plugins.md`](docs/design/ui-plugins.md).

### Mote Modules (src/motes/)

| File | Purpose |
|------|---------|
| `mote_impl.h` | Private shared header: `mixed_node_t`, `sim_mote_env_t` (runner glue bundle), per-kind module APIs |
| `mote_kinds.c` | Mote-kind registry: board kind → {boot, register_radio, ops} row |
| `msp430_elf_mote.c` | MSP430 boot policy (ELF load, ds2411/infomem/node-id patches, run-to-main), execute tick, CC2420 radio ops, full mote vtable (`msp430_elf_mote_ops`) |
| `arm_elf_mote.c` | ARM boot policy (cc2538/firefly/nrf52840/nrf54l15 wiring, FICR seeding, linkaddr patches), execute tick, radio ops, full mote vtable (`arm_elf_mote_ops`) |
| `native_cooja_mote.c` | Native Cooja mote: boot, full adapter table, tick helpers, SYNC radio ops |
| `js_app_mote.c` | JS app mote: boot, full adapter table, BATCH radio ops |

The mote vtable (`include/sim/sim_mote.h`, `sim_mote_ops_t`) abstracts the four
node kinds (MSP430/ARM/native/JS). All four kinds' ops tables are module-owned
now — Phase 6 M38 moved the MSP430/ARM execute/serial adapters into their
modules (the radio-bus dependency cleared in Phase 5, the GDB-stub dependency
in Phase 6 M37), retiring the runner-side injection. See
`docs/design/refactor-plan.md` §3.15–§3.23 for the completed Phase 1–10
milestones (Phase 5 extracted the RF-delivery policy into `sim_radio_bus.c`
and retired `--threads`; Phase 6 extracted the observation/optional features
into `src/services/` behind a `sim_service_ops_t` host and moved the emulated
adapters out; Phase 7 added config v2 over one `sim_normalized_config_t`;
Phase 8 routed the runner's board/mote-kind/service lookups through one
static `sim_registry_t`; Phase 10 shrank the runner to a pure frontend —
the emulated MSP430/ARM chip coupling moved behind mote ops
(`dump_diagnostics`/`apply_startup_delay`/`program_counter`) so the runner
includes no chip headers and never switches on `NODE_MSP430`/`NODE_ARM`;
Phase 9 added the dlopen plugin ABI (`csim_plugin.h` + `sim_plugin.c`, v1 =
register a service; example `plugins/packet_sink.c`). The native host-process
scheduling policy is the one documented deferral. **The staged refactor
(Phases 1–10) is complete.**

### MSP430 Source Files (src/msp430/)

| File | Purpose |
|------|---------|
| `msp430_cpu.c` | Core CPU: computed-goto interpreter, event queue, interrupt service, step/step_until |
| `msp430_config.c` | MCU configurations: F149, F1611, F2617, F5437, CC430F5137, FR5969 |
| `msp430_clock.c` | Clock module: BCS (DCOCTL/BCSCTL1) and CS (FR5xxx) variants, ACLK/SMCLK dividers |
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
| `nrf52840_soc.c` | nRF52840 SoC bundle: CLOCK/RTC/TIMER/UARTE/RADIO (EasyDMA)/RNG/TEMP/FICR |
| `nrf54l15_soc.c` | nRF54L15 SoC bundle: GRTC/DPPI/RADIO/EGU/TIMER/UARTE/FICR, **GPIO P0-P2**, and the **VPR/SPU FLPR-launch registers** (CPURUN/INITPC + SECATTR gate) |

### RISC-V Source Files (src/riscv/)

| File | Purpose |
|------|---------|
| `riscv_cpu.c` | RV32E interpreter (`rv32e_zicsr_zifencei`): base RV32I + CSR + `fence.i` + M-mode traps. Shares the host M33's memory/IO bus, so SRAM + peripherals are one address space. No M/C/F/D |
| `nrf54l_vpr.c` | Bridge: on the M33's VPR `CPURUN` edge, instantiate the FLPR over the shared bus and co-step it after each ARM execute slice (SoC-agnostic `coproc` hook on `arm_cpu_t`) |

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

**TX power (Phase 12):** a TXCTRL write pushes PA_LEVEL (`& 0x1f`, max 31) to the radio medium via the `radio_set_power` host hook; the medium scales effective range by `indicator/max` (Cooja UDGM). Firmware that holds max PA is byte-identical with the old fixed range.

**CRC:** CCITT-16 with bit reversal, matching CC2420 hardware behavior.

### Timer A/B

On-demand counter: not incremented every cycle. Instead, counter value is computed from elapsed cycles when read.

**Clock sources:** TCLK (external), ACLK (32768 Hz), SMCLK (DCO-derived), INCLK
**Modes:** Stop, Up (count to CCR0), Continuous (count to 0xFFFF), Up/Down
**Events:** CCR compare events scheduled in CPU event queue (cycle-based)

### Clock Module

Two variants selected by `mcu->clock_type`:

**BCS (classic, F1xx/F2xx; UCS on F5xxx is register-compatible)** — DCO frequency formula matches Java MSPSim:
```
dco_factor = (max_dco_freq - 1000) / 2048
freq = ((dco_freq << 5) + dco_mod + (rsel << 8)) * dco_factor + 1000
```
Default: DCOCTL=0x60, BCSCTL1=0x84 -> ~2.69 MHz (F1611). After firmware calibration: ~3.9 MHz.

**CS (FR5xxx)** — Password-protected (CSCTL0_H = 0xA5 unlocks). DCO frequency from a lookup table indexed by DCOFSEL/DCORSEL. Per-clock source select (SELA/SELS/SELM) and dividers (DIVA/DIVS/DIVM) in CSCTL2/CSCTL3.

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
| **fr5969** | MSP430FR5969 | No | eUSCI_A0 | MSP-EXP430FR5969 LaunchPad (FRAM, CS clock) |
| **cc2538dk** | CC2538 (ARM Cortex-M3) | Yes (on-chip) | UART0 | TI SmartRF06 + CC2538EM |
| **openmote** | CC2538 (ARM Cortex-M3) | Yes (on-chip) | UART0 | OpenMote board (same SoC as cc2538dk) |
| **zoul-firefly** | CC2538 (ARM Cortex-M3) | Yes (on-chip) + CC1200 (off-SoC sub-GHz) | UART0 | Zolertia Firefly — dual-band |
| **nrf52840-dongle** | nRF52840 (ARM Cortex-M4F) | Yes (on-chip 2.4 GHz) | UART0 (legacy window) | Nordic PCA10059 USB Dongle, M4F + FPv4-SP-D16, VTOR=0x1000 (Open Bootloader region at 0x0..0xfff) |
| **nrf52840-dk** | nRF52840 (ARM Cortex-M4F) | Yes (on-chip 2.4 GHz) | UART0 (legacy window) | Nordic PCA10056 Development Kit, same SoC as Dongle, VTOR=0x0, SEGGER VCP console |
| **nrf54l15-dk** | nRF54L15 (ARM Cortex-M33, ARMv8-M) | Yes (on-chip 2.4 GHz) | UARTE20 | Nordic nRF54L15-DK, 256 KB RAM, GRTC/DPPI fabric, VTOR=0x0 |
| **nrf54l15-dk + FLPR** | nRF54L15 **FLPR (RV32E, RISC-V)** coprocessor | — (uses the M33's radio) | shared SRAM | **Dual-core / cross-ISA**: the M33 (`flpr-host`) loads the FLPR blob into shared SRAM and releases it via the VPR `CPURUN` register; the RV32E core (`hello-vpr`) then runs **unmodified Contiki-NG** alongside the M33. ISA `rv32e_zicsr_zifencei` (no M/C). See [`docs/design/riscv-vpr-plan.md`](docs/design/riscv-vpr-plan.md) |

## MCU Configurations

| MCU | Address Space | RAM | Flash | Max DCO | MSP430X |
|-----|---------------|-----|-------|---------|---------|
| MSP430F149 | 64 KB | 2 KB @ 0x200 | 60 KB @ 0x1100 | 4.9 MHz | No |
| MSP430F1611 | 64 KB | 10 KB @ 0x1100 | 48 KB @ 0x4000 | 4.9 MHz | No |
| MSP430F2617 | 1 MB | 8 KB @ 0x1100 | 92 KB @ 0x3100 | 16 MHz | Yes |
| MSP430F5437 | 1 MB | 16 KB @ 0x1C00 | 256 KB @ 0x5C00 | 25 MHz | Yes |
| CC430F5137 | 1 MB | 4 KB @ 0x1C00 | 32 KB @ 0x8000 | 25 MHz | Yes |
| MSP430FR5969 | 64 KB | 2 KB @ 0x1C00 | 32 KB FRAM @ 0x4400 | 24 MHz | Yes |

## Multi-Node Simulation

Single-threaded, event-driven kernel (Cooja's scheduling model; Phases 1–3 of
`docs/design/refactor-plan.md`).  `sim_runtime_t` owns the simulation clock,
the unified `(time_ns, seq)`-ordered event queue, the radio medium/bus, and
the per-slot mote objects.

**Architecture:**
1. Initialize nodes: the board registry (`sim_board.c`) maps the firmware
   extension to a platform; load ELF, patch ds2411/node-id/linkaddr, run crt0
   to main, register mote + radio-endpoint ops
2. Kernel pump: `sim_runtime_run_until(sim, horizon, dispatch)` pops events in
   `(time, seq)` order, advancing `now_ns` to each event's exact time —
   `NODE_WAKEUP` (mote execute slice), `RX_BYTE` (per-byte radio delivery),
   `RADIO_TIMER` (RX-stall watchdog), `TEST_ACTION` (script time pins)
3. Mote execute: `ops->execute(m, now_ns)` runs one Cooja-style slice
   (MspMote.execute equivalent: clock-deviation jump, pinned sim_time,
   step_micros) and returns the mote's next wakeup time
4. RF TX: chip TX callbacks feed `sim_radio_bus_tx_byte()` (per-sender byte
   clock + frame assembler + medium filter), which dispatches per receiver
   delivery mode: SYNC (native), PER_BYTE (CC2420 / cc2538 / nrf52840 /
   nrf54l15 — one kernel RX_BYTE event per on-air byte; what makes TSCH's
   receiving_packet()-based slot timing work), BATCH (JS)

Per-mote cycle accounting inside the execute slices handles nodes running at
different CPU frequencies after DCO calibration.

### ARM Multi-Node (CC2538)

Same event-driven kernel as MSP430, with CC2538 RF Core for 802.15.4 radio:

1. Initialize nodes: load ELF, run crt0 to main, patch `linkaddr_node_addr` per node
2. Each node gets unique IEEE address (00:12:74:node_id:00:00:00:node_id)
3. RF Core TX callback feeds the radio bus; receivers take per-byte kernel events
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

### ARM (interpreter only)

| Benchmark | Speed |
|-----------|-------|
| CC2538 2-node RPL-UDP (60s sim) | ~300x real-time |
| nRF52840 2-node RPL-UDP (60s sim) | ~360x real-time |
| nRF52840 2-node TSCH (60s sim) | ~14x real-time |
