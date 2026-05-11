# nRF52840 USB Dongle Port — Status & Direction

> Strategic doc: where the port is, where it's going, and what's
> explicitly out of scope. For the device contract see
> [`SPEC.md`](SPEC.md).

## Current state — short version

**Pre-L0 scaffolding in progress; the architectural decision needed
before peripheral code lands has been resolved.** The contract —
[`SPEC.md`](SPEC.md) and this file — defines what the port covers,
how it splits across commits, and where the risks sit.

Concrete progress so far:

- `arm_config_t nrf52840_config` added to `src/arm/arm_config.{c,h}`:
  64 MHz, 1 MiB flash @ `0x00000000`, 256 KiB SRAM @ `0x20000000`,
  48 IRQs. Per nRF52840 PS v1.7.
- **`arm_platform_t` is now SoC-polymorphic.** SoC-specific peripherals
  live behind `plat->soc` + an `arm_soc_ops_t` vtable (init / destroy /
  set_console). CC2538 peripherals migrated into `cc2538_soc_t`
  (`include/arm/cc2538_soc.h`, `src/arm/cc2538_soc.c`); consumers reach
  them via the `arm_platform_cc2538(plat)` accessor.
- **Per-instance memory layout in `arm_cpu_t`.** `flash_base`, `flash_end`,
  `sram_base`, `sram_end`, `rom_size` are populated from the SoC config
  at init time. Every memory access in `arm_cpu.c` / `arm_elf.c` /
  `arm_nvic.c` consults these fields instead of CC2538-hardcoded macros
  (`ARM_FLASH_BASE`, etc.). The macros stay in `arm_cpu.h` as constants
  for `test_arm_correctness.c` (which always uses cc2538_config).
- **Vector-table discovery is SoC-aware.** `arm_config_t::vtor_default`:
  if non-zero, used directly at reset; if zero, falls back to the CC2538
  CCA convention (read flash_end - 0x2C + 8). `nrf52840_config` sets
  `vtor_default = 0x1000` because the dongle reserves 0x0..0xfff for
  the Open Bootloader.
- **`nrf52840_soc.{h,c}` skeleton landed** — empty `nrf52840_soc_t`,
  `nrf52840_soc_ops` (init / destroy / set_console no-ops), wired into
  the platform registry as `platform_nrf52840_dongle`.

**L0 / L1 / L2 reached.** Built two reference firmware ELFs via
`tools/build-device-firmware.sh --target nrf52840 --board dongle`:
  - `hello-world.nrf52840-dongle` (197 KiB) — default dongle build,
    console = USB-CDC.
  - `hello-world-uart.nrf52840-dongle` (118 KiB) — same source built
    with `NRF52840_NATIVE_USB=0`, console = legacy UART0 register
    window. **This is the one csim currently runs end-to-end.**

Then added two minimal peripheral stubs in `nrf52840_soc.c`:
  - **CLOCK** (0x40000000): TASKS_HFCLKSTART / TASKS_LFCLKSTART writes
    set the corresponding EVENTS_*STARTED registers immediately.
  - **UART0 legacy window** (0x40002000): writes to TXD (0x51C) call
    the platform's console callback and set EVENTS_TXDRDY (0x11C);
    firmware acks by writing 0 back to EVENTS_TXDRDY.

A smoke runner loads the UART ELF, hooks `set_console` to stdout,
resets, and steps. Result is the full Contiki-NG bring-up banner +
the hello message:

```
[INFO: Main      ] Starting Contiki-NG-release/v5.1-81-gad0d07381
[INFO: Main      ] - Routing: RPL Lite
[INFO: Main      ] - Net: sicslowpan
[INFO: Main      ] - MAC: CSMA
[INFO: Main      ] - 802.15.4 PANID: 0xabcd
[INFO: Main      ] - 802.15.4 Default channel: 26
[INFO: Main      ] Node ID: 0
[INFO: Main      ] Link-layer address: f4ce.3600.0000.0000
[INFO: Main      ] Tentative link-local IPv6 address: fe80::f6ce:3600:0:0
Hello, world
```

443 UART bytes emitted across ~836 K instructions in ~1.1 M cycles.
Then PC parks at 0x00004b30 — the next unmodelled peripheral spin
(probably RTC for Contiki's etimer process; investigate next).

Verification — zero regressions:
  - 68/68 MSP430 + 48/48 ARM correctness (33 base + 15 M4 DSP)
  - ARM Firefly bringup PASS, cc2538dk RPL-UDP 26 RX, 11.0× real-time

## L4 milestone — single-node application logic running

Two more peripherals + one fix lifted the firmware from idle into real
application code:

  - **RADIO state machine** (0x40001000): tasks (TXEN/RXEN/START/STOP/
    DISABLE/CCASTART/RSSISTART/EDSTART) drive transitions through
    DISABLED→RXRU→RXIDLE→RX, DISABLED→TXRU→TXIDLE→TX, with the relevant
    SHORTS auto-chained (READY_START, TXREADY_START, RXREADY_START,
    END_DISABLE, etc.). State entry fires READY/RXREADY/TXREADY/END/
    PHYEND/DISABLED events; events with INTENSET bits set raise the
    RADIO IRQ (vector 1). All transitions are instant (no rampup
    delay).
    **Not yet modelled**: PACKETPTR-based EasyDMA (TX bytes are
    dropped, RX never delivers external data), CRC verification,
    BCMATCH/MHRMATCH, RSSI/EDSAMPLE values. That's the work for the
    multinode integration.

  - **TIMER0..4** (0x40008000+): minimal model that satisfies
    `rtimer_arch_now()` (TASKS_CAPTURE[i] → CC[i] = current counter,
    counter advances at 16 MHz >> PRESCALER). Without this the radio
    driver's `RTIMER_BUSYWAIT(TXRU_DURATION_TIMER)` spun forever.

Empirical: `udp-client.nrf52840-dongle` runs **90 sim seconds in 15 s
wall** (≈6× real-time, ARM interpreter only). Prints the Contiki banner
and then 4 instances of `[INFO: App       ] Not reachable yet` — the
client's app timer firing every ~22 s, reporting "no RPL parent" because
no DIO has been received (no working radio data path yet, and no second
node to receive DIOs from).

That's L4 in spirit: the firmware reaches the app process, schedules
events, and runs them on time. CSMA/MAC layers also functional —
they accept the TX call without spin-locking.

## L6 reached — RPL-UDP exchanges hello/response between two nrf52840 nodes

```
41.378 [Node 2/ARM] Sending request 0 to fd00::f6ce:3601:f1f4:203b
41.459 [Node 1/ARM] Received request 'hello 0' from fd00::f6ce:3602:e3e9:4176
41.485 [Node 1/ARM] Sending response.
41.571 [Node 2/ARM] Received response 'hello 0' from fd00::f6ce:3601:...
59.743 [Node 2/ARM] Sending request 1 to fd00::f6ce:3601:f1f4:203b
```

Round-trip in 193 ms simulated, ~1× real-time wall.  All 445 existing
tests still pass (68 MSP430 + 48 ARM + 21 mock-host + 73 cc1200 +
235 radio_medium).

The four bugs that blocked convergence after the multinode harness
landed:

1. **NULL-deref in `emulated_rxfifo_available`** — was unconditionally
   calling `arm_platform_cc2538(plat)->rfcore` for any ARM node;
   crashed on first frame to an nrf52840 node. Fix: branch on cc2538
   vs nrf52840 and report "always available" for nRF (EasyDMA
   bypasses the FIFO entirely).
2. **RADIO INTENSET bit positions wrong** — CRCOK/CRCERROR/FRAMESTART
   were each off by 2 bits. The driver enables CRCOK+CRCERROR
   (mask 0x3000 = bits 12+13) so the IRQ fires on every frame; with
   the wrong positions the IRQ never fired and the receive queue
   starved.
3. **Cortex-M4F VFP not modelled** — Contiki's nrf52840 build emits
   VPUSH/VPOP/VLDR/VSTR/VMOV (and a smattering of arithmetic) for
   ABI marshalling around helper calls. Originally NOP'd; that
   silently corrupted call frames. Now `src/arm/arm_vfp.c` implements
   the FPv4-SP-D16 subset that real firmware uses (load/store, VMOV,
   VADD/VSUB/VMUL/VDIV/VABS/VNEG/VSQRT, VCMP, VCVT) and faults loudly
   on anything else.
4. **VFP encoding masks** — initial dispatcher masks were off by a
   bit in several places (caught by `-Wtautological-compare`); fixed.

What's missing for a full L5/L6 production run:
  - Move smoke runner into `arm-multinode-nrf` test_runner subcommand.
  - Add `firmware/nrf52840-dongle/PROVENANCE.md` entries for the udp
    pair (already there from the data-path commit).
  - Add a test_runner regression line so the build stays green.
  - Write a couple of M4 VFP correctness tests — the encoding tables
    were tricky and a regression suite makes future changes safer.

## Earlier checkpoints — multinode wiring

`test_mixed_multinode.c` now recognises `.nrf52840-dongle`, branches
`init_arm_node` on `arm_platform_cc2538` vs `arm_platform_nrf52840`,
and routes `mixed_deliver_rf_bytes` to the right chip per-platform.
For nRF nodes:

  - TX listener installed via `nrf_radio_set_tx_listener` (slot 0)
  - RX delivery via `nrf_radio_receive_byte` from the medium
  - `soc->ficr.deviceaddr0/1` and `soc->rng.prng_state` seeded with a
    per-node hash for unique IEEE EUI-64 + distinct CSMA backoff

Plus a fix to the RADIO state machine: `TASKS_TXEN` / `TASKS_RXEN` are
now accepted from any state except their own ramp-up/active state
(real Contiki driver triggers TXEN directly from RXIDLE after a STOP,
relying on hardware to handle the implicit transition). And a
correction to two SHORTS bit positions: `TXREADY_START` is bit 18
(was 19), `RXREADY_START` is bit 19 (was 20) — confirmed against
`nrf52840_bitfields.h`.

Two-node `udp-server.nrf52840-dongle + udp-client.nrf52840-dongle`
result:

  - Both nodes boot with **distinct IEEE link addresses**:
      Node 1 = `fe80::f6ce:3601:f1f4:203b`
      Node 2 = `fe80::f6ce:3602:e3e9:4176`
  - Radio state machine engages: `TASKS_RXEN → RXIDLE → SHORT
    RXREADY_START → TASKS_START → RX`, periodically interleaved with
    `TASKS_TXEN → TXIDLE → SHORT TXREADY_START → TASKS_START → TX
    (radio_emit_tx walks PACKETPTR, emits 4×0x00 + 0x7A SFD + frame
    + CRC) → TXIDLE`.
  - 1–6 sim seconds: clean. 7+ sim seconds: **segfault**, location
    not yet pinpointed.

Possibilities to investigate next session:
  - Some state path lets `radio_emit_tx` be called with a stale
    `PACKETPTR` that the (already added) bounds check doesn't catch.
  - SHORTS chain recursion overflowing the stack on a particular
    sequence (TX → END → END_DISABLE → DISABLED → DISABLED_TXEN → …).
  - Mismatch between the cc2538 `mixed_rf_tx_handler_radio` spectrum
    routing and the unregistered nRF radio slot.

Single-node `udp-client.nrf52840-dongle` runs cleanly for 30+ sim
seconds (unaffected — segfault is only in the two-node harness path).
CC2538 RPL-UDP regression: 26 direct RX in 60 s sim, 11.4× real-time
— no impact on existing platforms.

Once the segfault is fixed, the next obvious checkpoints are:
  - DIO arrival on Node 2 (RX path verification)
  - DAO + DODAG formation
  - First UDP packet delivered → "Sending request" / "Received reply"

## Earlier — data path + RNG + FICR landed

  - **PACKETPTR EasyDMA in RADIO** (TX): on TASKS_START in TX state,
    `radio_emit_tx()` walks the buffer at PACKETPTR (PHR + payload),
    computes the IEEE 802.15.4 CCITT-16 FCS, and emits the on-air
    byte sequence (4×0x00 preamble + 0x7A SFD + PHR + payload + CRC
    low + CRC high) through `soc->radio_tx_cb`.
  - **PACKETPTR EasyDMA in RADIO** (RX): `nrf_radio_receive_byte()`
    is a public API (called by the multinode harness) that frames
    incoming bytes through preamble→SFD→PHR→payload, writes accepted
    bytes into PACKETPTR-pointed RAM, and fires
    FRAMESTART/PAYLOAD/END/PHYEND/CRCOK on completion.
  - **RNG** (0x4000D000): TASKS_START → set EVENTS_VALRDY immediately;
    VALUE returns next byte from a per-node xorshift32 PRNG (seed
    initialised by SoC init, the harness can override).
  - **FICR** (0x10000000): DEVICEADDR0/1 + CODEPAGESIZE/CODESIZE +
    DEVICEADDRTYPE returned. The harness writes per-node DEVICEADDR
    values directly via the soc->ficr struct for unique IEEE EUI-64.

What remains:
  - **Multinode harness** (`test_mixed_multinode.c`): recognise the
    `.nrf52840-dongle` extension, init the platform, install the
    TX listener via `nrf_radio_set_tx_listener()`, fan-out
    `radio_medium` deliveries via `nrf_radio_receive_byte()`, set
    per-node `soc->ficr.deviceaddr0/1`.
  - **Debug RPL-UDP convergence**: per past porting experience, this
    is the unpredictable part — frame format mismatches, IRQ priority
    issues, timing dependencies typically surface here.

## Why this port, why this board

- **Why nRF52840:** large installed base (Particle Argon, Adafruit
  Feather, Nordic DK + Dongle, many third-party boards). Multi-protocol
  radio (802.15.4 + BLE) is a unique capability vs the existing
  CC2420/CC1200/CC2538 lineup. Single-chip — no off-SoC radio to
  emulate.
- **Why the Dongle (PCA10059) and not the DK (PCA10056):** smaller
  surface. No SEGGER, no LCD, no SD card, no QSPI flash, fewer LEDs
  and buttons. Same SoC = same heavy lifting on the radio + PPI;
  fewer board peripherals to model around it. Trade-off documented in
  [`SPEC.md`](SPEC.md): default console is USB-CDC, which we'll
  side-step in L0–L4 by building with UART console on the dongle's
  exposed pads.

## Scope split

The port has two independent risk areas. Both must land for L6, but
they can be developed in parallel after L0–L1 is green.

### Track A — Cortex-M4F support (CPU-side)

The existing ARM interpreter started Cortex-M3-only. M4 adds:
- DSP halfword multiply (SMUL{B,T}{B,T}, SMLA{B,T}{B,T}, SMULW{B,T},
  SMLAW{B,T}, SMLAL{B,T}{B,T}) — **landed**, 15 correctness tests
- DSP SIMD halfword/byte (SADD16/SSUB16/SADD8/SSUB8 + signed/unsigned
  + halving/saturating variants) — pending
- DSP saturating (QADD/QSUB/QDADD/QDSUB) — pending
- DSP packing (PKHBT/PKHTB) and bitfield variants — pending
- DSP misc (SMLAD/SMLSD/SMMUL/SMMLA/SMMLS, USAD8) — pending
- VFPv4 single-precision FPU — pending; not expected on the
  networking path
- Bit-band aliases (also exist on M3, may differ in detail) — TODO
- M4-specific MPU bits — TODO; Contiki rarely enables MPU

Strategy: implement DSP families upfront with correctness tests; add
FPU on-demand if firmware traps. Each family extends the existing
multiply/shift handler chain in `arm_cpu.c`. APSR_Q (sticky saturation
flag) is now defined.

### Track B — nRF52840 peripherals

In dependency order (each its own commit):

1. **Boot path**: `arm_config.c` entry, vector table, NVMC stub,
   POWER stub. Firmware halts at first peripheral access if these
   aren't there.
2. **Tick source**: CLOCK + RTC. Without RTC, Contiki's main loop
   never advances.
3. **Output**: GPIO + GPIOTE + UARTE0. L2 banner depends on this.
4. **RADIO** in 802.15.4 mode + EasyDMA. L4 onwards.
5. **PPI**. Required for TSCH. Not required for plain CSMA.
6. **Multinode glue**: per-node IEEE address patching (FICR.DEVICEADDR),
   register the radio with the medium, fan-out byte delivery.

The RADIO peripheral is the dominant risk — it has hardware shorts
(RXREADY → START → END → DISABLE chains), EasyDMA frame buffers, and
participates in PPI-driven TSCH timing. Expect this single item to
take as long as everything else combined, mirroring the CC1200
experience on Firefly.

## Out of scope

Explicitly **not** part of this port:

- **USB-CDC console.** The factory dongle uses USB-CDC for serial.
  Modeling enough USBD for CDC enumeration is real work
  (endpoint config, SETUP transfers, EasyDMA → bulk-IN endpoint).
  L0–L4 will run firmware built with UART-console-on-pads.
  USBD modeling is a separate follow-on, tracked as its own SPEC
  appendix if we get there.
- **BLE.** The nRF52840 radio is multi-protocol. We model only
  802.15.4 mode. BLE timing, LL state machine, advertising channels
  — not in scope. (csim's medium model is single-protocol.)
- **SoftDevice.** Nordic's protocol stack. Contiki on nRF runs
  bare-metal, no SoftDevice.
- **Bootloader / DFU.** csim loads ELFs directly into flash; the
  Open Bootloader region (the top of flash on factory dongles) is
  not exercised.
- **Cryptographic accelerators (CCM/AAR/ECB).** Not needed for
  RPL/UDP without link-layer security. If TSCH-encrypted firmware
  is requested, revisit.
- **External chips.** None on the Dongle. If a future board (e.g.
  Particle Argon = nRF52840 + ESP32 coprocessor) is requested, it
  becomes a separate SPEC.

## Risks

1. **PPI accuracy.** TSCH on nRF uses PPI for sub-microsecond timing
   between RADIO events and TIMER captures. csim's event queue is
   integer-ns; PPI dispatch must happen *before* the next CPU
   instruction step or TSCH slips. Plan: write a small PPI unit test
   harness early to pin the timing semantics before TSCH firmware
   sees the model.
2. **EasyDMA semantics.** Real hardware reads/writes the
   `*PACKETPTR`-pointed RAM at peripheral-controlled times. Firmware
   that mutates the buffer mid-transaction has implementation-defined
   behavior. csim should mirror "buffer is owned by the peripheral
   between START and END" or expose a clear violation diagnostic.
3. **M4-only opcodes encountered late.** If a single `VLDR` or
   `SMUL` shows up deep in a library function, debugging is annoying
   because the trap is far from the cause. Mitigation: surface
   `undef` with full PC + insn dump from day one.
4. **Contiki-NG nrf52840 dongle build path may not be vanilla.** The
   DK is the well-tested target; the Dongle has historically required
   board-specific patches in Contiki-NG. Expect to spend the first
   day on the build path before any csim work. If the build is
   broken upstream, file PRs (Firefly precedent — fix Contiki-NG, not
   csim).

## Files

- [`SPEC.md`](SPEC.md) — device contract, definition of done
- [`STATUS.md`](STATUS.md) — this file
- [`../../docs/porting-a-device.md`](../../docs/porting-a-device.md) —
  the L0–L6 process this port follows

(`HARDWARE-TEST.md`, `DATASHEET-FINDINGS.md`, `archive/` are left to
be created if and when the port reaches the corresponding stage.
Don't pre-create empty docs.)

## L3 milestone — Contiki main loop alive on RTC IRQ

After the hello-world banner, the firmware was parking at PC=0x4b30
which is the `dmb sy` immediately after `wfi` in `platform_idle`:

```
4b1e: bl  int_master_read_and_disable    @ disable IRQs
4b28: bl  process_nevents
4b2c: cbnz r0, 4b30                       @ skip wfi if events queued
4b2e: wfi                                 @ sleep until interrupt
4b30: dmb sy                              @ ← was stuck here
4b3a: b.w int_master_status_set           @ re-enable IRQs (takes pending IRQ)
```

Two pieces unblock the loop:

  - **RTC0** stub (0x4000B000) — 24-bit counter + TICK / OVRFLW /
    COMPARE[0..3] events + INTENSET/INTENCLR + PRESCALER + CC. The
    counter advances via a recurring TICK event scheduled on the ARM
    event queue; on each tick we latch EVENTS_TICK and (if INTENSET.TICK
    is set) raise the RTC0 NVIC IRQ (vector 11). Counter reads compute
    a value lazily from `(cycles - anchor_cycles) / tick_period_cycles`.
  - **WFI handling fix in arm_cpu.c** — Cortex-M `wfi` wakes on any
    pending interrupt regardless of PRIMASK. The previous code only
    cleared `cpu_off` if `arm_nvic_check_pending` actually took the
    exception, which it skips when PRIMASK=1. Result: with the
    `disable IRQ → wfi → re-enable` idle pattern, the CPU never woke.
    Fix: clear `cpu_off` on any pending IRQ; the IRQ fires later when
    PRIMASK clears. Affects every ARM platform; CC2538 just doesn't
    use this idle pattern.

The firmware is now alive end-to-end: PC oscillates between
`platform_idle` (0x4b30 wfi), the RTC IRQ entry (0x4d62), some etimer
pump path (0xbd58), and back. ~70× real-time speed. The hello-world
example prints once and idles forever, exactly as expected.

For the architectural reference, the boundary nRF peripherals plug
into:

```c
/* arm_platform.h */
typedef struct arm_soc_ops {
    const char *name;
    void (*init)(struct arm_platform *plat);
    void (*destroy)(struct arm_platform *plat);
    void (*set_console)(struct arm_platform *plat,
                        arm_uart_tx_callback cb, void *user_data);
} arm_soc_ops_t;

typedef struct arm_platform {
    arm_cpu_t           cpu;          /* common */
    arm_nvic_t          nvic;         /* common */
    arm_systick_t       systick;      /* common */
    sim_host_t          host;         /* populated by soc_ops->init */
    void               *soc;          /* per-SoC state */
    const arm_platform_config_t *config;
} arm_platform_t;
```

The L0 path is now unblocked. Order:

1. **`include/arm/nrf52840_soc.h` + `src/arm/nrf52840_soc.c`** — the
   SoC bundle. Initially: empty struct, `nrf52840_soc_ops` with init
   that does nothing useful and a stub `set_console` (writes to
   stderr). Mirrors `cc2538_soc.{c,h}` shape so the pattern stays
   uniform.
2. **`platform_nrf52840_dongle`** entry in `arm_platform.c`'s
   registry, pointing at `nrf52840_config` + `nrf52840_soc_ops`.
3. **Pick the first nrf ELF**: the simplest Contiki-NG nrf52840
   firmware that builds clean. Run it through `arm_load_elf` →
   `arm_cpu_reset` → `arm_step` and see what address the first
   peripheral access lands on. That tells us which peripheral to
   model first (almost certainly CLOCK or NVMC).
4. Iterate: stub the next peripheral, step further, repeat until we
   reach `main()`. That's L1.

Same empirical loop the Sky and Firefly ports used.
