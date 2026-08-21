# Running Zephyr firmware on the nRF52840 emulator (bring-up notes)

**Status:** spike done 2026-06-15, then **reverted** to keep the tree clean for
the in-progress refactor. This doc + `zephyr-bringup-uarte.patch` capture
everything so the work can be resumed. Nothing here is currently in the build.

## Goal

Demonstrate that csim's instruction-level emulation is OS-agnostic by booting an
**unmodified Zephyr** image on the existing nRF52840 emulator — the same emulator
that runs Contiki-NG firmware. This is the headline experiment for the "multi-OS
cross-level simulator" framing of the paper: a second RTOS, with its own drivers,
running unmodified with no OS-specific simulator code.

## What already works (no emulator changes)

An unmodified Zephyr v4.2 `hello_world` built for `nrf52840dk/nrf52840`:

- **loads and executes** on the nRF52840 ARM Cortex-M4F core,
- runs to `main`, sustains the correct 64 MHz HFCLK, no faults.

So the CPU core, reset/boot, vector table, and clock handle a foreign RTOS as-is.
The emulator selects the platform purely by the firmware filename extension
(`*.nrf52840-dk` → the `nrf52840-dk` platform); a Zephyr `zephyr.elf` is a
standard ELF32 ARM image and loads exactly like a Contiki one.

## Build environment (set up on this machine, 2026-06-15)

- Zephyr workspace: `/Users/joakimeriksson/work/zephyrproject` (Zephyr v4.2,
  `modules/hal/nordic` present, board `nrf52840dk/nrf52840`).
- Python venv: `/Users/joakimeriksson/work/zephyrproject/.venv` (python3.12;
  `west` + Zephyr `scripts/requirements.txt`). System python3.9 is too old for
  Zephyr 4.2 deps — use 3.10+.
- Host tools via Homebrew: `cmake ninja dtc` (+ `coreutils` for `gtimeout`).
- Toolchain: **gnuarmemb** using the existing Homebrew `arm-none-eabi-gcc` — no
  1 GB Zephyr SDK needed.

### Build + run

```sh
# Build Zephyr hello_world for the nRF52840 DK
cd /Users/joakimeriksson/work/zephyrproject
ZEPHYR_TOOLCHAIN_VARIANT=gnuarmemb GNUARMEMB_TOOLCHAIN_PATH=/opt/homebrew \
  .venv/bin/west build -p always -b nrf52840dk/nrf52840 \
  zephyr/samples/hello_world -d /tmp/zephyr-hello-nrf52840

# Hand it to csim (board chosen by the .nrf52840-dk extension)
cp /tmp/zephyr-hello-nrf52840/zephyr/zephyr.elf \
   <csim>/firmware/zephyr/hello-world-zephyr.nrf52840-dk
cd <csim>
./build/test_runner nrf52840-dk-multinode \
   firmware/zephyr/hello-world-zephyr.nrf52840-dk -t 2000
```

## The gap ladder (what Zephyr exercises that Contiki-NG never did)

Instruction-level emulation is OS-agnostic, so "adding Zephyr" is really
*emulator completeness*: filling peripheral/register paths Contiki-NG's drivers
never touched. Found so far, in boot order:

### 1. UARTE EasyDMA console TX — SOLVED (in the patch, reverted)

Zephyr's console uses the **UARTE EasyDMA** path (`CONFIG_UART_NRFX_UARTE=y`):
set `TXD.PTR` (0x544) + `TXD.MAXCNT` (0x548), trigger `TASKS_STARTTX` (0x008),
poll `EVENTS_ENDTX` (0x120). csim previously modelled only Contiki's legacy
byte-at-a-time `TXD` (0x51C), so Zephyr output went nowhere.

Fix (see `zephyr-bringup-uarte.patch`): extend the existing UART window at
`0x40002000` in `src/arm/nrf52840_soc.c` / `include/arm/nrf52840_soc.h` to model
the EasyDMA TX cluster — on `STARTTX`, stream `MAXCNT` bytes from SRAM at
`TXD.PTR` via `arm_read8` through the console callback, then latch
`ENDTX`/`TXSTARTED`. Pattern mirrors the existing nRF54L15 UARTE in
`src/arm/nrf54l15_soc.c`. Verified exercised by Zephyr (`ENABLE=8`, `STARTTX`
observed) and **no regression** to the Contiki-NG legacy path (still prints,
`uart_bytes=886`). `NRF_UART_TRACE=1` dumps UART-window writes.

NOTE: `TXD.MAXCNT` is 0 during the driver's init-time prime; real per-character
TX (`MAXCNT=1`) only happens once `printk` runs — which it doesn't yet, because:

### 2. RTC1 system timer — NEXT BLOCKER (not started)

Zephyr never reaches `printk`; it busy-loops in the kernel/scheduler. Root cause:
Zephyr's system timer is **RTC1 @ 0x40011000** (`CONFIG_NRF_RTC_TIMER=y`,
devicetree `rtc1: rtc@40011000`, tickless kernel). csim models **RTC0 @
0x4000B000** only. With no RTC1 COMPARE interrupt, the tickless kernel never
schedules the main thread, so the banner is never flushed.

To do: model RTC1 (separate 0x40011000 window). The existing RTC0 model in the
same file is a starting template, **but** Zephyr's `nrf_rtc_timer` driver is
COMPARE/CC-and-IRQ driven (not the simple periodic TICK that Contiki's RTC0 uses),
so the COMPARE-event + NVIC path is the part that needs real work.

### 3. Minor unmapped peripherals (harmless so far)

A throwaway `ARM_IO_TRACE` (2-line env-gated `fprintf` in the unmapped-IO
fallback of `arm_read32`/`arm_write32` in `src/arm/arm_cpu.c`; reverted) showed
Zephyr also pokes, with low/non-spinning counts: `0x50000xxx` (GPIO P0 pin cfg),
`0x4001exxx`/`0x4001fxxx` (NVMC / GPIOTE-ish), `0x4000617c`. None are spin-waited
on, so they aren't blockers yet — revisit after RTC1. Re-add that `ARM_IO_TRACE`
fprintf when resuming; it's the fastest way to find the next missing peripheral.

## Resume checklist

1. Re-apply the UARTE patch: `git apply docs/zephyr-bringup-uarte.patch`.
2. Implement RTC1 @ 0x40011000 (COMPARE/CC + RTC1 NVIC IRQ).
3. Re-add the `ARM_IO_TRACE` unmapped-IO fprintf to find the next gap.
4. Rebuild Zephyr `hello_world` per above; success = "Hello World! nrf52840dk"
   in the per-node console and `uart_bytes > 0`.
5. Then tiers T2 (802.15.4 link via the nRF RADIO + Nordic `nrf_802154` driver)
   and T3 (Thread / OpenThread + Contiki-NG↔Zephyr interop on one medium).
