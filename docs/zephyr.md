# Running Zephyr OS on Cooja-NG

Cooja-NG boots **Zephyr OS** on its emulated nRF52840, in addition to
Contiki-NG. The bundled example prints the classic banner:

```
*** Booting Zephyr OS build v4.4.0 ... ***
Hello World! nrf52840dk/nrf52840
```

## Run the bundled examples

```sh
# Single-thread hello_world
./build/test_runner nrf52840-dk-multinode \
    firmware/nrf52840-dk/zephyr-hello-world.nrf52840-dk -t 2000 -n 1

# Two threads + timers (samples/synchronization) — they alternate every ~565 ms
./build/test_runner nrf52840-dk-multinode \
    firmware/nrf52840-dk/zephyr-synchronization.nrf52840-dk -t 3000 -n 1

# 802.15.4 networking: stock echo_server <-> echo_client (UDP over 6LoWPAN/IPv6/RPL)
# Server logs "Received and replied"; full two-node round-trip, 0 timeouts.
./build/test_runner nrf52840-dk-multinode \
    firmware/nrf52840-dk/zephyr-echo-server.nrf52840-dk \
    firmware/nrf52840-dk/zephyr-echo-client.nrf52840-dk -t 40000
```

Pre-built Zephyr samples for `nrf52840dk/nrf52840`.  The synchronization sample
exercises the system timer (`k_msleep`) **and** thread context switching
(PendSV / MSP↔PSP banking).  The echo samples (`samples/net/sockets/echo_*`
with only the sample's own `overlay-802154.conf`) exercise the **on-chip
802.15.4 radio + the full 6LoWPAN/IPv6/RPL stack** end-to-end.

## How it was built

No `sudo` and no Zephyr SDK are needed — a Python venv plus the system
`arm-none-eabi-gcc` (`gnuarmemb` toolchain variant) is enough:

```sh
python3 -m venv --without-pip ~/zephyr-venv          # PEP 668: build pip inside a venv
~/zephyr-venv/bin/python get-pip.py                  # from bootstrap.pypa.io
~/zephyr-venv/bin/pip install west cmake ninja pyelftools
~/zephyr-venv/bin/west init ~/zephyrproject && cd ~/zephyrproject && west update
~/zephyr-venv/bin/pip install -r zephyr/scripts/requirements-base.txt

export PATH=~/zephyr-venv/bin:$PATH
export ZEPHYR_TOOLCHAIN_VARIANT=gnuarmemb GNUARMEMB_TOOLCHAIN_PATH=/usr
west build -b nrf52840dk/nrf52840 zephyr/samples/hello_world
```

No board overlay or config changes are needed — the stock image (default
**UARTE** EasyDMA console) runs as-is. Then run the resulting
`build/zephyr/zephyr.elf` (renamed to a `*.nrf52840-dk` path so the board
registry picks the platform).

## What the emulator provides

Booting Zephyr needed three additions to `src/arm/nrf52840_soc.c` beyond what
Contiki used (see commit history):

- **CLOCK** `HFCLKSTAT`/`LFCLKSTAT`/`LFCLKSRC` — Zephyr's `lfclk_spinwait()`
  polls `LFCLKSTAT.STATE`; csim previously modelled only the `*_STARTED`
  events, so the spin never exited (the actual boot blocker).
- **RTC1** (`0x40011000`, IRQ 17) — Zephyr's `nrf_rtc_timer` system clock
  (Contiki uses RTC0).
- **UARTE EasyDMA TX** (`TASKS_STARTTX` + `TXD.PTR`/`TXD.MAXCNT` →
  `EVENTS_ENDTX`/`TXSTOPPED`) — the default Zephyr console. csim's transfer is
  instantaneous, so it latches `TXSTOPPED` immediately (what the driver's
  `ENDTX→STOPTX` PPI link would produce; csim doesn't model PPI). The legacy
  non-DMA UART (Contiki's path, and Zephyr with a `nordic,nrf-uart` overlay)
  also still works.
- **MRS IPSR** (SYSm=5) now returns just the exception number, so Zephyr's
  `_isr_wrapper` dispatches the right ISR — without this every interrupt
  mis-dispatched and the tickless RTC1 clock never advanced.
- **Banked MSP/PSP** across exception entry/return + `CONTROL.SPSEL`, so the
  PendSV context switch works and threads actually run (Contiki used MSP only,
  so it never exercised this).

For the 802.15.4 echo, two more were needed:

- **TEMP sensor** (`0x4000C000`) — `nrf_802154` periodically measures die
  temperature for radio calibration and *blocks the system work-queue* on
  `device_sync_sem` until the `DATARDY` interrupt. With no TEMP model that read
  hung forever, so DAD never completed (`Network init failed -116`). csim
  completes it instantly: `TASKS_START` → `EVENTS_DATARDY` + IRQ, `TEMP` ≈ 25 °C.
- **UARTE EasyDMA RX** (`STARTRX`/`RXD.PTR`/`ENDRX`) — the input half of the
  console, for shell-driven samples (shared with the RIOT path, `docs/riot.md`).

### Bring-up debugging toolkit

These env-gated scopes (behaviour-neutral; in `src/arm/`) are what made the
above tractable — reach for them when porting a new OS or peripheral:

| Env var | What it shows |
|---|---|
| `ARM_MMIO_TRACE=1` | first access to each **unmapped peripheral page** (find missing peripherals) |
| `ARM_EXC_TRACE=1` | **exception entry/return** (exc/EXC_RETURN, SP, PC) — ISR dispatch + context switches |
| `NRF_RX_TRACE=1` | nRF radio RX framing |

Plus the end-of-run per-node `PC=0x…` (the ARM `program_counter` op) →
`arm-none-eabi-addr2line -e <elf> <pc>` resolves a boot spin to a source line.
The per-mote GDB stub is also available for interactive inspection.

## Limitations

- The stock echo samples have **no log backend in the 802.15.4 build** (their
  `printk`/LOG output is dropped — on real hardware too, in that config), so the
  echo server's own confirmation is what's visible; verify with the
  `Received and replied` log, `Total RF bytes`, and 0 timeouts rather than a
  boot banner.
- OpenThread (Thread/mesh on top of 802.15.4) is a further step — see
  `docs/design/zephyr-802154-plan.md` (needs the QSPI settings backend).
