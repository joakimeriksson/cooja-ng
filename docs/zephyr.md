# Running Zephyr OS on Cooja-NG

Cooja-NG boots **Zephyr OS** on its emulated nRF52840, in addition to
Contiki-NG. The bundled example prints the classic banner:

```
*** Booting Zephyr OS build v4.4.0 ... ***
Hello World! nrf52840dk/nrf52840
```

## Run the bundled example

```sh
./build/test_runner nrf52840-dk-multinode \
    firmware/nrf52840-dk/zephyr-hello-world.nrf52840-dk -t 2000 -n 1
```

The firmware is a pre-built `samples/hello_world` for `nrf52840dk/nrf52840`.

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
- The `ARM_MMIO_TRACE=1` diagnostic (logs the first access to each unmapped
  peripheral page) is how the missing registers were found — useful for any
  future firmware bring-up.

## Limitations

- Single-node console demo only; Zephyr's 802.15.4 networking on csim's radio
  medium is untried.
