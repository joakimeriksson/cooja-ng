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
west build -b nrf52840dk/nrf52840 zephyr/samples/hello_world \
    -- -DDTC_OVERLAY_FILE=legacy-uart.overlay
```

The overlay forces the **non-DMA legacy UART** (`uart_nrfx_uart`), whose
register window csim models:

```dts
/* legacy-uart.overlay */
&uart0 { compatible = "nordic,nrf-uart"; };
```

Then run the resulting `build/zephyr/zephyr.elf` (renamed to a
`*.nrf52840-dk` path so the board registry picks the platform).

## What the emulator provides

Booting Zephyr needed three additions to `src/arm/nrf52840_soc.c` beyond what
Contiki used (see commit history):

- **CLOCK** `HFCLKSTAT`/`LFCLKSTAT`/`LFCLKSRC` — Zephyr's `lfclk_spinwait()`
  polls `LFCLKSTAT.STATE`; csim previously modelled only the `*_STARTED`
  events, so the spin never exited (the actual boot blocker).
- **RTC1** (`0x40011000`, IRQ 17) — Zephyr's `nrf_rtc_timer` system clock
  (Contiki uses RTC0).
- The `ARM_MMIO_TRACE=1` diagnostic (logs the first access to each unmapped
  peripheral page) is how the missing registers were found — useful for any
  future firmware bring-up.

## Limitations

- **Console needs the legacy-UART overlay.** The default `nrf52840dk` console
  is **UARTE** (EasyDMA), which csim does not model yet — an unmodified Zephyr
  image boots but prints nothing. Modelling the UARTE TX path
  (`TASKS_STARTTX`/`TXD.PTR`/`TXD.MAXCNT` → `EVENTS_ENDTX`) would let stock
  images print without the overlay.
- Single-node console demo only; Zephyr's 802.15.4 networking on csim's radio
  medium is untried.
