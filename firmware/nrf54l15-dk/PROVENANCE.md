## hello-world.nrf54l15-dk

- **Source**: contiki-ng commit `f15d82e66b9ea205b5cddbf38c286b9b4649d14d`
- **Source path**: `examples/hello-world` (file: `hello-world.c`)
- **TARGET**: `nrf`
- **BOARD**: `nrf54l15/dk`
- **Toolchain**: Docker contiker/contiki-ng:latest
- **Built**: 2026-05-14T08:29:31Z by Joakim Eriksson
- **Build command**: `tools/build-device-firmware.sh --target nrf --board nrf54l15/dk --example examples/hello-world --output firmware/nrf54l15-dk/hello-world.nrf54l15-dk`

## udp-server.nrf54l15-dk

- **Source**: contiki-ng commit `f15d82e66b9ea205b5cddbf38c286b9b4649d14d`
- **Source path**: `examples/rpl-udp` (file: `udp-server.c`)
- **TARGET**: `nrf`
- **BOARD**: `nrf54l15/dk`
- **Toolchain**: Docker contiker/contiki-ng:latest
- **Built**: 2026-05-14T10:24:09Z by Joakim Eriksson
- **Build command**: `tools/build-device-firmware.sh --target nrf --board nrf54l15/dk --example examples/rpl-udp --output firmware/nrf54l15-dk/udp-server.nrf54l15-dk --source-file udp-server`

## udp-client.nrf54l15-dk

- **Source**: contiki-ng commit `f15d82e66b9ea205b5cddbf38c286b9b4649d14d`
- **Source path**: `examples/rpl-udp` (file: `udp-client.c`)
- **TARGET**: `nrf`
- **BOARD**: `nrf54l15/dk`
- **Toolchain**: Docker contiker/contiki-ng:latest
- **Built**: 2026-05-14T10:24:25Z by Joakim Eriksson
- **Build command**: `tools/build-device-firmware.sh --target nrf --board nrf54l15/dk --example examples/rpl-udp --output firmware/nrf54l15-dk/udp-client.nrf54l15-dk --source-file udp-client`

## udp-server-instr.nrf54l15-dk

- **Source**: contiki-ng commit `f15d82e66b9ea205b5cddbf38c286b9b4649d14d`
- **Source path**: `examples/rpl-udp` (file: `udp-server.c`, instrumented)
- **TARGET**: `nrf`
- **BOARD**: `nrf54l15/dk`
- **Toolchain**: host arm-none-eabi-gcc (built with `--local`)
- **Built**: 2026-05-15 by Joakim Eriksson during HW validation session
- **Build command**: `tools/build-device-firmware.sh --target nrf --board nrf54l15/dk --example examples/rpl-udp --source-file udp-server --output firmware/nrf54l15-dk/udp-server-instr.nrf54l15-dk --no-provenance --local`
- **Modification**: a `tick_test_process` is autostarted alongside `udp_server_process`. At boot it prints `[CFG] RTIMER_SECOND=… CLOCK_SECOND=… boot_rt=…`, then once per `etimer_set(CLOCK_SECOND)` prints `[T] uptime=Ns clock_time=N rtimer_now=N rt_delta=N`. Used as a ground-truth probe for GRTC tick rate — on real PCA10156 each `[T]` line emerges every 1 wall-clock second and `rt_delta` advances by ~1,000,000 (= 1 MHz GRTC SYSCOUNTER). The csim emulator's GRTC must reproduce both timings.

## shell.nrf54l15-dk

- **Source**: contiki-ng commit `14a3f574a37228cae84f50b7891acf1488604473`
- **Source path**: `examples/libs/shell` (file: `example.c`)
- **TARGET**: `nrf`
- **BOARD**: `nrf54l15/dk`
- **Toolchain**: host
- **Built**: 2026-09-05T22:33:17Z by Niclas Finne
- **Build command**: `tools/build-device-firmware.sh --target nrf --board nrf54l15/dk --example examples/libs/shell --output firmware/nrf54l15-dk/shell.nrf54l15-dk`

## spi-flash.nrf54l15-dk

- **Source**: contiki-ng commit `55e7ef6c889ed23a9d5c1da1a2e04395c3ee6c77`
- **Source path**: `examples/platform-specific/nrf/spi-flash` (file: `spi-flash.c`)
- **TARGET**: `nrf`
- **BOARD**: `nrf54l15/dk`
- **Branch**: `feature/nrf-spi-driver` (joakimeriksson/contiki-ng, PR contiki-ng/contiki-ng#3234) — not on `develop` yet
- **Make flags**: none beyond the example's own Makefile (`NRF_WITH_SPI=1`, `NRF_SPI_INSTANCES=00`)
- **Toolchain**: host `arm-none-eabi-gcc 15.2.1` (Arm GNU Toolchain 15.2.Rel1), built with `--local`
- **Built**: 2026-09-06T08:29:17Z by Joakim Eriksson
- **Build command**: `tools/build-device-firmware.sh --local --contiki-dir ~/work/contiki-ng --target nrf --board nrf54l15/dk --example examples/platform-specific/nrf/spi-flash --output firmware/nrf54l15-dk/spi-flash.nrf54l15-dk`
- **What it does**: every 5 s reads the on-board MX25R6435F over SPIM00 (SCK P2.01, MOSI P2.02, MISO P2.04, CS P2.05 as GPIO; 8 MHz mode 0) through `arch/cpu/nrf/dev/spi-arch.c`: JEDEC ID from a RAM and a flash-resident command buffer, the 4-byte SFDP signature, a 256-byte staged SFDP read, and a bit-rate sweep (1/2/4/8/16/20/32 MHz) that re-initialises the SPIM seven times.
- **Hardware oracle** (real PCA10156, 2026-09-06 — the emulation must print the same):

  ```
  nRF SPI flash test
    controller 0, 8000000 Hz, mode 00
    SCK P2.01  MOSI P2.02  MISO P2.04  CS P2.05
    JEDEC ID: c2 28 17  OK (MX25R6435F)
    flash-buf: c2 28 17  OK (DMA staging)
    SFDP:     53 46 44 50  OK
    long SFDP: 53 46 44 50 ... ff ff  OK (256 B staged)
    bit rates (requested, driver rounds down): 1:ok 2:ok 4:ok 8:ok 16:ok 20:ok 32:ok (MHz)
  SPI OK
  ```
- **Test**: `configs/test-spi-flash-nrf54l15-dk.json`

