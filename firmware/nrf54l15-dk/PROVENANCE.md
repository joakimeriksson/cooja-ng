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

