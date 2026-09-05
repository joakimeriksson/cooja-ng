## hello-world.nrf54l15-xiao

- **Source**: contiki-ng commit `f15d82e66b9ea205b5cddbf38c286b9b4649d14d`
- **Source path**: `examples/hello-world` (file: `hello-world.c`)
- **TARGET**: `nrf`
- **BOARD**: `nrf54l15/xiao`
- **Toolchain**: Docker contiker/contiki-ng:latest
- **Built**: 2026-05-14T10:32:08Z by Joakim Eriksson
- **Build command**: `tools/build-device-firmware.sh --target nrf --board nrf54l15/xiao --example examples/hello-world --output firmware/nrf54l15-xiao/hello-world.nrf54l15-xiao`

## udp-server.nrf54l15-xiao

- **Source**: contiki-ng commit `f15d82e66b9ea205b5cddbf38c286b9b4649d14d`
- **Source path**: `examples/rpl-udp` (file: `udp-server.c`)
- **TARGET**: `nrf`
- **BOARD**: `nrf54l15/xiao`
- **Toolchain**: Docker contiker/contiki-ng:latest
- **Built**: 2026-05-14T10:32:23Z by Joakim Eriksson
- **Build command**: `tools/build-device-firmware.sh --target nrf --board nrf54l15/xiao --example examples/rpl-udp --output firmware/nrf54l15-xiao/udp-server.nrf54l15-xiao --source-file udp-server`

## udp-client.nrf54l15-xiao

- **Source**: contiki-ng commit `f15d82e66b9ea205b5cddbf38c286b9b4649d14d`
- **Source path**: `examples/rpl-udp` (file: `udp-client.c`)
- **TARGET**: `nrf`
- **BOARD**: `nrf54l15/xiao`
- **Toolchain**: Docker contiker/contiki-ng:latest
- **Built**: 2026-05-14T10:32:38Z by Joakim Eriksson
- **Build command**: `tools/build-device-firmware.sh --target nrf --board nrf54l15/xiao --example examples/rpl-udp --output firmware/nrf54l15-xiao/udp-client.nrf54l15-xiao --source-file udp-client`

## secure-world-example.nrf54l15-xiao

- **Source**: contiki-ng commit `ec4aafd64df1b2da81483ac9c6ddd665410fda41`
- **Branch**: contiki-ng `nrf54l15-trustzone` (TrustZone-M support is not in upstream Contiki-NG yet)
- **Source path**: `examples/platform-specific/nrf/trustzone/secure-world` (file: `secure-world-example.c`)
- **TARGET**: `nrf`
- **BOARD**: `nrf54l15/xiao`
- **Toolchain**: host
- **Built**: 2026-09-05T22:34:09Z by Niclas Finne
- **Build command**: `tools/build-device-firmware.sh --target nrf --board nrf54l15/xiao --example examples/platform-specific/nrf/trustzone/secure-world --output firmware/nrf54l15-xiao/secure-world-example.nrf54l15-xiao --local`

## normal-world-example.nrf54l15-xiao

- **Source**: contiki-ng commit `ec4aafd64df1b2da81483ac9c6ddd665410fda41`
- **Branch**: contiki-ng `nrf54l15-trustzone` (TrustZone-M support is not in upstream Contiki-NG yet)
- **Source path**: `examples/platform-specific/nrf/trustzone/normal-world` (file: `normal-world-example.c`)
- **TARGET**: `nrf`
- **BOARD**: `nrf54l15/xiao`
- **Toolchain**: host
- **Built**: 2026-09-05T22:34:29Z by Niclas Finne
- **Build command**: `tools/build-device-firmware.sh --target nrf --board nrf54l15/xiao --example examples/platform-specific/nrf/trustzone/normal-world --output firmware/nrf54l15-xiao/normal-world-example.nrf54l15-xiao --local`

## normal-world-hang.nrf54l15-xiao

- **Source**: contiki-ng commit `ec4aafd64df1b2da81483ac9c6ddd665410fda41`
- **Branch**: contiki-ng `nrf54l15-trustzone` (TrustZone-M support is not in upstream Contiki-NG yet)
- **Source path**: `examples/platform-specific/nrf/trustzone/normal-world` (file: `normal-world-example.c`)
- **TARGET**: `nrf`
- **BOARD**: `nrf54l15/xiao`
- **Make args**: `DEFINES=NORMAL_WORLD_CONF_HANG_AT_ITERATION=2`
- **Toolchain**: host
- **Built**: 2026-09-05T22:34:45Z by Niclas Finne
- **Build command**: `tools/build-device-firmware.sh --target nrf --board nrf54l15/xiao --example examples/platform-specific/nrf/trustzone/normal-world --output firmware/nrf54l15-xiao/normal-world-hang.nrf54l15-xiao --make-args "DEFINES=NORMAL_WORLD_CONF_HANG_AT_ITERATION=2" --local`

## udp-server-ns.nrf54l15-xiao

- **Source**: contiki-ng commit `ec4aafd64df1b2da81483ac9c6ddd665410fda41`
- **Branch**: contiki-ng `nrf54l15-trustzone` (TrustZone-M support is not in upstream Contiki-NG yet)
- **Source path**: `examples/rpl-udp` (file: `udp-server.c`)
- **TARGET**: `nrf`
- **BOARD**: `nrf54l15/xiao`
- **Make args**: `TRUSTZONE=1`
- **Toolchain**: host
- **Built**: 2026-09-05T22:35:02Z by Niclas Finne
- **Build command**: `tools/build-device-firmware.sh --target nrf --board nrf54l15/xiao --example examples/rpl-udp --output firmware/nrf54l15-xiao/udp-server-ns.nrf54l15-xiao --source-file udp-server --make-args "TRUSTZONE=1" --local`

## udp-client-ns.nrf54l15-xiao

- **Source**: contiki-ng commit `ec4aafd64df1b2da81483ac9c6ddd665410fda41`
- **Branch**: contiki-ng `nrf54l15-trustzone` (TrustZone-M support is not in upstream Contiki-NG yet)
- **Source path**: `examples/rpl-udp` (file: `udp-client.c`)
- **TARGET**: `nrf`
- **BOARD**: `nrf54l15/xiao`
- **Make args**: `TRUSTZONE=1`
- **Toolchain**: host
- **Built**: 2026-09-05T22:35:19Z by Niclas Finne
- **Build command**: `tools/build-device-firmware.sh --target nrf --board nrf54l15/xiao --example examples/rpl-udp --output firmware/nrf54l15-xiao/udp-client-ns.nrf54l15-xiao --source-file udp-client --make-args "TRUSTZONE=1" --local`

## normal-world-fullplat.nrf54l15-xiao

- **Source**: contiki-ng commit `ec4aafd64df1b2da81483ac9c6ddd665410fda41`
- **Branch**: contiki-ng `nrf54l15-trustzone` (TrustZone-M support is not in upstream Contiki-NG yet)
- **Source path**: `examples/platform-specific/nrf/trustzone/normal-world` (file: `normal-world-example.c`)
- **TARGET**: `nrf`
- **BOARD**: `nrf54l15/xiao`
- **Make args**: `TZ_MINIMAL_NONSECURE_PLATFORM=0`
- **Toolchain**: host
- **Built**: 2026-09-05T22:35:39Z by Niclas Finne
- **Build command**: `tools/build-device-firmware.sh --target nrf --board nrf54l15/xiao --example examples/platform-specific/nrf/trustzone/normal-world --output firmware/nrf54l15-xiao/normal-world-fullplat.nrf54l15-xiao --make-args "TZ_MINIMAL_NONSECURE_PLATFORM=0" --local`

