## hello-world.nrf52840-dongle

- **Source**: contiki-ng commit `ad0d073818684fd0c6cdce2f01c3aac9331f812e`
- **Source path**: `examples/hello-world` (file: `hello-world.c`)
- **TARGET**: `nrf52840`
- **BOARD**: `dongle`
- **Toolchain**: Docker contiker/contiki-ng:latest
- **Built**: 2026-05-10T21:42:37Z by Joakim Eriksson
- **Build command**: `tools/build-device-firmware.sh --target nrf52840 --board dongle --example examples/hello-world --output firmware/nrf52840-dongle/hello-world.nrf52840-dongle`

## hello-world-uart.nrf52840-dongle

- **Source**: contiki-ng commit `ad0d073818684fd0c6cdce2f01c3aac9331f812e`
- **Source path**: `examples/hello-world` (file: `hello-world.c`)
- **TARGET**: `nrf52840`
- **BOARD**: `dongle`
- **Make args**: `NRF52840_NATIVE_USB=0`
- **Toolchain**: Docker contiker/contiki-ng:latest
- **Built**: 2026-05-10T22:05:42Z by Joakim Eriksson
- **Build command**: `tools/build-device-firmware.sh --target nrf52840 --board dongle --example examples/hello-world --output firmware/nrf52840-dongle/hello-world-uart.nrf52840-dongle --make-args "NRF52840_NATIVE_USB=0"`

## udp-server.nrf52840-dongle

- **Source**: contiki-ng commit `ad0d073818684fd0c6cdce2f01c3aac9331f812e`
- **Source path**: `examples/rpl-udp` (file: `udp-server.c`)
- **TARGET**: `nrf52840`
- **BOARD**: `dongle`
- **Make args**: `NRF52840_NATIVE_USB=0`
- **Toolchain**: Docker contiker/contiki-ng:latest
- **Built**: 2026-05-10T22:31:29Z by Joakim Eriksson
- **Build command**: `tools/build-device-firmware.sh --target nrf52840 --board dongle --example examples/rpl-udp --output firmware/nrf52840-dongle/udp-server.nrf52840-dongle --source-file udp-server --make-args "NRF52840_NATIVE_USB=0"`

## udp-client.nrf52840-dongle

- **Source**: contiki-ng commit `ad0d073818684fd0c6cdce2f01c3aac9331f812e`
- **Source path**: `examples/rpl-udp` (file: `udp-client.c`)
- **TARGET**: `nrf52840`
- **BOARD**: `dongle`
- **Make args**: `NRF52840_NATIVE_USB=0`
- **Toolchain**: Docker contiker/contiki-ng:latest
- **Built**: 2026-05-10T22:31:40Z by Joakim Eriksson
- **Build command**: `tools/build-device-firmware.sh --target nrf52840 --board dongle --example examples/rpl-udp --output firmware/nrf52840-dongle/udp-client.nrf52840-dongle --source-file udp-client --make-args "NRF52840_NATIVE_USB=0"`

