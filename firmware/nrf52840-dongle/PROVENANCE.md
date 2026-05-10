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

