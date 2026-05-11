# Device SPEC — `nRF52840 Development Kit (PCA10056)`

> Lightweight SPEC. The DK shares the SoC, peripheral model, ARM-M4
> support, and software stack with the [`nrf52840-dongle`](../nrf52840-dongle/SPEC.md);
> only board glue and bootloader assumptions differ. Read the dongle
> SPEC for the SoC details; this doc lists the deltas.

## Identity

- **SoC**: `nRF52840` (same as the Dongle — Cortex-M4F + 2.4 GHz radio)
- **Board name**: `nRF52840 Development Kit` (Nordic PCA10056)
- **Contiki-NG `TARGET`**: `nrf52840`
- **Contiki-NG `BOARD`**: `dk`
- **csim platform string**: `nrf52840-dk`
- **Reference docs**:
  - nRF52840 Product Specification v1.7 (shared with the Dongle).
  - PCA10056 user guide + schematic (Nordic).
  - Contiki-NG: `arch/platform/nrf52840/dk/`.

## Deltas vs the Dongle

| | Dongle (PCA10059) | DK (PCA10056) |
|---|---|---|
| Console | USB-CDC by default; we run with `NRF52840_NATIVE_USB=0` so output routes to the legacy UART register window | UARTE0 to SEGGER VCP — no `NRF52840_NATIVE_USB=0` needed |
| VTOR | 0x00001000 (Open Bootloader at 0x0..0xfff) | 0x00000000 (no bootloader region; SEGGER flashes app at 0x0) |
| LEDs | 1× mono (P0.6) + 1× RGB (P0.8, P1.9, P0.12), active-low | 4× LEDs at P0.13–P0.16, active-low |
| Buttons | 1× user button at P1.6 | 4× buttons at P0.11, P0.12, P0.24, P0.25 |
| Form factor | USB stick, no SEGGER | Full dev kit, SEGGER J-Link onboard, lots of headers |

Same SoC means the same `nrf52840_soc.c` peripheral set is used by both
boards. Only `arm_platform_config_t` differs (LED/button pinout,
`vtor_override`).

## Reference firmware

Pre-built ELFs under `firmware/nrf52840-dk/`:

- `hello-world.nrf52840-dk` — banner + idle, console = UARTE0.
- `udp-server.nrf52840-dk` — RPL-UDP server (RPL root), Contiki-NG `examples/rpl-udp`.
- `udp-client.nrf52840-dk` — RPL-UDP client.

Build command:

```sh
tools/build-device-firmware.sh --target nrf52840 --board dk \
    --example examples/rpl-udp --source-file udp-server \
    --output firmware/nrf52840-dk/udp-server.nrf52840-dk
```

No `NRF52840_NATIVE_USB=0` flag needed (unlike the Dongle build).

## Definition of done

- [x] `make clean && make` builds clean — same SoC code as Dongle.
- [x] Reuses the M4 DSP halfword multiply + FPv4-SP-D16 VFP support
      added for the Dongle port. No additional ISA work.
- [x] **L6** — two-node RPL-UDP:
      `./build/test_runner nrf52840-dk-multinode
       firmware/nrf52840-dk/udp-server.nrf52840-dk
       firmware/nrf52840-dk/udp-client.nrf52840-dk -t 60000`
      Result: 2 round-trips in 60 s, ~2× real-time. Output:
      `[Node 1] Received request 'hello 0' from fd00::f6ce:3602:e3e9:4176`
      `[Node 2] Received response 'hello 0' from fd00::f6ce:3601:f1f4:203b`
- [x] **Cross-board interop** — DK server + Dongle client (or vice
      versa) on the same medium also converges. Same on-air format
      (0x00 preamble + 0x7A SFD + PHR + payload + CRC), same channel,
      same SoC.
- [x] `docs/architecture.md` Platforms table includes the
      `nrf52840-dk` entry.
- [x] No regressions: 68/68 MSP430 + 74/74 ARM correctness, 21/21
      CC2420 + 73/73 CC1200 mock-host, 235/235 radio_medium.
- [ ] Not done (deferred, same as the Dongle): PPI for TSCH, USBD
      emulation, BLE / SoftDevice, double-precision FP arithmetic.
