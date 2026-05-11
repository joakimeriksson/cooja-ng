# nRF52840 Development Kit Port — Status

> Companion to [`SPEC.md`](SPEC.md). For the full SoC narrative and
> architectural deltas, see [`../nrf52840-dongle/STATUS.md`](../nrf52840-dongle/STATUS.md);
> the DK shares everything except board glue and VTOR.

## Current state — short version

**The port is complete.** Same SoC as the Dongle, so no new emulator
work — only an `arm_platform_config_t` entry for the board (LED/button
pinout + `vtor_override = 0`) and a `nrf52840-dk-multinode` test_runner
subcommand. L6 RPL-UDP converges between two DK nodes and across a
DK/Dongle pair.

```
41.459 [Node 1/ARM] Received request 'hello 0' from fd00::f6ce:3602:e3e9:4176
41.571 [Node 2/ARM] Received response 'hello 0' from fd00::f6ce:3601:f1f4:203b
59.796 [Node 1/ARM] Received request 'hello 1' from fd00::f6ce:3602:e3e9:4176
59.942 [Node 2/ARM] Received response 'hello 1' from fd00::f6ce:3601:f1f4:203b
```

## What it took

This port is the validation that the SoC-polymorphism refactor done
during the Dongle port actually pays off. Total delta to add the DK as
a new board:

  - **One platform-config entry** in `src/arm/arm_platform.c`
    (`platform_nrf52840_dk`).
  - **One field** moved: `vtor_default` migrated from
    `arm_config_t` (SoC-level) to `arm_platform_config_t::vtor_override`
    (board-level), because the Dongle has a bootloader region at 0x0
    and the DK does not. SoC config sets `vtor_default = 0` and lets
    the board override.
  - **One subcommand**: `nrf52840-dk-multinode` in `test_main.c`.
  - **Extension recognition**: `.nrf52840-dk` registered in
    `detect_node_type` and `init_arm_node`.

No new peripheral models. No new instruction handlers. No new tests
required (the existing 74 ARM correctness tests cover the SoC). The
DK shares the same `nrf52840_soc.c` and `arm_vfp.c` as the Dongle.

## Files

- [`SPEC.md`](SPEC.md) — board contract; lists only the deltas from
  the Dongle SPEC.
- [`../nrf52840-dongle/SPEC.md`](../nrf52840-dongle/SPEC.md) — SoC
  contract (read this for everything not in the deltas table).
- [`../nrf52840-dongle/STATUS.md`](../nrf52840-dongle/STATUS.md) — the
  full L0–L6 architectural deltas from the SoC port.
- `../../firmware/nrf52840-dk/PROVENANCE.md` — build commands for the
  hello-world / udp-server / udp-client ELFs.

## How to run

```sh
make
./build/test_runner nrf52840-dk-multinode \
    firmware/nrf52840-dk/udp-server.nrf52840-dk \
    firmware/nrf52840-dk/udp-client.nrf52840-dk \
    -t 60000

# Cross-board: DK + Dongle on the same medium also works.
./build/test_runner mixed-multinode \
    firmware/nrf52840-dk/udp-server.nrf52840-dk \
    firmware/nrf52840-dongle/udp-client.nrf52840-dongle \
    -t 60000
```
