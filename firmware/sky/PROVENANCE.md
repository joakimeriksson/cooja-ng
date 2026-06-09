## nullnet-broadcast.sky

- **Source**: contiki-ng commit `9aa6ecb055182fa8511df68d0ff1f7edc31ed4a9`
- **Source path**: `examples/nullnet` (file: `nullnet-broadcast.c`)
- **TARGET**: `sky`
- **Toolchain**: host
- **Built**: 2026-06-09T22:21:15Z by Joakim Eriksson
- **Build command**: `tools/build-device-firmware.sh --target sky --example examples/nullnet --output firmware/sky/nullnet-broadcast.sky --source-file nullnet-broadcast`

## cputest.sky / timertest.sky

- **Source**: prebuilt MSPSim-era fixtures copied verbatim from the
  Contiki-NG tree at `tools/cooja/firmware/sky/` (sources:
  `tools/cooja/tests/cputest.c`, `tools/cooja/tests/timertest.c`)
- **TARGET**: `sky`
- **Restored**: 2026-06-09 (removed in 1089e7d, restored for the
  `test_runner firmware` boot tests which otherwise SKIP them)
- Exercises instruction-level behavior (cputest) and timer A/B
  capture/compare paths (timertest); the boot test asserts clean
  execution, no expected-output substring.
