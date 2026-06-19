# Contributing to Cooja-NG

Thanks for your interest! Cooja-NG (`csim`) is a C re-implementation of the
parts of Cooja and MSPSim needed to run the Contiki-NG test suite headlessly.

## Ground rules

- **Behaviour-preserving by default.** Most changes must not alter simulation
  output. Verify with a cross-build diff against a pre-change binary on a
  representative config (e.g. `multinode firmware/sky/udp-server.sky
  firmware/sky/udp-client.sky -t 30000 -q`), ignoring wall-clock/perf lines.
- **Ports, not type switches.** The kernel is ports-and-adapters: never branch
  on a node's concrete type — add an op to the relevant vtable
  (`sim_mote_ops_t`, `sim_service_ops_t`, `sim_medium_ops_t`) instead. See
  [`docs/design/refactor-plan.md`](docs/design/refactor-plan.md).
- **Match the surrounding code** — comment density, naming, and idiom.

## Build & test

```sh
make                                  # O3 + LTO (GNU Lightning auto-detected)
make debug                            # O0 -g for debugging

./build/test_runner correctness       # MSP430 instruction tests
./build/test_runner arm-correctness   # ARM instruction tests
./build/test_runner radio-medium      # radio-medium policy tests
./build/test_runner radio-bus         # RF delivery tests
./build/test_runner cc1200-mock-host  # chip-driver tests
make plugins && tools/check-plugin.sh # plugin system smoke

# End-to-end against the upstream Contiki-NG Cooja suite:
tools/run-cooja-tests.sh
```

CI (`.github/workflows/test.yml`) runs the unit + firmware + multinode + plugin
suites on Linux and macOS; please keep it green.

## Pull requests

- One logical change per PR; describe what you verified (which suites, which
  cross-build diff).
- New peripherals/SoCs: see [`docs/porting-a-device.md`](docs/porting-a-device.md).
- New plugins: [`docs/design/ui-plugins.md`](docs/design/ui-plugins.md) and the
  `plugins/` examples.

## License

By contributing you agree your contributions are licensed under the project's
[BSD-3-Clause license](LICENSE).
