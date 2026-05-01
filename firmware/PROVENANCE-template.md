# Firmware Provenance — `<elf-filename>`

> Copy this file to `firmware/<board>/PROVENANCE.md` and fill in every
> field. Every committed `.<board>` ELF must have an accompanying
> `PROVENANCE.md` recording how it was built. Without this, a Contiki-NG
> upstream change that alters init order can silently break csim with no
> trace of which commit was used.

## Source

- **Project**: TODO (e.g. `contiki-ng/contiki-ng`)
- **Commit hash**: TODO (full 40-char SHA from `git rev-parse HEAD` in
  the source tree)
- **Source path**: TODO (e.g. `examples/hello-world/`,
  `examples/rpl-udp/`)

## Target

- **`TARGET=`**: TODO (e.g. `zoul`)
- **`BOARD=`**: TODO (e.g. `firefly`)
- **`MAKE_MAC`** / **`MAKE_NET`** / **`MAKE_RADIO`**: TODO (any
  non-default Contiki-NG configuration)
- **Other Make flags**: TODO (e.g. `CFLAGS_EXTRA=-DSOMETHING`)

## Toolchain

- **Compiler**: TODO (e.g. `arm-none-eabi-gcc 12.2.0`)
- **Newlib version**: TODO (if known)
- **Build host**: TODO (e.g. `Ubuntu 22.04 inside contiker/contiki-ng:latest`)

## Build command

The exact command that produced the ELF. Prefer one that runs verbatim
through `tools/build-device-firmware.sh`:

```sh
TODO: ./tools/build-device-firmware.sh \
        --target zoul \
        --board firefly \
        --example hello-world
```

## Build date

- **UTC**: TODO (`date -u +"%Y-%m-%dT%H:%M:%SZ"`)
- **Built by**: TODO (name or CI job)

## What this firmware does

A few sentences on the binary's behavior. Used by tests to know what
output to expect. Examples:

- "Prints `Hello, world` once and halts."
- "Blinks the green LED at 1 Hz forever."
- "Acts as RPL DAG root, accepts UDP `hello N` requests on port 8765,
  replies with `hello N`."

## Expected runtime output

Exact substring(s) test_runner should grep for to consider the firmware
working. Used by `test_firmware.c` and friends.

- TODO: substring 1
- TODO: substring 2 (in order)

## Notes

Anything else worth recording — known quirks, deliberate non-defaults,
why this firmware was chosen as the test fixture, etc.

- TODO: …
