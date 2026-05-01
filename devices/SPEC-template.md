# Device SPEC — `<board name>`

> Copy this file to `devices/<board-name>/SPEC.md` and fill in every field.
> If a field is unknown, leave it as `TODO:` rather than guessing — see
> `docs/porting-a-device.md` §3 for why guessing is the #1 source of port
> bugs.

## Identity

- **SoC**: TODO (e.g. `CC2538`, `MSP430F1611`)
- **Board name**: TODO (e.g. `Zoul Firefly`)
- **Contiki-NG `TARGET`**: TODO (e.g. `zoul`)
- **Contiki-NG `BOARD`**: TODO (e.g. `firefly`)
- **csim platform string**: TODO (lowercase, hyphenated; used as filename
  extension and `--platform` argument; e.g. `zoul-firefly`)
- **Reference docs**: TODO (datasheet PDFs with page numbers, Contiki-NG
  `arch/platform/<target>/<board>/` path)

## CPU

- **Architecture**: TODO (`arm-cortex-m3` / `msp430` / `msp430x` / new)
- **Frequency**: TODO Hz (typical, after firmware clock setup)
- **RAM**: TODO bytes @ TODO base address
- **Flash**: TODO bytes @ TODO base address
- **Reuses existing emulator?**: TODO (yes if `arm-cortex-m3` with CC2538;
  no if a new variant — adding a CPU is a much bigger scope)

## Console

- **Peripheral**: TODO (e.g. `UART0` / `USART1`)
- **Base address**: TODO (CPU memory map)
- **Baud**: TODO bps (typically 115200)
- **TX pin**: TODO (port + pin)
- **RX pin**: TODO (port + pin, optional for tests)

## LEDs

| Index | Name  | Port | Pin | Polarity |
|-------|-------|------|-----|----------|
| 0     | TODO  | TODO | TODO| TODO (active-high / active-low) |
| 1     | TODO  | TODO | TODO| TODO |
| 2     | TODO  | TODO | TODO| TODO |

## Buttons

| Index | Name  | Port | Pin | Polarity |
|-------|-------|------|-----|----------|
| 0     | TODO  | TODO | TODO| TODO |

(Delete the row above if no buttons.)

## Off-SoC chips

For each external chip on the SPI/SSI bus, fill in:

### `<chip name>` — TODO role (radio / flash / sensor)

- **Datasheet ref**: TODO (PDF + page)
- **Bus**: TODO (`SSI0` / `SSI1` on ARM; `USART0/1` on MSP430)
- **CSn pin**: TODO (port, pin, polarity — CC2538 chip selects are
  usually active-low)
- **Status pins** driven by chip back to MCU (one row per pin):

  | Signal | Port | Pin | Polarity | Maps to |
  |--------|------|-----|----------|---------|
  | TODO   | TODO | TODO| TODO     | TODO (e.g. FIFOP, SFD, GPIO0) |

- **Reset pin**: TODO (or "none / shared with VREG" if unmanaged)
- **Power/enable pin**: TODO (e.g. VREG_EN)
- **Interrupt routing**: TODO (which status pins fire MCU interrupts)
- **Notes**: TODO (anything firmware-specific — e.g. Contiki-NG driver
  expects the chip to be at a specific SSI instance)

(Delete this section entirely if there are no off-SoC chips. Repeat the
section for each chip.)

## Clock tree

- **Source**: TODO (e.g. external 32 MHz crystal, internal RC)
- **CPU clock divider**: TODO
- **What the firmware actually configures**: TODO (one or two lines on
  what the boot path writes to the clock control registers — read the
  Contiki-NG `arch/cpu/<cpu>/clock.c` to fill this in)

## Known firmware quirks

> List any non-obvious init steps, undocumented register writes, or
> ordering constraints the port will need to handle. See `docs/porting-a-
> device.md` §7 for the kinds of issue past ports have hit.

- TODO: …

(If none known, leave the bullet as `TODO: none observed yet`.)

## Reference firmware

Pre-built ELFs that the test harness loads. **Each must have a
`PROVENANCE.md` next to it** (copy `firmware/PROVENANCE-template.md`).

- `firmware/<board>/bringup.<board>` — bring-up firmware. Prints a known
  banner string, blinks LEDs in a fixed pattern, then halts. Used for
  L0–L4 tests. ~50 lines of Contiki-NG.
- `firmware/<board>/nullnet-broadcast.<board>` — 802.15.4 broadcast.
  Used for L5.
- `firmware/<board>/udp-server.<board>` and `udp-client.<board>` —
  RPL-UDP. Used for L6 (full-stack integration).

## Definition of done

Concrete `test_runner` subcommands that must pass for the port to be
considered complete. Paste the actual command output as evidence in the
PR.

- [ ] `make clean && make` builds with no new warnings
- [ ] `./build/test_runner <platform>-firmware` passes (covers L0–L4)
- [ ] `./build/test_runner <platform>-multinode firmware/<board>/nullnet-broadcast.<board> -t 20000 -q` shows ≥1 RX per node (L5)
- [ ] `./build/test_runner <platform>-multinode firmware/<board>/udp-server.<board> firmware/<board>/udp-client.<board> -t 60000` exchanges ≥1 hello/response (L6)
- [ ] `.github/workflows/test.yml` runs the new subcommands on PR
- [ ] No CPU/GPIO type leaks into off-SoC chip drivers — chips take
      `sim_host_t`
- [ ] `docs/architecture.md` Platforms table updated
