# Device SPEC — `Seeed Studio XIAO nRF54L15`

> Lightweight SPEC. The XIAO shares the SoC, peripheral model, M33
> support, and software stack with the [`nrf54l15-dk`](../nrf54l15-dk/SPEC.md)
> — only board glue differs (LEDs, button, console pins, an external
> RF switch). Read the DK SPEC for the SoC details; this doc lists the
> deltas.

## Identity

- **SoC**: `nRF54L15` (same as the DK — Cortex-M33 + 2.4 GHz radio)
- **Board name**: `Seeed Studio XIAO nRF54L15`
- **Contiki-NG `TARGET`**: `nrf`
- **Contiki-NG `BOARD`**: `nrf54l15/xiao`
- **csim platform string**: `nrf54l15-xiao`
- **Reference docs**:
  - **Seeed wiki** (pinout, schematic, getting-started):
    <https://wiki.seeedstudio.com/xiao_nrf54l15/>
  - **nRF54L15 datasheet**:
    <https://www.nordicsemi.com/Products/nRF54L15>
  - **Zephyr XIAO board page** (good cross-reference for pinout):
    <https://docs.zephyrproject.org/latest/boards/seeed/xiao_nrf54l15/>
  - **Contiki-NG port + status doc**:
    `arch/platform/nrf/nrf54l15/xiao/` — has its own `README.md`
    documenting which features work on real hardware.

## Deltas vs the DK

| | DK (PCA10156) | XIAO |
|---|---|---|
| Form factor | Full dev kit (~6×8 cm), SEGGER J-Link onboard | 21×17.8 mm, CMSIS-DAP via onboard SAMD11 |
| LEDs | 4× (P2.9, P1.10, P2.7, P1.14) | 1× red (P2.0) |
| Buttons | 4× (P1.13, P1.9, P1.8, P0.4) | 1× user (P0.0) |
| Console pads | UARTE20 TX=P1.4 / RX=P1.5 | UARTE20 TX=P1.9 / RX=P1.8 |
| RAM | 192 KiB (per port headers) | 256 KiB (per XIAO README) — same SoC variant, verify against PS |
| RF front end | None — direct antenna | **External RF switch**: PWR=P2.3, SEL=P2.5 |
| Debugger | SEGGER J-Link | CMSIS-DAP (SAMD11) |

Same SoC means the same `nrf54l15_soc.c` peripheral set is used by
both boards. Only `arm_platform_config_t` differs (LED/button pinout,
RF switch pins as platform-managed GPIO outputs that csim accepts but
ignores).

## RF switch — what to do with it

The XIAO uses an external SP3T (or similar) RF switch between the
nRF54L15 antenna pin and the SMA/PCB antenna because the package
needs different antenna paths for BLE vs 802.15.4 in some
configurations. The firmware drives:

  - **PWR (P2.3)** — switch power enable, set high before TX/RX.
  - **SEL (P2.5)** — select between 2.4 GHz path A vs B.

csim doesn't model RF switches (there's no real antenna). The
existing GPIO output-callback mechanism absorbs these writes silently
— same way `cc2538_gpio` handles board-level GPIO writes that don't
map to a chip driver.

## Reference firmware

Pre-built ELFs under `firmware/nrf54l15-xiao/`:

- `hello-world.nrf54l15-xiao` — banner + idle on UARTE20.
- `udp-server.nrf54l15-xiao` — RPL-UDP server.
- `udp-client.nrf54l15-xiao` — RPL-UDP client.

Build command (no special flags — same Docker build path as DK):

```sh
tools/build-device-firmware.sh --target nrf --board nrf54l15/xiao \
    --example examples/hello-world \
    --output firmware/nrf54l15-xiao/hello-world.nrf54l15-xiao
```

## Definition of done

- [ ] Reuses the M33 ISA support + `nrf54l15_soc.c` peripheral set
      added for the DK port. No additional ISA work expected.
- [ ] L6 — 2-node RPL-UDP between two XIAO nodes passes.
- [ ] Cross-board interop — DK server + XIAO client (or vice versa)
      forms a DODAG. Same on-air format, same SoC.
- [ ] `docs/architecture.md` Platforms table includes `nrf54l15-xiao`.
- [ ] No regressions on existing platforms.
- [ ] **Out of scope (deferred)**: RF switch behaviour — written by
      firmware but not modelled in csim. GPIO writes accepted silently.
