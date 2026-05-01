# `firmware/zoul-firefly/`

Pre-built Contiki-NG ELFs for the Zolertia Firefly (`TARGET=zoul`,
`BOARD=firefly`). The csim test harness loads any file in this
directory whose extension is `.zoul-firefly`.

See [`devices/zoul-firefly/SPEC.md`](../../devices/zoul-firefly/SPEC.md)
for the board specification and test ladder.

## Building

Use the helper script — it cross-compiles via Docker by default and
auto-stamps a `PROVENANCE.md` next to each ELF:

```sh
tools/build-device-firmware.sh \
    --target zoul \
    --board  firefly \
    --example examples/hello-world \
    --output firmware/zoul-firefly/bringup.zoul-firefly
```

## Required artifacts (per the porting checklist)

| ELF | Purpose | Test level |
|-----|---------|-----------|
| `bringup.zoul-firefly`            | Banner + LED blink + halt    | L0–L4 |
| `nullnet-broadcast.zoul-firefly`  | 802.15.4 broadcast (2 nodes) | L5    |
| `udp-server.zoul-firefly`         | RPL DAG root + UDP echo      | L6    |
| `udp-client.zoul-firefly`         | RPL child + UDP requester    | L6    |

Each ELF must have a sibling `PROVENANCE.md` (one per ELF) recording
Contiki-NG commit, `make` flags, toolchain, and expected stdout
substrings — see [`firmware/PROVENANCE-template.md`](../PROVENANCE-template.md).

## Radio note

The Firefly is a dual-radio board (CC2538 on-chip 2.4 GHz +
off-chip CC1200 sub-GHz). csim only emulates the 2.4 GHz radio for
now, so build firmware with the 2.4 GHz driver explicitly selected
(e.g. `MAKE_RADIO=cc2538-rf`) to avoid the Contiki dual-RF init path
poking the (un-emulated) CC1200 over SPI.
