## hello-world.nrf52840-dk

- **Source**: contiki-ng commit `ad0d073818684fd0c6cdce2f01c3aac9331f812e`
- **Source path**: `examples/hello-world` (file: `hello-world.c`)
- **TARGET**: `nrf52840`
- **BOARD**: `dk`
- **Toolchain**: Docker contiker/contiki-ng:latest
- **Built**: 2026-05-11T09:08:39Z by Joakim Eriksson
- **Build command**: `tools/build-device-firmware.sh --target nrf52840 --board dk --example examples/hello-world --output firmware/nrf52840-dk/hello-world.nrf52840-dk`

## udp-server.nrf52840-dk

- **Source**: contiki-ng commit `ad0d073818684fd0c6cdce2f01c3aac9331f812e`
- **Source path**: `examples/rpl-udp` (file: `udp-server.c`)
- **TARGET**: `nrf52840`
- **BOARD**: `dk`
- **Toolchain**: Docker contiker/contiki-ng:latest
- **Built**: 2026-05-11T09:09:05Z by Joakim Eriksson
- **Build command**: `tools/build-device-firmware.sh --target nrf52840 --board dk --example examples/rpl-udp --output firmware/nrf52840-dk/udp-server.nrf52840-dk --source-file udp-server`

## udp-client.nrf52840-dk

- **Source**: contiki-ng commit `ad0d073818684fd0c6cdce2f01c3aac9331f812e`
- **Source path**: `examples/rpl-udp` (file: `udp-client.c`)
- **TARGET**: `nrf52840`
- **BOARD**: `dk`
- **Toolchain**: Docker contiker/contiki-ng:latest
- **Built**: 2026-05-11T09:09:11Z by Joakim Eriksson
- **Build command**: `tools/build-device-firmware.sh --target nrf52840 --board dk --example examples/rpl-udp --output firmware/nrf52840-dk/udp-client.nrf52840-dk --source-file udp-client`

## zephyr-echo-server.nrf52840-dk / zephyr-echo-client.nrf52840-dk

- **Source**: Zephyr (`zephyrproject/zephyr`) `v4.4.0-5779-g32e0ab566c6`
- **Source path**: `samples/net/sockets/echo_server` and `.../echo_client`
- **BOARD**: `nrf52840dk/nrf52840`
- **Config**: STOCK — only the sample's own `overlay-802154.conf` (selects the
  on-chip 802.15.4 radio / `nrf_802154`); no other firmware changes.
- **Toolchain**: gnuarmemb (`arm-none-eabi-gcc`)
- **Built**: 2026-06-21 by Joakim Eriksson; stripped with `--strip-debug`.
- **Build command**:
  `west build -b nrf52840dk/nrf52840 samples/net/sockets/echo_{server,client} -- -DEXTRA_CONF_FILE=overlay-802154.conf`
- **Purpose**: regression test for stock-Zephyr 802.15.4 networking on csim's
  nRF52840 (DAD + two-node UDP echo). The server logs `Received and replied`.


## tsch-node.nrf52840-dk

- **Source**: contiki-ng commit `36a1ab5e3` (local tree, 2026-07-02)
- **Source path**: `examples/6tisch/simple-node` (file: `node.c`)
- **TARGET**: `nrf52840`
- **BOARD**: `dk`
- **Local patch**: `node.c` coordinator guard removed — `if(node_id == 1) NETSTACK_ROUTING.root_start();` compiled on ALL targets (upstream gates it to COOJA/Z1), so node 1 self-selects TSCH coordinator in the emulator
- **Toolchain**: arm-none-eabi-gcc 15.2.1 (host build)
- **Built**: 2026-07-02 by Joakim Eriksson / Claude
- **Build command**: `make TARGET=nrf52840 BOARD=dk` in examples/6tisch/simple-node
- **Purpose**: 2-node TSCH regression (association + held sync + EACK'd keepalives); node 1 becomes coordinator, node 2 associates
