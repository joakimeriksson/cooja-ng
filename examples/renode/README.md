# Renode ↔ csim co-simulation

Renode drives csim's simulation clock, and csim's whole 802.15.4 network
appears inside Renode as one memory-mapped device. Firmware running on a
Renode-emulated CPU can transmit into that network, receive from it, and read
a csim node's console — while the two simulators' clocks advance together.

No Renode-side code is needed: `CoSimulated.CoSimulatedPeripheral` is
Renode's standard external-simulator peripheral. Protocol and design:
[`../../docs/design/renode-cosim-plan.md`](../../docs/design/renode-cosim-plan.md).

## Files

| File | |
|---|---|
| `csim-host.repl` | A minimal Cortex-M4 machine: CPU, memory, a PL011 UART, and the csim window at `0x40100000` |
| `csim-rpl.resc` | Interactive run: watch a live RPL-UDP network from Renode through the register window |
| `csim-rpl-headless.resc` | The same run bounded by virtual time, for scripts and CI |
| `csim.repl`, `csim.resc` | The peripheral on its own, to drop into an existing platform |
| `csim_dev.h` | Guest-side view of the register window (mirrors `include/native/renode_dev.h`) |
| `firmware/` | The demo guest firmware: a bare-metal 802.15.4 sniffer |

## Watching csim's network from Renode

Two Tmote Sky motes run a real Contiki-NG RPL-UDP exchange inside csim; the
Renode firmware watches every frame of it through the register window.

Build the firmware once (needs `arm-none-eabi-gcc`):

```sh
make -C examples/renode/firmware
```

**Renode starts csim itself.** Setting `simulationFilePath` is what makes
Renode open the two sockets, launch the peer with them on its command line,
and shake hands — `tools/csim-renode-launch.sh` maps those arguments onto
csim's `--renode` flag. The scenario comes from the environment, because
Renode has no way to pass it through:

```sh
export CSIM_CONFIG=$PWD/configs/test-renode-rpl-sky.yaml
export CSIM_RENODE_FREQ_HZ=1000000
export CSIM_LOG=/tmp/csim.log          # keep csim's output out of Renode's

renode examples/renode/csim-rpl.resc
# then, in Renode's monitor:  start
```

`CSIM_RENODE_FREQ_HZ` must match `frequency` in `csim-host.repl`: the
`tickClock` message carries only a tick count, not the rate.

Headless, bounded by virtual time rather than wall-clock:

```sh
renode --disable-xwt --console --plain \
       -e "include @examples/renode/csim-rpl-headless.resc"
```

## What you should see

The firmware's UART output, with csim's simulation time on every line:

Actual output from a 20-second run against Renode 1.17, with csim's
simulation time on every line:

```
[renode] csim sniffer starting
[renode] csim device found: version=1 self_id=3 nodes=3
[renode] listening on channel 26 (interrupt-driven)
[renode] 3.738805 rx #1 node=1 ch=26 rssi=-26dBm len=95 DATA seq=198 dst=ff:ff src=00:12:74:01:00:01:01:01 6lowpan=IPHC
[renode] 7.613875 rx #2 node=2 ch=26 rssi=-26dBm len=100 DATA seq=126 dst=00:12:74:01:00:01:01:01 src=00:12:74:02:00:02:02:02 ackreq 6lowpan=IPHC
[renode] 7.617593 rx #3 node=1 ch=26 rssi=-26dBm len=3 ACK seq=126
[renode] t=10s frames=6 irqs=9 dropped=0
...
[renode] t=20s frames=19 irqs=31 dropped=0
```

The broadcast at 3.73 s is RPL multicast; the unicast at 7.61 s carries
`ackreq` and its acknowledgement comes back 3.7 ms later — the 802.15.4
turnaround, computed by csim's medium and observed from Renode.

csim's own run ends with its validators passing, because the RPL exchange
really happened while Renode was driving the clock:

```
Validator [PASS]: "Received request" matched 1/1 node=1
Validator [PASS]: "Received response" matched 1/1 node=2
renode: 20000 ticks, 252770 reads, 83954 writes, 34 irqs, 7 log flushes
```

Renode's own log also carries csim's node consoles, forwarded as log
messages, so the Sky motes' Contiki output appears in Renode:

```
[INFO] cosimulation_connection: Co-simulation: [node 2] [INFO: Main] Starting Contiki-NG...
    [node 1] [INFO: Main      ] - Routing: RPL Lite
```

## How the timing works

The peripheral is declared with `frequency: 1000000` and `limitBuffer: 1000`,
so every millisecond of Renode virtual time Renode sends a `tickClock` and
**blocks** until csim has simulated the same millisecond. Neither simulator
can run ahead of the other.

Every access to the register window is a synchronous round trip to the csim
process, which is why the demo firmware is interrupt-driven rather than
polling: it sits in `WFI` and touches the bus only when csim raises the
device's line. A tight polling loop would cost millions of round trips per
simulated second.
