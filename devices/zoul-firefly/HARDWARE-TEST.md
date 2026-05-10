# Hardware Test Plan — Zoul Firefly + CC1200

> Briefing for whoever (Claude or human) does the actual hardware test.
> Companion to [`STATUS.md`](STATUS.md); historical investigation
> trail in [`archive/L6-PLAN.md`](archive/L6-PLAN.md).

## What we're testing

**Headline question**: does the L6 RPL-UDP firmware (`udp-server-subghz` /
`udp-client-subghz`, both built with `ZOUL_CONF_USE_CC1200_RADIO=1`)
actually converge on real Zolertia Firefly hardware with default
Contiki settings?

**Why it matters**:
- If YES on hardware → csim has remaining emulation gaps. Compare
  csim's pcap output and trace against hardware behavior to find them.
- If NO on hardware → csim is more correct than we've been assuming.
  The "L6 doesn't converge in csim" investigation has been chasing a
  phantom — the firmware itself doesn't converge with these defaults
  on real radio hardware either.

## What's in the repo for testing

- `firmware/zoul-firefly/udp-server-subghz.zoul-firefly` — DAG root
- `firmware/zoul-firefly/udp-client-subghz.zoul-firefly` — DAG child
- `firmware/zoul-firefly/udp-server-led.zoul-firefly` — same but with
  RED LED 1 Hz heartbeat + BLUE LED toggle on RX (visual debugging)
- `firmware/zoul-firefly/udp-client-led.zoul-firefly` — same but with
  GREEN LED 1 Hz heartbeat + BLUE LED toggle on TX

The `-led` variants are physically distinguishable by LED color and
let you confirm the firmware is actually running.

## Equipment needed

- 2× Zolertia Firefly boards (CC2538 + CC1200)
- 2× USB cables
- A host with two USB ports (NOT through Proxmox passthrough — that
  silently drops CDC control transfers and breaks DTR/RTS toggling
  needed for clean reset cycles)
- Optional: third Firefly or CC1310 launchpad for Sensniff packet
  capture

## Setup

1. Plug both Fireflies into the Mac. Verify with `lsusb` that both
   appear as `Silicon Labs CP210x UART Bridge` and that
   `ls /dev/tty.usbserial-*` shows two distinct devices (their
   serials should differ — check `/dev/serial/by-id/`).
2. Install Contiki-NG's `cc2538-bsl` tool if not already available:
   ```sh
   git clone --recursive https://github.com/contiki-ng/contiki-ng
   # Tool lives at: contiki-ng/tools/cc2538-bsl/cc2538-bsl.py
   ```
3. Install `arm-none-eabi-objcopy` if not already (for ELF→bin):
   ```sh
   brew install arm-none-eabi-binutils    # macOS
   ```

## Flash + capture procedure

```sh
# Adjust paths/devices as needed
CONTIKI=/path/to/contiki-ng
BSL=$CONTIKI/tools/cc2538-bsl/cc2538-bsl.py
BOARD0=/dev/tty.usbserial-XXXX     # first Firefly
BOARD1=/dev/tty.usbserial-YYYY     # second Firefly

# 1. Convert ELFs to .bin
arm-none-eabi-objcopy -O binary firmware/zoul-firefly/udp-server-led.zoul-firefly /tmp/server-led.bin
arm-none-eabi-objcopy -O binary firmware/zoul-firefly/udp-client-led.zoul-firefly /tmp/client-led.bin

# 2. Flash
python3 $BSL -e -w -v -b 460800 -a 0x00200000 -p $BOARD0 /tmp/server-led.bin
python3 $BSL -e -w -v -b 460800 -a 0x00200000 -p $BOARD1 /tmp/client-led.bin

# 3. Press RESET button on BOTH boards.
#    BOARD0 should blink RED at 1 Hz (server alive).
#    BOARD1 should blink GREEN at 1 Hz (client alive).

# 4. Capture UART for 90 s (covers ~10 RPL trickle intervals).
( stty 115200 raw -echo < $BOARD0 ; cat $BOARD0 | tee /tmp/board0.log ) &
( stty 115200 raw -echo < $BOARD1 ; cat $BOARD1 | tee /tmp/board1.log ) &
sleep 90
kill %1 %2
```

## What to look for

### Outcome A — Hardware converges

Look for in either log:
```
[INFO: App       ] Sending request 0 to ...
[INFO: App       ] Received request 'hello 0' from ...
[INFO: App       ] Received response 'hello 0' from ...
```

If you see this within 90 s → csim has emulation gaps to close. Next
step: capture a Sensniff pcap of the same exchange and compare with
csim's pcap (csim has `--pcap` on `test_runner`).

### Outcome B — Hardware does NOT converge

Both boards print only:
```
[INFO: App       ] Not reachable yet
```
every ~9 s, forever. No DAG forms.

**This is significant**: it means the Contiki RPL-UDP firmware with
default `ZOUL_CONF_USE_CC1200_RADIO=1` config doesn't converge in
2-node CSMA on real hardware either. csim was correct. The L6 issue
is a Contiki tuning problem, not a csim emulation gap.

In that case, the productive next steps are:
- Tune `CSMA_CONF_MAX_FRAME_RETRIES`, `CSMA_CONF_MIN_BE`,
  `CSMA_CONF_MAX_BE` upward
- Lengthen RPL trickle minimum interval
- Document that csim faithfully reproduces this real-world behavior
- Mark L6 as "passes when firmware is properly tuned for sub-GHz"

### Outcome C — Mixed signal

LEDs blink (firmware running) but no RPL convergence and no obvious
errors. Same as Outcome B for practical purposes — captures real
hardware's behavior of "valid firmware, doesn't converge."

## Comparing against csim

To run the same test in csim:
```sh
./build/test_runner zoul-firefly-multinode \
    firmware/zoul-firefly/udp-server-subghz.zoul-firefly \
    firmware/zoul-firefly/udp-client-subghz.zoul-firefly \
    -t 90000 -d 200 \
    --pcap /tmp/csim.pcap
```

Compare:
- UART output content + cadence (csim should produce same `Not reachable yet` interval as hardware)
- pcap (open both in Wireshark, compare DIO/DAO/DIS sequence + timing)

## Known csim quirks worth checking against hardware

These are simulator approximations that hardware will reveal as
correct or wrong:

1. **CC1200 strobe transition times** in `src/arm/cc1200.c`:
   - `SIDLE = 50 µs`, `SRX/STX = 200 µs`, `SCAL = 720 µs`
   - These were sourced from Contiki's RTIMER_BUSYWAIT_UNTIL constants
     and CC1200 datasheet SWRU346B. Hardware measurement (logic
     analyzer on GDO0 configured to MARC_2PIN_STATUS_0) would
     confirm or correct.

2. **CC1200 byte period at 50 kbps** in `src/arm/cc1200.c`:
   `CC1200_BYTE_PERIOD_NS = 160000` (160 µs/byte).
   - Should be exactly correct per the radio config.

3. **MARC≠RX byte drops**. csim's `cc1200_receive_byte` drops bytes
   whenever the chip isn't in MARC_RX. Real hardware has AGC + demod
   pipeline that may catch the START of the next frame's preamble
   even during brief MARC transitions. If hardware converges where
   csim doesn't, this is the first thing to inspect.

4. **csim's `schedule_emulated_wakeup`** uses `current_sim_ns` (not
   `cpu->sim_time_ns`) to anchor cross-node scheduling — recent fix
   for `tsch-drift-z1`. Hardware doesn't have this issue (no shared
   sim_time concept), but if hardware behaves notably differently
   from csim on TSCH, this would be a place to look.

## Outcome (2026-05-06)

The hardware run reframed L6 from "csim emulation gap" to "upstream
Contiki-NG firmware bug" and produced the two PR branches now tracked
in [`STATUS.md`](STATUS.md). The investigation trail that this run
closed out is archived in [`archive/L6-PLAN.md`](archive/L6-PLAN.md)
and [`archive/CC1200-RX-ACK-CHAIN.md`](archive/CC1200-RX-ACK-CHAIN.md).
