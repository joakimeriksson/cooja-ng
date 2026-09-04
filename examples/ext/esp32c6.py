#!/usr/bin/env python3
"""
ESP32-C6 node -- an unmodified ESP-IDF image (ROM, bootloader, FreeRTOS,
Contiki-NG on IDF) emulated by esp32sim, driven by csim in exact lock-step.

This is only a launcher: esp32sim-c6 --cooja speaks the protocol in
docs/design/external-nodes-plan.md §4 itself (hello/step/stop in, one done
per message out, tx at the cycle the guest wrote TX_START, rx injected at its
own time, idle skipped to the next device deadline).  csim keys external nodes
on the .py extension, so a Python file is the natural place to say which
binary and which firmware to run:

    { "firmware": "examples/ext/esp32c6.py", "id": 2, "x": 2.0, "y": 0.0 }

Environment:
    ESP32SIM_C6      the esp32sim-c6 binary
                     (default ~/work/esp32sim/target/release/esp32sim-c6)
    CONTIKI_C6_DIR   an ESP-IDF build directory holding bootloader/,
                     partition_table/ and the app image + ELF
                     (default ~/work/esp32/esp32-contiki/build-nullnet)
    ESP32C6_APP      the app's base name in that directory (default esp32-blink)
    ESP32C6_ARGS     extra esp32sim-c6 arguments, whitespace-separated
                     (e.g. "--cooja-verbose", or "--console all")

The mask ROM ELF comes from ~/.espressif/tools/esp-rom-elfs (or pass --rom in
ESP32C6_ARGS).  `--stub bb_init=0` skips the PHY blob's baseband calibration,
a set of handshakes on undocumented registers that would spin forever; the
802.15.4 MAC model does not depend on it.  The MAC address follows the csim
node id, so a run is a function of the config alone.
"""
import os
import shlex
import sys

HOME = os.path.expanduser("~")
BIN = os.environ.get("ESP32SIM_C6", f"{HOME}/work/esp32sim/target/release/esp32sim-c6")
BUILD = os.environ.get("CONTIKI_C6_DIR", f"{HOME}/work/esp32/esp32-contiki/build-nullnet")
APP = os.environ.get("ESP32C6_APP", "esp32-blink")

args = [BIN, "--cooja", "--boot", "rom", "--flash-mb", "2", "--no-dump",
        "--stub", "bb_init=0",
        "--bootloader", f"{BUILD}/bootloader/bootloader.bin",
        "--ptable", f"{BUILD}/partition_table/partition-table.bin",
        "--app", f"{BUILD}/{APP}.bin",
        "--elf", f"{BUILD}/{APP}.elf"]
args += shlex.split(os.environ.get("ESP32C6_ARGS", ""))

for path in (BIN, f"{BUILD}/{APP}.bin"):
    if not os.path.exists(path):
        # A missing dependency must be loud: an external node that never
        # starts would otherwise look like a silent one.
        sys.stderr.write(f"esp32c6.py: {path} not found "
                         "(set ESP32SIM_C6 / CONTIKI_C6_DIR)\n")
        sys.exit(2)

sys.stderr.flush()
os.execv(BIN, args)
