#!/usr/bin/env python3
"""
ESP32-C6 running the standard 6LoWPAN/RPL/UDP stack -- the rpl-udp client of
Contiki-NG, on an unmodified ESP-IDF image, emulated by esp32sim.

Identical to esp32c6.py except for which build directory it runs: this one
defaults to an RPL/UDP build rather than the NullNet one, so a config can
pick the network stack without setting an environment variable.

    { "firmware": "examples/ext/esp32c6-rpl.py", "id": 2, "x": 3.0, "y": 0.0 }

Why it matters which: NullNet is Contiki-specific glue with no network layer,
so interoperating over it proves only that 802.15.4 frames cross. The RPL
build runs IPv6 over 6LoWPAN with RPL routing and UDP on top -- IETF
protocols -- which is what makes cross-simulator interoperation a claim about
standards rather than about one project's example code.

Environment: as esp32c6.py, but CONTIKI_C6_DIR defaults to build-rpl.
"""
import os
import shlex
import sys

HOME = os.path.expanduser("~")
BIN = os.environ.get("ESP32SIM_C6", f"{HOME}/work/esp32sim/target/release/esp32sim-c6")
BUILD = os.environ.get("CONTIKI_C6_DIR", f"{HOME}/work/esp32/esp32-contiki/build-rpl")
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
        sys.stderr.write(f"esp32c6-rpl.py: {path} not found "
                         "(set ESP32SIM_C6 / CONTIKI_C6_DIR)\n")
        sys.exit(2)

sys.stderr.flush()
os.execv(BIN, args)
