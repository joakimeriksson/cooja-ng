# Zephyr nRF52840 DAD-stall — test / debug plan

Stock Zephyr `echo_server` / `echo_client` (with the sample's own
`overlay-802154.conf`) reach `net_config` but **never complete Duplicate Address
Detection** on csim's emulated nRF52840 → `Network initialization failed (-116)`.
Reproduces for a lone node *and* two nodes.

**Goal:** decide whether this is a **csim emulator bug** or **stock-firmware /
config behaviour**, then fix the former. Firmware stays STOCK — all fixes go in
csim. (Only the sample overlay + a net-debug log overlay for visibility.)

## Already verified — do NOT re-investigate
- Radio CCA/TX work (6 TX, 6 `nrf_802154_transmitted_raw`, 0 failed); CLOCK /
  RTC1 ISR / `sys_clock_announce_locked` run; the nrf_802154 critical-section
  counter balances `0→1→0` (not stuck).
- The context-switch instructions are **correct**: `LDRD`/`STRD` (field decode +
  body), `MSR`/`MRS PSP`/`MSP`/`CONTROL`/SPSEL, exception entry/return stacking.
  So this is **not** a broken-instruction bug like the ADC/SDIV ones.
- `dad_timeout` (the DAD-completion work, `0xf424` in echo-s-nd) **never runs** →
  the address stays `tentative`. A workq handler (`mgmt_event_work_handler`) is
  stuck (dispatch `0x1fdb4`=8 vs return `0x1fdb6`=7). `k_yield` *returns*, so it
  is a **blocked callback**, not a lost-from-ready-queue scheduler bug.
- Config: `NET_MGMT_EVENT_THREAD=y` (dedicated net-mgmt thread, separate from
  `k_sys_work_q` which owns DAD), `NET_MGMT_EVENT_QUEUE_SIZE=5`, `TC_TX_COUNT=0`.

## Track A — GDB stub (sandbox; do first, no hardware)
Answers *"where is csim wrong?"* — cuts through the multi-thread ambiguity that
stopped the trace-from-outside approach. The runner supports `--gdb [node:]port`
+ `--gdb-wait` (`src/services/gdb_service.c`).

```sh
./build/test_runner nrf52840-dk-multinode /tmp/echo-s-nd.nrf52840-dk -n 1 \
    -t 30000 --gdb 3333 --gdb-wait &
arm-none-eabi-gdb /tmp/echo-s-nd/zephyr/zephyr.elf -ex 'target remote :3333'
```
In GDB: list the Zephyr threads (thread-aware helper, or walk `_kernel.threads` /
the ready queue manually), find the **`k_sys_work_q`** thread, and read its
blocked state + the object it is pended on.
- A thread waiting on a sem/event csim **never delivers** → **csim bug**: fix it.
- Everything *legitimately* blocked, no csim fault visible → go to Track B.

## Track B — two real nRF52840-DK boards (Mac; tiebreaker)
Answers *"is csim wrong at all?"* Flashable, host-independent firmware (copy the
`.hex` from this Linux box, or rebuild on the Mac):
- server: `/tmp/echo-s-nd/zephyr/zephyr.hex`
- client: `/tmp/echo-client-nd/zephyr/zephyr.hex`

Rebuild (if needed):
```sh
source <venv>/bin/activate
export ZEPHYR_TOOLCHAIN_VARIANT=gnuarmemb GNUARMEMB_TOOLCHAIN_PATH=/usr   # or the SDK
west build -p -b nrf52840dk/nrf52840 -d build/echo-s-nd \
    zephyr/samples/net/sockets/echo_server \
    -- -DEXTRA_CONF_FILE="overlay-802154.conf;netdbg.conf"
```
(`netdbg.conf` = `CONFIG_LOG`/`LOG_MODE_IMMEDIATE` + `NET_*_LOG_LEVEL_DBG` so DAD
prints over the SEGGER VCP.)

Flash + monitor:
```sh
nrfjprog --ids                          # the two board serial numbers
west flash -d <dir> --dev-id <SNR>      # or: nrfjprog --program zephyr.hex --chiperase -r
screen /dev/tty.usbmodem<...> 115200    # monitor each board's console
```

**Simplest decisive test first — lone `echo_server` on ONE board.** A lone node
cannot have a duplicate, so DAD *should* complete:
- address goes `tentative → preferred`, "Run echo server", **no -116** → **csim
  bug** (Track A localizes it).
- times out `-116` exactly like csim → **firmware/config**; csim is faithful —
  stop digging in csim.

Then `echo_server` + `echo_client` on two boards for the full UDP echo
round-trip (watch both consoles for the request/echo).

## Decision
```
Track A finds a csim fault  → fix csim, done.
Track A inconclusive        → Track B.
  Track B lone-server completes DAD → csim bug (back to Track A with the fact).
  Track B lone-server times out too → not csim; revisit the firmware/config.
```
