# Porting a Device to csim

This guide is for anyone — human or agent — adding a new device to csim. A "device" can mean a new board (same CPU, different glue), a new SoC (new CPU emulator + platform), or a new off-SoC chip (e.g. an external radio or flash). Pick your scope first; the rest of the guide branches from there.

## 1. Pick the scope

| You're adding | New CPU emulator? | New chip driver? | New platform glue? | Effort |
|---|---|---|---|---|
| A board that reuses an existing SoC and chips (e.g. another CC2538-based board) | No | No | Yes | ~½ day |
| A board with an existing SoC + new off-SoC chip (e.g. CC2538 + CC1200) | No | Yes | Yes | days |
| A board with a new SoC (different ARM/MSP430 variant) | Maybe | Maybe | Yes | days–weeks |
| Brand new CPU architecture (e.g. RISC-V) | Yes (large) | Maybe | Yes | weeks+ |

If you're not sure, default to the smallest scope. Most ports look like row 1 once you actually enumerate what's reused.

## 2. Reference ports

Before writing anything new, read the closest existing port end-to-end. Pattern-matching to an existing port is faster and safer than designing from the datasheet alone.

- **MSP430 reference**: `firmware/sky/`, `src/msp430/msp430_platform.c`, `include/msp430/msp430_platform.h`, `src/msp430/cc2420.c` (off-SoC chip example).
- **ARM Cortex-M3 reference**: `firmware/cc2538dk/`, `src/arm/arm_platform.c`, `include/arm/arm_platform.h`, `src/arm/cc2538_rfcore.c` (on-chip radio example).
- **Architecture overview**: `docs/architecture.md`.
- **CPU↔peripheral API and off-SoC chip recipe**: same file, sections "Emulation API" and "Off-SoC chips".

## 3. Write a SPEC before code

Copy [`devices/SPEC-template.md`](../devices/SPEC-template.md) to `devices/<board>/SPEC.md` and fill it in **first**. The SPEC is the contract; if it's vague, ports fail in subtle ways.

The template covers identity, CPU, console, LEDs, buttons, off-SoC chips, clock tree, known firmware quirks, reference firmware, and definition of done. If a field is unknown, leave the literal `TODO:` marker rather than guessing — unknown fields are the #1 source of port bugs.

## 4. Repository conventions

| Kind of file | Location | Naming |
|---|---|---|
| New CPU header | `include/<arch>/<arch>_*.h` | e.g. `arm_cpu.h` |
| New CPU source | `src/<arch>/<arch>_*.c` | |
| New peripheral on existing CPU | `include/<arch>/<chip>_<peripheral>.h` and `src/<arch>/<chip>_<peripheral>.c` | e.g. `cc2538_uart.h` |
| Off-SoC chip header | `include/<arch>/<chip>.h` (today) — see §6 for cross-CPU plan | e.g. `cc2420.h` |
| Off-SoC chip source | `src/<arch>/<chip>.c` | |
| Platform glue | extend `<arch>_platform.{c,h}` with a new entry in the platform table | |
| Pre-built firmware | `firmware/<board>/<name>.<board>` | extension == platform name (auto-detect in `test_mixed_multinode.c`) |
| Tests | extend `test/test_main.c` and add entries in `test_firmware.c` / `test_mixed_multinode.c` | |
| Makefile | add new `.c` files to the appropriate `*_SOURCES` list | |

The Makefile does not auto-discover sources. New `.c` files must be added to `MSP430_SOURCES`, `ARM_SOURCES`, `COMMON_SOURCES`, etc.

## 5. The test ladder

Don't gate the port on "RPL-UDP works" — that's six layers away from anything the agent or porter wrote. Build cheap, ordered checkpoints and only proceed when each one is green.

| Level | Test | What it proves | Expected wall time |
|---|---|---|---|
| **L0** | ELF loads cleanly, reset vector points into flash | Memory map is right | <100 ms |
| **L1** | Reset handler runs to `main()` without faulting | Stack init + SystemInit OK | <1 s |
| **L2** | Console UART prints an expected banner string | UART base address + GPIO mux right | <2 s |
| **L3** | LEDs toggle in a known sequence | GPIO port/pin mapping right | <5 s |
| **L4** | Contiki-NG `Starting Contiki-NG-...` and timer init lines appear | Clocks, SysTick/Timer, IRQ controller all OK | <10 s |
| **L5** | 2-node `nullnet-broadcast`: each node logs ≥1 RX | Radio TX/RX path | <30 s sim |
| **L6** | 2-node RPL-UDP: ≥1 hello/response exchange | Full stack | <60 s sim |

Each level needs a binary that exercises *only* the lower-level requirements. That means writing a small bring-up firmware:

```
firmware/<board>/bringup.<board>      ~50 lines of Contiki-NG:
                                        - prints a known banner
                                        - blinks LEDs in a fixed pattern
                                        - then while(1)
```

Build it once with [`tools/build-device-firmware.sh`](../tools/build-device-firmware.sh) — the script handles the Contiki-NG cross-compile (Docker by default, or `--local` with your host toolchain) and stamps a `PROVENANCE.md` next to the ELF so future Contiki upstream changes can't silently rebase what the test loads. Commit the ELF + the provenance entry, then add a test entry that asserts the banner string appears. This binary unblocks L0–L4 without touching the radio.

Beneath L0 sits one more layer that's worth filling in for any new chip driver: **mock-host unit tests** (see §"Test layering" below). They let you exercise an off-SoC chip's state machine without a real CPU at all.

## 6. Step-by-step

### 6.1 Add the platform config

For an ARM board, extend `arm_platform.c`'s table and the `arm_platform_config_t` struct (`include/arm/arm_platform.h`) with the new entry. For MSP430, do the same in `msp430_platform.c` / `msp430_platform.h`.

The new entry encodes everything from the SPEC: console UART index, LED port/pin, button port/pin, off-SoC chip wiring config (analogous to `msp430_cc2420_config_t`).

### 6.2 Build the host vtable

Every platform that hosts off-SoC chips fills in a `sim_host_t` once at init. Pattern from `src/msp430/msp430_platform.c`:

```c
static int64_t arm_host_now_ns(void *cpu) { return ((arm_cpu_t *)cpu)->sim_time_ns; }
static void    arm_host_schedule_ns(void *cpu, cpu_event_t *ev, int64_t fire_ns) {
    arm_schedule_event_ns((arm_cpu_t *)cpu, ev, fire_ns);
}
/* ... cancel, set_input_pin, force_irq_edge ... */

plat->host.cpu            = &plat->cpu;
plat->host.gpio           = &plat->gpio;
plat->host.now_ns         = arm_host_now_ns;
plat->host.schedule_ns    = arm_host_schedule_ns;
plat->host.cancel         = arm_host_cancel;
plat->host.set_input_pin  = arm_host_set_input_pin;
plat->host.force_irq_edge = arm_host_force_irq_edge;
```

Then pass `&plat->host` to each chip's `*_init(chip, host)` call.

### 6.3 Wire peripherals (clocks → UART → GPIO → LEDs → radio)

Always in this order. Each layer depends on the previous:

1. **Clocks**. Get the CPU running at the right frequency. If you skip this, every timing-sensitive test (UART baud, radio symbol period) will be off by a constant factor and you'll spend hours chasing it.
2. **UART**. The console is the cheapest visibility you have — it makes L2 possible. Worth getting right early even if firmware doesn't use it for much.
3. **GPIO**. LEDs are the simplest functional test of GPIO output. Buttons are the simplest test of GPIO input.
4. **Timers**. Required for L4 (Contiki-NG init does timer setup). Skipping or stubbing breaks SysTick / rtimer / etimer.
5. **Radio**. Hardest piece. Save for last. Don't start L5/L6 until L0–L4 are all green.

### 6.4 Off-SoC chips

If the device has an off-SoC chip (radio, flash, sensor):

1. Implement the chip driver in `src/<arch>/<chip>.c` and have it take `const sim_host_t *host` — never `*_cpu_t` / `*_gpio_t` directly. This is what makes the same chip portable to a different CPU later.
2. The platform installs the SPI exchange callback (`*_usart_set_spi_exchange` on MSP430, equivalent on ARM SSI when added) that routes by CSn.
3. The platform watches `gpio->output_callback` for transitions on chip control pins (CSn, VREG, RESET) and forwards to the chip's API.
4. The chip drives status pins back via `host->set_input_pin(...)`. Edge-driven IRQ requirements use `host->force_irq_edge(...)`.

See `src/msp430/cc2420.c` for the canonical example — every CPU/GPIO touch point goes through `r->host->...`, no direct types.

### 6.5 Radio integration

If the chip transmits on the air:

1. Register a TX listener via the chip's `*_set_rf_listener(chip, cb, data)`.
2. The multinode driver (`test/test_mixed_multinode.c`) routes those bytes through the radio medium and back into other nodes' chips via `*_receive_byte()`.
3. Frame format must match: 4× `0x00` preamble + SFD + length + payload + CRC. SFD value is chip-specific (CC2420 uses `0x7A`, IEEE 802.15.4 standard is `0xA7`).
4. The radio medium (`src/common/radio_medium.c`) keys on `node_id` today. If your device has two radios, see the `TODO(dual-radio)` markers in `include/common/radio_medium.h` — that work isn't done yet.

### 6.6 Test runner entries

For each new firmware that should run in CI, add an entry in the relevant table:

- `test/test_firmware.c` — for single-node boot tests (L0–L4).
- `test/test_main.c` — wire up new subcommands like `<board>-firmware` if needed.
- `test/test_mixed_multinode.c` — auto-detects platform from filename extension. A new `.zoul` extension needs an explicit case in the dispatch.

Add the new test commands to `.github/workflows/test.yml` so PRs run them.

## 7. Test layering — where each kind of test belongs

| Layer | What | Example | When to use |
|---|---|---|---|
| Below L0 | Chip-driver unit tests via [`mock_sim_host_t`](../include/common/mock_sim_host.h) | [`test/test_mock_host.c`](../test/test_mock_host.c) | Any new off-SoC chip. Drive the chip's state machine without a real CPU; assert exact `set_input_pin` / `force_irq_edge` / `schedule_ns` call sequences. Catches bugs that won't surface until L4–L6 otherwise. |
| L0–L1 | Instruction-level CPU tests | `test_runner correctness` / `arm-correctness` | Reuse existing tests if reusing the SoC. Add new tests only when you've added a new CPU emulator. |
| L2–L4 | SoC peripheral tests via the bring-up firmware | `test_runner <platform>-firmware` | Mandatory for every port. The bring-up firmware exists to make these levels reach. |
| L5–L6 | Radio + full Contiki stack | `test_runner <platform>-multinode` | Run real Contiki firmware through the radio medium. The integration oracle. |

Don't skip the chip-driver unit tests if your port adds an off-SoC chip — they pay for themselves the first time a state-machine bug would otherwise surface as "RPL doesn't converge after 60 seconds."

## 8. Pitfalls catalog (what previous ports got wrong)

These are real issues from past porting work. Watch for the *category*, not just the literal bug:

- **Register offset mismatches.** Datasheets sometimes list register addresses relative to different bases (peripheral base vs. SFR base). Triple-check by reading the firmware's actual access pattern before trusting the datasheet table. Example: CC2538 FFSM registers were at the wrong offset in early code — silent failure, dropped writes.
- **Bit-position errors.** A single off-by-one in a status flag bit (e.g. FIFO at bit 7 vs. bit 5) will boot fine, then fail under specific load patterns. Fail mode: silent, intermittent.
- **Stale time during multi-cycle batches.** `sim_time_ns` is updated at the *end* of `step_until` batches, not after every instruction. Peripherals reading it during a busy-wait will see a frozen value. Compute from cycles + freq instead. Example: sleep timer `dl-miss` errors.
- **Address filter / auto-ACK.** ACKs sent without checking destination address generate spurious traffic and break MAC layers that expect strict ACK matching. Always implement the dest PAN/short/extended check before generating ACK.
- **RXFIFO overflow during replay.** Buffered RX bytes from concurrent transmissions (collisions on real hardware) must be discarded, not replayed when the radio re-enters RX. Replaying triggers overflows that real hardware never sees.
- **Symbol/byte timing not in nanoseconds.** All radio timing should be in ns and use `*_schedule_event_ns()`, not cycles. Otherwise DCO calibration breaks the timing. Already a project-wide invariant; don't break it.
- **Forgotten init quirks.** Real firmware often does undocumented init writes to "reserved" registers. If the firmware halts early, log every IO write before the halt — the missing one is usually obvious in retrospect.
- **Synchronous side effects in chip-driver byte handlers — fix with events, not `step_node_until`.** When a chip driver receives an on-air byte and needs to surface a state change to firmware (raise an IRQ, drop a status pin, transition state), the change MUST be scheduled via `host->schedule_ns()` rather than acted on synchronously inside the byte-delivery callback. The simulator's main loop is event-driven: it advances `sim_time_ns` to the next scheduled event, then steps each node up to that time. If a chip driver pends an IRQ synchronously and schedules nothing, the receiver CPU will not be woken until its next periodic wakeup — by which point the sender's MAC-level ACK_WAIT has typically already expired. Symptom: the firmware appears to ignore RX'd frames; ACKs are never sent in time; CSMA retransmits and the network never converges. Anti-pattern: working around this by inserting `step_node_until(...)` calls in the harness's per-byte delivery path. Always check the canonical pattern in `src/msp430/cc2420.c` (every state inflection has a `HOST_SCHEDULE_NS` call). This was the root cause of the original CC1200/Firefly L6 RPL-UDP non-convergence — `src/arm/cc1200.c`'s end-of-frame GDO0 drop was inline; moving it onto a `frame_done_event` scheduled one byte-period after the last on-air CRC byte was the fix. See git log for `cc1200: event-driven RX frame-done`.
- **L6 convergence failure ≠ csim bug. Validate firmware on hardware first.** When 2-node L6 (RPL-UDP) doesn't converge after L0–L5 are all green, the temptation is to chase csim emulation gaps. Stop and run the *same* firmware on real hardware before any deeper csim diagnosis. The Firefly/CC1200 port spent days investigating "rx_incoming during turnaround" and "Node 1 CPU starvation" as csim fidelity gaps; on real Firefly silicon the same firmware showed the *same* convergence failure, which immediately re-pointed the investigation at upstream Contiki-NG. Two firmware bugs followed: `CSMA_CONF_ACK_WAIT_TIME` (5 ms) was below the actual cc1200 ACK round-trip on SUN FSK 50 kbps (~12.5 ms), and `pending_packet()` polled tightly enough to starve the cc1200 RX IRQ chain over SPI. Neither needed csim changes. Rule of thumb: if you've burned >1 hour of investigation without a hypothesis pinning the failure to csim specifically, run on hardware.
- **Don't add fidelity buffers to mask firmware races.** It's tempting to mirror `src/msp430/cc2420.c`'s `rx_incoming[]` buffer in any new chip driver where bytes get dropped during state transitions. Confirm first that the buffer corresponds to a *documented* hardware behavior — cc2420's buffers bytes during a well-defined RX→TX→RX turnaround window per the datasheet — and is not just a workaround for a firmware-side race that csim is faithfully exposing. `devices/zoul-firefly/archive/CC1200-RX-ACK-CHAIN.md` proposed exactly such a buffer for cc1200; with hindsight it would have hidden a real CSMA bug. csim's job is to reproduce hardware faithfully, including its firmware-level bugs. A faithful simulator's silver bullet is to make hardware bugs reproducible, not to paper over them.
- **Datasheet citations are mandatory for chip-behavior commits.** Every commit that changes a chip driver's externally observable behavior (signal output, state transition, register semantics, timing) should cite a datasheet page in the commit body. If you can't, it's a guess and should not land. The cc1200 port had at least one such guess (`1a694cd cc1200: drive GDO0/GDO2 on sync match regardless of IOCFG selection`) which was reverted in `71acbb4` once the IOCFG semantics were properly modeled. The `devices/zoul-firefly/DATASHEET-FINDINGS.md` artifact pattern (a table of citations per finding) emerged as a recovery mechanism — make it the rule from day one, not a way to back-fill correctness later.
- **A new device often exposes pre-existing simulator infrastructure debt — scope it separately.** The CC1200 port triggered a 15-commit refactor of the radio medium to support per-radio multi-channel state (`8a2d03b` → `72665bb`), which fixed a pre-existing "TSCH channel matching is fake" gap that affected every test on every platform. When you find this pattern (a device's correct behavior requires changing simulator infrastructure that was always wrong), finish the port first against whatever the medium currently exposes, then file the infrastructure fix as its own series. Bundling them ties two unrelated risks together and makes review nearly impossible.

When you hit a new bug, add it here.

## 9. Definition of done — checklist

A port is considered done when **all** of the following are true. Paste actual command output as evidence in the PR:

- [ ] `devices/<board>/SPEC.md` exists, no `TODO` fields remain.
- [ ] `make clean && make` succeeds with zero new warnings.
- [ ] Firmware ELFs committed under `firmware/<board>/` (bring-up + at least one networking firmware) **with a `PROVENANCE.md` next to them** (auto-stamped by `tools/build-device-firmware.sh`).
- [ ] If a new off-SoC chip was added: at least one mock-host unit test in `test/test_<chip>.c` exercising the chip's state machine without a real CPU.
- [ ] `./build/test_runner <platform>-firmware` passes (covers L0–L4).
- [ ] `./build/test_runner <platform>-multinode <bringup>.<board> -t 5000 -q` passes (L5 if applicable).
- [ ] If radio-equipped: 2-node nullnet-broadcast logs ≥1 RX per node within 30 s sim.
- [ ] If full stack: 2-node RPL-UDP exchanges ≥1 hello/response within 60 s sim.
- [ ] `.github/workflows/test.yml` runs the new tests on PR.
- [ ] No CPU/GPIO type leaks into off-SoC chip drivers (chips take `sim_host_t`).
- [ ] `docs/architecture.md` Platforms table updated with the new entry.
- [ ] Every commit that changes externally observable chip-driver behavior cites a datasheet page in the commit message body. Guesses don't land.
- [ ] If you suspected a csim emulation gap and weren't sure: at least one run of the same firmware on real hardware, with the result recorded in a `HARDWARE-TEST.md` next to the SPEC. Always do this before declaring an L6 failure to be a csim bug.

## 10. Closing out a port

A port doesn't end when L6 turns green — it ends when the project tree no longer leaks investigation noise into the next contributor's reading. Stale plan docs are worse than no plan docs because they read as if there's open work that doesn't exist. When the port is done:

1. **Archive the trail.** Move investigation/diagnostic docs (`L*-PLAN.md`, `*-RX-CHAIN.md`, intermediate audits, anything that was written *during* the investigation) into `devices/<board>/archive/`. Keep only the steady-state docs (`SPEC.md`, `STATUS.md`, `DATASHEET-FINDINGS.md`, `HARDWARE-TEST.md` if a real hardware test happened) at the top level of `devices/<board>/`.
2. **Add an `archive/README.md`** that says, in 2-3 sentences, what the archived docs were for and what their final disposition was (resolved, superseded, etc.). The point is to make it cheap for a future reader to decide "I don't need to read this."
3. **Update `STATUS.md` to reflect the closed state.** Re-read it cold. If any sentence implies open work that isn't open, fix it. The Zoul `STATUS.md` originally said "see L6-PLAN.md for the operational task list" weeks after L6 was resolved — actively misleading.
4. **Single closeout commit.** "<board>: archive investigation trail, port complete" — makes the close-out self-documenting in `git log`.

Reference example: [`devices/zoul-firefly/archive/`](../devices/zoul-firefly/archive/) and the surrounding `STATUS.md` rewrite.

## 11. For autonomous agents

If you're driving a port without continuous human supervision:

1. **Stop on ambiguity.** If the SPEC has a `TODO` or the datasheet contradicts itself, halt and report — never guess. A guess that boots is worse than a halt that reports the gap.
2. **Work bottom-up.** Don't write radio code until L4 passes. Don't write peripheral code until L1 passes. Skipping levels makes failures multi-causal and ten times harder to diagnose.
3. **Show evidence per level.** When you claim a level passed, paste the literal `test_runner` output. "Probably works" is not a level pass.
4. **Bound your edits.** If you've made >2000 lines of changes without a green test, stop and summarize what you tried. Long edit chains compound errors.
5. **Reuse, don't reimplement.** Same SoC = reuse the SoC source files. Same chip = reuse the chip source file (now possible thanks to `sim_host_t`). New code is a last resort.
6. **No CC2538 / MSP430 emulator changes.** Unless the SPEC explicitly says the SoC is new or buggy, edits to existing CPU emulators are out of scope. Any change there is a regression risk for every existing port.
7. **Commit per level.** One commit per L0/L1/.../L6 makes review and rollback trivial. Squash at the end if you prefer.

The autonomous loop should look like: scaffolding → L0–L2 → L3–L4 → radio (L5–L6), with a human checkpoint after L4 before the agent touches radio code. Radio bugs are expensive to debug; a human glance before committing the agent to that phase usually saves hours.
