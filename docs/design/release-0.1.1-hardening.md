# Release Hardening Plan — 0.1.1 Stabilization

Status: **complete (2026-07-25)** — shipped in the `v0.1.0` release, not as a
separate 0.1.1 (see the amended §0 note below). Companion to
[`refactor-plan.md`](refactor-plan.md). Source: full subsystem audit (kernel /
ARM / MSP430+common / motes-native-riscv / services-ui-runner), plus the
radio-CRC fix already landed as `6bc0402`.

**Progress:** Tranche A (A1–A6), Tranche B (B1–B7), Tranche C, and the §5
config triage (T4) are **done** and merged to `main`. **T3 is fixed**, not
deferred: the nRF54L15 TX-completion event is now scheduled in cycles
(`71a85d7`), and the follow-on multi-hop failure on *both* nRF radios — an
aborted RX leaving `psdu_being_received` set — is fixed in `8b7d84c`. Two-node
and 3/4-node chains route end-to-end on nRF52840 and nRF54L15, each with a
regression config.

**Still open:** the §4 stand-alone unit tests (ws-frame, elf-malformed,
FLPR-WFI, NVIC-dual-pending, periodic-GPTimer). Those fixes are covered only
by integration gates today; DADD is the one with its own regression test. This
is the plan's stated bar that the release does **not** yet meet.

## 0. Framing and key decisions

- ~~**`v0.1.0` is already tagged (2026-06-22).** Do **not** move or re-cut it.
  This work ships as **`v0.1.1`**.~~ **Superseded (2026-07-25):** the earlier
  `v0.1.0` tag was never published, so rather than ship a 0.1.1 on top of an
  unreleased 0.1.0, the tag is re-cut on the current `main` and this audit
  ships *inside* `v0.1.0` as the first public release. The document keeps its
  0.1.1 filename and item numbering for traceability. No CLI, config, or
  plugin-ABI changes are in scope — every item below is a correctness, memory-
  safety, robustness, or internal-refactor fix. (§0.x SemVer note in the
  CHANGELOG permits behaviour adjustments in a minor bump if we later decide to
  call it 0.2.0; 0.1.1 is the honest label for "same features, fixed".)
- **Release gate = correctness + safety, not completeness.** A finding blocks
  0.1.1 if it (a) produces wrong simulation results on a supported/advertised
  path, (b) is an externally reachable crash/corruption, or (c) corrupts memory
  on plausible input. Everything else is scheduled but non-blocking.
- **Every blocker fix lands with a regression test** (§4). Most of these bugs
  exist *because* there is no test on that path; a release that fixes them
  without locking them is not stabilized. This is the difference between 0.1.1
  and "0.1.0 with patches."
- **Discipline carries over from `refactor-plan.md`:** the broad test gate (§3)
  is mandatory before merging each tranche; determinism reproducibility check is
  in the gate; a regressed tranche is reverted, not fixed forward.
- **Big refactors are out of scope for 0.1.1** and tracked separately as a 0.2.0
  track (§5). They are divergence risks, not release blockers, and doing them
  under release pressure is how the next bug gets introduced.

## 1. Tranche A — Release blockers: correctness & memory safety

Each: **file:line → fix → test → risk**. Land in ~this order (safety-reachable
first). All are S/M effort.

**A1. WebSocket server: validate payload length; kill the OOB `memmove`.** `HIGH`
`src/ui/ws_server.c:242,272`. `int total = header_len + (int)payload_len`
truncates a 64-bit attacker-controlled length negative; the `recv_len < total`
guard then passes and `memmove(buf, buf+total, recv_len-total)` runs wild.
- Fix: reject any frame with `payload_len > RECV_BUF - header_len` (close the
  client); keep length arithmetic in `uint64_t`/`size_t`; while here, enforce the
  client-mask-required rule and drop frames whose declared length exceeds the
  buffer instead of force-closing mid-stream.
- Test: new `test_ws_frame` unit suite feeding crafted frame headers (the
  0x80000000-length frame, an unmasked frame, a 126/127 boundary) to the parser.
- Risk: low; parser is self-contained. Effort S.

**A2. Native mote: clamp all firmware-controlled lengths.** `HIGH`
`src/motes/native_cooja_mote.c:347` (RX `memcpy` no clamp — sibling
`native_node.c:341` clamps), fed by `native_node.c:311` (`frame_len =
*simOutSize`, unclamped) and `:289` (`simLoggedLength`, unclamped read).
- Fix: clamp to the documented `[128]` buffer at all three sites; make the fast
  path and the deferred path share one clamp helper so they can't diverge again.
- Test: extend the mixed-multinode harness (or a focused unit) with a frame
  ≥129 bytes; assert no corruption and a bounded copy.
- Risk: low. Effort S.

**A3. nRF54L15: per-instance SUBSCRIBE/EGU/TIMER binding context.** `HIGH`
`src/arm/nrf54l15_soc.c:731,995,1856` — `egu_sub_bindings[]`,
`radio_sub_bindings[]`, `timer_cc_ctx[2][]` are file-scope `static`, keyed by
slot not node, so in a multi-node sim the last node to program a slot owns every
other node's callback. (nrf52840 keys off per-`soc` heap structs — correct;
this is 54L-only.)
- Fix: move the three tables into `nrf54l15_soc_t` (per instance); update the
  DPPI/timer registrations to pass `&soc->...[idx]`.
- Test: **coupled with A3-test** — bring up a 2-node nRF54L15 scenario (today
  only `chain-3node-nrf54l15-dk.json` exists and it fails; see §2/T3). A green
  2-node 54L15 test is the proof this is fixed *and* closes a real coverage gap.
- Risk: medium (touches the DPPI fan-out wiring). Effort M.

**A4. Malformed-ELF overflow guards.** `HIGH (robustness)`
`src/msp430/msp430_elf.c:11`, `src/arm/arm_elf.c` — `paddr + size > max_mem` on
unchecked `uint32_t` from `p_paddr`/`p_filesz` wraps and returns a wild pointer
that `elf_load_segments` then `fread`s into. Also `elf_loader.c:110` uses
`sh_link`/`sh_size` unvalidated (multi-GB `malloc` DoS).
- Fix: `paddr < max_mem && size <= max_mem - paddr` (no addition) in both route
  callbacks; validate `sh_link < e_shnum` and cap `sh_size` in `elf_find_symbol`.
- Test: `test_elf_malformed` — hand-built ELF headers with wrapping `p_paddr`,
  oversized `p_filesz`, out-of-range `sh_link`; assert clean rejection, no write.
- Risk: low (rejection path). Effort S.

**A5. MSP430 `DADD`: real BCD add + carry; de-duplicate the ALU switch.** `HIGH`
`src/msp430/msp430_cpu.c:2045` and the copy at `:592` — `dst+src+carry` binary,
carry flag never set. Wrong for any BCD firmware.
- Fix: implement per-nibble BCD addition setting C on decimal overflow. **Do it
  once** by collapsing the interpreter ALU switch (`:1977`) and
  `execute_decoded` (`:529`) onto a shared helper — the duplication is *why* the
  bug exists in two places (this is the one refactor pulled into Tranche A
  because it's the correct vehicle for the fix, per the "add the fix at the
  seam" principle).
- Test: add BCD cases to `test_correctness` (`0x09 DADD 0x01 = 0x10`, carry
  chains, C flag) — the suite currently has **zero** BCD coverage.
- Risk: medium (shared-helper refactor). Effort M. Validate byte-identical on
  the non-DADD opcodes via the existing 72 tests + `JIT_VERIFY`.

**A6. `sim_config.c` `ftell` guard.** `MED (cheap, blocker-adjacent)`
`src/sim/sim_config.c:36` — unchecked `ftell` returns −1 on a directory →
`malloc(0)`, `fread(SIZE_MAX)`, `buf[-1]='\0'`.
- Fix: `if (len < 0 || len > SANE_MAX) { error }`.
- Test: `sim_config_load` on a directory path returns −1 cleanly.
- Risk: none. Effort S.

## 2. Tranche B — Correctness: peripheral & CPU edge cases

Strongly recommended for 0.1.1; narrower triggers than A. Ship if they fit;
otherwise document in Known Issues (§6) and pull to 0.1.2. Each has a test target.

- **B1. ARM NVIC PendSV/SysTick share one `pending_exception` slot.** `MED`
  `arm_nvic.c:59`. A SysTick landing on a pending PendSV drops the context
  switch. Fix: separate pending latches (bitmask) for the two system exceptions;
  take by priority. Test: pend both, step, assert both taken in priority order.
  Relevant to Zephyr/RTOS scheduling. Effort M.
- **B2. RISC-V WFI + machine-IRQ order.** `MED` `riscv_cpu.c:190,214,191`. WFI
  won't wake with `MIE=0` and a pending source (spec: WFI resumes regardless of
  MIE); and priority is coded MEI>MTI>MSI vs spec MEI>MSI>MTI. Fix: wake the
  hart when `mip & mie != 0` independent of global MIE; correct the tie-break.
  Test: FLPR unit — WFI with MIE=0 + pending timer resumes. Effort S.
- **B3. CC2538 GPTimer raises no IRQ, ignores periodic mode.** `MED`
  `cc2538_gptimer.c:11,167`. Fix: wire `ta_event.callback` → `arm_nvic_set_pending`;
  reload on `mode==0x02`. Test: firmware or unit that arms a periodic GPTimer
  interrupt and counts fires. Effort M.
- **B4. MSP430 JIT ADDC drops the V flag.** `MED` `msp430_jit.c:143`. Fix:
  either emit V for ADDC or add ADDC to the `can_inline` reject list (`:65`)
  alongside SUBC/DADD — simplest correct option. Test: `JIT_VERIFY` warm-block
  parity on an ADDC-then-JGE sequence. Effort S.
- **B5. MSP430 RRCM carry bit (`msp430_cpu.c:1284`, off-by-two vs RRAM/RRUM) +
  Timer Up/Down modeled as Continuous (`msp430_timer.c:322`).** `MED/LOW`.
  RRCM: confirm expected bit against Java MSPSim, then match the sibling ops.
  Up/Down: model count-to-CCR0-and-back. Tests: instruction test for RRCM;
  timer-cadence test for MC=3. Effort M (verify-gated).
- **B6. SMC cache-invalidation window narrower than a block.** `MED`
  `msp430_cpu.c:73`. A write into a compiled block's interior (blocks span ≤32
  insns) doesn't invalidate it. Fix: invalidate any block whose `[start, start+
  len)` covers the write, or widen the clear to `MAX_BLOCK_SIZE`. JIT-build only.
  Test: RAM-resident self-patching routine. Effort M.
- **B7. Bounds/ordering hygiene in shared infra.** `LOW/MED`. Clamp
  `radio_medium` `node_count` to `RADIO_MEDIUM_MAX_NODES` (`radio_medium.c:245`);
  in `sim_eq_schedule_gen` (`sim_event_queue.c:99`) remove-existing before the
  capacity check so a full-queue reschedule replaces instead of dropping. Tests:
  128+ node guard; full-queue reschedule keeps the mote alive. Effort S.

## 3. Tranche C — Robustness & hygiene

Non-blocking; batch into 0.1.1 if time permits, else 0.1.2. Grouped by theme.

- **Resource-leak / cleanup paths:** `native_node.c:87` dlopen+temp leak on
  `dlsym` failure (add `dlclose`+`unlink`); `sim_external_command.c:64` fork-fail
  leaves observer subscribed + FILE\* open; runner JS-script failure paths
  (`test_mixed_multinode.c:1953,2002`) skip `ss_cleanup`/`pcap_close`/
  `gdb_destroy`/`ui_destroy`. Fix: single `goto fail` cleanup ladder in each.
- **Native `/tmp` dylib → `mkstemp`.** `native_node.c:60` predictable name +
  symlink-following `fopen("wb")` before `dlopen` (CWE-377/59). Local-only, but
  cheap to fix with `mkstemp` + `fchmod`.
- **`sim_service` re-entrancy guard is `assert`-only** (`sim_service.c:73`) —
  vanishes under `-DNDEBUG`. Make it a real early-return + counter.
- **File-write error handling:** `pcap_writer.c:53` ignores every `fwrite`;
  `energest_engine.c:96` `publish_panel` can emit truncated (invalid) JSON. Add
  short-write checks / a truncation-safe closer.
- **Input hardening:** board detection keys on last dot in the whole path
  (`sim_board.c:46`) — use the basename; UI `move` node-index
  (`websocket_ui_service.c:62`) trusts `valueint` — bounds-check.
- **Low-severity CPU/periph:** NVIC IPR read 3 bytes past `ipr[]`
  (`arm_nvic.c:42`); SVC is a NOP (no SVCall) — document as unsupported unless an
  advertised RTOS path needs it; MSP430 CS HFXT case returns DCO
  (`msp430_clock.c:153`) — document or model.

## 4. Test additions that lock the release (do alongside A/B)

The release's real deliverable is that these paths now have coverage:

- `test_ws_frame` — WebSocket frame-parser unit (A1).
- `test_elf_malformed` — crafted-ELF rejection (A4).
- MSP430 BCD/DADD + RRCM cases in `test_correctness` (A5/B5).
- **2-node nRF54L15 multinode test** — new mode/config; proves A3 and closes the
  biggest platform coverage gap (§2/T3 below).
- FLPR WFI-wake unit (B2); periodic-GPTimer-IRQ test (B3); NVIC dual-pending
  test (B1).
- Re-run the issue #5 EWSN-style contention scenario and record the parse-
  failure count → 0 in the release notes (validates `6bc0402` end-to-end).

## 5. Pre-existing failing configs — triage before tag

These fail on unmodified `main` today (per the `preexisting-failing-configs`
memory) and must each be either fixed or explicitly listed as Known Issues:

- **T1/T2. `chain-4node-nrf52840-dk.json` / `-dongle.json`** — 0 receptions.
  Investigate (likely multi-hop RX/ramp timing, unrelated to CRC). If not fixed
  for 0.1.1 → Known Issue: "nRF52840 4-node multi-hop chains do not route."
- **T3. nRF54L15 multinode radio RX is broken — ROOT-CAUSED (2026-07-07).**
  2-node `configs/test-2node-nrf54l15-dk.json` never routes: the client stays
  "Not reachable yet" and never transmits (it only TXes after receiving a DIO,
  which never arrives). Verified **identical on the pre-CRC tree (d4c7b73)** and
  pre-A3 — so it is neither an A3 nor a `6bc0402` regression; 54L15 two-node
  radio reception has never worked. The sender transmits fine and the bus
  delivers the bytes (confirmed via `CSIM_TRACE_RADIO`); the failure is entirely
  in the 54L15 **receive** model, a stack of ≥2 layered bugs (via
  `NRF54L_RADIO_TRACE`):
  1. **Deferred-disable timeout fires mid-frame.** On the driver's
     BCMATCH→`TASKS_DISABLE`, the model defers the disable via a fixed **5 µs**
     safety timeout (`nrf54l15_soc.c` ~1313). A frame's air-time is 32 µs *per
     byte* × 20+ bytes, so the 5 µs timeout always fires mid-frame, snaps the
     radio to DISABLED, and the remaining bytes land in a re-armed phase-0
     parser as garbage — the frame never completes. (The in-code comment even
     claims "5 µs ... well above the ... byte period (32 µs)," which is
     backwards.) Fix direction: make it a *bytes-stopped* watchdog (reset on
     each RX byte while `rx_disable_pending`, with a period > 32 µs), so a live
     frame completes naturally and only a truly-aborted one trips it.
  2. **Completed frames fail CRC.** With the timeout extended past a frame's
     air-time (experiment), the frame completes but fires **CRCERROR**, so it is
     still dropped. The RX byte stream shows intermittent **double-delivery**
     (same byte at the same cycle twice in the trace) — reception is corrupted,
     not a CRC-convention mismatch. Needs its own investigation.
  This is deep radio-timing work in the most fragile part of the model, **not a
  release blocker** (the FLPR single-node feature is unaffected). Recommend
  shipping 0.1.1 with it as a documented Known Issue and a focused follow-up.
  A3 remains correct and orthogonal (it removes cross-node binding-state
  sharing, which would corrupt a *working* multinode 54L15 the moment T3 is
  fixed).
- **T4. `mixed-sky-native.json`** — references `firmware/cooja/udp-client.cooja`
  which was never built. Fix the config to point at an existing `.cooja` (e.g.
  `udp-sender.cooja`) or drop it. Trivial; do it.

## 6. Release mechanics (0.1.1)

1. Land Tranche A + its tests; run the full broad gate (below).
2. Land as much of Tranche B/C as fits; document the rest in Known Issues.
3. Do the §5 config triage.
4. `CHANGELOG.md`: add a `## [0.1.1] — <date>` section — `### Fixed` (A/B/C by
   subsystem), a **Known Issues** block (unfixed T1–T3, deferred B items,
   unsupported SVC/HFXT). Reference `6bc0402` under Fixed → radio.
5. Update `CLAUDE.md` only where behaviour changed (none of these change the
   documented test commands; the nRF radio CRC note may warrant a line).
6. Tag `v0.1.1` **after** the gate is green on the release commit.

**Broad gate (mandatory before tagging), reusing `refactor-plan.md` §10:**
```
./build/test_runner correctness              # + new BCD/RRCM cases
./build/test_runner arm-correctness          # 153+
./build/test_runner radio-medium             # 241
./build/test_runner cc1200-mock-host         # 73
./build/test_runner firmware; arm-firmware
# 2-node: sky nullnet + rpl-udp; cc2538dk/nrf52840-dk/dongle rpl-udp
# TSCH: cc2538dk + nrf52840-dk association + held-sync
# Zephyr echo (0 timeouts); nRF54L15 FLPR dual-core; NEW 2-node 54L15
./build/test_runner test <new unit suites>   # ws_frame, elf_malformed, ...
make cooja-tests                             # must not drop below 81/81
# determinism: same seed → identical event trace
```

## 7. Execution sequence & parallelism

- **Wave 1 (parallel, independent files):** A1 (ws_server), A4 (elf), A6
  (config), B2 (riscv), B4 (msp430 jit). All small, no shared surface.
- **Wave 2:** A2 (native clamps) + C native-cleanup together; A5 (DADD + ALU
  de-dup) as a focused change with `JIT_VERIFY`.
- **Wave 3:** A3 (54L15 statics) + the new 2-node 54L15 test together — the
  highest-risk change, done once the safety fixes are in and the gate is trusted.
- **Wave 4:** remaining B (B1/B3/B5/B6/B7) + C hygiene + §5 config triage, as
  capacity allows; then §6 release mechanics.

Each wave ends on the broad gate. A wave that regresses the gate is reverted per
the §11.5 rollback rule, not fixed forward.

## 8. Explicitly deferred to a 0.2.0 refactor track (NOT in 0.1.1)

Divergence-risk cleanups, valuable but not release blockers:

- **D1. nRF SoC RX/event/shorts consolidation** (~700 near-duplicate lines
  between `nrf52840_soc.c` / `nrf54l15_soc.c`; only TX-emit is shared today and
  the two are already diverging). Highest-value refactor; do it *after* A3 so we
  consolidate correct code.
- **D2. Complete the "pure frontend" runner** — `test_mixed_multinode.c` still
  has ~143 chip/kernel reach-ins (mote_cc2420, hand-rolled SFD parsing, bus_host
  accounting, chip stat statics). Contradicts the Phase-10 "pure frontend" claim.
- **D3. Move chip wire-format knowledge out of the kernel radio bus**
  (`sim_radio_bus.c:19-137` hardcodes CC2420/CC1200 framing + band convention).
- **D4. Duplication collapses:** clock-deviation block (×4 across mote modules),
  tickless-counter idiom (×6 across timers — each with its own wrap handling),
  `native_radio.c:18` CRC/bitrev (last un-consolidated copy), a shared PHY-timing
  constants header (32µs/byte, 192µs turnaround, ramp cycles).
- **D5. Dead-code removal:** runner "Phase Timing" (4 always-0 phases),
  `js_node.h` `line_buf`/`line_pos` + `NODE_OPAQUE_TAG`, `native_radio.c:107`
  unreachable state, `msp430_cpu.c:756` `JIT_VERIFY` block if unused.
- **D6. FLPR launch primitive** can't express CPURUN 1→0 stop / re-launch /
  INITPC refresh (`nrf54l_vpr.c:35`) — model limitation; only matters if
  firmware restarts the coprocessor.
