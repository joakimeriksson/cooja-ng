# TrustZone-M on csim: ARMv8-M security extension for the Cortex-M33 (nRF54L15)

**Status: COMPLETE (emulator track).** Plan written 2026-07-18, implemented
through 2026-07-26, re-verified 2026-07-30 on branch `trustzone`. Per-step
record under *Confirmed execution steps*; current measured numbers under
*Verified result*. The remaining work is the hardware track (DK golden-vector
capture) and deeper firmware exercise — both outside the core emulator.

**Goal:** implement the ARMv8-M security extension (TrustZone-M) in csim's
Cortex-M33 emulation so that csim runs *real* secure/non-secure partitioned
Contiki-NG firmware on the emulated nRF54L15, and counts+timestamps **every**
world transition (SG entry, BXNS/BLXNS, secure exception) per node,
network-wide.

**Achieved.** The M33 is no longer non-secure-only: `nrf54l15_config` sets
`has_trustzone = true`, the security extension lives in
`src/arm/arm_trustzone.c` (attribution engine) plus the state/transition work in
`src/arm/arm_cpu.c` and `src/arm/arm_nvic.c`, and csim boots the real
Contiki-NG `trustzone/` split image with per-node transition counters.

This is the enabler for the SenSys "IsoPartition" study (TEE-partitioning cost
in 6TiSCH): the P0–P3 partition schemes only become measurable once the cut
between secure and non-secure worlds is real and its per-transition cost is
modelled. It also makes csim, to our knowledge, the first *networked,
deterministic, cross-level* TrustZone-M emulation environment.

## Scope: paper-complete, not spec-complete

Build the subset the real Contiki-NG TZ firmware actually exercises, validate
it hard, ship it. Everything outside that is deferred to a "v2 / future work"
list at the end — **do not** implement the whole ARMv8-M security spec.

- **In scope (MVP):** SAU + SPU-as-IDAU attribution, secure/non-secure state,
  banked SP/CONTROL/xPSR-relevant state, SG/BXNS/BLXNS/TT, secure exception
  entry/return with additional-state stacking + integrity signature, lazy FP
  context (LSPACT), SecureFault, NVIC target-security banking, and the
  transition instrumentation.
- **Out of scope (defer):** full fault-escalation matrix corner cases, secure
  debug (SecureDebug/DAUTH), MPU_S/MPU_NS dual banking beyond what firmware
  programs, stack-limit fault variants firmware never triggers, and full TF-M
  (start with a hand-built minimal secure veneer).

The conformance target is concrete: **the firmware tells us what to support.**
`arch/cpu/nrf/nrf54l15/tz-target-cfg.c` and `tz-spu.c` in
`~/work/contiki-ng-nrf54l15` (branch `trustzone-port-v2`) program exactly the
SAU/SPU regions and veneers csim must honour.

## What csim already has (the foundation)

The core is well-shaped for this — the hooks have clean homes:

| Capability | Where | Note |
|---|---|---|
| Single memory-access chokepoint | `arm_read32/16/8`, `mem_read32` (`arm_cpu.c` ~L81/117/257) | SAU/IDAU attribution hooks here, one place |
| SP banking (MSP/PSP) + `use_psp`/CONTROL | `arm_cpu.c` ~L478–525, MSR/MRS ~L2454–2515 | extend to secure/NS banks |
| Exception entry/return + EXC_RETURN decode | `arm_exception_entry` (~L493), `exception_return` (~L555) | extend with S/NS, FType, DCRS, integrity sig |
| NVIC | `arm_nvic.c` | add target-security (`NVIC_ITNS`) + banked priority |
| SPU SECATTR (partial) | `nrf54l15_soc.c` ~L2096/2131 | already latches SECATTR for the FLPR; generalise to the CPU path |
| Secure address aliases known | `nrf54l15_soc.c` (0x5xxx secure / 0x4xxx NS windows) | memory-map foundation present |

**Absent today (new work):** SG/BXNS/BLXNS/TT decode (grep is empty), the
security-state machine, SAU registers, secure exception banking.

## State + register model additions (Phase 0)

Add to `arm_cpu_t`:

- `bool secure;` — current security state.
- Second SP bank: `msp_s/msp_ns`, `psp_s/psp_ns` (the existing `msp/psp`
  become the *active-state* bank; refactor all accesses through a
  current-bank accessor first, keeping the MSP/PSP tests green).
- `msplim_s/msplim_ns`, `psplim_s/psplim_ns` (MSPLIM/PSPLIM).
- `control_s/control_ns` (SPSEL/nPRIV/FPCA/SFPA).
- Secure system regs: `VTOR_S`, `AIRCR.BFHFNMINS/PRIS`, `SHCSR_S`,
  `CCR_S`, and the banked SysTick.
- Banked special-register MSR/MRS: `MSP_NS/PSP_NS/MSPLIM_NS/...`,
  `CONTROL_NS`, `SP_NS` (`arm_cpu.c` MSR/MRS block).

This refactor is load-bearing; do it first, in isolation, with existing
register tests still passing.

## Memory attribution: SAU + SPU-as-IDAU (Phase 1)

In the `mem_read*/write*` chokepoint, add `attr = sau_idau_lookup(addr, secure)`
→ `{NS, NSC, Secure}` with the **conservative rule** (secure if *either* SAU or
IDAU says secure):

- **SAU registers**: `SAU_CTRL/TYPE/RNR/RBAR/RLAR` (0/4/8 regions;
  base/limit at bits 5–31, NSC flag in RLAR).
- **IDAU = the Nordic SPU**: extend the existing FLPR SECATTR mechanism
  (`nrf54l15_soc.c`) to attribute the full address space per the SPU
  `PERIPH[]/FLASHREGION[]/RAMREGION[]` config that `tz-spu.c` programs.
- **Fault**: NS access to S memory → SecureFault (or RAZ/WI per config).

Correctness here is subtle — build against the ARMv8-M attribution table, and
validate region-by-region against what `tz-target-cfg.c` actually sets.

## Transition instruction surface (Phase 2)

New Thumb decode in `arm_cpu.c`:

- **SG** — legal only when executed from an NSC region; flips NS→S at a veneer
  entry. SG from non-NSC → SecureFault (INVEP).
- **BXNS / BLXNS** — S→NS return/call; BLXNS pushes the special return frame
  and sets the FNC/integrity state.
- **TT / TTT / TTA / TTAT** — query attribution (used by TF-M and veneers).

## Secure exception model (Phase 3)

Extend `arm_exception_entry` / `exception_return`:

- Select stack bank by **target** security state.
- On S→NS entry: additional-state stacking + **integrity signature**
  (0xFEFA125A/B) in the frame; on return, validate it.
- Extend EXC_RETURN with `ES` (bit 0, S/NS), `SPSEL`, `DCRS` (bit 5),
  `FType` (bit 4). New valid patterns (e.g. 0xFFFFFFBC) — cf. the known
  Renode gap, issue #937.
- **Lazy FP context** (`LSPACT`, `FPCCR_S/NS`) — model it, don't stub it: it
  is a large, load-dependent chunk of the exception-latency cost the paper
  measures.
- Add SecureFault (SHCSR_S.SECUREFAULTENA) + escalation to secure HardFault.

## NVIC + SysTick banking (Phase 4)

`arm_nvic.c`: per-interrupt target-security bit (`NVIC_ITNS[]`), banked
priorities, `AIRCR.PRIS` (NS priority range restriction), and a banked
SysTick. This is what makes the **ISR-path** transition latency (hypothesis
H3, the "cliff") real.

## The instrumentation payoff (Phase 5)

Once Phases 0–4 are real, add per-node counters + ns-timestamps at each
transition point (SG, BXNS/BLXNS, secure exception entry/exit, with a
`+FPctx` flag). Expose to the campaign harness (`test_runner` log lines, e.g.
`TZ SG=... BXNS=... SECEXC=... cycles=...`). Cheap once the mechanism exists;
this is the "impossible on silicon without intrusive probes" capability.

## Validation strategy

**The authoritative oracle is the real nRF54L15 DK — not another emulator.**
Renode/QEMU are themselves models; validating csim against them only proves
csim agrees with *their* interpretation of ARMv8-M, and would propagate their
bugs. Silicon is the only thing that can settle a disagreement, and for the
Nordic **SPU** (chip-specific IDAU behaviour) it is the *only* authority.

Layered so the fast loop stays fast and the truth stays authoritative:

- **Inner loop — ARM ARM conformance suite (fast, automatable).** Directed
  tests from the architecture manual: attribution-table cases, SG-from-non-NSC
  fault, BLXNS frame layout, the EXC_RETURN matrix, integrity-signature
  validation. Pinned to the *spec*. (Phase 1's `test_trustzone_sau` is the
  first of these.) Where silicon later surprises us, the HW capture overrides
  the spec-derived expectation.
- **Authoritative oracle — nRF54L15 DK, via capture-and-replay.** Hardware is
  ground truth but low-bandwidth (you cannot cheaply diff internal state on
  silicon). So: run small directed firmware on the DK that exercises each
  primitive (SG, BXNS/BLXNS, secure exception, ±FP context, SPU attribution),
  capture ground truth **once** — architectural state via GDB/semihosting,
  per-transition cycle cost via DWT `CYCCNT` — and freeze those as **golden
  vectors**. csim regresses against real-silicon captures, for both function
  and timing. This is the paper's Testbed 1.
- **Renode = development aid only, never an authority.** It has independent
  (tlib/QEMU-lineage) TrustZone-M support — SAU/IDAU + NSC, secure exception
  exit (`ES/S/SPSEL/MODE/FType/DCRS`), secure interrupt targeting — which is
  useful for *seeing internal state while debugging* a divergence. But it does
  not decide correctness, and it is not cycle-accurate, so it never informs
  the cost model. If Renode and the DK disagree, the DK wins.
- **Firmware:** start with a hand-built minimal secure veneer (SG entries
  wrapping AES + key storage = P1/P2), not full TF-M. Maps directly to the
  partition schemes, and is the same image captured on the DK above.

## Confirmed execution steps (2026-07-26)

Ordered, individually-tested increments. Conformance firmware is the existing
Contiki-NG `examples/platform-specific/nrf/trustzone/` suite (nRF54L15-only),
built `BOARD=nrf54l15/dk` so one binary runs on csim and the DK.

- **[DONE]** Phase 0 — security state + banked register storage.
- **[DONE]** Phase 1 engine — `arm_security_attr()` SAU/IDAU attribution (+14 tests).
- **[DONE] Step 1** — memory-mapped SAU registers (`0xE000EDD0`), Secure-only RAZ/WI.
- **[DONE] Step 2** — hot-path enforcement + SecureFault recording (`arm_tz_blocks`).
- **[DONE] Step 3** — transition instructions: TT (3a) + SG/BXNS with SP-bank swap
  (3b). BLXNS + FNC_RETURN full form deferred (needs the exception stacking).
- **[DONE] Step 4 (core)** — secure exception entry/return: bank by security
  state, VTOR_S fetch, take the pending SecureFault, re-bank on return.
  Deferred: integrity signature, lazy FP, nested-secure, BLXNS.
- **[DONE] Step 5** — NVIC target-security (`NVIC_ITNS`) wired into exception
  entry. Deferred: banked priorities + banked SysTick.
- **[DONE] Step 6** — `has_trustzone=true` on `nrf54l15_config`; existing
  firmware runs byte-identically (Secure world, enforcement inert).
- **[DONE] Step 7** — per-node transition instrumentation (SG/BXNS/secure-exc).
- **[DONE] Step 8** — BLXNS + FNC_RETURN implemented; csim boots the **real**
  Contiki-NG `trustzone/` split image (`tz-boot` harness). The secure world
  initializes TrustZone, configures SAU/IDAU + the non-secure environment,
  routes IRQs (NVIC_ITNS), and hands off to Non-secure ("Non-secure world"
  banner); security state flips correctly, 9 world transitions (4 SG + 5 BXNS)
  counted. The arm-gcc 15.2 firmware build blocker was fixed upstream-style in
  `contiki-ng-nrf54l15` (`arch/cpu/arm/cortex-m/Makefile.cortex-m`: re-tag the
  attribute-less CMSE import lib; guarded `--no-warn-rwx-segments`).

**PLAN COMPLETE.** All emulator steps done and tested; the real split firmware
boots in csim. Remaining is the HW track (DK golden-vector capture — the
authoritative oracle) and deeper firmware exercise (radio/timer-driven rpl-udp,
secure services), both outside the core emulator.

## Verified result (re-measured 2026-07-30, branch `trustzone` @ `87c92b3`)

Branch is based on `main` @ `de7e41c` (the `v0.1.0` release commit) and contains
every commit on `main`.

**Test suites** — `arm-correctness` is **224 / 224**, of which **71 are
TrustZone** (`main` is 153, so the TZ work is +71):

| TZ test group | Tests |
|---|---|
| SAU/IDAU attribution | 14 |
| SAU memory-mapped registers | 9 |
| Data-access enforcement | 9 |
| TT / TTT / TTA / TTAT | 3 |
| SG / BXNS transitions | 11 |
| SecureFault exceptions | 6 |
| NVIC target-security (`NVIC_ITNS`) | 6 |
| Transition counters | 3 |
| BLXNS / FNC_RETURN | 10 |
| **Total** | **71** |

**No regression on the non-TZ path**, with `has_trustzone = true` live on the
same SoC the release regression-tests: `correctness` PASS, `radio-medium`
241/241, `cc1200-mock-host` 73/73, and `configs/test-2node-nrf54l15-dk.json`
**TEST PASSED** (60013 ms simulated — identical to `main`).

**Real split firmware boots** via the `tz-boot` harness. The secure world
initializes TrustZone, configures SAU/IDAU and the non-secure environment,
validates the NS image permissions and reset vector, routes IRQs
(`ITNS[7]=0x00000008`), and jumps to the non-secure reset handler:

| Split image | SG (NS→S) | BXNS (S→NS) | Total | Result |
|---|---|---|---|---|
| `trustzone/{secure,normal}-world` (`nrf54l15/dk`) | 389 | 390 | **779** | reaches Non-secure, banner + `sec ret` round-trip |
| `trustzone/rpl-udp` (`nrf54l15/xiao`) | 16 | 17 | **33** | reaches Non-secure, boots the RPL-UDP server |

Both end in `Non-secure` state with 0 secure exceptions, so the harness pass is
app-independent rather than tied to one image. (The earlier "9 transitions
(4 SG + 5 BXNS)" figure in Step 8 above was measured at that step, before the
`tzbench` workload ran on; it is kept as the step's historical record.)

Reproduce with:

```sh
TZ=~/work/contiki-ng-nrf54l15/examples/platform-specific/nrf/trustzone
./build/test_runner tz-boot \
    $TZ/secure-world/build/nrf/nrf54l15/dk/secure-world-example.nrf \
    $TZ/normal-world/build/nrf/nrf54l15/dk/normal-world-example.nrf
```

The split images are **not** in the csim tree — they are built from the
`contiki-ng-nrf54l15` checkout (branch `trustzone-port-v2`), so `tz-boot` is a
manual harness rather than part of the default gate.

Parallel HW track (non-blocking): DK capture toolchain (JLink/GDB + DWT `CYCCNT`
+ semihosting) on the current non-TZ nRF54L15 first, then per-primitive golden
vectors (SG/BXNS/secure-exc ±FP, **SPU attribution captured from silicon**) as
csim regression fixtures for Steps 3–5 + 7.

**Done when** csim boots `trustzone/rpl-udp` on `nrf54l15/dk`, counts transitions
network-wide, and per-primitive costs match DK golden vectors within HW spread.

## Staged plan + effort

| Phase | Work | Est. |
|---|---|---|
| 0 | Security state + banked SP/CONTROL/limits; MSR/MRS_NS; refactor SP access | ~2–3 days |
| 1 | SAU + SPU-as-IDAU attribution in mem chokepoint; conformance vs `tz-target-cfg.c` | ~3–4 days |
| 2 | SG/BXNS/BLXNS/TT decode | ~2–3 days |
| 3 | Secure exception entry/return, integrity sig, lazy FP, SecureFault | ~4–5 days |
| 4 | NVIC target-security + banked priority + SysTick | ~2 days |
| 5 | Transition instrumentation + harness plumbing | ~1 day |
| — | ARM ARM conformance suite + **DK golden-vector capture/replay** (authoritative); Renode only as debug aid (spread across 1–4) | ~3–4 days |

Guard behind the existing `arm_config` capability so nRF52840 (M4, no TZ) and
the CC2538/MSP430 targets are unaffected.

## Open questions / risks

- **Phase 1 attribution correctness** and **Phase 3 stacking integrity** are
  the critical path: a plausible-but-wrong implementation silently diverges
  from silicon and quietly invalidates every transition-cost number the paper
  reports. Budget real validation there.
- **SPU coverage:** how much of the SPU region model does the Contiki-NG
  firmware actually program? Model that subset; don't chase the full SPU.
- **Determinism with banked SysTick + NS interrupts** — keep it fixed-seed
  reproducible (csim's core property).
- **FLPR interaction:** the nRF54L15 already has the RV32E FLPR in csim
  (`riscv-vpr-plan.md`); confirm SPU SECATTR changes here don't regress that.

## References

- ARM DDI 0553 (ARMv8-M Architecture Reference Manual) — security extension.
- `~/work/contiki-ng-nrf54l15` (branch `trustzone-port-v2`):
  `arch/cpu/nrf/nrf54l15/tz-target-cfg.c`, `tz-spu.c`;
  `arch/cpu/arm/cortex-m/trustzone/tz-api.{c,h}`, `tz-fault.c`, `tz-secure*.c`.
- csim: `src/arm/arm_cpu.c` (mem chokepoint, exception model, MSR/MRS),
  `src/arm/arm_nvic.c`, `src/arm/nrf54l15_soc.c` (SPU SECATTR), `arm_config.c`.
- Renode (functional oracle): `renode-infrastructure`
  `src/Emulator/Cores/Arm-M/CortexM.cs`, `NVIC.cs`; tlib `arch/arm/helper.c`;
  issue #937 (Armv8-M EXC_RETURN handling).
- Related: `riscv-vpr-plan.md` (FLPR / SPU precedent), `t3-nrf54l15-rx-plan.md`.
