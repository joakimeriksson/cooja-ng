# ARM Interpreter Performance Plan

Status: **plan** (2026-08-01). Companion to [`refactor-plan.md`](refactor-plan.md)
(§8 D5 overlaps Tier 0 here).

**Why now.** ARM is the strategic platform — CC2538, nRF52840, nRF54L15,
TrustZone-M, and the FLPR host all run on it — but it is the *only* emulated ISA
in csim with no acceleration beyond the plain interpreter. MSP430 has a decoder
plus a GNU Lightning JIT; RISC-V got a memoized fetch/decode path (`da4c718`,
~28%). ARM has neither. The legacy platform is the fast one.

Everything below is ordered by **measured evidence → confidence → cost**, not by
how interesting the work is. Tiers 0–1 are near-free and already measured; Tier 2
is a well-understood port of an in-tree technique; Tier 3 is a real project.

---

## 0. What is actually measured

All numbers are from this machine (Apple Silicon, `-O3 -flto -mcpu=native`)
unless stated. **Nothing in this plan is inferred from reading code alone.**

### 0.1 Profile — where ARM time goes

`zephyr-synchronization.nrf52840-dk`, single node, 120 s sim, `sample` over 5 s
(3454 samples, self time):

| Symbol | Self | Note |
|---|---|---|
| `arm_step` | 76.5% (2643) | the interpreter core — expected |
| `condition_passed` | **10.5% (364)** | declared `static inline`, **not inlined** |
| `mach_absolute_time` | **7.0% (241)** | wall-clock syscall in the event pump |
| `arm_step_until` | 2.3% (78) | |
| `mixed_dispatch_event` | 0.9% (30) | kernel |
| `arm_mote_execute` | 0.7% (25) | |
| everything else | ~2% | event queue, radio bus, kernel |

Two of the top three are **accidents, not costs of emulation**:

- `condition_passed` appears as its own symbol with 364 samples. The inliner
  gives up on it inside the ~3000-line `arm_step`, so every conditional
  instruction (and every IT-block body) pays a call.
- `mach_absolute_time` traces to `test_mixed_multinode.c:2526` and `:2539` —
  **two clock reads per event-pump iteration**, feeding only the "Phase Timing"
  report. That report has one non-zero phase (`step (CPU)`, ~87%) and four
  always-zero ones (`distribute`, `deliver`, `flush/output`, `channel sync`) —
  the same dead code `refactor-plan.md` §8 **D5** already flags.

### 0.2 Tier-0 fix — measured, not predicted

`__attribute__((always_inline))` on `condition_passed` + dropping the two
per-event clock reads. 5 reps, 60 s sim, same binary flags:

| | median | min | max |
|---|---|---|---|
| baseline | 3.003 s | 2.831 s | 3.236 s |
| Tier 0 | **2.737 s** | **2.651 s** | 2.769 s |

**~8% faster, and the distributions do not overlap** (baseline min 2.831 >
Tier-0 max 2.769), so the effect is larger than the run-to-run noise.
`arm-correctness` stayed 153/153. Two one-line changes.

### 0.3 External reference — Renode 1.16.1 (same host, same ELFs)

Both simulators run our `firmware/nrf52840-dk/zephyr-*` images; instruction
counts agree within 4.5%, so the workloads are comparable.

**CPU-bound** (`zephyr-synchronization`), slope over 10 s → 60 s virtual so
process startup cancels:

| | wall / virtual-s | instr / virtual-s | **MIPS** | ×real-time |
|---|---|---|---|---|
| csim | 0.0486 s | 7.53 M | **154.9** | 20.6× |
| Renode | 0.0976 s | 7.86 M | **80.6** | 10.2× |

csim is **1.92× faster** than Renode's tlib (a QEMU-derived JIT) — while being a
pure interpreter. Worth re-checking on x86-64, where Renode is better tuned.

**Idle-dominated** (`zephyr-hello-world`): csim 0.02 s flat at both 10 s and
60 s virtual (event queue jumps to the next event); Renode 2.26 s → 6.22 s.
That gap measures **idle-skip policy, not engine speed**, and must always be
labelled as such — the same distinction that made the FLPR demo go 0.15× → 714×
with no interpreter change at all.

### 0.4 The measurement gap

**There is no ARM benchmark.** `./build/test_runner bench` prints *"MSP430 C
Emulator Performance Benchmarks"* and covers MSP430 only. Every ARM figure in
this document was produced by hand-timing `nrf52840-dk-multinode`. That is not a
gate, it is not in CI, and it cannot detect a regression.

---

## 1. Prerequisite — `arm-bench` (do this first)

Nothing else in this plan is verifiable without it.

- New mode in `test_main.c`, mirroring `run_benchmarks()`: fixed instruction
  budget per case, report wall ms + MIPS + instruction count, plus the CSV
  summary block `bench` already emits.
- Cases: a synthetic ALU/branch loop (isolates dispatch), a Thumb-2-heavy loop
  (isolates the `t32_decode` nest), an IT-block loop (isolates
  `condition_passed`), and two firmware runs (`zephyr-synchronization`,
  `udp-server.nrf52840-dk`) for a realistic mix.
- Report **MIPS** as the primary number and ×real-time as secondary, with a note
  that ×real-time is idle-policy-sensitive.
- Wire into CI as a *reported* number, not a hard gate initially — machine
  variance on shared runners would make it flaky. Revisit once we have a
  baseline distribution.

Effort: **~half a day.** It also gives PGO a training workload (§3).

---

## 2. Tier 0 — free, already measured (~8%)

**2.1 Force-inline `condition_passed`.**
`static inline __attribute__((always_inline))`. The function is tiny and
branch-only; the inliner is declining it purely because `arm_step` is enormous.
*Risk: none. Verified 153/153.*

**2.2 Remove the per-event wall-clock reads.**
Two `get_time_ms()` calls per event-pump iteration exist only to populate
`time_step`. Options, preferred first:

1. **Delete the Phase Timing report** together with its four always-zero phases
   (`refactor-plan.md` §8 D5 already schedules this) and keep the single useful
   number — total wall time — which is measured once at the ends.
2. If the per-phase breakdown is wanted for debugging, gate the whole block
   behind `CSIM_PHASE_TIMING=1` so the default path makes no syscall.

*Risk: low, but it is user-visible output — it changes what a normal run prints.
Decide (1) vs (2) before implementing.*

**Acceptance:** `arm-bench` improves ≥5%; `arm-correctness` 153/153; determinism
guard byte-identical (this touches the event pump, so that guard is mandatory).

---

## 3. Tier 1 — repair PGO, and train it on ARM

`CLAUDE.md` advertises `make pgo` as **"~40% faster"**. It does not compile:

```
test/test_mixed_multinode.c:73:10: fatal error: 'mote_impl.h' file not found
```

**Root cause: `PGO_CFLAGS` and the PGO source list are hand-duplicated copies of
the real build that drifted.** They predate Phases 6–8, so relative to `CFLAGS`
they are missing `-I src/motes`, `-D_GNU_SOURCE`, and four whole source groups
(`$(SIM_SOURCES)`, `$(SERVICES_SOURCES)`, `$(MOTES_SOURCES)`,
`$(QUICKJS_SOURCES)`).

**And even when it worked, it trained on the wrong thing:** the profile run is
`test_runner bench` + `test_runner correctness` — **both MSP430-only**. Every ARM
hot path was laid out blind. The advertised 40% has never been measured for ARM.

Work:

1. **Make drift impossible.** One `ALL_SOURCES` list shared by both the object
   build and the PGO build; `PGO_CFLAGS = $(filter-out -flto -MMD -MP,$(CFLAGS))`
   instead of a hand-copied flag string. QuickJS needs its private
   `-DCONFIG_VERSION=...` `-w` (it has its own object rule today), so either
   pass those in the single-command build or — cleaner — make `pgo` a **recursive
   `make`** that reuses the normal per-object rules with added
   `-fprofile-instr-{generate,use}`. The recursive form is preferred: it cannot
   drift again by construction.
2. **Train on ARM too:** add `arm-correctness` and a short
   `nrf52840-dk-multinode zephyr-synchronization` run to the profile step, merging
   all `.profraw` files.
3. **Measure the real ARM delta** with `arm-bench` and correct the CLAUDE.md
   claim to whatever it actually is, per ISA.

Note: `xcrun llvm-profdata` fails under a restricted sandbox
(`couldn't create cache file`); use `llvm-profdata` directly, or document the
requirement.

Effort: **~1 day.** Confidence: high that it *builds*; **unknown** what it buys
on ARM until measured — that is the point of the task.

---

## 4. Tier 2 — ARM decoded-instruction cache

**The structural gap:** ARM decodes inline on *every* execution. MSP430 has a
stateless decoder (`msp430_decode.c`) feeding a JIT; RISC-V memoizes. ARM does
neither — `arm_step` fetches, computes `top5`, dispatches through
`thumb_dispatch[32]`, and for Thumb-2 (`top5 >= 29`) falls into `t32_decode` and
a nest of `switch (op1)/(op2)/(op_dp)` — all re-executed identically every time
the same instruction runs.

**Correction to an easy assumption:** the RISC-V win is *not* directly portable.
`da4c718` memoizes `rvc_expand()` in a **64 K table keyed on the 16-bit
compressed word** — legal only because compressed expansion is a pure function of
a 16-bit value. Thumb-2 is 32-bit; a value-keyed table is impossible.

So ARM needs a **PC-keyed decoded-instruction cache**, i.e. the MSP430 model:

- `decoded_arm_insn_t` holding class/handler-index + pre-extracted operand
  fields, cached at `pc >> 1`, sized to the flash window.
- Hit → jump straight to the handler with fields already extracted, skipping the
  entire decode nest. Miss → decode once, fill, execute.
- Optionally extend to basic blocks later (MSP430's `basic_block` with
  `MAX_BLOCK_SIZE 32` is the template) — but do the per-instruction cache first;
  it is where the decode saving lives.

**Correctness hazard — do not skip.** `arm_write8/16/32` accept writes into
`[flash_base, flash_end)`, so firmware (e.g. via NVMC) *can* self-modify. A
PC-keyed cache must be invalidated on any write into that range, exactly as
`msp430_cpu.c:73 cache_invalidate()` does — and note csim already shipped a bug
here once (the MSP430 SMC invalidation only covered a fixed 6-byte window; fixed
in the 0.1.0 hardening audit). Reuse that lesson: invalidate by **actual byte
span**, not a fixed window.

**Expected gain:** RISC-V got 28% from a much cheaper decode. ARM's decode is
heavier (Thumb-2 + IT-block state), so plausibly more — but this is an
expectation, not a measurement, and Tier 0/1 land first precisely so the baseline
is honest.

Effort: **~1 week.** Self-contained in `src/arm/`, no ABI change, no kernel
change. Determinism must be byte-identical (cycle counts are accumulated at 32
sites in `arm_step`; the cached path must reproduce them exactly).

---

## 5. Tier 3 — ARM JIT (GNU Lightning)

Biggest win, biggest risk. Only start after Tier 2 exists and `arm-bench` is a
trusted number.

Everything needed is already in the tree:

- GNU Lightning is wired into the build (auto-detected via pkg-config; the JIT is
  optional and the interpreter is the fallback).
- `src/msp430/msp430_jit.c` is a complete working template: hot-block threshold
  via `MSPSIM_JIT_THRESHOLD`, `block_exec_count[pc>>1]`, compiled-block cache,
  invalidation on write, and — importantly — the discipline of compiling **only
  blocks where every instruction is inlineable**, with a documented exclusion
  list (`SUBC`, `DADD`, memory operands, SR/PC writes, PUSH/CALL/RETI, and
  `ADDC`, excluded during the hardening audit because its carry-in left no
  register free to compute the overflow flag).

That last point is the real lesson to carry over: **the MSP430 JIT shipped a
warm-block-only divergence** — a bug that only appeared after a block went hot.
An ARM JIT has strictly more state to get wrong (APSR flags including `GE`, IT
blocks, banked SP under TrustZone). Mitigations, non-negotiable:

- A `JIT_VERIFY` mode that runs interpreter and JIT in lockstep and diffs
  architectural state per block, run over the whole firmware corpus.
- Start with the narrowest possible profitable subset: flag-setting data
  processing and unconditional branches on registers only. Explicitly exclude IT
  blocks, all memory operands, `PC`/`SP` writes, and anything TrustZone-relevant
  in v1.
- Determinism is a **gated guarantee** in this project
  (`tools/check-determinism.sh` is in CI). A JIT that perturbs cycle accounting
  breaks it. Cycle counts must match the interpreter exactly, block for block.

Effort: **3–4 weeks.** Do not start it under release pressure.

---

## 6. Sequencing, risk, rollback

| Step | Effort | Confidence | Gate |
|---|---|---|---|
| `arm-bench` | ½ day | — (enabler) | builds; numbers stable across 5 reps |
| Tier 0 | ½ day | **measured ~8%** | arm-bench ≥5%; 153/153; determinism |
| Tier 1 PGO | 1 day | build: high; gain: unknown | `make pgo` works; ARM delta measured |
| Tier 2 decode cache | ~1 week | high (in-tree precedent) | full gate + determinism byte-identical |
| Tier 3 JIT | 3–4 weeks | medium | JIT_VERIFY clean over corpus + full gate |

**Mandatory gate for anything touching `arm_step` or the event pump:**
`arm-correctness` (153), `correctness`, `radio-medium` (241), `cc1200` (73),
`arm-firmware`, the cc2538/nRF52840/nRF54L15 multinode configs, TSCH ×2, the
Cooja suite (**93/93**), and `tools/check-determinism.sh`. Per
`refactor-plan.md` §11.5, a tier that regresses the gate is **reverted, not fixed
forward**.

**What this plan deliberately does not do:** chase the 311× idle-skip number as
if it were engine speed, or quote `make pgo`'s "~40%" for ARM until someone has
measured it. Both are currently in our docs and neither is supported for ARM.

---

## 7. Open questions

1. **Phase Timing report — delete or gate?** (§2.2) Changes visible output.
2. **x86-64 cross-check of the Renode comparison.** The 1.92× MIPS result is
   single-platform; Renode's arm64 .NET build may be less optimized than its
   primary target. jftest4 can settle it.
3. **Is `make pgo` used by anyone?** If it is dead, deleting it is cheaper than
   repairing it — but then the CLAUDE.md "~40% faster" claim must go too.
4. **Does any supported firmware actually write to flash at run time?** If not,
   Tier 2's invalidation can be a cheap assertion rather than a hot-path check —
   but it must still be *correct*, not merely absent.
