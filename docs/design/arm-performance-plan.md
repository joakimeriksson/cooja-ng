# ARM Interpreter Performance Plan

Status: **in progress** (2026-08-01). §1 `arm-bench` and §3 Tier 1 (PGO) are
**done and measured**; Tier 0, 2 and 3 are open. Companion to
[`refactor-plan.md`](refactor-plan.md) (§8 D5 overlaps Tier 0 here).

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

## 1. Prerequisite — `arm-bench` — **DONE**

Landed as `test/test_arm_benchmark.c`, mode `./build/test_runner arm-bench`.
Nothing else in this plan was verifiable without it.

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

It also gives PGO a training workload (§3).

### 1.1 Measured baseline (Apple Silicon, `-O3 -flto -mcpu=native`)

Median of 5 in-run iterations, 20 M instructions each:

| Benchmark | What it isolates | MIPS |
|---|---|---|
| `it-block` | IT blocks / `condition_passed` | ~202 |
| `alu-reg` | Thumb-16 computed-goto dispatch | ~200 |
| `branch` | conditional branches | ~202 |
| `mem-ldr-str` | `arm_read32`/`arm_write32` | ~176 |
| **`thumb2-dp`** | **the `t32_decode` switch nest** | **~164** |
| `fw-zephyr-sync` | real firmware (nRF52840) | ~196 |
| `fw-cc2538-udp` | real firmware (CC2538) | ~153 |

**`thumb2-dp` is the slowest synthetic case — ~18% below the Thumb-16 paths.**
That is direct support for Tier 2 (§4): the 32-bit decode nest is measurably
more expensive per instruction than the 16-bit dispatch, which is exactly what
a decoded-instruction cache removes.

### 1.2 The runner costs ~27% on top of the interpreter

`fw-zephyr-sync` runs the *same* Zephyr ELF as the hand-timed measurement in
§0.3, but bare — no kernel, no event pump, no radio:

| Path | MIPS |
|---|---|
| `arm-bench` (bare `arm_step`) | **~196** |
| `nrf52840-dk-multinode` (full runner) | **~155** |

So roughly **27% of wall time on a real single-node run is spent outside the
interpreter**, in the kernel/event-pump path. That is consistent with the
profile in §0.1 (`arm_step` 76.5% self) and it is what Tier 0 attacks — the two
`mach_absolute_time` calls per event-pump iteration are in exactly this 27%.

### 1.3 Noise floor — read before gating on this

Run-to-run spread of the per-run medians, 3 consecutive runs on a *loaded*
laptop:

| Benchmark | spread |
|---|---|
| `thumb2-dp` | 2.4% |
| `it-block` | 2.6% |
| `fw-zephyr-sync` | 6.7% |
| `branch` | 7.1% |
| `fw-cc2538-udp` | 7.4% |
| `alu-reg` | 8.2% |
| `mem-ldr-str` | **20.3%** |

**Tier 0's ~8% gain is at or below this noise floor for a single A/B run.**
Any tier must therefore be validated with repeated runs comparing medians, not
one before/after pair — and CI should report the number, not hard-gate on it,
until a quiet-machine baseline distribution exists. `mem-ldr-str` is too noisy
to gate on at all in its current form.

### 1.4 The guard, and what it does not cover

Each synthetic loop increments `r7` once per pass and nothing else writes it,
so `r7` must equal `INSTRUCTIONS / instructions-per-iteration`; a mismatch is
reported as **BROKEN** and makes the suite exit non-zero, rather than timing
garbage.

Validated by injection, not assumed — and it earned its keep immediately:

- Deleting one instruction from `alu-reg` (declared 10/iter, actual 9) trips
  it: `iters=2222222` vs `expected=2000000`.
- It caught a real error while the file was being written: `mem-ldr-str` was
  declared at 6 instructions/iteration when it is 7.

**Limit, stated so it is not over-trusted:** it detects changes to loop *length*
or *control flow*, not a wrong instruction that is flow-neutral. Injecting
`0xDEAD` is *not* caught — this emulator executes an undefined instruction as a
silent no-op instead of faulting, so the loop keeps its shape. (That no-op
behaviour is itself a small fidelity gap worth a separate look; a real M3 takes
UsageFault.) Semantic correctness of encodings is `arm-correctness`'s job, not
this benchmark's.

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

## 3. Tier 1 — repair PGO, and train it on ARM — **DONE**

Landed. `make pgo` works again on both clang and gcc, trains on ARM as well as
MSP430, and the measured gain is far larger than the claim it replaced.

### 3.1 Measured result

`arm-bench` medians of 3 runs; the **holdout** is wall-clock on
`configs/chain-3node-nrf52840-dk.json` (240 s sim, ~152 M instructions), which
is deliberately **not** in the training set.

| | plain | PGO | speedup |
|---|---|---|---|
| **macOS / clang, Apple Silicon** | | | |
| MSP430 `register-alu` | 392.9 | 640.0 MIPS | 1.63× |
| MSP430 `branch-heavy` | 444.5 | 664.7 MIPS | 1.50× |
| ARM `alu-reg` | 199.7 | 339.7 MIPS | 1.70× |
| ARM `thumb2-dp` | 163.4 | 265.7 MIPS | 1.63× |
| ARM `it-block` | 211.5 | 343.4 MIPS | 1.62× |
| ARM `branch` | 202.5 | 347.9 MIPS | 1.72× |
| ARM `mem-ldr-str` | 165.2 | 336.8 MIPS | 2.04× |
| ARM `fw-zephyr-sync` | 195.4 | 330.2 MIPS | 1.69× |
| ARM `fw-cc2538-udp` | 149.9 | 255.2 MIPS | 1.70× |
| **holdout** (untrained) | 1.22 s | 0.79 s | **1.55×** |
| **Linux / gcc, x86-64** | | | |
| ARM `alu-reg` | 130.0 | 189.4 MIPS | 1.46× |
| ARM `thumb2-dp` | 121.2 | 143.5 MIPS | 1.18× |
| ARM `fw-zephyr-sync` | 141.4 | 185.7 MIPS | 1.31× |
| **holdout** (untrained) | 1.58 s | 1.34 s | **1.18×** |

**Read the holdout, not `arm-bench`.** `arm-bench` is *in* the training set, so
its 1.6–2.0× is an optimistic upper bound — the compiler has seen exactly those
loops. **1.55× (clang) / 1.18× (gcc) on untrained work is the honest number**,
and it is still much better than the "~40%" the docs claimed. Correctness is
unaffected: `correctness`, `arm-correctness` 153, `radio-medium` 241,
`cc1200-mock-host` 73 all pass under PGO on both toolchains, with zero profile
warnings from gcc.

Note the clang/gcc gap is large and consistent. Not investigated; the plausible
cause is that clang's PGO does more aggressive layout/inlining of the big
computed-goto interpreter than gcc's. Worth a look if gcc becomes the primary
distribution build — CI uses gcc on Linux.

### 3.2 What was wrong, and what changed

`CLAUDE.md` advertised `make pgo` as **"~40% faster"**. It did not compile:

```
test/test_mixed_multinode.c:73:10: fatal error: 'mote_impl.h' file not found
```

**Root cause: `PGO_CFLAGS` and the PGO source list were hand-duplicated copies of
the real build that drifted.** They predated Phases 6–8, so relative to `CFLAGS`
they were missing `-I src/motes`, `-D_GNU_SOURCE`, and four whole source groups
(`$(SIM_SOURCES)`, `$(SERVICES_SOURCES)`, `$(MOTES_SOURCES)`,
`$(QUICKJS_SOURCES)`).

**And even when it worked, it trained on the wrong thing:** the profile run was
`test_runner bench` + `test_runner correctness` — **both MSP430-only**. Every ARM
hot path was laid out blind, so the advertised 40% had never meant anything for
ARM.

What changed:

1. **Drift made structurally impossible.** `pgo` is now a **recursive `make`**
   that reuses the normal per-object rules, so there is no second copy of the
   flags or the source list to fall out of sync. A single `PGO_FLAGS` variable
   (empty for a normal build) is threaded through `CFLAGS`, `LDFLAGS` and the
   QuickJS rule — which is why QuickJS's private `-DCONFIG_VERSION` / `-w` no
   longer need duplicating. A single-command build cannot express that per-file
   flag, which is what broke the first repair attempt.
2. **gcc support.** The old target was clang-only
   (`-fprofile-instr-generate` + `llvm-profdata merge`). It now detects the
   compiler and uses `-fprofile-generate` / `-fprofile-use -fprofile-correction`
   under gcc, where `.gcda` files are written beside the objects — so step 3
   deletes only `*.o`, not the profile data. This matters because **CI builds
   with gcc on Linux**.
3. **Training set** (`PGO_TRAIN_*`): `bench` + `correctness` (MSP430, as before)
   plus `arm-correctness` (ARM instruction mix), `arm-bench` (the five ARM hot
   paths), and one real `nrf52840-dk-multinode zephyr-synchronization` firmware
   run. The firmware run is what keeps the profile from overfitting to the
   synthetic loops.
4. `PROFDATA` is overridable (`make pgo PROFDATA=/path/to/llvm-profdata`) —
   `xcrun` is not always usable, and it fails outright under a restricted
   sandbox with `couldn't create cache file`.

Cost: `make pgo` takes ~27 s (clang, Apple Silicon) / ~48 s (gcc -j2, x86-64).

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
| ~~Tier 1 PGO~~ **DONE** | — | — | **1.55× clang / 1.18× gcc on an untrained workload**; suites green on both |
| Tier 2 decode cache | ~1 week | high (in-tree precedent) | full gate + determinism byte-identical |
| Tier 3 JIT | 3–4 weeks | medium | JIT_VERIFY clean over corpus + full gate |

**Mandatory gate for anything touching `arm_step` or the event pump:**
`arm-correctness` (153), `correctness`, `radio-medium` (241), `cc1200` (73),
`arm-firmware`, the cc2538/nRF52840/nRF54L15 multinode configs, TSCH ×2, the
Cooja suite (**93/93**), and `tools/check-determinism.sh`. Per
`refactor-plan.md` §11.5, a tier that regresses the gate is **reverted, not fixed
forward**.

**What this plan deliberately does not do:** chase the 311× idle-skip number as
if it were engine speed — that measures event-queue policy, not the interpreter,
and it stays labelled as such. (The companion offender, `make pgo`'s unmeasured
"~40%", is now settled: §3.1 replaced it with per-toolchain numbers from an
untrained holdout, and both `README.md` and `CLAUDE.md` were corrected.)

---

## 7. Open questions

1. **Phase Timing report — delete or gate?** (§2.2) Changes visible output.
2. **x86-64 cross-check of the Renode comparison.** The 1.92× MIPS result is
   single-platform; Renode's arm64 .NET build may be less optimized than its
   primary target. jftest4 can settle it.
3. ~~**Is `make pgo` used by anyone?**~~ **Resolved by repairing it.** It is
   referenced only from `CLAUDE.md` and `README.md` (not from CI or any script),
   but at 1.55× it is worth keeping and worth wiring into a release build. The
   "~40% faster" claim has been replaced with the measured per-toolchain
   numbers in both files.
4. **Does any supported firmware actually write to flash at run time?** If not,
   Tier 2's invalidation can be a cheap assertion rather than a hot-path check —
   but it must still be *correct*, not merely absent.
