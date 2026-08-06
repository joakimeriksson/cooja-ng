# ARM Interpreter Performance Plan

Status: **every tier in this plan is done and measured** (2026-08-07).
Companion to [`refactor-plan.md`](refactor-plan.md) (§8 D5 overlaps Tier 0
here).

## Summary — what shipped, and what is left

**Interpreter** (§2–§4): 2.939 s → 1.940 s on the ARM runner workload, **34%**,
from four independent changes — force-inlining `condition_passed`, gating the
per-instruction debug facilities, and hoisting the flash window and the
GDB/ROM-trap checks out of the fetch path. A fifth (a decoded-instruction
cache) was **dropped when measurement refuted its premise**, and a sixth
(branch consolidation) was reverted as noise.

**JIT** (§5): a Thumb-16 decoder, a GNU Lightning code generator, guarded SRAM
memory operations, and two dispatcher fixes. Cycle-exact — `CSIM_ARM_JIT=0`
and `=1` produce byte-identical output, which is required because determinism
is a gated guarantee and Lightning is an optional dependency.

| Workload | JIT off | JIT on | |
|---|---|---|---|
| `zephyr-synchronization`, 1 node, 60 s sim (arm64) | 2.05 s | **0.58 s** | **3.5x** |
| `fw-zephyr-sync` MIPS (x86-64) | 190.3 | **1170.5** | **6.15x** |
| `alu-reg` / `mem-ldr-str` / `branch` (arm64) | | | 4.46x / 4.11x / 2.06x |
| Contiki-NG ARM firmware (chain-3/4node, cc2538 RPL-UDP) | | | **~1.00x** |

That works out to **4.2 host cycles per emulated instruction** where the JIT
applies, against the MSP430 JIT's 7.8.

**The one number that did not move is the important one, and §5.11–§5.13
decompose it — ending in a profile that should have been taken first.** The
71.5% of instructions the JIT compiles on cc2538 account for **~5% of the
runtime**; the 28.5% it cannot — MMIO, 64-bit division traps, Thumb-2 — hold
~75%. Three successive attempts to give the JIT more to do (memory ops, the
cheap Thumb-16 classes, trace formation) each returned nothing or less than
nothing, and one profile explains all three. **The JIT is finished as a line of
work here; the remaining wins are in the interpreter** (§5.13).

**Next steps, in the order the measurements justify** — reordered after §5.11,
which refuted the previous ordering's premise:

~~Cheap Thumb-16 classes~~ **DONE, bought nothing** (`d5ba3b3`, §5.12).
~~Block linking~~ **BUILT AND REVERTED — a 9% regression** (§5.13).

**The JIT is finished as a line of work.** A profile (§5.13.3) shows the 71.5%
of instructions it compiles are ~5% of the runtime; the expensive ones — MMIO,
64-bit division traps, Thumb-2 — are what is left. More coverage, linking or
Thumb-2 all optimise that 5%.

1. **Interpreter work, which is where the time actually is.** The first result
   from taking the profile seriously was `handle_fw_trap`: a per-instruction
   call the compiler declined to inline, **1.16x on chain-3node-nRF52840**
   (`a8a8b70`, §5.13.4). Two more named in the same profile:
   - **MMIO reads** (`arm_read32` ~7%, plus `find_io_region` inside
     `arm_step_interpreter`). A linear region scan per access; a direct-mapped
     lookup or a per-page table is the obvious replacement.
   - **The Thumb-2 decode nest**, which is the slowest interpreted path
     (`thumb2-dp` is the slowest `arm-bench` case) and 12–23% of these streams.
2. **`arm_step_until`'s convergence tail (§5.7b).** 79.8% of `arm_step` calls
   arrive with a budget of 1 instruction; ~3M call frames per 3 s run to execute
   one instruction each. Timing-sensitive, so it needs the full gate.
3. ~~**Disable the JIT where coverage is low.**~~ **Checked and refused.** The
   profile made this look promising — on Contiki-NG/nRF52840 `arm_step` +
   `arm_jit_run` are 7.5% of runtime for 13% coverage — but measuring it
   directly after the ROM-trap hoist, the JIT is still a net positive
   everywhere it was suspect (7 paired reps each): chain-3node-nRF52840 1.01x,
   chain-4node 1.00x, cc2538 1.05x. The dispatcher costs close to what it
   saves; it does not cost more. Leave it on.
4. **Multi-instruction block verification (§5.10).** `arm-decode` and `arm-jit`
   are both exhaustive over *single* encodings; block **composition** —
   register liveness across instructions, the loop back-edge, join patching —
   is covered only by `CSIM_ARM_JIT_VERIFY=1` over the firmware corpus.

**What is NOT on this list, and why:** more instruction coverage as a way to
speed up Contiki-NG ARM. §5.12 tested that directly — ~8% of the executed
stream added, nothing measurable out — and the probe explains it: the blocks
are short because of *branch density*, and every extra class buys a fraction of
an instruction. **Coverage is a prerequisite for linking, not a substitute for
it.**

Also not on the list: raising these platforms to Zephyr-like numbers. Zephyr's
3.5x comes from a **self-loop** entered once and run ~1000 iterations natively
(§5.12.1); Contiki-NG protocol code has no such loop, and its static block
ceiling (2.4–3.6) is no better than Zephyr's (2.1). Anyone planning against a
3x number for these platforms is planning against the wrong number.

Read §5.10 before touching `arm_jit.c` — it documents a GNU Lightning x86-64
backend bug that made every N-reading condition silently take the wrong branch
while every test on that host stayed green.

**Why now.** ARM is the strategic platform — CC2538, nRF52840, nRF54L15,
TrustZone-M, and the FLPR host all run on it — but it is the *only* emulated ISA
in csim with no acceleration beyond the plain interpreter. MSP430 has a decoder
plus a GNU Lightning JIT; RISC-V got a memoized fetch/decode path (`da4c718`,
~28%). ARM has neither. The legacy platform is the fast one.

Everything below is ordered by **measured evidence → confidence → cost**, not by
how interesting the work is. Tiers 0–1 were near-free; Tier 2 was planned as a
port of an in-tree technique and was **dropped when measurement refuted its
premise** (§4.1), with the effort redirected to what the same measurement
pointed at instead; Tier 3 is a real project.

A note on how to read this document: several claims in it have been withdrawn or
corrected by later measurement, and those corrections are left in place rather
than edited out. That is deliberate — the failure mode this plan exists to avoid
is spending a week on a refactor justified by a number nobody re-checked.

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
per-event clock reads. First estimate, 5 reps, 60 s sim:

| | median | min | max |
|---|---|---|---|
| baseline | 3.003 s | 2.831 s | 3.236 s |
| Tier 0 | 2.737 s | 2.651 s | 2.769 s |

That read as **~8%** with non-overlapping distributions — but the two arms were
measured **at different times**, hours apart, not back to back. Re-measured
properly on one machine state (§2.3) it is **~5.9%**. The original figure is
kept here as the estimate it was; **§2.3 is the number to cite.** The
methodological point is in §1.3, and it applied to this very measurement.

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
That was read as direct support for Tier 2 (a decoded-instruction cache).
**Withdrawn** — see §4.1: real firmware is 78–99.9% Thumb-16, so this synthetic
loop does not represent the executed instruction mix, and the Amdahl ceiling for
caching t32 decode is ~0.4–2%. The measurement is sound; the inference from it
was not.

### 1.2 The runner is ~27% slower on the same ELF — but *not* from runner overhead

`fw-zephyr-sync` runs the *same* Zephyr ELF as the hand-timed measurement in
§0.3, but bare — no kernel, no event pump, no radio:

| Path | MIPS |
|---|---|
| `arm-bench` (bare `arm_step`) | **~196** |
| `nrf52840-dk-multinode` (full runner) | **~155** |

**Correction (measured 2026-08-05).** An earlier draft of this section read that
"~27% of wall time on a real single-node run is spent outside the interpreter,
in the kernel/event-pump path," and used that to size Tier 0. **That attribution
was wrong.** With Tier 0's `CSIM_PHASE_TIMING=1` now reporting the split
directly:

| Workload | inside the event pump | outside it |
|---|---|---|
| `zephyr-synchronization`, 1 node, 60 s | **95.3%** | 4.7% |
| `chain-3node-nrf52840-dk`, 240 s | **96.5%** | 3.5% |

So only ~4–5% is runner scaffolding (service polls, progress, UI). The 27% MIPS
gap is **real emulation work the bare benchmark doesn't do** — RTC/TIMER
interrupts firing, radio events, event-queue dispatch — not overhead to be
optimized away. Tier 0's clock-read removal targeted the small slice, which is
consistent with it landing at ~6% rather than the ~8% first estimated.

The lesson for the tiers below: a MIPS gap between two paths is not evidence of
overhead until something attributes it. Tier 2/3 estimates should be sized
against `arm_step` self time (§0.1), not against this gap.

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
CI should report the number, not hard-gate on it, until a quiet-machine
baseline distribution exists. `mem-ldr-str` is too noisy to gate on at all in
its current form.

#### Measurement protocol — repeated runs are *not* enough

This machine drifts on a timescale of minutes, which defeats the obvious
method. Three separate times in this document a change was sized by running
5 reps of A, then 5 reps of B:

| Claim | by blocked A/B | by interleaved paired A/B | verdict |
|---|---|---|---|
| Tier 0 | ~8% | 5.9% | overstated |
| debug-branch consolidation "ceiling" (§4.2) | ~6% | — | overstated |
| debug-branch consolidation, actual (§4.4) | 1.6% | **3 wins / 9 pairs** | **not real** |

**The protocol that works:** build both binaries, then alternate
`A,B,A,B,…` for N pairs and count how often B beats A *within its own pair*.
Drift then hits both arms equally. A change that is real wins nearly every
pair (Tier 0: 4 of 5 runs below the baseline *minimum*; the §4.2 debug gate:
zero distribution overlap). A change that wins 3 of 9 pairs is noise no matter
what the medians say.

Blocked A/B is still fine for large effects (the §4.2 gate at 14.6% was
unambiguous either way). It is unreliable in the 0–6% band, which is exactly
where most micro-optimizations land.

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

## 2. Tier 0 — **DONE** (~6% measured)

**2.1 Force-inline `condition_passed`.**
`static inline __attribute__((always_inline))`. The function is tiny and
branch-only; the inliner is declining it purely because `arm_step` is enormous.
*Risk: none. Verified 153/153.*

**2.2 Phase Timing — four dead phases deleted, the live one gated.**
The two `get_time_ms()` calls per event-pump iteration existed only to populate
`time_step`. Resolved by evidence rather than preference:

- `time_distribute`, `time_deliver`, `time_flush`, `time_channel_sync` were
  written **zero times** anywhere in the runner and always printed `0.0 ms`
  (`refactor-plan.md` §8 D5). Deleted.
- `time_step` is genuinely live and genuinely useful — it is what says whether a
  workload is interpreter-bound — so it is kept behind `CSIM_PHASE_TIMING=1`.
  The default path now makes no syscall; the diagnostic is one env var away and
  works on any machine without a profiler. It earned its keep immediately: it is
  what refuted the §1.2 overhead claim.

`tools/check-config-equivalence.sh` filtered this block as noise, so removing it
could not affect that guard (re-verified: **EQUIVALENT**, 1599 lines).

### 2.3 Measured result

Back-to-back on one machine state, 5 reps each, `zephyr-synchronization` 1 node
60 s through `nrf52840-dk-multinode` (the workload that exercises both the
interpreter and the pump):

| | median | min | max |
|---|---|---|---|
| baseline | 2.939 s | 2.838 | 3.208 |
| Tier 0 | **2.767 s** | 2.614 | 2.873 |

**~5.9%**, with **4 of 5** Tier-0 runs below the baseline *minimum*.

Two honesty notes:

- The first estimate was ~8%, from a **single** before/after pair. Re-measured
  properly it is ~6%. This is precisely the §1.3 noise-floor warning applying to
  its own author — a single A/B pair on this codebase is not a measurement.
- **`arm-bench` cannot see most of this change** and should not be used to gate
  it: the synthetic loops call `arm_step` directly with no kernel, so only the
  `always_inline` half is visible there, and it lands inside the noise floor
  (−4.4% to +7.3% across cases, all within run-to-run spread). The runner
  workload is the correct instrument for Tier 0.

**Gate (all green):** `correctness`, `arm-correctness` 153/153, `radio-medium`
241/241, `cc1200-mock-host` 73/73, `arm-bench` exit 0, determinism run-twice
byte-identical, config v1↔v2 equivalent, Cooja suite 93/93.

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

## 4. Tier 2 — **DROPPED as scoped**, replaced by the debug-facility gate (14.6%)

### 4.1 The decode-cache premise did not survive measurement

The plan called for a PC-keyed decoded-instruction cache, sized off one number:
`thumb2-dp` running ~18% slower than the Thumb-16 loops (§1.1). Before spending
a week on a refactor that fuses/unfuses ~1100 lines of decode-and-execute, the
premise was checked directly by counting the executed instruction mix
(temporary `CSIM_DECODE_STATS` instrumentation, since reverted):

| Workload | Thumb-2 share | mean t32 chain depth |
|---|---|---|
| `zephyr-synchronization` (nRF52840) | **0.1%** | 5.25 |
| Contiki RPL-UDP (CC2538) | **4.5%** | 4.58 |
| Zephyr `echo_server` 802.15.4 (nRF52840) | **14.7%** | 6.10 |
| Contiki RPL-UDP (nRF52840) | **22.2%** | 7.14 |

**Real firmware is 78–99.9% Thumb-16.** Two consequences kill the idea:

1. **A cache cannot help the majority case.** Thumb-16 "decode" is
   `top5 = hw1 >> 11` plus a computed goto into `thumb_dispatch[32]` — about two
   operations. A PC-keyed cache lookup (index, load, validity test, branch)
   costs *the same or more*. csim's Thumb-16 dispatch is already at the floor;
   there is nothing to memoize. This is the real disanalogy with MSP430, whose
   decode genuinely is expensive (addressing modes, extension words) — not the
   Thumb-2-is-32-bit point flagged earlier.
2. **The addressable minority is small.** Even on the most Thumb-2-heavy
   workload (22.2%), the *fixable* part is the chain walk, not the execute. With
   `thumb2-dp` only 18% off the Thumb-16 pace, the Amdahl ceiling is
   `0.222 × 18% ≈ 4%` for removing the entire gap, and roughly **0.4–2%** for
   removing just the chain walk — against ~1 week of work and a real determinism
   risk.

Dropped. The `thumb2-dp` number was a true measurement of a *synthetic* loop
that turned out not to represent real code; §1.1's "direct support for Tier 2"
claim is withdrawn.

### 4.2 What the measurement pointed at instead — **DONE, 14.6%**

All the Thumb-16 benchmarks cluster at ~200 MIPS regardless of what the
instruction actually does (ALU 199, branch 198, IT-block 205), which says the
per-instruction **fixed overhead** dominates, not the operation. Reading
`arm_step`'s 244-line preamble, five debug facilities were being checked on
*every* emulated instruction:

| Facility | Cost per instruction |
|---|---|
| `arm_trace_step()` | load + predicted branch (already guarded) |
| **PC watchpoints** (`ARM_PC_WATCH`) | init test + **loop setup**, unguarded |
| **Zephyr thread dump** (`ZEPHYR_THREADS`) | init test + 3-term condition, unguarded |
| **Memory watch** (`ARM_MEM_WATCH`) | init test + address test, unguarded |
| `arm_sp_audit_check()` | load + predicted branch (already guarded) |

All are off in every normal run. The three unguarded ones now sit behind a
single cached `arm_debug_facilities_on()`, which does the `getenv` work once and
is thereafter one predictable branch.

This region had bitten the project before: the `ARM_WILD_TRAP` site right below
carries a comment recording that an *unlatched* `getenv()` there was once ~35%
of simulation wall time. Same class of bug, three more instances.

**Measured** back-to-back, 5 reps, `zephyr-synchronization` 1 node 60 s:

| | median | min | max |
|---|---|---|---|
| Tier 0 | 2.716 s | 2.625 | 2.744 |
| + debug gate | **2.319 s** | 2.264 | **2.433** |

**14.6%, with completely non-overlapping distributions** (new max 2.433 < old
min 2.625) — a far stronger signal than Tier 0's. Unlike Tier 0 this *is*
visible to `arm-bench`, because it is in the interpreter core rather than the
event pump: **+17.0% `alu-reg`, +18.2% `thumb2-dp`, +21.2% `fw-zephyr-sync`,
+13.5% `fw-cc2538-udp`** against the pre-Tier-0 baseline.

Facilities re-verified working after the change: `ARM_MEM_WATCH` still logs
byte changes, `ARM_PC_WATCH` still reports hit counts, `ZEPHYR_THREADS` still
runs clean.

**Cumulative Tier 0 + this: 21.1%** on the runner workload
(2.939 s → 2.319 s).

### 4.3 If a decode cache is ever revisited

Only worth it for a workload profile that is genuinely Thumb-2-heavy, and only
after `CSIM_DECODE_STATS`-style evidence that it is. Even then, scope it to the
`t32_decode` chain alone — never to Thumb-16 — and note that a cheap
intermediate exists: a `switch (op1)` pre-dispatch would cut the mean chain
depth from ~7 to ~3 on Contiki workloads without any decode/execute refactor.
Verify the 12 arms are genuinely mutually exclusive first; the trailing
`(hw1 & 0xEC00) == 0xEC00` VFP arm looks like a catch-all and may overlap.

### 4.4 Negative result — consolidating the *guarded* debug branches is worth ~0

After §4.2 landed, four debug checks remained in the per-instruction path:
`arm_trace_step()`, the new `arm_debug_facilities_on()` block,
`arm_sp_audit_check()`, and the `ARM_WILD_TRAP` test. Folding the last two into
the first looked like a free extra win — a cross-time "ceiling" experiment
(delete them outright) suggested ~6%.

It is not a win. Properly interleaved, 9 paired runs:

| | median | min | max |
|---|---|---|---|
| §4.2 as committed | 2.353 s | 2.166 | 2.546 |
| + consolidation | 2.316 s | 2.258 | 2.404 |

1.6% by median, but **consolidated was faster in only 3 of 9 pairs** — i.e.
slower more often than not. Reverted.

**Why, and the generalisable lesson:** `arm_sp_audit_check`, `arm_trace_step`
and the wild-trap test were *already* `__builtin_expect`-guarded single
predictable branches. A perfectly-predicted not-taken branch is essentially
free on an out-of-order core — it costs a slot that was idle anyway. The 14.6%
in §4.2 did **not** come from "fewer branches"; it came specifically from the
three *unguarded* facilities, above all the PC-watchpoint **loop setup**
(`for (int i = 0; i < arm_pcw_count; i++)`), which the compiler cannot hoist
and which is not a simple predicted branch.

So the rule for anything further in this path: **look for unguarded work, loop
setups, and unlatched `getenv`/syscalls — not for branch count.** Removing
already-predicted branches from a hot loop is a non-optimization.

## 5. Tier 3 — ARM JIT — **DONE**, incl. memory ops; Thumb-2 is what remains

### 5.0 What a JIT is actually worth here — measure before committing weeks

GNU Lightning was not installed on either dev machine, so **nothing in this
tree had ever been JIT-compiled**. Installing it (`brew install lightning` on
the Mac; already present at /usr/local on jftest4) made the existing MSP430 JIT
measurable — same binary, JIT on versus `MSPSIM_JIT_THRESHOLD` set impossibly
high:

| Host | interpreter | + JIT | speedup |
|---|---|---|---|
| Apple Silicon, clang | 445 MIPS | 557 MIPS | **1.25×** (5/5 pairs, p=0.031) |
| Linux x86-64, gcc | 266 MIPS | 490 MIPS | **1.84×** |

**A JIT is strongly platform-dependent in this codebase.** The M-series core
handles the indirect-branch-heavy interpreter loop so well that there is much
less for generated code to beat; the x86-64/gcc interpreter is slower, so the
JIT has more headroom. Anyone sizing this work from the 1.84× figure alone
will be disappointed on Apple Silicon.

### 5.1 Why ARM is slower than MSP430 — decomposed

The obvious framing ("MSP430 is faster, build a JIT to catch up") was wrong
twice over: MSP430 reaches its numbers *without* a JIT on the Mac, and part of
its lead is inherent. Ceiling experiment, removing each per-instruction
obligation ARM has that MSP430 does not:

| variant | MIPS |
|---|---|
| ARM, as committed | 239 |
| − GDB check, − ROM traps | 259 |
| − IT-block machinery | 263 |
| − all three | **321** |
| MSP430 (interpreter) | **421** |

The 1.76× gap ≈ **1.34× removable overhead × 1.31× inherent**. The removable
part shipped (§5.2). The residual ~1.3× is the IT-block state machine and ISA
complexity — not removable by tuning, which is the honest argument *for* a JIT
rather than against one.

### 5.2 Shipped from that analysis

- **Flash window hoisted** out of the fetch path (`86d29b5`): `cpu->flash` is a
  pointer, so any store in the loop may alias it and force a reload every
  instruction. MSP430 has always cached `memory`/`max_mem` in locals.
  **4.3%**, 15/20 pairs, p=0.021.
- **GDB-stub and ROM-trap checks hoisted** (`8c3cb60`): both set once (at GDB
  attach / ELF load) but re-tested every instruction. MSP430's interpreter has
  neither obligation — 0 GDB checks vs ARM's 5. **15.9%**, 12/12 pairs,
  p=0.0002. GDB behaviour verified unchanged by diffing the full RSP exchange.

### 5.3 The real blocker was the decoder, and it is now built (`c40c994`)

The MSP430 JIT is built on `msp430_decode.c` → `decoded_insn_t` →
`msp430_jit_compile`. **ARM had no decoder at all** — decode is fused into
execution, and `arm_decode.h` was a dead 21-line stub referenced from nowhere.
Landed:

- `arm_decode.c` — Thumb-16 decoder + `arm_decode_block()` with an
  `all_supported` flag the compiler must honour.
- `arm_execute_decoded()` — reference semantics the generated code must
  reproduce, in `arm_cpu.c` so it shares the interpreter's flag helpers.
- `test_runner arm-decode` — differential test, now in CI (0.29 s).

**Subset: 18880 of 65536 encodings.** Small on purpose — anything unsupported
is refused and interpreted, so it can never run *wrong*, only slowly. ADC/SBC
are excluded by name: carry-in is exactly what broke the MSP430 JIT.

**Verification is differential and exhaustive**, not by inspection: all 65536
halfwords × 6 randomised register/flag states through both paths, comparing
r0–r15, APSR and cycles. 269952 comparisons, 0 failures, identical on
Linux/gcc. Validated by fault injection — a flag-only BIC bug (invisible to
`arm-correctness`) is caught immediately.

### 5.4 Codegen — **DONE** (`06f62b3`)

*The figures in this section are the first-cut ones; §5.8 supersedes them after
memory ops and the dispatcher fixes.*

Measured interleaved (`CSIM_ARM_JIT=0` vs `1`), 7 pairs, Apple Silicon @ 4.39 GHz:

| benchmark | JIT off | JIT on | speedup | cycles/insn | paired |
|---|---|---|---|---|---|
| `alu-reg` | 276.8 | **1367.1** MIPS | **4.94x** | 15.9 -> **3.2** | 7/7 |
| `fw-zephyr-sync` (real Zephyr) | 300.9 | **371.5** MIPS | **1.23x** | 14.6 -> **11.8** | 7/7 |
| `it-block`, `branch`, `mem-ldr-str`, `thumb2-dp`, `fw-cc2538-udp` | | | ~1.00x | | |

Linux/x86-64 confirms it: `alu-reg` 184 -> 837 MIPS (4.5x).

**3.2 host cycles per emulated instruction beats the MSP430 JIT's 7.8** — the
generated code is not the limit; coverage is.

**Native self-loops are where the win is.** A conditional branch back to the
block head compiles to a native loop under an iteration budget, instead of one
call per iteration. Measured `avg_block` = **999.5** instructions per entry on
zephyr-synchronization, against 9.9 for straight-line blocks. That shape
dominates firmware: a temporary top5 histogram showed `SUBS`+`BNE` is **99.8%**
of executed instructions there.

### 5.5 Cycle-exactness cost a design change — and caught a bug in my dispatcher

`CSIM_ARM_JIT=0` and `=1` produce **byte-identical** output on
`test-2node-nrf54l15-dk`, `chain-3node`/`chain-4node-nrf52840-dk` and
`chain-3node-nrf54l15-dk`. Not optional: determinism is a gated guarantee, and
a JIT that shifted timing would make results depend on whether GNU Lightning
happened to be installed on the build machine.

The first dispatcher ran the interpreter in fixed 32-instruction batches
between cache probes. **That alone shifted simulation timing** — a TX moved
4.240 s -> 3.078 s — with the compile threshold set so high that nothing
compiled, so batching was the only variable:

| dispatcher | vs no dispatcher |
|---|---|
| batch = 32 | **DIFFERS** |
| batch = caller's full budget | byte-identical |

`arm_step`'s contract is "up to `count` instructions", and the interpreter's
WFI/event handling is sensitive to that budget. The fix: hand over the full
budget and probe once per `arm_step` call — ample, since a 3 s Zephyr run makes
4.18M such calls.

*The lesson generalises.* The isolation run — dispatcher present, codegen
disabled — is what separated "my batching" from "my code generation". Without
it the natural conclusion would have been that the codegen was wrong.

### 5.6 Memory operations — **DONE** (`d39ccf4`), and what they cost to get right

Loads and stores were 35% of what stopped a block early, and unlike everything
else in the subset they can *fail*. A store into an IO window fires a
peripheral callback that can raise an interrupt or reschedule an event, which
breaks the property the whole block design rests on: nothing observable happens
between block entry and block exit.

The shape that preserves it is a **guarded side exit**. Inline the SRAM case
behind two tests — alignment, and `(uint32)(addr - sram_base) < sram_size`,
which excludes flash, ROM, the bit-band alias and every peripheral window in
one unsigned compare — and on a miss leave the block at that instruction with
state exactly as if only the preceding ones had run. The interpreter redoes it
through the full IO path. **Correctness therefore never depends on the guard
being generous, only on it being sound**: a guard that rejects too much costs
speed, not accuracy.

Consequences worth knowing before repeating this:

- A block's cost stopped being its length. Memory ops charge 2 cycles (the
  central charge plus the handler's own), everything else 1, so blocks carry a
  cycle prefix-sum table and the dispatcher reconstructs both counts from it.
- `LDR Rt,[PC,#imm]` gets its own class: the address is a compile-time
  constant, flash is read-only in this model, and the cache is flushed on
  reset — so it resolves the region once and emits **no guard at all**.
- Decoder support went 22528 -> 44992 encodings; `arm-decode` is now **269952**
  differential comparisons, 0 failed.

**The verifier had to learn to rewind memory.** Its first run against
`ldr r3,[sp,#4] / adds r3,#1 / str r3,[sp,#4]` reported r3 off by exactly one,
every time. Nothing was wrong with the generated code: the verifier replayed
the block over the memory the *first* run had already updated. Re-running a
block is only a valid comparison if the whole input state is restored, and
memory joined that state the moment stores were compiled. The snapshot is
skipped for load-only blocks.

### 5.7 The two limits that actually mattered — and neither was instruction coverage

*(a) is the min-block cliff; (b), referred to elsewhere as §5.7b, is the
`arm_step` budget granularity.*

With memory ops landed, coverage on `zephyr-synchronization` was still 26.3%.
The instinct is "add more instruction classes". The measurement says otherwise:
**72% of executed instructions on cc2538 RPL-UDP already decode**. Two
structural limits were doing the damage.

**(a) The fallback is all-or-nothing per `arm_step` call.** When the dispatcher
finds no block at the current PC it hands the interpreter the caller's *entire*
remaining budget — which it must, because chopping that budget changes
simulation timing (§5.5). So one uncompilable PC costs every compilable block
that would have followed it in that call. Refusing a 1-instruction block at the
PC after a Thumb-2 instruction does not cost one instruction, it costs the rest
of the slice. That makes the minimum-block-length knob a cliff, not a
trade-off:

| `CSIM_ARM_JIT_MIN_BLOCK` | insns via compiled code | wall clock |
|---|---|---|
| 4 | 26.3% | 1.55 s |
| 2 | 26.3% | 1.59 s |
| **1** | **98.3%** | **0.58 s** |

Compiling everything keeps the dispatcher chaining block to block and it stops
falling back at all. The cost was measured rather than assumed: **163 live
compiled blocks, +4.8 MB peak RSS** — a firmware image's working set is nothing
like the 65536-slot cache.

**(b) A budget the JIT could never satisfy.** Histogramming `arm_step`'s
`count` argument on cc2538 RPL-UDP:

| `count` | share of 6.85M calls |
|---|---|
| **1** | **79.8%** |
| 5–8 | 20.1% |
| everything else | 0.1% |

`arm_step_until` single-steps once it is within 10 cycles of its target, and
that tail is most of its calls. A 4-instruction block — the average on that
firmware — can never run on a budget of 1, so roughly 61% of all executed
instructions were unreachable by the JIT regardless of coverage.

The fix is to notice that **the instruction count was only ever a proxy for the
cycle target**. A block is now admitted when its whole measured cost fits in
`cpu->cycle_limit`, even if it exceeds the instruction budget. That is
*stricter* than what the interpreter offers, not looser: `arm_step(cpu, 1)`
overshoots by however many cycles one instruction happens to cost (2 for a
load, 12 for exception entry, 20 for a divide), whereas a block is admitted
only if it provably fits.

### 5.8 Result

`arm-bench`, 7 paired interleaved reps, Apple Silicon @ 4.39 GHz:

| benchmark | JIT off | JIT on | speedup | host cycles/insn | paired |
|---|---|---|---|---|---|
| `alu-reg` | 272.7 | **1216.4** MIPS | **4.46x** | 16.1 -> **3.6** | 7/7 |
| `mem-ldr-str` | 262.0 | **1076.7** MIPS | **4.11x** | 16.8 -> **4.1** | 7/7 |
| **`fw-zephyr-sync`** (real Zephyr) | 296.9 | **1033.3** MIPS | **3.48x** | 14.8 -> **4.2** | 7/7 |
| `branch` | 275.5 | **568.6** MIPS | **2.06x** | 15.9 -> **7.7** | 7/7 |
| `it-block` | 288.3 | 292.4 | 1.01x | | 3/7 |
| `thumb2-dp` | 219.8 | 209.7 | 0.95x | | 2/7 |
| `fw-cc2538-udp` | 171.0 | 168.4 | 0.98x | | 3/7 |

Whole-runner workloads, 5 paired reps:

| workload | JIT off | JIT on | speedup |
|---|---|---|---|
| `zephyr-synchronization`, 1 node, 60 s sim | 2.05 s | **0.60 s** | **3.42x** |
| `chain-3node`/`chain-4node-nrf52840-dk` | 1.04 / 1.18 s | 1.04 / 1.17 s | 1.00x |
| cc2538 2-node RPL-UDP, 120 s sim | 0.24 s | 0.24 s | 1.00x |
| `chain-3node-nrf54l15-dk` | 0.81 s | 0.82 s | 0.99x |

Linux/x86-64 (jftest4, gcc) agrees and is larger, because its interpreter
baseline is lower. **One interleaved pair, not a paired median** — the seven-rep
protocol above was run on Apple Silicon only:

| benchmark | JIT off | JIT on | |
|---|---|---|---|
| `fw-zephyr-sync` | 190.3 | **1170.5** MIPS | **6.15x** |
| `alu-reg` | 188.5 | 882.6 MIPS | 4.68x |
| `mem-ldr-str` | 187.5 | 669.7 MIPS | 3.57x |
| `branch` | 197.5 | 369.8 MIPS | 1.87x |
| `thumb2-dp`, `it-block`, `fw-cc2538-udp` | | | ~1.00x |

**The flat cases are honest and the reason is the same one.** They are Thumb-2
dense, so almost nothing compiles and they pay the probe: average block length
is **1.7–2.4 instructions on Contiki-NG ARM firmware against 34.9 on Zephyr**.
Thumb-2 is the whole remaining gap on that firmware family, and it is the next
lever — not more Thumb-16 classes, of which only push/pop and the hi-register
forms are left.

### 5.9 Verification

- **`CSIM_ARM_JIT_VERIFY=1`** lockstep (registers, APSR, cycles, and now SRAM):
  **0 mismatches** over `arm-bench`, `arm-correctness`, `zephyr-synchronization`,
  cc2538 RPL-UDP, `chain-3node-nrf52840-dk` and `test-2node-nrf54l15-dk`.
- **JIT on/off byte-identical** (wall-clock lines excluded) on
  `test-2node-nrf54l15-dk`, `chain-3node`/`chain-4node-nrf52840-dk`,
  `chain-3node-nrf54l15-dk`, `chain-4node-cc2538dk`. On the 60 s
  `zephyr-synchronization` run the two agree to the cycle: 3813589583 cycles,
  453300376 instructions, same final PC.
- **`arm-decode`**: 269952 differential comparisons, 0 failed, both hosts.
- **`arm-jit`** (new, §5.10): 359936 generated-code comparisons over 44992
  encodings, 0 failed, both hosts.
- **TSCH passes on both** `cc2538dk` and `nrf52840-dk` — the timing-sensitive
  case, and the one that would break first if a block ran past an event.
- **Full gate on Linux/x86-64** (the host the bug in §5.10 lived on): every
  suite above, all five configs cycle-exact, lockstep clean on three workloads,
  MSP430 determinism check, both TSCH tests, and the **Cooja suite 93/93,
  0 failed, 0 skipped** — including all eight `17-tun-rpl-br` border-router
  tests under `--with-tun`.

### 5.10 The bug that justified all of it, found on the second host

The gap flagged here in the previous revision — "`emit_cond()` is the
highest-risk function in the JIT and is covered only by whatever the corpus
happens to execute" — turned out to be occupied.

```
jit_andi(dst, src, 0x80000000) returns 0 on x86-64 GNU Lightning 2.2.3.
```

`emit_cond()` extracted the N flag exactly that way, so **MI, PL, GE, LT, GT
and LE all evaluated as though N were clear** on x86-64. Isolated in a
standalone Lightning program with no csim involved: `ori`, `bmci`, `bmsi` and
`andi` with `0xFFFFFFFF` are all correct on the same operand, and the ARM64
backend is correct throughout. The fix is a logical right shift, which carries
no immediate.

**What it looked like from the outside is the part worth remembering.** On the
host where it was wrong, every signal being watched stayed green: `arm-decode`
269952/269952, `arm-correctness` 153/153, the full Cooja suite 93/93, and
simulations producing entirely plausible output. They simply took the wrong
branch sometimes. Two properties kept it hidden — it is **host-specific**, so
the development machine could not find it, and it only bites where short
conditional tails are compiled, so it was dormant at `MIN_BLOCK=4` and appeared
the instant that became 1. It surfaced only because `CSIM_ARM_JIT_VERIFY=1` was
run on the other architecture, where it reported 902007 mismatches in which
**r15 was the only register that differed and the flags agreed** — the JIT
computing N correctly and then reading it back as zero.

**`arm-jit` now closes it.** For every one of the 65536 Thumb halfwords the
decoder accepts, compile a one-instruction block and run the generated code
against the interpreter over 8 register/flag states, comparing r0–r15, APSR and
cycles: 44992 encodings, **359936 comparisons, 0 failed on both arm64 and
x86-64**. The conditional-branch encodings supply all 14 condition codes
against random flags; memory encodings are driven both at SRAM addresses (the
inline path) and at wild ones, exercising 67583 guarded side exits and checking
each leaves state untouched.

`arm-decode` could never have caught this, and the distinction generalises:
`arm-decode` validates the *description* of an instruction, while the JIT ships
a **third** implementation — the machine code Lightning emits from that
description. Only running that code tests it, and only on the host it was
emitted for.

**Remaining gap:** neither suite runs multi-instruction blocks, so a bug in how
blocks are *composed* (register liveness across instructions, the loop
back-edge, join patching) is still corpus-covered only. `CSIM_ARM_JIT_VERIFY=1`
over the firmware corpus is what covers that today.

### 5.11 Why Contiki-NG ARM firmware is hard — decomposed

The summary above used to say these workloads are flat "because they are
Thumb-2 dense". That is true of one of them and false of the other, and the
real answer is structural. Executed-instruction mixes, measured by decoding
every instruction as the interpreter retires it:

| | Zephyr sync (nRF52840) | Contiki-NG cc2538 | Contiki-NG nRF52840 |
|---|---|---|---|
| speedup | **3.5x** | 1.14x | 1.00x |
| JIT-decodable | 94.1% | 69.1% | 64.4% |
| block-terminating branches | 9.1% | 10.5% | **25.8%** |
| Thumb-2 | 3.4% | **6.3%** | **23.3%** |
| single largest class | `SUBS #imm` **80.8%** | `LDR [SP,#imm]` 22.8%, `NOP` 18.0% | `B<cond>` 25.8% |
| mean compiled block | **34.9** | 3.3 | 1.8 |

**5.11.1 The Zephyr number is a spin loop, and should be read as one.**
80.8% of that workload's executed instructions are a single `SUBS Rd,#imm`, and
8.4% are the `B<cond>` closing the loop around it. The JIT compiles that pair
into a *native* loop that runs under an iteration budget without re-entering —
999.5 instructions per block entry on the pure delay loop. So 3.5x is a real
measurement of a real Zephyr image, but the mechanism is "this firmware
busy-waits", not "Zephyr code compiles well in general". It flatters the JIT
and the comparison table should not be read as Zephyr-vs-Contiki.

**5.11.2 The JIT's leverage is amortising block entry, and Contiki-NG's code
shape denies it that.** Entering a compiled block costs a cache probe, a call,
a prologue saving the callee-saved registers, an xPSR load, and the mirror
image on exit — call it 15–25 host cycles against an interpreter costing ~15
cycles *per instruction*. At Zephyr's 34.9 instructions per entry that is
noise. At Contiki-NG's 1.8–3.3 it is most of the win.

And block length there is not primarily a coverage problem. **25.8% of what
Contiki-NG executes on nRF52840 is a conditional branch** — a block must end at
one — so even with 100% instruction coverage the mean block would be under 4.
The workload is protocol code: short functions, frequent calls and returns,
data-dependent branching. That is the shape a basic-block JIT has least to
offer. Zephyr's delay loop is the opposite extreme.

**5.11.3 Coverage is real but secondary, and this was tested rather than
argued.** `NOP` was 18.0% of cc2538's stream and unsupported, so adding it (a
class that emits nothing) was a clean single-variable experiment:

| | before | after |
|---|---|---|
| coverage | 28.5% | **71.4%** |
| mean block | 2.4 | 3.3 |
| wall clock | 0.240 s | 0.210 s (**1.14x**, 7/7 paired) |

**A 2.5x jump in coverage bought 14%.** That is the ceiling asserting itself,
and it is the number to remember before committing weeks to any further
instruction class.

**5.11.4 Some of these workloads are barely interpreter-bound at all.**
`CSIM_PHASE_TIMING=1` reports ~95% "step (CPU)" for both Contiki-NG workloads,
but that phase includes event dispatch and WFI fast-forward. Dividing
instructions by step time and comparing against the interpreter-only rate from
`arm-bench` separates them:

| | instructions / step time | interpreter-only (`arm-bench`) | actually interpreting | ceiling for a perfect JIT |
|---|---|---|---|---|
| cc2538 2-node RPL-UDP | 58 MIPS | 170 MIPS | ~34% | **~1.5x** |
| chain-3node-nRF52840 | 126 MIPS | ~200 MIPS | ~63% | **~2.5x** |

The rest is radio bytes, the event queue, and idle time being skipped — none of
which a faster interpreter or a JIT touches. `chain-4node-cc2538dk` is further
along the same axis and does not move at all (1.01x).

**5.11.5 What this means for planning.** The four effects compound: a third to
two thirds of wall time is outside the interpreter; a quarter of what is left
is branches that end blocks; short blocks do not repay entry; and the remaining
uncovered classes are each a few percent. **No amount of codegen work makes
Contiki-NG ARM look like the Zephyr number.** The honest targets are ~1.5x on
cc2538 and ~2.5x on nRF52840, and interpreter work — which is subject to none
of these ceilings — competes well against JIT work for the same effort.

### 5.12 The cheap classes bought nothing, and the probe that explains why

After `NOP`, the remaining cheap Thumb-16 classes were added together
(`d5ba3b3`): high-register ADD/CMP/MOV, CBZ/CBNZ, SXTB/SXTH/UXTB/UXTH, ADR,
`ADD Rd,SP,#imm`, `ADD/SUB SP,#imm`. That is **~8% of everything Contiki-NG
executes** on both platforms, verified to 410400 generated-code comparisons.

**Result: no measurable speedup.**

| | coverage | mean block | wall clock (7 pairs) |
|---|---|---|---|
| cc2538 2-node RPL-UDP | 71.4% → 71.5% | 3.3 → 3.3 | 1.14x → 1.10x |
| chain-3node-nRF52840 | 12.4% → 13.2% | 1.8 → 1.8 | 1.00x → 1.02x |

Removing a block-stopper only lengthens a block if the *next* instruction is
also supported, and it usually is not. CBZ/CBNZ is the clean illustration: it
used to end a block by being unsupported and now ends it by being a
terminator, so the block gains the CBZ itself and nothing else.

**5.12.1 Probe: how long could blocks be, under any coverage?**

Rather than guess at Thumb-2's value, walk forward from every executed PC and
count how far a block *could* reach under three assumptions. (Static block
length — one pass, not counting loop iterations.)

| assumption | Contiki nRF52840 | Contiki cc2538 | Zephyr sync |
|---|---|---|---|
| today | 1.09 | 3.04 | 1.68 |
| **+ Thumb-2 supported** | **2.23** | **3.45** | 1.96 |
| + every non-branch instruction | 2.41 | 3.63 | 2.10 |
| *(= branch-only ceiling)* | | | |

Two things fall out, and the second one reframes this whole section.

**Thumb-2 is most of the remaining headroom on nRF52840 (1.09 → 2.23 of a 2.41
ceiling) and almost none on cc2538 (3.04 → 3.45).** The blanket "do Thumb-2"
recommendation was wrong in the same way the blanket "Contiki is Thumb-2 dense"
claim was.

**And Zephyr's ceiling is 2.10 — lower than cc2538's.** Yet Zephyr runs 3.5x
and cc2538 1.1x. So *static block length is not what distinguishes them*, and
§5.11.2 was reaching for the wrong variable. What distinguishes them is that
Zephyr's hot block is a **self-loop**: it is entered once and runs ~1000
iterations inside compiled code, so the entry cost is amortised a thousandfold.
The `avg_block=31.7` reported by `CSIM_ARM_JIT_STATS` counts iterations, not
instructions decoded — it was measuring the loop, not the block.

Contiki-NG protocol code has no such loop. It has a branch every 2.4–3.6
instructions and returns to the dispatcher at each one.

**5.12.2 How fast is the compiled path actually, per instruction?**

This can be derived from measurements rather than modelled. cc2538 spends ~34%
of wall time interpreting (§5.11.4), runs 71.5% of instructions through
compiled code, and comes out 1.10x overall. Solving for the per-instruction
speedup *f* of the compiled path:

```
1.10 = 1 / (0.66 + 0.34 × (0.285 + 0.715/f))   →   f ≈ 1.6
```

**At a static block length of 3.3, compiled code is only ~1.6x faster per
instruction than interpreting it.** Block entry — the cache probe, the call,
the prologue saving callee-saved registers, the xPSR load and its mirror on
exit — is eating most of the 4–5x the generated code achieves inside a long
block (§5.8: 3.6 host cycles/instruction on `alu-reg`, against ~15 interpreted).

Applying the same arithmetic forward: Thumb-2 taking nRF52840 from 1.09 to 2.23
would lift *f* to roughly 1.4, and with 63% of wall time interpreted that is
**about 1.15x overall — two weeks for 15%.**

**5.12.3 What this says to do instead: block linking, not more coverage.**

If per-entry cost is the binding constraint, the fix is to stop paying it. The
standard technique is **block chaining** — a compiled block ends by jumping
*directly* into the next compiled block instead of returning to the dispatcher,
so a chain of 2-instruction blocks runs without re-entering C at all. It makes
block length largely irrelevant, which is exactly the property these workloads
need, and it benefits every platform rather than one.

It is not free: something must still bound how long a chain runs, because a
block contains no event check. The natural shape is a cycle-budget test at each
chain entry (the data for it — `cycles_total` per block and `next_event_cycle`
— already exists, §5.7), plus unlinking on cache flush.

**Recommended order, revised:** block linking first; Thumb-2 only afterwards,
and then for nRF52840's sake specifically. Thumb-2 before linking would be
buying instructions for blocks that cannot pay for themselves.

### 5.13 Block linking: built, measured, **reverted** — and the profile that ended the JIT line of work

§5.12.3 recommended block linking. It was built and it is a regression.

**5.13.1 What was built, and why it is a trace and not a link.** GNU Lightning
cannot express a jump from one compiled function into another: the target's
prologue pushes a frame nothing pops, so a chain would leak stack per link. The
equivalent that *is* expressible is to build the chain at compile time — keep
decoding across the branch and emit the whole run as one function with one
prologue. `B` is followed to its target and generates no code at all;
`B<cond>`/`CBZ` continue down the fall-through with the taken path becoming a
side exit, reusing the mechanism the guarded memory accesses already had. A
branch to somewhere already in the trace ends it, which keeps traces
straight-line and preserves the self-loop shape worth 3.5x on Zephyr.

**5.13.2 It did what it was supposed to and lost anyway.**

| cc2538 2-node | basic blocks | traces |
|---|---|---|
| instructions per block entry | 3.3 | **5.3** |
| block entries | 1.96M | **999K** |
| coverage | **71.5%** | 58.3% |

Measured as a single variable in one build, interleaved, 7 pairs:

| | blocks | traces | |
|---|---|---|---|
| cc2538 2-node RPL-UDP | 0.200 s | 0.220 s | **0.91x** — traces won 1/7 |
| chain-3node-nRF52840 | 0.970 s | 0.980 s | 0.99x |
| zephyr-sync | 0.560 s | 0.560 s | 1.00x |

Entries halved and each covered 60% more instructions, exactly as the probe
predicted — and coverage *fell*, because a longer trace needs more cycle
headroom before the next scheduled event, and 79.8% of `arm_step` calls arrive
with about ten cycles of it (§5.7b). Traces are entered less often than the
blocks they replace. Recovering that needs an in-trace cycle check so a trace
can be entered and stop early; the arithmetic in §5.12.2 puts the result at
~1.14x against 1.10x today. Not worth the complexity in the highest-risk file
in the tree. **Reverted** (patch kept out of tree).

**5.13.3 The profile nobody had run.** Every estimate in §5.11–§5.12 rested on
inferring the interpreter's share of wall time from MIPS ratios. Sampling the
running processes instead:

| Contiki-NG cc2538, JIT on | share |
|---|---|
| `arm_step_interpreter` | ~75% |
| `handle_fw_trap` | ~11% |
| `arm_read32` (MMIO) | ~7% |
| **compiled JIT code** | **~5%** |

**The 71.5% of instructions the JIT compiles account for about 5% of the
runtime.** They are the cheap ones. The 28.5% it does not compile — MMIO
accesses through `find_io_region` into a device callback, 64-bit division
traps, Thumb-2 — are roughly 37x more expensive apiece and hold ~75%. That
single number explains every null result in §5.12 and §5.13 at once, and it
should have been measured before any of that work rather than after.

On Contiki-NG/nRF52840 the same profile also shows **`arm_step` + `arm_jit_run`
at 7.5% of runtime for 13% instruction coverage** — the JIT dispatcher there
costs close to what it saves. Close to, but not more than: measured after the
ROM-trap hoist, `CSIM_ARM_JIT=1` still beats `=0` on every workload where this
looked doubtful (chain-3node 1.01x, chain-4node 1.00x, cc2538 1.05x, 7 paired
reps each), so it stays on.

**5.13.4 The immediate payoff.** `handle_fw_trap` at ~10% was a per-instruction
call the compiler declined to inline — the `condition_passed` defect from Tier
0 all over again, hidden because the *guard* had been hoisted while the call had
not. Two register compares in place of the call: **1.16x on chain-3node-nRF52840
(7/7 paired), 1.05x on cc2538** (`a8a8b70`). That is a larger win on those
platforms than the memory ops, the NOP class, the cheap Thumb-16 classes and
trace formation put together.

**5.13.5 Conclusion: the ARM JIT is finished as a line of work.** It is worth
3.4x on loop-shaped firmware and it has harvested essentially everything
reachable on protocol firmware, where the instructions it can compile are not
where the time is. Further coverage, linking or Thumb-2 all optimise the 5%.

## 6. Sequencing, risk, rollback

| Step | Effort | Confidence | Gate |
|---|---|---|---|
| `arm-bench` | ½ day | — (enabler) | builds; numbers stable across 5 reps |
| ~~Tier 0~~ **DONE** | — | — | **~5.9% measured** (4/5 runs below baseline min); full gate green |
| ~~Tier 1 PGO~~ **DONE** | — | — | **1.55× clang / 1.18× gcc on an untrained workload**; suites green on both |
| ~~Tier 2 decode cache~~ **DROPPED** | — | premise refuted (§4.1) | — |
| **Tier 2′ debug gate** **DONE** | ½ day | — | **14.6% measured**, non-overlapping; full gate green |
| ~~branch consolidation~~ **REVERTED** | — | 3 wins / 9 paired runs | not a real effect (§4.4) |
| **Tier 3 decoder** **DONE** | ½ day | — | **269952 differential comparisons, 0 failures**; in CI |
| **Tier 3 codegen** **DONE** | 1 day | — | 4.46x ALU, cycle-exact, JIT_VERIFY 0 mismatches |
| **Tier 3b memory ops + dispatch** **DONE** | 1 day | — | **3.42x on real Zephyr firmware** (§5.8); 269952 decode comparisons |
| **`arm-jit` generated-code suite** **DONE** | ½ day | — | **359936 comparisons over 44992 encodings, 0 failures**, both hosts; in CI |
| **Tier 3c NOP** **DONE** | 1 h | — | **1.14x on cc2538**, 18% of its stream (§5.11.3) |
| **Tier 3d cheap Thumb-16 classes** **DONE** | ½ day | — | **no measurable speedup** — kept as a linking prerequisite (§5.12) |
| ~~Tier 3e block linking~~ **REVERTED** | 1 day | — | **0.91x on cc2538** — entries halved, coverage fell (§5.13.2) |
| ~~Tier 3f Thumb-2~~ **DROPPED** | — | optimises the 5% the JIT already reaches (§5.13.3) | — |
| **ROM-trap hoist** **DONE** | 1 h | — | **1.16x on chain-3node-nrf52840**, 7/7 paired; byte-identical (§5.13.4) |
| MMIO region lookup | ~2 days | `arm_read32` ~7% + `find_io_region` (§5.13.3) | full gate, byte-identical |
| `arm_step_until` convergence tail | ~2 days | 79.8% of calls carry a budget of 1 (§5.7) | timing-sensitive — full gate, byte-identical |
| ~~flash-window hoist~~ **DONE** | — | 15/20 pairs, p=0.021 | **4.3%** |
| ~~gdb/ROM-trap hoist~~ **DONE** | — | 12/12 pairs, p=0.0002 | **15.9%** |

**Mandatory gate for anything touching `arm_step` or the event pump:**
`arm-correctness` (153), `arm-decode` (269952), `arm-jit` (359936),
`correctness`, `radio-medium` (241), `cc1200` (73), `arm-firmware`, the
cc2538/nRF52840/nRF54L15 multinode configs, TSCH ×2, the Cooja suite
(**93/93**, `--with-tun`), and `tools/check-determinism.sh`.

**And for anything touching the JIT specifically:** `CSIM_ARM_JIT=0` vs `=1`
byte-identical on the five multinode configs, `CSIM_ARM_JIT_VERIFY=1` clean
over the firmware corpus, **and both of those repeated on the other host
architecture** — §5.10 is a bug that was correct on arm64 and silently wrong on
x86-64, and no single-host gate could have caught it. Per
`refactor-plan.md` §11.5, a tier that regresses the gate is **reverted, not fixed
forward**.

**What this plan deliberately does not do:** chase the 311× idle-skip number as
if it were engine speed — that measures event-queue policy, not the interpreter,
and it stays labelled as such. (The companion offender, `make pgo`'s unmeasured
"~40%", is now settled: §3.1 replaced it with per-toolchain numbers from an
untrained holdout, and both `README.md` and `CLAUDE.md` were corrected.)

---

## 7. Open questions

1. ~~**Phase Timing report — delete or gate?**~~ **Resolved:** the four
   always-zero phases are deleted, the one live phase is gated behind
   `CSIM_PHASE_TIMING=1`. See §2.2.
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
5. **Should the GNU Lightning `andi 0x80000000` bug (§5.10) go upstream?** It is
   worked around here and the tree has no other use of that immediate (the
   remaining `F_N` uses are `ori`/`bmci`, both verified correct on both
   backends), so nothing is blocked either way — but it is a live defect in
   2.2.3 that another project will hit.
6. **Is `remaining >= cb->length` still needed at all?** The cycle-target
   relaxation (§5.7b) made the instruction budget the looser of two bounds on
   every path that sets `cycle_limit`. If no caller depends on the instruction
   cap, the dispatcher could drop it and simplify — but that is a contract
   change to `arm_step`, so it needs the byte-identical gate to say so.
