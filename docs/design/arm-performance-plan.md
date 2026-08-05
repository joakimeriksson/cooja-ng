# ARM Interpreter Performance Plan

Status: **in progress** (2026-08-05). §1 `arm-bench`, §2 Tier 0, §3 Tier 1 (PGO)
and §4 (the debug-facility gate, which replaced the dropped decode cache) are
**done and measured** — **21.1% cumulative** on an ARM runner workload. Tier 3
(JIT) is open. Companion to
[`refactor-plan.md`](refactor-plan.md) (§8 D5 overlaps Tier 0 here).

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
| ~~Tier 0~~ **DONE** | — | — | **~5.9% measured** (4/5 runs below baseline min); full gate green |
| ~~Tier 1 PGO~~ **DONE** | — | — | **1.55× clang / 1.18× gcc on an untrained workload**; suites green on both |
| ~~Tier 2 decode cache~~ **DROPPED** | — | premise refuted (§4.1) | — |
| **Tier 2′ debug gate** **DONE** | ½ day | — | **14.6% measured**, non-overlapping; full gate green |
| ~~branch consolidation~~ **REVERTED** | — | 3 wins / 9 paired runs | not a real effect (§4.4) |
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
