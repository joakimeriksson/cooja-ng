# Cooja-NG as the Contiki-NG regression-test tool (post-5.2)

Status: **proposed** (2026-08-17). Written against Contiki-NG `develop`
at `v5.2-4-gd5610d807` and csim `main` after the ARM-performance (PR #9),
TrustZone (PR #8) and cc2538-clock (PR #7) merges.

## 1. Where we already are — the starting position is strong

The measured facts this plan builds on, none of them aspirational:

- **Contiki-NG 5.2's entire Cooja simulation suite is 93 `.csc` tests, and
  csim passes 93/93** — 0 failed, 0 skipped, including all eight
  `17-tun-rpl-br` border-router tests under `--with-tun`
  (`tools/run-cooja-tests.sh`, verified repeatedly on x86-64/Linux as part of
  the standing jftest4 gate). Every test uses UDGM, which csim implements
  with Cooja's semantics (including TX-power-scaled range).
- The suite runs from **one static C binary** — no JVM, no gradle, no Docker
  requirement of its own. Contiki-NG's current CI spends per-job effort
  building/caching Java Cooja before the first simulated second.
- csim is **deterministic by gate**: two runs are byte-identical, JIT on or
  off, and that property is CI-enforced (`check-determinism.sh` + the JIT
  cycle-exactness gates).
- Speed: the 85 non-TUN tests complete in a few minutes wall (the TUN tests
  are bounded by real external processes — tunslip6 + native border router —
  not by simulation speed). On the emulated-platform side, csim runs cc2538
  RPL-UDP at ~300x real-time vs Renode's 0.94x default / 4.5x fast-idle
  (`tools/renode-compare/run.sh`, plan §0.3b: **~70x faster than Renode's
  best configuration**).

What Contiki-NG's CI does today (`.github/workflows/build.yml`): a 17-way
category matrix on ubuntu + macos; the simulation categories
(`simulation-base`, `ipv6`, `ieee802154`, `rpl-lite`, `rpl-classic`,
`tun-rpl-br`, `script-base`, `security-protocols`) run Java Cooja headless
inside a privileged Docker container, 1–15 min per job (tun-rpl-br is the
15-min tail). `04-renode-simulation` runs exactly one workload —
`rpl-udp/cc2538dk` — under Renode.

### 1b. Direction: `.csc` is a bridge, not the destination

The native JSON config (v2) is the primary format; `csc2json.py` exists so
the transition period needs no hand-porting of 93 upstream tests. The
intended end state is that Contiki-NG's tests carry native csim configs and
the converter retires with Java Cooja. Until then the converter is a
correctness boundary and is held to the fail-loudly contract above — it may
refuse a `.csc`, it may never mistranslate one silently.

## 2. Goal and non-goals

**Goal:** after 5.2, Contiki-NG PR CI runs its simulation regression tests
through csim, with Java Cooja retired from the PR-gating path (kept available
for interactive use — this plan is about the *headless test runner* role).

**Non-goals:** replacing Cooja's GUI/interactive role; changing what the
tests assert; porting the compile-only or native-run categories (they don't
involve a simulator).

## 3. Plan

### Phase 0 — binary releases of cooja-ng (~1 day, prerequisite for Phase 1)

Contiki-NG CI should never compile cooja-ng. The `make` itself is under a
minute, but a source build imports our failure modes into their PR traffic —
toolchain matching, `pkg-config`, and the GNU Lightning source fetch from
`ftp.gnu.org`, an external dependency that will eventually flake in someone
else's PR and make cooja-ng look broken. The pitch upstream is "one static
binary, zero build system"; a download step keeps that literally true.

Two properties make binaries unusually clean here:

- **The JIT is cycle-exact by gate**, so a Lightning-enabled binary and an
  interpreter-only one produce byte-identical simulation output. Shipping
  Lightning statically linked costs nothing in reproducibility.
- The only runtime deps beyond libc are vendored (QuickJS, cJSON); `dlopen`
  is used only by the optional plugin system, which the Cooja suite never
  touches — so a mostly-static build (or fully static, plugins compiled
  out) covers the container case.

Release contents — a small tarball per platform, not a bare binary, because
the scripts are the interface: `test_runner` + `tools/run-cooja-tests.sh` +
`tools/csc2json.py` + `tools/build-test-firmware.sh`. Firmware compilation
deliberately stays on the contiki-ng side: that is *their* code under test.

Targets: `linux-x86_64` (their privileged Docker container), `macos-x64` and
`macos-arm64` (their simulation categories also run natively on macOS
runners). Built and checksummed by a cooja-ng release workflow — which is
also the mechanism behind "cut a pinned tag" below. Version pinning becomes
a release-asset URL bump via normal PR.

### Phase 1 — shadow job in Contiki-NG CI (non-gating, ~2–3 days)

Add one workflow job to `contiki-ng/.github/workflows/build.yml`:

1. Download the **pinned cooja-ng release tarball** (Phase 0) — no checkout,
   no compile. Source build stays documented as the fallback path only.
2. `CONTIKI_DIR=$GITHUB_WORKSPACE tools/run-cooja-tests.sh --with-tun`
   inside the same privileged container the existing jobs use (TUN already
   works there — that's how the Java tun-rpl-br job runs).
3. `continue-on-error: true` and a summary artifact. **Non-gating on
   purpose:** the point of this phase is to collect divergence data over
   weeks of real PR traffic, not to block anyone.

Deliverable is a small PR to contiki-ng plus an RFC issue explaining the
motivation (speed, determinism, no-JVM, emulated-platform coverage). Engage
the maintainers (nvt in particular) at this step, not after — buy-in is the
actual critical path of this whole plan, and the shadow job is designed to
be a zero-risk ask.

### Phase 2 — divergence triage + drift protection (rolling, 2–4 weeks of shadow traffic)

- Every shadow-vs-Java disagreement gets root-caused and recorded: emulator
  gap, converter gap, or test flakiness (Java Cooja tests already carry a
  "Failed simulations for no obvious reason, Cooja issue?" retry comment in
  upstream CI — csim's determinism should *reduce* this class, and each such
  case is an argument in our favor worth documenting).
- ~~**Harden `csc2json.py` to fail loudly**~~ **DONE** (pulled forward from
  Phase 2 — see the fail-loudly commit). The converter is strict by default:
  any radio-medium class, plugin, mote type, declared `moteinterface`,
  `interface_config`, or simulation-level element outside the allowlists
  inventoried from the 5.2 suite is a fatal `ConversionError` (`--lax`
  downgrades, exploration only). The suite runner fails the run on ANY
  error (conversion, firmware-build, test-without-assertions — the old
  silent SKIP/exit-0 paths), and the JS engine now treats a script that
  ends without a `testOK()`/`testFailed()` verdict as a failure. Verified:
  93/93 real files still convert and pass; four crafted mutants (MRM
  medium, PowerTracker plugin, unknown moteinterface, unknown sim element)
  each fail conversion and turn the suite red. The only accepted skips are
  the explicit opt-outs: TUN tests without `--with-tun`, missing firmware
  under `--no-build`.
- Add a **nightly csim-side job that runs the suite against contiki-ng
  `develop` HEAD** (jftest4 or a scheduled GitHub workflow), so we see
  upstream drift before their CI does. Track the delta between "tests in
  their tree" and "tests we pass" as a number that must stay 0.
- Seed semantics: upstream runs `BASESEED=1 RUNCOUNT=1`. Document what csim
  does with the csc random seed today and, if it ignores it, wire it through
  — multi-seed runs (`RUNCOUNT>1`) become nearly free at csim speeds and are
  a Phase-4 selling point.

### Phase 3 — flip to gating + replace the Renode category (~1 week incl. review)

- Promote the csim job to required for the simulation categories; move the
  Java Cooja simulation jobs to a nightly/weekly schedule for one release
  cycle as the reference implementation, then retire them from CI.
- **Replace `04-renode-simulation`** with csim running the same
  `rpl-udp/cc2538dk` firmware. Same ELF, same assertion, ~70x faster, and it
  exercises an emulator the project can actually debug (the Renode leg needed
  a boot-ROM, a SYS_CTRL model and IEEE addresses just to boot stock
  firmware — `tools/renode-compare/run.sh` documents all three gaps).
- Define the divergence policy in writing: during the transition Java Cooja
  remains the reference for MSP430-timing questions; after retirement, csim
  is the reference and carries the burden of proof via its own differential
  gates (arm-decode/arm-jit/lockstep/determinism).

### Phase 4 — test what Java Cooja never could (incremental, after the flip)

This is the payoff that justifies the migration beyond CI minutes:

- **Emulated modern platforms in PR CI**: cc2538, nRF52840, nRF54L15 configs
  — today Contiki-NG's ARM platforms are compile-tested only (plus the one
  Renode workload). csim's chain/RPL/TSCH configs make them
  behavior-tested.
- **TSCH timing tests** (~25 s wall for the nrf52840 association+sync test)
  — timing-sensitive coverage upstream CI has never had.
- **Multi-seed robustness runs** as a nightly: `RUNCOUNT=10` across the
  suite is affordable at 100–300x real-time.
- Longer term: Zephyr-interop tests, nRF54L15 dual-core (FLPR), TrustZone-M
  — already working in csim, all invisible to Java Cooja.
- `make pgo` build for the CI runner if suite wall time ever matters
  (measured 1.55x clang / 1.18x gcc).

## 4. Risks and open questions

| risk | mitigation |
|---|---|
| Upstream buy-in | Phase 1 is deliberately zero-risk (non-gating, no removal); RFC issue + maintainer engagement before any gating change |
| Two implementations of Cooja semantics drifting | converter fails loudly on unknown features; nightly run against develop HEAD; delta-must-be-0 tracking |
| Test-script JS engine parity (QuickJS vs Cooja's engine) | covered empirically by 93/93 today; any new script API use surfaces in the shadow job |
| csim bug masks a real regression | determinism + the differential gates make csim failures reproducible and debuggable — the opposite failure mode of the current "Cooja issue?" retries |
| cooja-ng release/versioning for CI pinning | Phase 0 binary releases; pin by release-asset URL; bump via normal PR |
| binary/runner ABI drift (glibc, macOS versions) | mostly-static Linux build; macOS binaries built on the oldest supported runner image; checksums in the release |
| TUN in GitHub runners | already solved — upstream's own tun-rpl-br job runs privileged Docker |

## 5. Effort summary

| phase | effort | gate |
|---|---|---|
| 0. binary release workflow | ~1 day | tagged release with linux + macos tarballs, checksummed; suite passes from the unpacked tarball on a clean machine |
| 1. shadow job + RFC | 2–3 days | job green in shadow, maintainers engaged |
| 2. triage + hardening | rolling, low | 0 unexplained divergences over N weeks |
| 3. gating flip + Renode replacement | ~1 week | upstream approval |
| 4. new coverage | incremental | each addition through normal PR review |
