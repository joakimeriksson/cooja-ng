# Running the Contiki-NG Cooja tests with Cooja-NG

Cooja-NG runs Contiki-NG's `tests/*/*.csc` simulation tests headlessly, from a
single static binary, without a JVM. This is the user guide; the roadmap for
making it Contiki-NG's CI tool is in
[`design/contiki-ng-testing-plan.md`](design/contiki-ng-testing-plan.md).

Everything here assumes a Contiki-NG checkout (5.2 or `develop`) and the
embedded toolchains its firmware builds need (`msp430-gcc`, the native host
compiler, `arm-none-eabi-gcc` for ARM tests). Cooja-NG itself needs only
`python3` at run time.

## 1. Get Cooja-NG

Either the prebuilt release — no build step:

```sh
curl -fsSL https://github.com/joakimeriksson/cooja-ng/releases/download/v0.2.2/cooja-ng-v0.2.2-linux-x86_64.tar.gz | tar xz
cd cooja-ng-v0.2.2-linux-x86_64          # macos-x64 / macos-arm64 tarballs exist too
```

or from source (`git clone … && make`, GNU Lightning auto-detected). Both give
the same layout: `build/test_runner` plus `tools/run-cooja-tests.sh`,
`tools/csc2json.py`, `tools/build-test-firmware.sh`, `tools/test-border-router.sh`.

## 2. Run tests

`tools/run-cooja-tests.sh` is the entry point. It takes the Contiki-NG tree
from `CONTIKI_DIR` (or `csim.conf`, or `../contiki-ng`) and can be invoked
from any working directory — it runs from its own tree, so a `make -C
tests/<category>` in Contiki-NG or a CI step needs no `cd`.

```sh
export CONTIKI_DIR=/path/to/contiki-ng

tools/run-cooja-tests.sh                                  # all non-TUN tests (85 in 5.2)
tools/run-cooja-tests.sh --with-tun                       # all 93, incl. border-router (needs TUN: sudo or setcap on tunslip6)
tools/run-cooja-tests.sh 14-rpl-lite                      # one category
tools/run-cooja-tests.sh 07-simulation-base/01-cooja-hello-world   # one test
tools/run-cooja-tests.sh 14-rpl-lite -v                   # show each test's output
```

For each `.csc` the runner: converts it to Cooja-NG's native JSON config
(`csc2json.py`), builds any firmware the test needs from the Contiki-NG tree
(cached under `firmware/`; `--no-build` to skip, `--clean` to force), runs the
simulation with the test's JavaScript attached, and prints `PASS`/`FAIL` per
test plus a summary.

## 3. Seeds

```sh
tools/run-cooja-tests.sh 14-rpl-lite --seed 3
```

`--seed N` is Cooja's `--random-seed`: it overrides the `randomseed` in every
`.csc` for that run. The same seed gives a byte-identical simulation
(determinism is a gated guarantee), and different seeds change every random
decision — radio loss when the medium's success ratios are below 1.0, the
per-node startup-delay spread, and anything a test script draws from
`sim.getRandomSeed()` / `java.util.Random`. Seeds are positive integers; `0`
is rejected rather than silently meaning "the config's own".

This is what makes Contiki-NG's `BASESEED`/`RUNCOUNT` loop faithful (§5).

## 4. Logs

```sh
tools/run-cooja-tests.sh 07-simulation-base --logdir $CONTIKI_DIR/tests
```

`--logdir DIR` writes `DIR/<category>/<csc-name>.testlog` per test — the
same layout as Contiki-NG's `tests/` tree, so pointing it at that tree puts
every log beside its `.csc` (their `.gitignore` already covers `*.testlog`).
The category level is not cosmetic: the same `.csc` basename exists in
several categories (`07-rpl-random-rearrangement` in both `14-rpl-lite` and
`15-rpl-classic`), and a flat directory would silently overwrite.

Each `.testlog` is the full runner output: every mote's console lines, the
test script's `log.log()` output (`[JS] …`), the verdict, and the run
statistics. A test that could not even run — conversion error, firmware
build failure — gets a `.testlog` with the error, so a red test never leaves
nothing behind.

## 5. Exit codes: the fail-loudly contract

The runner exits non-zero if **anything** is wrong, and prints why. There is
no result that is green by accident:

| situation | what happens |
|---|---|
| a test's script calls `testFailed()` or its timeout expires | `FAIL`, exit 1 |
| the `.csc` uses a radio medium, plugin, mote type, mote interface or element Cooja-NG does not implement | `ERROR (conversion failed)` naming the feature, exit 1 |
| a firmware build fails | `ERROR (firmware build failed)`, exit 1 |
| a converted test has no assertions | `ERROR (no test criteria)`, exit 1 |
| a script ends without calling `testOK()`/`testFailed()` | `FAIL: script ended without a verdict`, exit 1 |
| the test pattern matches no `.csc` at all (typo, wrong `CONTIKI_DIR`, renamed test) | error, exit 2 — never "0 tests, all OK" |
| TUN tests without `--with-tun`; missing firmware with `--no-build` | `SKIP` — the only two, and both are explicit opt-outs |

When a new upstream `.csc` uses a feature Cooja-NG lacks, the fix is to add
real support or port that test to Cooja-NG's native JSON config — never to
loosen the check. `csc2json.py --lax` exists for local exploration and must not
be used in CI.

## 6. From inside a Contiki-NG tree: `make -C tests/… SIMULATOR=cooja-ng`

Contiki-NG developers run tests as `make -C tests/07-simulation-base`, which
includes `tests/Makefile.simulation-test` — a `tests` target that loops
`BASESEED..BASESEED+RUNCOUNT`, invokes Java Cooja per `.csc`, appends
`TEST FAIL: <csc> seed N` to a `summary` file, and a `summary` target that
prints "All tests OK" or the failures and sets the exit code.

The change below adds a `SIMULATOR=cooja-ng` switch to that file: the
variables and fetch rule go after `GRADLE ?= …`, and the existing `tests:`
rule is wrapped in `ifeq/else/endif` (wrapping, not appending — a second
`tests:` rule would make GNU make warn "overriding recipe" on every run).
`summary`, the seed loop, "All tests OK" and the exit code are untouched, so
nothing downstream — developer habit, CI log parsing — changes. The default
stays `cooja`. *(This is the Contiki-NG-side change; it lives in the Contiki-NG
tree, not here. It was validated against Contiki-NG 5.2's
`Makefile.simulation-test` with `RUNCOUNT=2`.)*

```make
# ---- Cooja-NG switch --------------------------------------------------------
# make -C tests/<category> SIMULATOR=cooja-ng   (default: cooja = Java Cooja)
SIMULATOR        ?= cooja
COOJA_NG_VERSION ?= v0.2.2
COOJA_NG         ?= $(CONTIKI)/tools/cooja-ng
COOJA_NG_RUN      = $(COOJA_NG)/tools/run-cooja-tests.sh
COOJA_NG_URL      = https://github.com/joakimeriksson/cooja-ng/releases/download/$(COOJA_NG_VERSION)

UNAME_S := $(shell uname -s)
UNAME_M := $(shell uname -m)
ifeq ($(UNAME_S),Darwin)
  COOJA_NG_PLATFORM ?= macos-$(if $(filter arm64,$(UNAME_M)),arm64,x64)
else
  COOJA_NG_PLATFORM ?= linux-x86_64
endif
COOJA_NG_PKG = cooja-ng-$(COOJA_NG_VERSION)-$(COOJA_NG_PLATFORM)

# Download the pinned release once, checksum-verified. No compile step.
$(COOJA_NG_RUN):
	@mkdir -p $(dir $(COOJA_NG)) && cd $(dir $(COOJA_NG)) && \
	  curl -fsSL -O $(COOJA_NG_URL)/$(COOJA_NG_PKG).tar.gz && \
	  curl -fsSL -O $(COOJA_NG_URL)/SHA256SUMS && \
	  (sha256sum --ignore-missing -c SHA256SUMS 2>/dev/null || shasum -a 256 --ignore-missing -c SHA256SUMS) && \
	  tar xzf $(COOJA_NG_PKG).tar.gz && rm -rf $(COOJA_NG) && mv $(COOJA_NG_PKG) $(COOJA_NG) && \
	  rm -f $(COOJA_NG_PKG).tar.gz SHA256SUMS

.PHONY: cooja-ng-fetch
cooja-ng-fetch: $(COOJA_NG_RUN)
# -----------------------------------------------------------------------------

# Replace the existing `tests: $(TESTS)` rule with this block — the
# `else` branch IS the existing Java Cooja recipe, unchanged.
ifeq ($(SIMULATOR),cooja-ng)
# One run per .csc per seed; a non-zero exit (FAIL or ERROR — Cooja-NG never
# skips silently) appends to `summary`, exactly like the Java Cooja branch.
tests: $(TESTS) | $(COOJA_NG_RUN)
	@for (( SEED=$(BASESEED); SEED < $$(( $(BASESEED) + $(RUNCOUNT) )); SEED++ )); do \
	  for t in $^; do \
	    CONTIKI_DIR=$(realpath $(CONTIKI)) $(COOJA_NG_RUN) \
	      $(notdir $(CURDIR))/$${t%.csc} --seed $$SEED --logdir $(realpath $(CURDIR)/..) >/dev/null || \
	      echo "TEST FAIL: $$t seed $$SEED" >> summary; \
	  done; \
	done
else
tests: $(TESTS)
	@for (( SEED=$(BASESEED); SEED < $$(( $(BASESEED) + $(RUNCOUNT) )); SEED++ )); do \
          $(GRADLE) --no-watch-fs --parallel --build-cache -p $(CONTIKI)/tools/cooja run -Dslf4j.provider=ch.qos.logback.classic.spi.LogbackServiceProvider --args="--no-gui --contiki=$(realpath $(CONTIKI)) --logdir=$(dir $(realpath $<)) --random-seed=$$SEED $(realpath $^)" || \
            echo "TEST FAIL: $^ seed $$SEED" >> summary; \
         done
endif
```

Then, from the Contiki-NG tree:

```sh
make -C tests cooja-ng-fetch                              # once; pins COOJA_NG_VERSION
make -C tests/07-simulation-base SIMULATOR=cooja-ng       # same output shape as Java Cooja
make -C tests/14-rpl-lite SIMULATOR=cooja-ng RUNCOUNT=10  # ten seeds — cheap at 100–300x real-time
```

Notes: the runner's own `PASS`/`FAIL` lines go to `>/dev/null` here because
`summary` is the contract; drop the redirect to see them. `--logdir` is the
`tests/` root (the category's parent), so per-test logs land as
`<csc-name>.testlog` in the category directory, exactly where Java Cooja's do
and where the existing `clean` rule removes them.
`RUNCOUNT>1` is only meaningful because `--seed` exists — without it every
iteration would silently rerun the `.csc`'s own seed.

## 7. In CI (GitHub Actions)

The same command is the CI job. A minimal non-gating "shadow" job for a
Contiki-NG workflow, alongside the existing Java Cooja jobs:

```yaml
  cooja-ng:
    name: cooja-ng/${{ matrix.test }}
    runs-on: ubuntu-latest
    continue-on-error: true          # shadow mode while collecting divergence data
    strategy:
      fail-fast: false
      matrix:
        test: [simulation-base, ipv6, ieee802154, rpl-lite, rpl-classic, script-base, security-protocols]
    steps:
      - uses: actions/checkout@v4
      - run: make -C tests cooja-ng-fetch
      - run: make -C tests/??-${{ matrix.test }} SIMULATOR=cooja-ng
      - uses: actions/upload-artifact@v4
        if: always()
        with: { name: testlogs-${{ matrix.test }}, path: tests/??-${{ matrix.test }}/*.testlog }
```

Run it inside the same privileged container the Java `tun-rpl-br` job uses to
get the TUN tests as well (`--with-tun` in the invocation). Once it has run
green over enough PR traffic, drop `continue-on-error` and it is the gate.

## 8. Troubleshooting

- **`ERROR … (conversion failed): unsupported radio medium class …`** — the
  test needs something Cooja-NG does not implement. Add support, or port the
  test to a native JSON config ([`test-format.md`](test-format.md)). Do not
  use `--lax` in CI.
- **`ERROR … (firmware build failed)`** — the Contiki-NG build for that
  target failed; run `tools/build-test-firmware.sh --from-json …` or the
  test's `make` line by hand to see the compiler output.
- **TUN tests skipped** — pass `--with-tun`; `tunslip6` needs root or
  `sudo setcap cap_net_admin+eip tools/serial-io/tunslip6` once.
- **Two runs differ** — they should not with the same seed. That is a bug in
  Cooja-NG; report it with both `.testlog`s.
