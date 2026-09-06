# Simulation Configuration and Test Format

Cooja-NG simulations are described by a config file — **YAML** (`.yaml` /
`.yml`, the primary format) or JSON (`.json`, still fully supported; every
existing JSON config keeps working unchanged). One file defines the nodes,
the radio medium and, optionally, an automated test. This document describes
the schema, the strictness rules, the test engine, saving a running setup, and
the `csc2json.py` converter for importing Cooja `.csc` files.

The two formats are the same schema: JSON is a subset of YAML, and both go
through one validator and one parser. What YAML adds is what people actually
want in a config: **comments**, **literal blocks** for the test script
(`js_script_inline: |` — the script verbatim, no escaping), and readable
one-line node entries in flow style. Examples in the tree:
`configs/chain-4node-sky.yaml`, `configs/test-js-hello.yaml`,
`configs/test-rpl-udp-sky.yaml`, `configs/medium-plugin-gilbert-elliott.yaml`
— each a twin of a `.json` next to it, and CI proves the pairs simulate
byte-identically.

## Strictness: a config that loads is a config that was understood

The loader rejects — with a `file:line:column` message — anything it would
otherwise have to guess about. There is no key it ignores silently:

| the file contains | what happens |
|---|---|
| a key the schema does not know (`tx_rang:`) | error naming the key and where (`config.medium`) |
| a key of the wrong type (`timeout_ms: "60000"`) | error: must be a number, got a string |
| a duplicate key (JSON or YAML) | error |
| a `v1` key in a `v2` file or vice versa (`medium` vs `radiomedium`, `plugins` without `version: 2`) | error, with the hint |
| YAML anchors/aliases (`&x` / `*x`), explicit tags (`!!str`), a second document (`---`) | error |
| YAML 1.1 booleans — `yes`, `no`, `on`, `off`, `Yes`, `TRUE`, `~`-less `Null` … | error: write `true`/`false`/`null` in lowercase, or quote it to mean the string. (This is the "Norway problem": `country: NO` must never read as `false`.) |
| `.inf`, `.nan`, hex/octal (`0x1f`), `1_000` | error: plain decimal only |

Plain scalars follow YAML 1.2 core typing: `true`/`false`, `null`/`~`,
integers and decimals are typed; everything else is a string. Anything
quoted (`"007"`, `'true'`) or in a block scalar is always a string. The only
keys the runtime does not read are the ones `csc2json.py` writes for the
firmware build tooling (`nodes[].build`, `nodes[]._mote_type_desc`,
`mote_types[].description`) and free-text `description`/`note` fields.

`test_runner config-reject test/configs/invalid/*` runs the fixtures that
each contain exactly one forbidden thing; CI requires all of them to fail.

## Saving a running setup

```sh
./build/test_runner test configs/chain-4node-sky.yaml --save-config my-setup.yaml
```

`--save-config` writes, at the end of the run, the configuration that was
**actually running** — not the file that was loaded: node positions come from
the radio medium (so a node moved by a test action is saved where it ended
up), the node list is the live one (nodes the script added, minus the ones it
removed), and the duration and seed are the effective ones (`-t` and `--seed`
win over the file). The file starts with a comment saying where and when it
was saved, and the loader re-reads it before reporting success. The saved
file plus its seed reproduces the run.

It works without a config too: `test_runner mixed-multinode a.sky b.sky
--save-config setup.yaml` turns a command line into a config file.

## Config tooling

```sh
./build/test_runner config-convert   old.json new.yaml     # canonical v2 YAML, header says where it came from
./build/test_runner config-roundtrip configs/*.json configs/*.yaml  # load -> YAML -> load must be lossless + idempotent
./build/test_runner config-reject    test/configs/invalid/*         # every file must be refused
```

The writer always emits **v2**: a v1 config is lifted (mote types are
synthesized from the distinct firmware paths, keeping the order of any
existing `mote_types` so `getMoteTypes()[i]` in a JS script still refers to
the same type).

## Config File Structure

```yaml
version: 2                 # omit for the legacy v1 layout (see below)
title: My test
timeout_ms: 60000
seed: 1
startup_delay_ms: 1000
speed: 10.0
medium: { ... }            # v1: radiomedium
mote_types: [ ... ]        # v2 only: named types the nodes reference
nodes: [ ... ]
test: { ... }
```

### Top-Level Fields

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `title` | string | — | Display name for the simulation |
| `timeout_ms` | int | 20000 | Total simulation duration in milliseconds |
| `seed` | int | 0 | Random seed (0 = not set) |
| `startup_delay_ms` | int | 0 | Each node gets a random startup delay in `[0, startup_delay_ms)` |
| `speed` | float | 0 | Real-time speed multiplier for UI mode (0 = default 10x) |

### Radio Medium

```yaml
medium:                    # v1: radiomedium
  type: udgm
  tx_range: 50.0           # metres
  interference_range: 100.0
  success_ratio_tx: 1.0
  success_ratio_rx: 1.0
```

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `type` | string | — | `"udgm"` for Unit Disk Graph Medium |
| `tx_range` | float | 50.0 | Transmission range in meters |
| `interference_range` | float | 100.0 | Interference range in meters |
| `success_ratio_tx` | float | 1.0 | TX success probability (0.0–1.0) |
| `success_ratio_rx` | float | 1.0 | RX success probability (0.0–1.0) |

Two nodes can communicate if their distance is within `tx_range`. Frames from nodes within `interference_range` (but beyond `tx_range`) cause collisions.

### Nodes

```yaml
# v2: nodes reference a named mote type
mote_types:
  - { name: server, firmware: firmware/cc2538dk/udp-server.cc2538dk }
  - { name: client, firmware: firmware/cc2538dk/udp-client.cc2538dk }
nodes:
  - { type: server, id: 1, x: 0.0, y: 0.0 }
  - { type: client, id: 2, x: 30.0, y: 30.0 }

# v1: nodes carry the firmware path directly
nodes:
  - { firmware: firmware/cc2538dk/udp-server.cc2538dk, id: 1, x: 0.0, y: 0.0 }
  - { firmware: firmware/cc2538dk/udp-client.cc2538dk, id: 2, x: 30.0, y: 30.0 }
```

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `firmware` | string | **required** | Path to firmware ELF file |
| `id` | int | auto (index+1) | Node ID, used for MAC address and test step filtering |
| `x`, `y` | float | 0, 0 | Position in meters (for radio medium) |
| `clock_deviation` | float | 1.0 | Clock speed factor (Cooja MspClock deviation) |
| `peripherals` | list | board defaults | Off-SoC SPI chips on this node, see below |

`peripherals` names the SPI chips hanging off the node's SPI masters.
Leave it out to get the board's own set (nRF54L15-DK: the on-board
MX25R6435F flash on SPIM00 with CS P2.05, and an ENC28J60 on SPIM22 with
CS P1.12); give a list — even an empty one — to replace that set.  Only
the nRF54L15 boards act on it today; other boards print a note and ignore
it.

```yaml
nodes:
  - firmware: firmware/nrf54l15-dk/spi-flash.nrf54l15-dk
    id: 1
    peripherals:
      - { chip: mx25r6435f, spim: 0,  cs: P2.05 }   # SPIM00
      - { chip: enc28j60,   spim: 22, cs: P1.12 }   # SPIM22
```

| Field | Type | Description |
|-------|------|-------------|
| `chip` | string | `mx25r6435f` (64 Mbit NOR flash) or `enc28j60` (Ethernet controller) |
| `spim` | int | SPIM instance id as in the peripheral name: `0` = SPIM00, `22` = SPIM22, `30` = SPIM30 |
| `cs` | string | Chip-select GPIO as the firmware prints it, `P<port>.<pin>` (the driver toggles it as a plain GPIO) |

Node type is auto-detected from the firmware file extension:

| Extension | Platform |
|-----------|----------|
| `.sky` | MSP430 (Tmote Sky + CC2420 radio) |
| `.cc2538dk` | ARM Cortex-M3 (CC2538 + on-chip 802.15.4 radio) |
| `.cooja` | Native Cooja mote (via dlopen) |

## Test Section

The `test` section defines automated pass/fail criteria. A simulation with a `test` section exits with code 0 on pass, 1 on failure.

```yaml
test:
  steps: [ ... ]
  fail_on: [ ... ]
  timeout_is_success: false
  actions: [ ... ]
```

### Test Steps

Steps are evaluated **sequentially**. The test engine waits for step 0 to match before checking step 1, and so on. When all steps match, the test passes.

```yaml
steps:
  - { wait: "Hello, world" }
  - { wait: "Data received from", node: 1, count: 3, timeout_ms: 600000 }
```

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `wait` | string | **required** | Substring to match in node console output |
| `node` | int | -1 (any) | Only match output from this node ID |
| `count` | int | 1 | Number of matches required before advancing |
| `timeout_ms` | int | 0 | Per-step timeout (0 = use global `timeout_ms`) |

**Matching behavior:**
- Each line of console output (UART) is checked against the current step's `wait` pattern using substring match (`strstr`).
- If `node` is set, only output from that node ID is considered.
- The `count` field requires the pattern to match N times before the step passes. This is useful for waiting for repeated events (e.g., "wait for 3 data packets").
- When a step matches, the engine advances to the next step. When all steps are done, the test passes.

### Fail-On Patterns

```yaml
fail_on: ["FAILED", "assertion failed", "stack overflow"]
```

An array of substrings. If **any** console output line from **any** node contains one of these patterns, the test **immediately fails** — regardless of step progress.

`fail_on` is checked **before** step matching on every line. This is useful for catching error conditions like test framework failures or runtime assertions.

### Timeout Behavior

The test engine has two timeout modes:

**Default (`timeout_is_success: false`):** If the simulation reaches `timeout_ms` before all steps complete, the test **fails**.

**Timeout-is-success (`timeout_is_success: true`):** If the simulation reaches `timeout_ms` without hitting a `fail_on` pattern, the test **passes**. This is for tests that verify the *absence* of errors over a time period (e.g., "run RPL for 10 minutes with zero packet loss").

```yaml
test:
  timeout_is_success: true
  fail_on: ["packet loss", "parent switch: -> (NULL"]
```

**No-steps tests:** If `steps` is empty (or absent) and `timeout_is_success` is not set, the test passes if no `fail_on` pattern is hit during the simulation. This is equivalent to a `fail_on`-only test.

### Validators

Validators are pattern counters that run throughout the entire simulation and are checked at the end. Unlike steps (which are sequential and end the test early on completion), validators never affect test flow — they just count and are evaluated at timeout.

```yaml
test:
  timeout_is_success: true
  validators:
    - { pattern: Data, min_count: 16 }
    - { pattern: "fd00::", min_count: 4, node: 1 }
```

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `pattern` | string | **required** | Substring to match in console output (`strstr`) |
| `min_count` | int | 1 | Minimum matches required to pass |
| `node` | int | -1 (any) | Only count matches from this node ID |

**How they work:**

- Every console output line is checked against every validator's pattern (independently of steps).
- Each match increments that validator's counter.
- When the simulation ends and the test would otherwise pass (timeout_is_success, or all steps completed), all validators are checked.
- If **any** validator's count is below its `min_count`, the test **fails**.

**Validators vs. steps:** Steps model sequential events ("first see X, then see Y"). Validators model aggregate counts ("see X at least N times total"). Use validators when the Cooja script counts messages over the full run, e.g. `if(seenMsgs > 15) testOK()`.

**Output:**

```
Validator [PASS]: "Data" matched 23/16
Validator [FAIL]: "fd00::" matched 2/4 node=1
TEST FAILED: validator "fd00::" matched 2/4 times
```

### Timed Actions

Actions execute at specific simulation times, enabling dynamic scenarios like topology changes, serial input, and node reboot.

```yaml
actions:
  - { at_ms: 60000,  type: move, node: 4, x: 58.0, y: 108.0 }
  - { at_ms: 120000, type: send, node: 1, data: "rpl-set-root 1\n" }
  - { at_ms: 120000, type: send_all, data: "ip-addr\n" }
  - { at_ms: 180000, type: remove, node: 3 }
  - { at_ms: 200000, type: add, node: 3, mote_type: 0 }
```

#### Move Action

Repositions a node and recomputes radio neighbors.

| Field | Type | Description |
|-------|------|-------------|
| `at_ms` | int | Simulation time to execute (ms) |
| `type` | string | `"move"` |
| `node` | int | Node ID to move |
| `x`, `y` | float | New position in meters |

#### Send Action

Sends data to a node's UART RX (serial input). This is how you send shell commands to firmware that has a CLI.

| Field | Type | Description |
|-------|------|-------------|
| `at_ms` | int | Simulation time to execute (ms) |
| `type` | string | `"send"` |
| `node` | int | Node ID to send to |
| `data` | string | Data to send (include `\n` for newline) |

#### Send All Action

Sends data to all active nodes' UART RX. Useful for broadcast queries like `ip-addr`.

| Field | Type | Description |
|-------|------|-------------|
| `at_ms` | int | Simulation time to execute (ms) |
| `type` | string | `"send_all"` |
| `data` | string | Data to send to every active node |

#### Remove Action

Removes a node from the simulation (stops stepping it). The node can be re-added later with `add`.

| Field | Type | Description |
|-------|------|-------------|
| `at_ms` | int | Simulation time to execute (ms) |
| `type` | string | `"remove"` |
| `node` | int | Node ID to remove |

#### Add Action

Re-adds a previously removed node by rebooting it (re-runs firmware from reset). The node resumes at current simulation time, not from t=0.

| Field | Type | Description |
|-------|------|-------------|
| `at_ms` | int | Simulation time to execute (ms) |
| `type` | string | `"add"` |
| `node` | int | Node ID to reboot/add |

Actions must be ordered by `at_ms`. They are executed in order as the simulation clock advances past each action's timestamp.

## Test Engine Evaluation Order

On every line of console output from any node:

1. Check all `fail_on` patterns. If any match → **FAIL** immediately.
2. Update all validator counters (increment count for each matching validator).
3. Check current step's `wait` pattern (with optional `node` filter). If match → increment match count.
4. If match count reaches step's `count` → advance to next step.
5. If all steps completed → check validators → **PASS** or **FAIL**.

On each simulation time step:

6. Execute any pending actions whose `at_ms` has been reached.
7. Check per-step timeout. If exceeded → **FAIL**.

On simulation end (global `timeout_ms` reached):

8. If `timeout_is_success` or no steps defined → check validators → **PASS** or **FAIL**.
9. Otherwise → **FAIL** (incomplete steps).

Validator check: if any validator's count is below its `min_count`, the test fails even if steps/timeout would have passed.

## Examples

### Simple message check

```yaml
timeout_ms: 5000
nodes:
  - { firmware: firmware/cc2538dk/hello-world.cc2538dk, id: 1 }
test:
  steps:
    - { wait: "Hello, world" }
```

Passes when node 1 prints "Hello, world". Fails if 5 seconds elapse without the message.

### Multi-node broadcast

```yaml
timeout_ms: 60000
radiomedium:
  type: udgm
  tx_range: 50.0
nodes:
  - { firmware: firmware/cc2538dk/nullnet-broadcast.cc2538dk, id: 1, x: 0, y: 0 }
  - { firmware: firmware/cc2538dk/nullnet-broadcast.cc2538dk, id: 2, x: 25, y: 0 }
test:
  steps:
    - { wait: Received, node: 1 }
    - { wait: Received, node: 2 }
```

Passes when both nodes have received a broadcast from the other.

### RPL convergence (timeout-is-success)

```yaml
timeout_ms: 1000000
seed: 1
startup_delay_ms: 1000
radiomedium:
  type: udgm
  tx_range: 50.0
  interference_range: 50.0
nodes:
  - { firmware: firmware/cc2538dk/root-node.cc2538dk, id: 3, x: 0, y: 0 }
  - { firmware: firmware/cc2538dk/sender-node.cc2538dk, id: 2, x: 130, y: 146 }
  - { firmware: firmware/cc2538dk/receiver-node.cc2538dk, id: 1, x: 7, y: -26 }
test:
  timeout_is_success: true
  fail_on:
    - "packet loss"
```

Runs for 1000 seconds of simulated time. Passes if no "packet loss" message appears.

### Data structure self-test (fail-on only)

```yaml
timeout_ms: 10000
nodes:
  - { firmware: firmware/cc2538dk/test-ringbufindex.cc2538dk, id: 1 }
test:
  fail_on:
    - FAILED
  steps:
    - { wait: DONE }
```

Fails immediately if any output contains "FAILED". Passes when "DONE" appears.

### RPL data exchange validation (validators)

```yaml
timeout_ms: 2000000
radiomedium:
  type: udgm
  tx_range: 50.0
nodes:
  - { firmware: firmware/cooja/root-node.cooja, id: 3, x: 0, y: 0 }
  - { firmware: firmware/cooja/sender-node.cooja, id: 2, x: 30, y: 0 }
  - { firmware: firmware/cooja/receiver-node.cooja, id: 1, x: 7, y: -26 }
test:
  timeout_is_success: true
  validators:
    - { pattern: Data, min_count: 16 }
```

Runs for 2000 seconds. Passes only if at least 16 lines containing "Data" appear in console output. This validates that RPL converges and UDP data exchange actually happens, rather than just "no crash".

### Node reboot with serial query

```yaml
timeout_ms: 300000
nodes:
  - ...
test:
  timeout_is_success: true
  validators:
    - { pattern: "fd00::", min_count: 4 }
  actions:
    - { at_ms: 1000, type: send, node: 4, data: "rpl-set-root 1\n" }
    - { at_ms: 61000, type: send_all, data: "ip-addr\n" }
    - { at_ms: 63000, type: remove, node: 4 }
    - { at_ms: 64000, type: add, node: 4 }
    - { at_ms: 65000, type: send, node: 4, data: "rpl-set-root 1\n" }
    - { at_ms: 125000, type: send_all, data: "ip-addr\n" }
```

Sets node 4 as RPL root, queries all nodes for IP addresses, reboots the root, re-queries. Validates that at least 4 `fd00::` address lines appear (meaning nodes have RPL addresses).

### Dynamic topology with timed actions

```yaml
timeout_ms: 300000
radiomedium:
  type: udgm
  tx_range: 50.0
nodes:
  - { firmware: firmware/cc2538dk/root-node.cc2538dk, id: 1, x: 0, y: 0 }
  - { firmware: firmware/cc2538dk/node.cc2538dk, id: 2, x: 25, y: 0 }
  - { firmware: firmware/cc2538dk/node.cc2538dk, id: 3, x: 50, y: 0 }
test:
  actions:
    - { at_ms: 120000, type: move, node: 3, x: 200, y: 0 }
    - { at_ms: 240000, type: move, node: 3, x: 50, y: 0 }
  steps:
    - { wait: "route restored", node: 3 }
```

Moves node 3 out of range at 120s, back in range at 240s, and waits for it to re-establish its route.

## Converting Cooja `.csc` Files

The `tools/csc2json.py` script converts Cooja simulation files to Cooja-NG's JSON config format (which the loader takes as-is; `test_runner config-convert` turns it into YAML).

### Usage

```sh
# Convert a .csc file to JSON
python3 tools/csc2json.py test.csc --firmware-dir firmware/cc2538dk -o out.json

# List firmware files needed by a .csc file
python3 tools/csc2json.py test.csc --list-firmware

# Show conversion warnings
python3 tools/csc2json.py test.csc --firmware-dir firmware/cc2538dk --warn
```

### What Gets Converted

| Cooja JS | csim JSON |
|----------|-----------|
| `TIMEOUT(ms)` | `timeout_ms` |
| `TIMEOUT(ms, if(cond) testOK())` | `timeout_is_success: true` + `validators` |
| `TIMEOUT(ms, if(seenMsgs > 15) testOK())` | `validators: [{"pattern": "Data", "min_count": 16}]` |
| `TIMEOUT(ms, if(lastMsg != -1) testOK())` | `validators: [{"pattern": "Data", "min_count": 1}]` |
| `WAIT_UNTIL(id == N && msg.contains("X"))` | `{"wait": "X", "node": N}` |
| `WAIT_UNTIL(msg.contains("X"))` | `{"wait": "X"}` |
| `if(msg.contains("X")) testFailed()` | `"fail_on": ["X"]` |
| `if(msg.contains("DONE")) break; testOK()` | `{"wait": "DONE"}` |
| `GENERATE_MSG` + `setCoordinates` | `actions` with `type: "move"` |
| `GENERATE_MSG` + `removeMote/addMote` | `actions` with `type: "remove"/"add"` |
| `write(sim.getMoteWithID(N), "cmd")` | `actions` with `type: "send"` |
| `write(motes[i], "cmd")` (broadcast) | `actions` with `type: "send_all"` |

### Unsupported Features

Some Cooja test scripts use features that can't be fully auto-converted. The converter flags these in an `unsupported_features` array in the output JSON:

- `generateMote` — creating new mote instances at runtime (without a matching addMote)
- `java.util.Random` — Java random number generator in script logic

Complex validation logic (e.g., collecting IP addresses and cross-checking per-node) requires manual validators in the JSON config.

## Running Tests

```sh
# Single test
./build/test_runner mixed-multinode configs/my-test.json -v

# All Cooja tests (requires firmware to be built)
./tools/run-cooja-tests.sh /path/to/contiki-ng

# Specific test directory
./tools/run-cooja-tests.sh /path/to/contiki-ng "14-rpl-lite"

# Build firmware for tests
./tools/build-test-firmware.sh /path/to/contiki-ng
```

### Test Runner Options

| Option | Description |
|--------|-------------|
| `-v` | Verbose: show all console output |
| `-q` | Quiet: suppress progress output |
| `-t ms` | Override simulation duration |
| `-n count` | Override node count |

### Limits

| Resource | Limit |
|----------|-------|
| Nodes | 128 |
| Test steps | 32 |
| Fail-on patterns | 8 |
| Validators | 8 |
| Actions | 64 |
| Pattern length | 255 chars |
