# Simulation Test Format

csim uses JSON configuration files to define simulations and automated tests. This document describes the full config schema, the test engine behavior, and the `csc2json.py` converter for importing Cooja `.csc` files.

## Config File Structure

```json
{
    "title": "My test",
    "timeout_ms": 60000,
    "seed": 1,
    "startup_delay_ms": 1000,
    "speed": 10.0,
    "radiomedium": { ... },
    "nodes": [ ... ],
    "test": { ... }
}
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

```json
"radiomedium": {
    "type": "udgm",
    "tx_range": 50.0,
    "interference_range": 100.0,
    "success_ratio_tx": 1.0,
    "success_ratio_rx": 1.0
}
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

```json
"nodes": [
    { "firmware": "firmware/cc2538dk/udp-server.cc2538dk", "id": 1, "x": 0.0, "y": 0.0 },
    { "firmware": "firmware/cc2538dk/udp-client.cc2538dk", "id": 2, "x": 30.0, "y": 30.0 }
]
```

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `firmware` | string | **required** | Path to firmware ELF file |
| `id` | int | auto (index+1) | Node ID, used for MAC address and test step filtering |
| `x`, `y` | float | 0, 0 | Position in meters (for radio medium) |

Node type is auto-detected from the firmware file extension:

| Extension | Platform |
|-----------|----------|
| `.sky` | MSP430 (Tmote Sky + CC2420 radio) |
| `.cc2538dk` | ARM Cortex-M3 (CC2538 + on-chip 802.15.4 radio) |
| `.cooja` | Native Cooja mote (via dlopen) |

## Test Section

The `test` section defines automated pass/fail criteria. A simulation with a `test` section exits with code 0 on pass, 1 on failure.

```json
"test": {
    "steps": [ ... ],
    "fail_on": [ ... ],
    "timeout_is_success": false,
    "actions": [ ... ]
}
```

### Test Steps

Steps are evaluated **sequentially**. The test engine waits for step 0 to match before checking step 1, and so on. When all steps match, the test passes.

```json
"steps": [
    { "wait": "Hello, world" },
    { "wait": "Data received from", "node": 1, "count": 3, "timeout_ms": 600000 }
]
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

```json
"fail_on": ["FAILED", "assertion failed", "stack overflow"]
```

An array of substrings. If **any** console output line from **any** node contains one of these patterns, the test **immediately fails** — regardless of step progress.

`fail_on` is checked **before** step matching on every line. This is useful for catching error conditions like test framework failures or runtime assertions.

### Timeout Behavior

The test engine has two timeout modes:

**Default (`timeout_is_success: false`):** If the simulation reaches `timeout_ms` before all steps complete, the test **fails**.

**Timeout-is-success (`timeout_is_success: true`):** If the simulation reaches `timeout_ms` without hitting a `fail_on` pattern, the test **passes**. This is for tests that verify the *absence* of errors over a time period (e.g., "run RPL for 10 minutes with zero packet loss").

```json
"test": {
    "timeout_is_success": true,
    "fail_on": ["packet loss", "parent switch: -> (NULL"]
}
```

**No-steps tests:** If `steps` is empty (or absent) and `timeout_is_success` is not set, the test passes if no `fail_on` pattern is hit during the simulation. This is equivalent to a `fail_on`-only test.

### Timed Actions

Actions execute at specific simulation times, enabling dynamic scenarios like topology changes or serial input.

```json
"actions": [
    { "at_ms": 60000,  "type": "move", "node": 4, "x": 58.0, "y": 108.0 },
    { "at_ms": 120000, "type": "send", "node": 1, "data": "rpl-set-root 1\n" }
]
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

Actions must be ordered by `at_ms`. They are executed in order as the simulation clock advances past each action's timestamp.

## Test Engine Evaluation Order

On every line of console output from any node:

1. Check all `fail_on` patterns. If any match → **FAIL** immediately.
2. Check current step's `wait` pattern (with optional `node` filter). If match → increment match count.
3. If match count reaches step's `count` → advance to next step.
4. If all steps completed → **PASS**.

On each simulation time step:

5. Execute any pending actions whose `at_ms` has been reached.
6. Check per-step timeout. If exceeded → **FAIL**.

On simulation end (global `timeout_ms` reached):

7. If `timeout_is_success` → **PASS**.
8. If no steps defined and no `fail_on` hit → **PASS**.
9. Otherwise → **FAIL** (incomplete steps).

## Examples

### Simple message check

```json
{
    "timeout_ms": 5000,
    "nodes": [
        { "firmware": "firmware/cc2538dk/hello-world.cc2538dk", "id": 1 }
    ],
    "test": {
        "steps": [
            { "wait": "Hello, world" }
        ]
    }
}
```

Passes when node 1 prints "Hello, world". Fails if 5 seconds elapse without the message.

### Multi-node broadcast

```json
{
    "timeout_ms": 60000,
    "radiomedium": { "type": "udgm", "tx_range": 50.0 },
    "nodes": [
        { "firmware": "firmware/cc2538dk/nullnet-broadcast.cc2538dk", "id": 1, "x": 0, "y": 0 },
        { "firmware": "firmware/cc2538dk/nullnet-broadcast.cc2538dk", "id": 2, "x": 25, "y": 0 }
    ],
    "test": {
        "steps": [
            { "wait": "Received", "node": 1 },
            { "wait": "Received", "node": 2 }
        ]
    }
}
```

Passes when both nodes have received a broadcast from the other.

### RPL convergence (timeout-is-success)

```json
{
    "timeout_ms": 1000000,
    "seed": 1,
    "startup_delay_ms": 1000,
    "radiomedium": { "type": "udgm", "tx_range": 50.0, "interference_range": 50.0 },
    "nodes": [
        { "firmware": "firmware/cc2538dk/root-node.cc2538dk",     "id": 3, "x": 0, "y": 0 },
        { "firmware": "firmware/cc2538dk/sender-node.cc2538dk",   "id": 2, "x": 130, "y": 146 },
        { "firmware": "firmware/cc2538dk/receiver-node.cc2538dk", "id": 1, "x": 7, "y": -26 }
    ],
    "test": {
        "timeout_is_success": true,
        "fail_on": ["packet loss"]
    }
}
```

Runs for 1000 seconds of simulated time. Passes if no "packet loss" message appears.

### Data structure self-test (fail-on only)

```json
{
    "timeout_ms": 10000,
    "nodes": [
        { "firmware": "firmware/cc2538dk/test-ringbufindex.cc2538dk", "id": 1 }
    ],
    "test": {
        "fail_on": ["FAILED"],
        "steps": [
            { "wait": "DONE" }
        ]
    }
}
```

Fails immediately if any output contains "FAILED". Passes when "DONE" appears.

### Dynamic topology with timed actions

```json
{
    "timeout_ms": 300000,
    "radiomedium": { "type": "udgm", "tx_range": 50.0 },
    "nodes": [
        { "firmware": "firmware/cc2538dk/root-node.cc2538dk", "id": 1, "x": 0, "y": 0 },
        { "firmware": "firmware/cc2538dk/node.cc2538dk",      "id": 2, "x": 25, "y": 0 },
        { "firmware": "firmware/cc2538dk/node.cc2538dk",      "id": 3, "x": 50, "y": 0 }
    ],
    "test": {
        "actions": [
            { "at_ms": 120000, "type": "move", "node": 3, "x": 200, "y": 0 },
            { "at_ms": 240000, "type": "move", "node": 3, "x": 50,  "y": 0 }
        ],
        "steps": [
            { "wait": "route restored", "node": 3 }
        ]
    }
}
```

Moves node 3 out of range at 120s, back in range at 240s, and waits for it to re-establish its route.

## Converting Cooja `.csc` Files

The `tools/csc2json.py` script converts Cooja simulation files to csim JSON format.

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
| `TIMEOUT(ms, if(cond) testOK())` | `timeout_is_success: true` |
| `WAIT_UNTIL(id == N && msg.contains("X"))` | `{"wait": "X", "node": N}` |
| `WAIT_UNTIL(msg.contains("X"))` | `{"wait": "X"}` |
| `if(msg.contains("X")) testFailed()` | `"fail_on": ["X"]` |
| `if(msg.contains("DONE")) break; testOK()` | `{"wait": "DONE"}` |
| `GENERATE_MSG` + `setCoordinates` | `actions` with `type: "move"` |
| `write(sim.getMoteWithID(N), "cmd")` | `actions` with `type: "send"` |

### Unsupported Features

Some Cooja test scripts use features that csim doesn't support. The converter flags these in an `unsupported_features` array in the output JSON:

- `removeMote` / `addMote` — dynamic node creation/removal (node reboot tests)
- `generateMote` — creating new mote instances at runtime
- `java.util.Random` — Java random number generator in script logic

Tests with these features may still work if the unsupported code paths aren't critical to the test outcome (e.g., commented-out removeMote calls).

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
| Actions | 64 |
| Pattern length | 255 chars |
