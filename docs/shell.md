# The Cooja-NG shell and script engine

`--shell` gives a live simulation an interactive command line; `--script FILE`
runs the same commands from a file, with blocking waits and a pass/fail
verdict, so **one firmware with a Contiki-NG shell can serve many tests**: the
Cooja-NG shell types commands into the simulated node's console and asserts on
what the node prints.

```sh
./build/test_runner test configs/shell-nrf54l15-dk.yaml --shell            # interactive
./build/test_runner test configs/shell-nrf54l15-dk.yaml --shell --paused   # prompt first, `run` to start
./build/test_runner test configs/shell-nrf54l15-dk.yaml --script test/scripts/shell-nrf54l15.cnsh
printf 'sendln 1 help\nexpect 1 "Shows this help" 5s\nexit\n' \
    | ./build/test_runner test configs/shell-nrf54l15-dk.yaml --shell       # piped session
tools/check-shell.sh                                                       # smoke check
```

Every simulation mode that takes a config or firmware list (`test`,
`mixed-multinode`, the `*-multinode` wrappers) accepts the flags.

## Flags

| flag | meaning |
|---|---|
| `--shell` | read commands from stdin.  On a terminal: line editing, history (`~/.cooja-ng_history`, or `$CSIM_SHELL_HISTORY`; empty disables), tab completion of command names.  From a pipe: plain lines, each executed command echoed as `> cmd`, EOF = `exit`. |
| `--script FILE` | run FILE at simulation start, with or without `--shell`.  Without `--shell` the run ends when the script passes, fails or reaches its end. |
| `--paused` | start paused (needs `--shell`, `--script` or `--ui` to resume). |
| `--speed N` / `--speed max` / `--realtime` | wall-clock pacing: N simulated seconds per wall second; `max` = unpaced (the headless default; the live UI and the serial bridge default to 10x). |

With `--shell` the run has no duration: it ends at `exit`.  An explicit `-t`
**pauses** the simulation at that time instead of ending the run (`run`
continues), and the config's `timeout_ms` is ignored, with a note at start.
Without `--shell` (including `--script` alone) the duration ends the run as
before.

The default prompt shows the simulation time and state:
`cooja 12.345s> `, `cooja 12.345s [paused]> `, `cooja 12.345s [expect]> `.
Node console lines print between prompts in the same
`  12.345 [Node 1/ARM] text` format as a headless run.

## Commands

Node selectors: `1`, `1,3`, `2-5`, `all`, and `any` where a match is meant.
Times: `5s`, `250ms`, `1500us`, `1.5s`, `2m`; a bare number is milliseconds;
`+2s` is relative to now.  Words with spaces are quoted (`"..."` decodes
`\n \t \\ \" \xHH`, `'...'` is literal); `#` starts a comment.

**Simulation control**

| command | |
|---|---|
| `run [duration]` | resume; with a duration, pause again after it (holds the script until then) |
| `pause` | stop dispatching events (services and input keep running) |
| `step [N\|duration]` | run exactly N events (default 1) or a duration, then pause |
| `speed [ratio\|max\|realtime]` | wall-clock pacing; no argument prints it |
| `status`, `time`, `nodes` | state summary; simulation time; the node table |
| `exit`, `quit` | end the run: normal teardown, test reports, `--save-config` |

**Nodes**

| command | |
|---|---|
| `add <firmware\|type> [id] [x y]` | add a node from a config v2 mote-type name or a firmware path; id defaults to max+1; it boots and runs from now |
| `remove <nodes>` | stop nodes for good (`reboot` revives them) |
| `move <id> <x> <y>` | set a position (metres), neighbours recomputed |
| `reboot <nodes>` | destroy + re-initialize from the same firmware, clock re-seeded to now |

**Console**

| command | |
|---|---|
| `log [on\|off [nodes] \| only <nodes>]` | which nodes' console lines print here (default all; none under `-q`) |
| `log-file <path> [nodes]`, `log-file off [path]`, `log-file` | append nodes' console lines to a file (same line format, flushed per line); close; list |
| `send <nodes> <text...>` | console input, escapes honoured, no newline added |
| `sendln <nodes> <text...>` | `send` + one `\n`, i.e. one Contiki-NG shell command (the Contiki shell ends a line on `\n` *or* `\r`, so `\r\n` would be two commands) |

Input is delivered the way each platform's model paces it (nRF54L15: one
UARTE byte per character time; MSP430: baud-paced; unconsumed bytes are
retried automatically).

**Scheduling**

| command | |
|---|---|
| `at <time> <command...>` | run a command at an exact simulation time |
| `every <period> <command...>` | run a command periodically (first after one period) |
| `atq`, `atrm <id>\|all` | list / cancel scheduled commands |

**Scripting** (see below)

| command | |
|---|---|
| `source <file>` | run a script file (nested up to 8 deep) |
| `expect <nodes\|any> "<pattern>" [timeout]` | block until a console line contains the pattern (substring); the timeout (default 30 s, `set expect-timeout`) fails the script |
| `sleep <duration>`, `wait-until <time>` | block for a duration / until a time |
| `assert time <op> <t>`, `assert nodes <op> N`, `assert node <id> active\|removed\|exists`, `assert count "<pat>" <op> N` | checks (`== != < <= > >=`); a false assert fails the script |
| `pass`, `fail [message]` | end the script with a verdict |
| `fail-on "<pattern>" [nodes\|any]` | fail as soon as a console line contains the pattern |
| `count "<pattern>" [nodes\|any]` | count matching lines from now on, for `assert count` |
| `on <nodes\|any> "<pattern>" <command...>` | run a command whenever a line matches (e.g. `on any "SecureFault" fail "unexpected fault"`) |
| `set [expect-timeout <duration>]`, `echo`, `save-config <file.yaml>`, `help [command]` | |

## Scripts

A script is one command per line.  Commands run **sequentially in
simulation time**: non-blocking commands run back to back at the same
instant, a blocking command (`expect`, `sleep`, `wait-until`, `run <dur>`,
`step`) holds the stream until it is satisfied, and then the next line runs
at exactly that instant.  So

```
sendln 1 help
expect 1 "Shows this help" 4s
```

arms the expect before the node has executed a single instruction after the
input, and the script is race-free.  Deadlines and matches are pinned on the
event queue, so a scripted run is deterministic and byte-identical across
runs (`tools/check-shell.sh` checks that).

While a script or a blocking command holds the stream, lines typed at the
prompt queue behind it.  Two escape hatches: a line starting with `!` runs
immediately if the command is safe to interleave (`status`, `nodes`, `log`,
`log-file`, `pause`, `run`, `step`, `speed`, `at`, `atq`, `atrm`, `echo`,
`help`, `exit`), and Ctrl-C aborts the script.

**Exit codes.**  A script fails on an `expect` timeout, a false `assert`,
`fail`, a matched `fail-on`, or any command error inside a script file
(unknown node, bad syntax, unreadable `source`); the process then exits 1 and
prints `--- Script Results ---` like the JSON test runner.  Reaching the end
of the script without `pass`/`fail` is a pass.  A script still blocked when
the run ends (duration reached, or `exit` typed at the prompt) is reported as
"did not complete" and fails.  Without any script or verdict command the shell
does not touch the exit code.

Example, `test/scripts/shell-nrf54l15.cnsh`:

```
count "Command not found" 1
wait-until 2s
sendln 1 help
expect 1 "Shows this help" 4s
sendln 1 ip-addr
expect 1 "Node IPv6 addresses" 4s
sendln 1 no-such-command
expect 1 "Command not found" 2s
assert count "Command not found" == 1
assert time < 4s
pass
```

## Notes

- The shell coexists with `--ui`: pause/play/speed in the browser and at the
  prompt act on the same state; node lines still reach the browser console.
- `--gdb --gdb-wait` blocks inside the GDB service; the prompt is frozen
  until the debugger connects.
- Line editing is the vendored linenoise (`lib/linenoise/README.md`), so the
  shell works in every build; `rlwrap` is not needed but harmless.
- The Contiki-NG shell prompt (`#<lladdr>> `) has no trailing newline, so it
  appears as a prefix of the node's next line; substring `expect` is not
  affected.
- Not yet available from the shell (planned follow-ups): memory/register
  peek and poke, per-node TrustZone counters, radio-medium knobs.

## Implementation

`src/sim/sim_control.c` — the one implementation of add/move/remove/reboot/
send/pause/run-for/step/speed, on a small bundle of runner primitives; the
JSON action executor, the JS action executor, the WebSocket UI and the shell
all call it.  `src/services/shell_parse.c` (tokenizer, times, selectors — pure,
unit-tested), `shell_commands.c` (the table), `shell_script.c` (the command
stream, blocking, `at`/`on` queues), `shell_service.c` (terminal/pipe I/O,
prompt, console routing, service glue).  `test/test_shell.c` runs the parser
and the engine against a mock control bundle (`test_runner shell`).
