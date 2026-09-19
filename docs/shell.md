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
python3 tools/check-shell-tty.py                                           # terminal-only paths
```

Every simulation mode that takes a config or firmware list (`test`,
`mixed-multinode`, the `*-multinode` wrappers) accepts the flags.

## Flags

| flag | meaning |
|---|---|
| `--shell` | read commands from stdin.  On a terminal: line editing, history (`~/.cooja-ng_history`, or `$CSIM_SHELL_HISTORY`; empty disables), tab completion of command names.  From a pipe: lines run strictly in order, like a script (see *Pipes*), each echoed as `> cmd`; EOF = `exit`. |
| `--script FILE` | run FILE at simulation start, with or without `--shell`.  Without `--shell` the run ends when the script passes or fails, or when it reaches its end and every `at`/`every` it scheduled has fired (or the duration ends the run). |
| `--paused` | start paused (needs `--shell`, `--script` or `--ui` to resume). |
| `--wall-timeout <dur>` | end the run after that much **wall-clock** time (exit code 6).  A bare number is **seconds** here — a wall-clock bound like `timeout(1)`'s, and the form the agent-sim-protocol spec writes (`--wall-timeout 600`); `500ms`, `2m`, `1.5s` are accepted too.  Never influences the simulation — it only stops it, through the normal teardown (reports, `--save-config`).  A session waiting for input that never comes stops too. |
| `--speed N` / `--speed max` / `--realtime` | wall-clock pacing: N simulated seconds per wall second; `max` = unpaced (the headless default; the live UI and the serial bridge default to 10x). |

With `--shell` the run has no duration: it ends at `exit`.  At a terminal an
explicit `-t` **pauses** the simulation at that time instead of ending the run
(`run` continues); from a pipe, where nothing can type `run`, `-t` ends the run
as it does without `--shell`.  The config's `timeout_ms` is ignored, with a
note at start.  The `-t` horizon is independent of `run`/`pause`/`step`: a
`run 500ms` before it does not cancel it.  Without `--shell` (including
`--script` alone) the duration ends the run as before.

SIGINT and SIGTERM end the run through the normal teardown (reports,
`--save-config`); a second signal kills the process.  While an external clock
source drives the simulation (Renode co-simulation), `pause`, `run`, `step`,
`speed`, `sleep`, `wait-until`, `at` and `every` are refused: the master owns
time.

The default prompt shows the simulation time and state:
`cooja 12.345s> `, `cooja 12.345s [paused]> `, `cooja 12.345s [expect]> `.
Node console lines print between prompts in the same
`  12.345 [Node 1/ARM] text` format as a headless run.

## Commands

Node selectors: `1`, `1,3`, `2-5`, `all` (`*`), and `any` where a match is
meant.
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
| `send <nodes> <text...>` | console input, escapes honoured, spacing kept as typed, no newline added |
| `sendln <nodes> <text...>` | `send` + one `\n`, i.e. one Contiki-NG shell command (the Contiki shell ends a line on `\n` *or* `\r`, so `\r\n` would be two commands) |

Input is delivered the way each platform's model paces it (nRF54L15: one
UARTE byte per character time; MSP430: baud-paced; unconsumed bytes are
retried automatically, up to 512 queued bytes per node — beyond that `send`
reports an error).  A line reaching `max-line` bytes (default 128, Contiki-NG's
serial-line buffer) prints a warning, since the node would truncate it;
`set max-line 0` silences it.

**Scheduling**

| command | |
|---|---|
| `at <time> <command...>` | run a command at an exact simulation time |
| `every <period> <command...>` | run a command periodically (first after one period) |
| `at list`, `at clear <id>\|all` | list / cancel scheduled commands (`at` and `every`); `atq` and `atrm` are aliases, as in the Unix commands |

`at`, `every` and `on` run one command beside the command stream, so they
refuse the commands that would hold it: `expect`, `sleep`, `wait-until`,
`step`, `source`, and `run` with a duration.  Put such sequences in a script.
An error in a scheduled command fails the script only if a script file
scheduled it, and the message names both (`at #3 (test.cnsh:4): ...`).

**Scripting** (see below)

| command | |
|---|---|
| `source <file>` | run a script file (nested up to 8 deep); from a script, a relative path is looked up next to that script first, then in the working directory |
| `expect <nodes\|any> "<pattern>" [timeout]` | block until a console line contains the pattern (substring); the timeout (default 30 s, `set expect-timeout`) fails the script |
| `sleep <duration>`, `wait-until <time>` | block for a duration / until a time |
| `assert time <op> <t>`, `assert nodes <op> N`, `assert node <id> active\|removed\|exists`, `assert count "<pat>" <op> N` | checks (`== != < <= > >=`); a false assert fails the script |
| `pass`, `fail [message]` | end the script with a verdict |
| `fail-on "<pattern>" [nodes\|any]` | fail as soon as a console line contains the pattern |
| `count "<pattern>" [nodes\|any]` | count matching lines from now on, for `assert count` |
| `on <nodes\|any> "<pattern>" <command...>` | run a command whenever a line matches (e.g. `on any "SecureFault" fail "unexpected fault"`) |
| `set [expect-timeout <duration> \| max-line <bytes>]`, `echo <text...>`, `save-config <file.yaml>`, `help [command]` | `save-config` records the time run so far as `timeout_ms` |

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

While a script or a blocking command holds the stream, lines typed at a
terminal prompt queue behind it.  Two escape hatches: a line starting with `!`
runs immediately if the command is safe to interleave (`status`, `time`,
`nodes`, `log`, `log-file`, `pause`, `run`, `step`, `speed`, `at`, `every`,
`atq`, `atrm`, `set`, `save-config`, `echo`, `help`, `exit`, `quit`) — `!run 500ms` and `!step` run beside the stream
without holding it — and Ctrl-C aborts the script.  A mistyped `!` command
prints an error but never fails the running script.

**Paused while blocked.**  Simulated time does not advance while paused, so a
blocked `sleep`, `wait-until` or `expect` can only continue after a resume.
At a terminal the shell says so once (`type !run to continue`).  If nothing can
resume the run — a pipe, whose lines queue behind the block, or `--script`
alone, and no web UI — the shell fails the script as a deadlock instead of
hanging.

**Exit codes.**  A failing script prints `--- Script Results ---` like the
JSON test runner and the process exit code says *what* failed — the same
table the agent-sim-protocol session spec uses, so a shell test and a session
report a failure the same way and a broken script is never read as a
firmware failure:

| code | meaning | in the shell |
|---|---|---|
| 0 | pass | the script ended without a failure |
| 1 | assertion | `expect` timeout, false `assert`, `fail`, matched `fail-on`, "did not complete" |
| 2 | invalid request | unknown command, bad syntax or selector, unknown node, unreadable `source`, a deadlock |
| 5 | guest failure | (not used by the shell) |
| 6 | wall timeout | `--wall-timeout` ended the run |
| 7 | cancelled | Ctrl-C, SIGINT or SIGTERM while a command or script was in flight |

Reaching the end of the script without `pass`/`fail` is a pass, and `exit`
inside a script file ends it normally.  A command still blocked, or a script
file not yet finished, when the run ends — duration reached, `exit` typed at
the prompt, or a signal — is reported as "did not complete" and fails.
Without any script or verdict command the shell does not touch the exit code.

**Matchers.**  The verdict vocabulary is the protocol's, under shorter names:

| shell | agent-sim-protocol |
|---|---|
| `expect <nodes> "<pat>"` | `log_contains` |
| `assert count "<pat>" <op> N` | `event_count` |
| `wait-until <time>`, `sleep <dur>` | `time` |
| `fail-on "<pat>"`, `on ... fail` | `invariants` |
| `assert node <id> active\|removed` | node state predicates |

## Pipes

When stdin is not a terminal, `--shell` reads it **synchronously**: whenever
the command stream is idle the simulation waits for the next line.  A piped
session therefore behaves exactly like a script — each command runs at a
simulation time decided by the commands before it, never by how fast the
host reads the pipe — and two runs of the same input are byte-identical.
Simulated time advances only through blocking commands (`run`, `sleep`,
`wait-until`, `expect`) or after EOF's implied `exit`.  A pipe's `!` lines run
in order like any other line; a driver process that writes commands over
time should use `run <duration>` / `sleep` to move the simulation between
them.

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
- An `on` command that sends to a node whose output matches the same pattern
  again (for example an echoing shell) feeds back on itself; more than 16
  firings between two slices are dropped with a warning.
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
and the engine against a mock control bundle (`test_runner shell`);
`tools/check-shell.sh` runs the end-to-end checks and `tools/check-shell-tty.py`
the terminal-only ones through a pseudo-terminal.
