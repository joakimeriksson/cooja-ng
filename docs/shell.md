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
| `status`, `time`, `nodes`, `stats` | state summary; simulation time; the node table; RF/console counters and per-node cycles and instructions |
| `exit`, `quit` | end the run: normal teardown, test reports, `--save-config` |

**Nodes**

| command | |
|---|---|
| `add <firmware\|type> [id] [x y]` | add a node from a config v2 mote-type name or a firmware path; id defaults to max+1; it boots and runs from now |
| `remove <nodes>` | stop nodes for good (`reboot` revives them) |
| `move <id> <x> <y>` | set a position (metres), neighbours recomputed |
| `reboot <nodes>` | destroy + re-initialize from the same firmware, clock re-seeded to now |

**Environment**

| command | |
|---|---|
| `radio`, `radio range <tx> [interference]`, `radio success <tx> [rx]` | show the medium; change UDGM range (metres) or success ratios (0-1); neighbour lists are recomputed |
| `link <a> <b> off\|on`, `link <a> -> <b> off\|on`, `links` | cut or restore a radio link both ways or one way, whatever the distance; list cut links.  A cut filters delivery only: the cut sender still occupies the air at the receiver, so its frames collide there as before |
| `neighbors [node]` | who each node hears |
| `pcap <file>\|off` | start or stop an 802.15.4 capture |
| `clock <node> [deviation]` | show or set a node's clock deviation (1.0 exact; 1.00002 runs 20 ppm fast) |
| `leds [nodes]` | LED states |
| `gpio <node> <port>.<pin> high\|low\|pulse [duration]\|release` | drive an input pin: MSP430 `P1.0`-`P10.7`, CC2538 `A.0`-`D.7` (raises the pin interrupt the firmware configured), nRF54L15 `P0`-`P2` (the IN register only; no GPIOTE interrupt is modelled); nRF52840 has no GPIO model.  `release` stops forcing the pin: on the nRF54L15 IN follows OUT again (loopback); MSP430 and CC2538 keep the last level (nothing to release).  Not `button release`, which drives the button's inactive level |
| `button <node> press\|release\|click [duration]` | the board's user button, respecting active-low wiring (Sky P2.7, Z1 P2.5, CC2538DK PA3, nRF boards per board file); `click` releases after 100 ms |
| `restart` | restart from the configuration: configured nodes only, links restored, `at` queue cleared, scripts aborted; lines after it run against the new simulation.  Inside a script file it is the script's end, like `exit`; refused from `at`/`every`/`on` |
| `ui <port>` | start the live web UI now |

**Console**

| command | |
|---|---|
| `log [on\|off [nodes] \| only <nodes>]` | which nodes' console lines print here (default all; none under `-q`) |
| `log-file <path> [nodes]`, `log-file off [path]`, `log-file` | append nodes' console lines to a file (same line format, flushed per line); close; list |
| `send <nodes> <text...>` | console input, escapes honoured, spacing kept as typed, no newline added |
| `sendln <nodes> <text...>` | `send` + one `\n`, i.e. one Contiki-NG shell command (the Contiki shell ends a line on `\n` *or* `\r`, so `\r\n` would be two commands) |
| `cmd [-e "<pat>"] [-f "<pat>"] [-c <var> "<re>"] [-t <timeout>] <node> [text...]` | `sendln`, then **block until the node prints its shell prompt** again. `-e`: the output before the prompt must contain the pattern; `-f`: it must not; `-c`: the first output line matching the regex sets `$var`; either failing, or no prompt within the timeout (default `expect-timeout`), fails the script. A bare `cmd <node>` just waits for a prompt. |
| `console <node>` | talk to one node directly (terminal only): lines you type go to the node, its console bytes come back unprefixed, other nodes' lines are hidden. Ctrl-C is sent to the node rather than ending the run. `~.` on its own line, or Ctrl-D, returns to the shell. The simulation keeps running at its current speed meanwhile. |

**Prompts.**  A shell prompt has no newline, so it never becomes a log line
that `expect` could see.  `cmd` watches the node's raw console bytes instead:
the text since the last newline must match the glob `set prompt` (default
`#*> `, Contiki-NG's `#<lladdr>> `; for example `"> "` for RIOT, `"uart:~$ "`
for Zephyr), and the console must then stay quiet for 2 ms of simulated time —
a prompt followed at once by more text on the same line is output, not the end
of the command.  A shell that forwards to another shell may never show a usable prompt: the
TrustZone secure world's `ns` passes the Normal world's output on one complete
line at a time, so the Normal world's prompt only appears in front of its
*next* output line.  Drive `ns` with `sendln` and `expect`:

```
cmd -e "Shows this help" 1 help          # the Secure world's own shell
sendln 1 ns help                         # forwarded to the Normal world
expect 1 "rpl-global-repair" 2s
```

Send nothing until the node's shell is up, or the line is lost and `cmd`
times out.

Input is delivered the way each platform's model paces it (nRF54L15: one
UARTE byte per character time; MSP430: baud-paced; unconsumed bytes are
retried automatically, up to 512 queued bytes per node — beyond that `send`
reports an error).  A line reaching `max-line` bytes (default 128, Contiki-NG's
serial-line buffer) prints a warning, since the node would truncate it;
`set max-line 0` silences it.

**History and watches**

| command | |
|---|---|
| `tail [-n N] [nodes]` | the last N (default 20) console lines, from the 2000 the shell remembers |
| `grep [-re] [-c] "<pattern>" [nodes]` | remembered console lines matching a substring or regex; `-c` prints only the count |
| `on list`, `on clear <n>\|all` | list and remove `on` / `fail-on` / `count` watches |
| `transcript <file>\|off`, `history save <file>` | append every command line you type or pipe to a file (replayable with `--script`); save the terminal's line-editing history |

**Variables**

| command | |
|---|---|
| `var`, `var <name>`, `var <name> <value...>`, `var -d <name>` | list, show, set (spacing kept), delete |
| `capture <var> <nodes\|any> "<regex>" [timeout]` | block until a console line matches the extended regex; group 1 (or the whole match) goes into `$var` |

`$name` and `${name}` expand in every command line before it is parsed —
also inside double quotes, never inside single quotes, after `\`, or in a
comment.  A value is always one word with every byte literal: quotes, `#`,
spaces and control characters in a captured line cannot break the command
(`sendln 2 ping $addr` sends exactly what was captured).  `$$` is a literal `$`, so `at +5s echo $$x` expands `$x` when the
`at` fires rather than when it is scheduled.  An undefined variable is an
error.  `cmd -c <var> "<regex>"`, `expect -c <var>`, `sym -c`, `reg -c` and
`mem -c` store into variables too; `assert var <name> <op> <value>` compares
(numerically when both sides are numbers, else `==`/`!=` as text).

```
cmd -c addr "(fe80::[0-9a-f:]+)" 1 ip-addr
sendln 2 ping $addr
expect 2 "Received ping reply" 5s
```

**Scheduling**

| command | |
|---|---|
| `at <time> <command...>` | run a command at an exact simulation time |
| `every <period> <command...>` | run a command periodically (first after one period) |
| `at list`, `at clear <id>\|all` | list / cancel scheduled commands (`at` and `every`); `atq` and `atrm` are aliases, as in the Unix commands |

`at`, `every` and `on` run one command beside the command stream, so they
refuse the commands that would hold it: `cmd`, `expect`, `expect-not`,
`capture`, `expect-fault`, `expect-halt`, `sendfile`, `sleep`, `wait-until`, `step`, `source`,
and `run` with a duration — and `restart`, which would discard whatever the
stream is waiting on (a pending `run 2s` included).  Put such sequences in a
script.
An error in a scheduled command fails the script only if a script file
scheduled it, and the message names both (`at #3 (test.cnsh:4): ...`).

**Scripting** (see below)

| command | |
|---|---|
| `source <file>` | run a script file (nested up to 8 deep); from a script, a relative path is looked up next to that script first, then in the working directory |
| `expect [-re] [-n N] [-c <var>] <nodes\|any> "<pattern>" [timeout]` | block until N console lines (default 1) contain the pattern — a substring, or with `-re` a POSIX extended regex; `-c` captures group 1 (or the match; for a substring, the line); the timeout (default 30 s, `set expect-timeout`) fails the script |
| `expect-not [-re] <nodes\|any> "<pattern>" <duration>` | block for the duration; a matching line fails the script at once |
| `sendfile [-t <timeout>] <node> <path>` | send a file to a node line by line, each as a `cmd` (waits for the prompt); lines go verbatim, no comments or variables |
| `sleep <duration>`, `wait-until <time>` | block for a duration / until a time |
| `assert time <op> <t>`, `assert nodes <op> N`, `assert node <id> active\|removed\|exists`, `assert count "<pat>" <op> N` | checks (`== != < <= > >=`); a false assert fails the script |
| `pass`, `fail [message]` | end the script with a verdict |
| `fail-on "<pattern>" [nodes\|any]` | fail as soon as a console line contains the pattern |
| `count "<pattern>" [nodes\|any]` | count matching lines from now on, for `assert count` |
| `on [--once] <nodes\|any> "<pattern>" <command...>` | run a command whenever (with `--once`: the first time) a line matches (e.g. `on any "SecureFault" fail "unexpected fault"`) |
| `repeat <count> [var]` … `end` | run the lines up to `end` count times; `$var` counts 1..count |
| `if <condition>` … [`else` …] `end` | conditions as for `assert`: `time`, `nodes`, `node`, `count`, `mem`, `var` |
| `set [expect-timeout <duration> \| max-line <bytes> \| prompt "<glob>"]`, `echo <text...>`, `save-config <file.yaml>`, `help [command]` | `save-config` records the time run so far as `timeout_ms` |

**Inspecting a node** (ARM nodes; the debugger's view)

| command | |
|---|---|
| `sym [-c <var>] <node> <symbol>` | address of a symbol in the node's firmware, then its Secure-world image (any node kind) |
| `mem [-w] [-c <var>] <node> <addr\|sym[+off]> [count]` | hexdump bytes (default 64), or native little-endian words with `-w` (32-bit ARM, 16-bit MSP430; default 8) |
| `mem [-w] <node> <addr\|sym> = <value...>` | write bytes (or words); ARM flash is refused, as on hardware; MSP430 writes are limited to RAM |
| `reg [-c <var>] <node> [name]`, `reg <node> <name> = <value>` | ARM: r0-r12 sp lr pc xpsr primask basepri faultmask, plus msp_s psp_s msp_ns psp_ns control_s control_ns on ARMv8-M; writes r0-r12, sp, lr, pc, xpsr — e.g. to inject a fault. MSP430: pc sp sr r3-r15, all writable |
| `tz <node>` | TrustZone-M: security state, SG / BXNS / secure-exception counters, SFSR+SFAR decoded, SAU regions, banked stacks |
| `faults <node>` | HardFault / MemManage / BusFault / UsageFault / SecureFault entry counts, the last fault's pc and the security state it came from, SFSR/SFAR |
| `expect-fault <node> [kind[,kind]\|any] [timeout]` | block until the node takes one of those faults (checked every simulated ms); the timeout fails the script |
| `assert mem <node> <addr\|sym> <op> <word>`, `assert node <id> secure\|non-secure` | memory (native word) and security-state checks |
| `break <node> <addr\|sym[+off]>` | stop the node before it executes that address and pause the simulation (ARM) |
| `watch <node> <addr\|sym[+off]> [bytes]` | stop when 1-4 bytes of SRAM change; the report names the pc of the instruction that wrote them (ARM) |
| `break list`, `break clear <n>\|all` | list breakpoints and watchpoints (with hit counts, and where a node is halted) and remove them |
| `continue [node]` | release halted nodes — a breakpoint is not hit again on the way out — and resume |
| `expect-halt <node> [timeout]` | block until the node hits a breakpoint or watchpoint |

A hit halts that node where it is and pauses the simulation at the next slice
boundary (other nodes may finish the slice they are in).  `run` resumes the
others with the node still halted; `continue` releases it.  The time the
others ran on is not replayed: the node resumes at the current instant with
its clock — timers and radio included — that much behind.  An armed node runs
in the interpreter, never the JIT, and costs one out-of-line check per
instruction; nodes without breakpoints are unaffected.  Breakpoints survive a
reboot of the node.  MSP430 nodes have `mem`/`reg` but no breakpoints.

Memory access bypasses TrustZone (it is the debugger's view, like a probe on
the SWD port); on ARM, reads of peripheral registers go through the
peripheral model, once per register (a word), and can have its read side
effects; on an nRF54L15 the debugger accesses as Secure, so a Non-secure alias
does not latch a permission error in the guest.  An MSP430 node's `mem` reads
the memory image, not its peripheral models.  CFSR/HFSR are not modelled; the fault
counters are, and they survive a SoC reset.

Example, `test/scripts/tz-securefault-nrf54l15-xiao.cnsh` on
`configs/shell-tz-nrf54l15-xiao.yaml`:

```
wait-until 3s
assert node 1 non-secure
tz 1
reg 1 pc = 0x00001000                     # branch the Normal world into Secure flash
expect-fault 1 securefault 100ms
faults 1
pass
```

## Scripts

A script is one command per line.  Commands run **sequentially in
simulation time**: non-blocking commands run back to back at the same
instant, a blocking command (`cmd`, `expect`, `expect-not`, `capture`,
`expect-fault`, `expect-halt`, `sendfile`, `sleep`, `wait-until`, `run <dur>`,
`step`) holds
the stream until it is satisfied, and then the next line runs at exactly that
instant.  So

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
runs immediately if the command is safe to interleave — the commands `help`
marks with `!`: state and settings, `run`/`pause`/`step`/`speed`, the
environment (`radio`, `link`, `gpio`, `button`, ...), `continue`, `exit` —
so `!run 500ms` and `!step` run beside the stream without holding it — and
Ctrl-C aborts the script.  A mistyped `!` command
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
| 1 | assertion | `expect` timeout, a failed `cmd`, false `assert`, `fail`, matched `fail-on`, "did not complete" |
| 2 | invalid request | unknown command, bad syntax or selector, unknown node, unreadable `source`, a deadlock; also a run that never starts — a `--script` file that cannot be opened, a bad flag value, an unknown option, a config that does not load (reported on stderr, before any results block) |
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
| `expect <nodes> "<pat>"`, `expect -re`, `expect -n N` | `log_contains`, `log_matches`, `count`; the window starts when `expect` is armed (`since: previous`) |
| `count "<pat>"` + `assert count "<pat>" <op> N` | `log_contains` with `count`; the window starts at `count`.  (The protocol's `event_count` counts event *types* such as `tx`/`rx`/`exception`, which the shell cannot count yet.) |
| `wait-until <time>`, `sleep <dur>` | `time` |
| `fail-on "<pat>"`, `expect-not`, `on ... fail` | `invariants` (Cooja-NG's `fail_on`; the protocol's `no_event` is type-based) |
| `assert node/mem/var`, `expect-halt`, `expect-fault`, `cmd`, `capture`, `on`, `every` | simulator-specific, outside the closed condition set (esp32sim advertises `probe_reached`/`memory_value`; Cooja-NG advertises none yet) |

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

`repeat`, `if`, `else` and `end` work in script files (not at the prompt) and
nest up to 8 deep; blocking commands inside a loop resume the loop.  A run of
non-blocking lines yields to the simulation every 10 000 lines, so a very long
loop spreads over a few scheduler slices rather than one instant:

```
repeat 5 i
  cmd -c rtt "time=([0-9]+)" 2 ping $addr
  if var rtt > 100
    echo slow round trip $i: $rtt ms
  end
end
```

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

The same test with `cmd`, `test/scripts/shell-nrf54l15-cmd.cnsh`:

```
wait-until 1s
cmd -e "Shows this help" 1 help
cmd -e "Node IPv6 addresses" -f "Command not found" 1 ip-addr
cmd -e "Command not found" 1 no-such-command
cmd 1                      # a bare line: just wait for the prompt
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
- Not yet available from the shell: single-instruction stepping, MSP430
  breakpoints, Nordic GPIOTE (pin interrupts from `gpio`/`button` on nRF).

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
