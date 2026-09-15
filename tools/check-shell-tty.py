#!/usr/bin/env python3
"""Drive the Cooja-NG shell through a pseudo-terminal (docs/shell.md).

tools/check-shell.sh covers everything a pipe can reach.  These are the
terminal-only paths: the linenoise editor (tab completion, history, Ctrl-C,
Ctrl-D), prompt redraw around node output, "!cmd" beside a blocked command
stream, and the "blocked while paused" hint.

    python3 tools/check-shell-tty.py        # exit status 0 = all checks passed
"""
import os
import pty
import re
import select
import sys
import tempfile
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BIN = os.path.join(ROOT, "build", "test_runner")
CFG = "configs/shell-nrf54l15-dk.yaml"
ANSI = re.compile(rb"\x1b\[[0-9;?]*[A-Za-z]")
failures = 0


class Session:
    def __init__(self, args, history):
        env = dict(os.environ, TERM="xterm", CSIM_SHELL_HISTORY=history)
        self.pid, self.fd = pty.fork()
        if self.pid == 0:
            os.chdir(ROOT)
            os.execve(BIN, [BIN, "test", CFG, "--shell", "-q"] + args, env)
        self.raw = b""

    def read_for(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            ready, _, _ = select.select([self.fd], [], [], 0.05)
            if not ready:
                continue
            try:
                data = os.read(self.fd, 65536)
            except OSError:
                return
            if not data:
                return
            self.raw += data

    def send(self, data, wait=0.5):
        os.write(self.fd, data.encode() if isinstance(data, str) else data)
        self.read_for(wait)

    def finish(self, wait=10.0):
        end = time.time() + wait
        while time.time() < end:
            pid, status = os.waitpid(self.pid, os.WNOHANG)
            if pid:
                self.read_for(0.2)
                return os.waitstatus_to_exitcode(status)
            self.read_for(0.1)
        os.kill(self.pid, 9)
        os.waitpid(self.pid, 0)
        return None

    @property
    def text(self):
        return ANSI.sub(b"", self.raw).decode("utf-8", "replace")


def check(name, ok):
    global failures
    print(("ok   " if ok else "FAIL ") + name)
    if not ok:
        failures += 1


def main():
    if not os.path.exists(BIN):
        print("build/test_runner not found; run make first")
        return 2
    tmp = tempfile.mkdtemp(prefix="csim-shell-tty.")
    history = os.path.join(tmp, "history")

    # 1. Editor basics: completion, node output while the prompt is live,
    #    Ctrl-C, history recall, Ctrl-D ends the run cleanly.
    s = Session(["--realtime"], history)
    s.read_for(1.5)
    s.send("stat")
    s.send("\t")
    s.send("\r", 0.6)
    s.send("log on 1\r", 0.4)       # -q starts with console output off
    s.send("sendln 1 help\r", 1.5)
    s.send("xyz garbage", 0.3)
    s.send(b"\x03", 0.4)
    s.send(b"\x1b[A", 0.3)          # up arrow recalls "sendln 1 help"
    s.send(b"\x15", 0.2)            # Ctrl-U clears it
    s.send("nodes\r", 0.6)
    s.send(b"\x04", 0.5)            # Ctrl-D on an empty line = exit
    code = s.finish()
    check("prompt shown", "cooja " in s.text)
    check("tab completes 'stat' to status", "> status" in s.text and "state:" in s.text)
    check("node output while editing", "[Node 1/ARM]" in s.text)
    check("Ctrl-C discards the line", "^C (type exit to quit)" in s.text)
    check("nodes table", "shell.nrf54l15-dk" in s.text)
    check("Ctrl-D exits with status 0", code == 0)
    check("history saved to disk",
          os.path.exists(history) and "sendln 1 help" in open(history).read())
    check("output post-processing kept (no staircase)", b"\r\n" in s.raw)

    # 2. A typo typed beside a running script must not fail the script.
    script = os.path.join(tmp, "slow.cnsh")
    with open(script, "w") as f:
        f.write("sleep 2s\npass\n")
    s = Session(["--realtime", "--script", script], history)
    s.read_for(0.8)
    s.send("!stauts\r", 0.5)
    s.read_for(2.5)
    s.send("exit\r", 0.5)
    code = s.finish()
    check("!typo reports an error", "unknown command 'stauts'" in s.text)
    check("!typo does not fail the script", "SCRIPT PASSED" in s.text and "SCRIPT FAILED" not in s.text)
    check("script run exits 0", code == 0)

    # 3. `!exit` while an expect is still waiting: the run ends, and the
    #    unfinished expect fails the verdict.
    s = Session(["--realtime"], history)
    s.read_for(1.0)
    s.send('expect 1 "never printed" 100s\r', 0.8)
    s.send("!exit\r", 0.5)
    code = s.finish()
    check("!exit while blocked: did not complete", "did not complete" in s.text)
    check("!exit while blocked: exit status 1", code == 1)

    # 4. Paused with the stream blocked: a hint, and !run releases it.
    s = Session(["--realtime"], history)
    s.read_for(1.0)
    s.send("pause\r", 0.4)
    s.send("sleep 500ms\r", 0.6)
    s.send("echo after-sleep\r", 0.3)
    check("blocked-while-paused hint", "type !run to continue" in s.text)
    s.send("!run\r", 1.5)
    s.send("exit\r", 0.5)
    code = s.finish()
    check("!run releases the sleep", "after-sleep" in s.text)
    check("paused session exits 0", code == 0)

    # 5. Terminal stdin, piped stdout ("--shell | tee log"): there is no
    #    line editor, but a human can still type, so the session must stay
    #    asynchronous — the simulation runs while nothing is typed.
    r, w = os.pipe()
    pid, fd = pty.fork()
    if pid == 0:
        os.dup2(w, 1)
        os.dup2(w, 2)
        os.close(r)
        os.close(w)
        os.chdir(ROOT)
        os.execve(BIN, [BIN, "test", CFG, "--shell", "-q", "--realtime"],
                  dict(os.environ, TERM="xterm", CSIM_SHELL_HISTORY=history))
    os.close(w)
    time.sleep(2.0)                 # type nothing: the sim should advance
    os.write(fd, b"status\r")
    time.sleep(1.0)
    os.write(fd, b"exit\r")
    out = b""
    end = time.time() + 10
    while time.time() < end:
        ready, _, _ = select.select([r], [], [], 0.2)
        if not ready:
            continue
        chunk = os.read(r, 65536)
        if not chunk:
            break
        out += chunk
    os.close(r)
    os.waitpid(pid, 0)
    text = ANSI.sub(b"", out).decode("utf-8", "replace")
    m = re.search(r"time: ([0-9.]+) s", text)
    check("piped stdout: the simulation runs between commands (%s)" %
          (m.group(1) if m else "no status"),
          bool(m) and float(m.group(1)) > 1.0)

    print("check-shell-tty: %s" % ("OK" if failures == 0 else "%d FAILED" % failures))
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
