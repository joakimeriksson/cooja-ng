#!/usr/bin/env python3
"""
Disturber (jammer) node — the smallest possible external data-driven node.

Speaks the protocol in docs/design/external-nodes-plan.md §4 directly on
stdin/stdout: no library, no dependencies, ~30 lines of logic.  It never
listens; it just puts a burst of junk on the air every JAM_PERIOD_MS of
*simulation* time, which collides with any frame in flight and makes the
neighbours' CRC fail.

Use it to answer "how does my protocol behave under interference?" without
touching the emulator:

    { "firmware": "examples/ext/jammer.py", "x": 10, "y": 0 }

Knobs (environment, so no config-schema change is needed):
    JAM_PERIOD_MS  how often to transmit          (default 100)
    JAM_LEN        bytes of junk per burst        (default 40, ~1.3 ms on air)
    JAM_CHANNEL    802.15.4 channel               (default 26)
"""
import json
import os
import sys

PERIOD_NS = int(os.environ.get("JAM_PERIOD_MS", "100")) * 1_000_000
JAM_LEN   = int(os.environ.get("JAM_LEN", "40"))
CHANNEL   = int(os.environ.get("JAM_CHANNEL", "26"))

# Deliberate garbage, not a valid MAC frame — that is the whole point of a
# jammer.  Fixed content so the run stays deterministic (§8: csim guarantees
# determinism only if the peer is deterministic — no random, no wall clock).
JUNK = bytes((i * 7 + 0x5A) & 0xFF for i in range(JAM_LEN)).hex()


def done(t, wake, out=()):
    """Exactly one reply per hello/step: when we are done and when to wake us."""
    sys.stdout.write(json.dumps({"type": "done", "t": t, "wake": wake,
                                 "out": list(out)}) + "\n")
    sys.stdout.flush()


def main():
    next_jam = 0
    for line in sys.stdin:
        msg = json.loads(line)
        kind = msg.get("type")

        if kind == "hello":
            # Stagger multiple jammers by node id so they do not fire in lockstep.
            next_jam = (msg["id"] % 10) * 1_000_000
            done(0, next_jam, [{"type": "log", "t": 0,
                                "line": "jammer up, every %d ms on ch %d"
                                        % (PERIOD_NS // 1_000_000, CHANNEL)}])

        elif kind == "step":
            t = msg["t"]                      # simulation nanoseconds, csim's clock
            out = []
            if t >= next_jam:
                out.append({"type": "tx", "t": t, "ch": CHANNEL, "frame": JUNK})
                out.append({"type": "log", "t": t,
                            "line": "jam %d B" % JAM_LEN})
                next_jam = t + PERIOD_NS
            done(t, next_jam, out)            # `wake` = when we want the next step

        elif kind == "stop":
            break


if __name__ == "__main__":
    main()
