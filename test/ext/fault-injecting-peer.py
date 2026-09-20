#!/usr/bin/env python3
"""A fault-injecting external node for tools/check-ext-peer.sh.

PEER_MODE picks the fault:

  stuck   answer every step with a wake at the step's own time, forever
  rewind  answer every step with a wake 1 ms before the step's time
  inf     answer the first step with done.t = 1e400 (infinite once parsed)
  ok      no fault: wake 1 ms after each step (the control)
"""
import json
import os
import sys

MODE = os.environ.get("PEER_MODE", "ok")


def reply(line):
    sys.stdout.write(line + "\n")
    sys.stdout.flush()


def done(t, wake):
    reply(json.dumps({"type": "done", "t": t, "wake": wake, "out": []}))


for line in sys.stdin:
    msg = json.loads(line)
    kind = msg.get("type")
    if kind == "hello":
        done(0, 1000000)
    elif kind == "step":
        t = msg["t"]
        if MODE == "stuck":
            done(t, t)
        elif MODE == "rewind":
            done(t, max(0, t - 1000000))
        elif MODE == "inf":
            # json.dumps cannot write it; a peer in another language can.
            reply('{"type":"done","t":1e400,"wake":null,"out":[]}')
        else:
            done(t, t + 1000000)
    elif kind == "stop":
        break
