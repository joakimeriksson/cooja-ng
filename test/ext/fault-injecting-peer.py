#!/usr/bin/env python3
"""A fault-injecting external node for tools/check-ext-peer.sh.

PEER_MODE picks the fault:

  stuck   answer every step with a wake at the step's own time, forever
  rewind  answer every step with a wake 1 ms before the step's time
  chatty  like stuck, but with a console line in every reply, so each
          exchange has output and only the idle-step bound catches it
  inf     answer the first step with done.t = 1e400 (infinite once parsed)
  tx-inf  answer the first step with a tx event stamped t = 1e400
  tx-ninf answer the first step with a tx event stamped t = -1e400
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
        elif MODE == "chatty":
            reply(json.dumps({"type": "done", "t": t, "wake": t, "out": [
                {"type": "log", "t": t, "line": "still here"}]}))
        elif MODE == "rewind":
            done(t, max(0, t - 1000000))
        elif MODE == "inf":
            # json.dumps cannot write it; a peer in another language can.
            reply('{"type":"done","t":1e400,"wake":null,"out":[]}')
        elif MODE in ("tx-inf", "tx-ninf"):
            # The stamp must be rejected as a time before it is compared
            # with the slice start: converting it is undefined behavior.
            stamp = "1e400" if MODE == "tx-inf" else "-1e400"
            reply('{"type":"done","t":%d,"wake":%d,"out":[{"type":"tx",'
                  '"t":%s,"frame":"4188"}]}' % (t, t + 1000000, stamp))
        else:
            done(t, t + 1000000)
    elif kind == "stop":
        break
