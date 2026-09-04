#!/usr/bin/env python3
"""
Sniffer node — receives every frame in range, logs it in hex and decodes the
IEEE 802.15.4 MAC header.

The counterpart to jammer.py: that one only transmits, this one only listens.
Together they cover both directions of the protocol in
docs/design/external-nodes-plan.md §4.

It never transmits and never asks for a wakeup (`wake: null`), so it costs
nothing between frames -- csim steps it only when something arrives.

    { "firmware": "examples/ext/sniffer.py", "x": 2.0, "y": 0.0 }

Each received frame is reported on the node's console:

    rx #1  from=1  rssi=-65 dBm  ch=26  len=17
        DATA seq=86  ver=1  dst=abcd/ffff  src=(same)/0101.0100.0174.1200
        hdr  41d8 56cd abff ff00 1274 0100 0101 01
        data 0000

Set SNIFFER_HEX=0 to log just the decoded header line.
"""
import json
import os
import sys

SHOW_HEX = os.environ.get("SNIFFER_HEX", "1") != "0"

FRAME_TYPES = {0: "BEACON", 1: "DATA", 2: "ACK", 3: "CMD"}
ADDR_MODES = {0: "none", 1: "resvd", 2: "short", 3: "ext"}


def fmt_addr(raw):
    """802.15.4 sends addresses least-significant byte first; Contiki prints
    them most-significant first, in dotted 2-byte groups.  Reverse so the two
    can be compared by eye against the node consoles."""
    b = raw[::-1]
    return ".".join(b[i:i + 2].hex() for i in range(0, len(b), 2))


def decode_mac(f):
    """Decode as much of the MAC header as the frame control field promises.
    Returns (summary, header_len).  Never raises: a sniffer must survive
    malformed and truncated frames, which is most of what it will hear."""
    if len(f) < 3:
        return "too short for a MAC header", 0

    fcf = f[0] | (f[1] << 8)
    ftype = fcf & 0x7
    sec = (fcf >> 3) & 1
    pending = (fcf >> 4) & 1
    ack_req = (fcf >> 5) & 1
    pan_comp = (fcf >> 6) & 1
    dst_mode = (fcf >> 10) & 0x3
    version = (fcf >> 12) & 0x3
    src_mode = (fcf >> 14) & 0x3
    seq = f[2]

    parts = ["%s seq=%d" % (FRAME_TYPES.get(ftype, "type%d" % ftype), seq)]
    if ack_req:
        parts.append("ack_req=1")
    if pending:
        parts.append("pending=1")
    if sec:
        parts.append("secured=1")
    if version:
        parts.append("ver=%d" % version)

    i = 3
    try:
        if dst_mode:
            dst_pan = f[i:i + 2][::-1].hex()
            i += 2
            n = 2 if dst_mode == 2 else 8
            dst = fmt_addr(f[i:i + n])
            i += n
            parts.append("dst=%s/%s" % (dst_pan, dst))
        if src_mode:
            if pan_comp and dst_mode:
                src_pan = "(same)"          # compressed: reuse the dest PAN
            else:
                src_pan = f[i:i + 2][::-1].hex()
                i += 2
            n = 2 if src_mode == 2 else 8
            src = fmt_addr(f[i:i + n])
            i += n
            parts.append("src=%s/%s" % (src_pan, src))
        if not dst_mode and not src_mode:
            parts.append("no addresses (mode %s/%s)"
                         % (ADDR_MODES[dst_mode], ADDR_MODES[src_mode]))
    except (IndexError, ValueError):
        parts.append("<truncated header>")
        return "  ".join(parts), min(i, len(f))

    if i > len(f):
        parts.append("<truncated header>")
        i = len(f)
    return "  ".join(parts), i


def grouped_hex(b, group=2, per_line=16):
    """Hex in 2-byte groups, so header fields line up by eye."""
    lines = []
    for off in range(0, len(b), per_line):
        chunk = b[off:off + per_line]
        lines.append(" ".join(chunk[i:i + group].hex()
                              for i in range(0, len(chunk), group)))
    return lines


out = []


def log(line):
    out.append({"type": "log", "t": 0, "line": line})


def done(t, wake=None):
    for ev in out:
        ev["t"] = t
    sys.stdout.write(json.dumps({"type": "done", "t": t,
                                 "wake": wake, "out": out}) + "\n")
    sys.stdout.flush()
    out.clear()


def main():
    count = 0
    for line in sys.stdin:
        msg = json.loads(line)
        kind = msg.get("type")

        if kind == "hello":
            log("sniffer up, listening")
            done(0)                      # wake=None: only step me on traffic

        elif kind == "step":
            t = msg["t"]
            for ev in msg.get("in", []):
                if ev.get("type") != "rx":
                    continue
                count += 1
                frame = bytes.fromhex(ev["frame"])
                summary, hdr_len = decode_mac(frame)

                log("rx #%d  from=%s  rssi=%s dBm  ch=%s  len=%d"
                    % (count, ev.get("from"), ev.get("rssi"),
                       ev.get("ch"), len(frame)))
                log("    " + summary)
                if SHOW_HEX:
                    # Split at the header boundary the decode found, so the
                    # addressing fields and the payload are told apart.
                    for hx in grouped_hex(frame[:hdr_len]):
                        log("    hdr  " + hx)
                    for hx in grouped_hex(frame[hdr_len:]):
                        log("    data " + hx)
            done(t)                      # still no wakeup of our own

        elif kind == "stop":
            break


if __name__ == "__main__":
    main()
