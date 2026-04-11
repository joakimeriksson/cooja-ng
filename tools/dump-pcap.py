#!/usr/bin/env python3
"""
Quick pcap dumper for csim 802.15.4 captures.

Reads a libpcap-format file (classic, ns-precision magic 0xa1b23c4d),
prints a one-line summary per frame, and decodes enough of the IEEE
802.15.4 MAC header to show frame type, source/destination short
addresses, and the next protocol layer if recognizable.

Usage:
    python3 tools/dump-pcap.py path/to/capture.pcap
"""
import struct
import sys

PCAP_MAGIC_NSEC = 0xa1b23c4d
PCAP_MAGIC_USEC = 0xa1b2c3d4

LINKTYPE_IEEE802_15_4_WITHFCS = 195

# IEEE 802.15.4 frame types
FRAME_TYPE = {
    0: "BEACON",
    1: "DATA",
    2: "ACK",
    3: "MAC_CMD",
    5: "MULTIPURPOSE",
}


def decode_802_15_4(buf: bytes) -> str:
    """Decode the MHR of an 802.15.4 frame and return a one-line summary."""
    if len(buf) < 3:
        return f"runt ({len(buf)}B)"
    fcf = buf[0] | (buf[1] << 8)
    seq = buf[2]
    ftype = fcf & 0x7
    sec = (fcf >> 3) & 1
    pending = (fcf >> 4) & 1
    ack_req = (fcf >> 5) & 1
    panid_compress = (fcf >> 6) & 1
    dst_mode = (fcf >> 10) & 0x3   # 0=none, 2=short, 3=ext
    src_mode = (fcf >> 14) & 0x3
    type_str = FRAME_TYPE.get(ftype, f"T{ftype}")
    flags = []
    if ack_req: flags.append("AR")
    if pending: flags.append("FP")
    if sec:     flags.append("SEC")
    flag_str = ",".join(flags) if flags else "-"

    # Try to extract addresses
    off = 3
    dst = src = None
    dst_pan = src_pan = None
    if dst_mode == 2:  # short
        if len(buf) >= off + 4:
            dst_pan = (buf[off] | (buf[off+1] << 8))
            dst = (buf[off+2] | (buf[off+3] << 8))
            off += 4
    elif dst_mode == 3:  # extended (8 bytes)
        if len(buf) >= off + 10:
            dst_pan = (buf[off] | (buf[off+1] << 8))
            dst = ":".join(f"{b:02x}" for b in buf[off+2:off+10])
            off += 10
    if src_mode == 2:
        if not panid_compress:
            if len(buf) >= off + 2:
                src_pan = buf[off] | (buf[off+1] << 8)
                off += 2
        if len(buf) >= off + 2:
            src = buf[off] | (buf[off+1] << 8)
            off += 2
    elif src_mode == 3:
        if not panid_compress:
            if len(buf) >= off + 2:
                src_pan = buf[off] | (buf[off+1] << 8)
                off += 2
        if len(buf) >= off + 8:
            src = ":".join(f"{b:02x}" for b in buf[off:off+8])
            off += 8

    addr = ""
    if src is not None and dst is not None:
        addr = f" {src}->{dst}"
    elif dst is not None:
        addr = f" ->{dst}"
    elif src is not None:
        addr = f" {src}->?"

    return f"{type_str:10s} seq={seq:3d} {flag_str:8s}{addr}  ({len(buf)}B)"


def main() -> int:
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} capture.pcap", file=sys.stderr)
        return 1
    path = sys.argv[1]

    with open(path, "rb") as f:
        hdr = f.read(24)
        if len(hdr) != 24:
            print("ERROR: pcap too short", file=sys.stderr)
            return 1
        magic, vmaj, vmin, tz, sigfigs, snaplen, linktype = struct.unpack(
            "<IHHiIII", hdr
        )
        if magic == PCAP_MAGIC_NSEC:
            ts_unit = "ns"
        elif magic == PCAP_MAGIC_USEC:
            ts_unit = "us"
        else:
            print(f"ERROR: bad magic 0x{magic:08x}", file=sys.stderr)
            return 1

        print(f"PCAP: {path}")
        print(f"  version {vmaj}.{vmin}, linktype {linktype}, "
              f"snaplen {snaplen}, ts={ts_unit}")
        print()

        n = 0
        while True:
            rh = f.read(16)
            if not rh:
                break
            if len(rh) != 16:
                print("WARNING: truncated record header", file=sys.stderr)
                break
            ts_sec, ts_frac, incl_len, orig_len = struct.unpack("<IIII", rh)
            data = f.read(incl_len)
            if len(data) != incl_len:
                print("WARNING: truncated record data", file=sys.stderr)
                break
            n += 1
            if magic == PCAP_MAGIC_NSEC:
                t = ts_sec + ts_frac / 1e9
            else:
                t = ts_sec + ts_frac / 1e6
            summary = decode_802_15_4(data) if linktype == LINKTYPE_IEEE802_15_4_WITHFCS else f"({orig_len}B)"
            print(f"  {n:4d}  {t:11.6f}s  {summary}")

        print()
        print(f"  {n} frames")
    return 0


if __name__ == "__main__":
    sys.exit(main())
