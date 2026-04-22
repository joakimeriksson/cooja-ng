// JS application mote: periodic broadcaster + receive logger.
//
// Sends a small 802.15.4 data frame every 2 seconds and logs every
// frame it receives. Demonstrates the abstract-mote level — no real
// firmware, just script logic running against the same radio medium
// as the MSP430, ARM, and native motes.

const SEND_INTERVAL_NS = 2_000_000_000; // 2 s
const NS_PER_S         = 1_000_000_000;

let seq = 0;

function init() {
    mote.log("js mote " + mote.id + " starting");
    mote.scheduleWakeup(0); // first execute() at t=0
}

function execute(t_ns) {
    // Build a minimal IEEE 802.15.4 data frame.
    // Frame Control: data frame, no security, no PAN-ID compression,
    //                short src/dst addressing, intra-PAN.
    //   FCF lo = 0x41 (data, intra-PAN)
    //   FCF hi = 0x88 (dst short, src short)
    //   seq    = seq++
    //   dst PAN = 0xABCD
    //   dst addr = 0xFFFF (broadcast)
    //   src addr = mote.id (16-bit)
    // Payload: "JS<id>:<seq>"
    const text = "JS" + mote.id + ":" + seq;
    const hdr  = [0x41, 0x88, seq & 0xFF,
                  0xCD, 0xAB,           // dst PAN (LE)
                  0xFF, 0xFF,           // dst addr (broadcast)
                  mote.id & 0xFF, (mote.id >> 8) & 0xFF];
    const bytes = new Uint8Array(hdr.length + text.length + 2 /* placeholder FCS */);
    bytes.set(hdr, 0);
    for (let i = 0; i < text.length; i++)
        bytes[hdr.length + i] = text.charCodeAt(i);
    // FCS bytes are placeholder — host PHY layer recomputes/ignores.
    bytes[bytes.length - 2] = 0;
    bytes[bytes.length - 1] = 0;

    mote.send(bytes);
    mote.log("tx seq=" + seq + " t=" + (t_ns / NS_PER_S).toFixed(3) + "s");
    seq++;

    mote.scheduleWakeup(SEND_INTERVAL_NS);
}

function receivedPacket(bytes) {
    // Parse src addr (offset 7-8) and trailing payload as ASCII.
    if (bytes.length < 11) return;
    const src = bytes[7] | (bytes[8] << 8);
    let text = "";
    for (let i = 9; i < bytes.length - 2; i++) {
        const c = bytes[i];
        text += (c >= 0x20 && c < 0x7F) ? String.fromCharCode(c) : ".";
    }
    mote.log("rx from=" + src + " len=" + bytes.length + " text='" + text + "'");
}
