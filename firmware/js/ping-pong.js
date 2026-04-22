// JS application mote: ping/pong with RTT measurement.
//
// The lowest-id node is the pinger. It sends a PING frame, records the
// send time, and waits for a PONG echo carrying the same sequence
// number. Other nodes are responders: on PING they immediately reply
// with PONG.
//
// This single script implements both roles. The role is chosen by
// mote.id, so launching N copies (1..N) yields one pinger and N-1
// responders, all running the same code.

const PING_INTERVAL_NS = 1_000_000_000; // 1 s between pings
const NS_PER_MS        = 1_000_000;
const NS_PER_S         = 1_000_000_000;

const PINGER_ID        = 1;
const TYPE_PING        = 0xA0;
const TYPE_PONG        = 0xA1;

// 802.15.4 header for a broadcast data frame.
const FCF_LO = 0x41, FCF_HI = 0x88;
const PAN_LO = 0xCD, PAN_HI = 0xAB;
const BCAST_LO = 0xFF, BCAST_HI = 0xFF;

let seq           = 0;
let pendingSeq    = -1;
let pendingTxNs   = 0;
let stats         = { sent: 0, received: 0, totalRttMs: 0 };

function buildFrame(type, payloadByte) {
    const f = new Uint8Array(13);
    f[0]  = FCF_LO;  f[1]  = FCF_HI;
    f[2]  = seq & 0xFF;
    f[3]  = PAN_LO;  f[4]  = PAN_HI;
    f[5]  = BCAST_LO; f[6] = BCAST_HI;
    f[7]  = mote.id & 0xFF; f[8] = (mote.id >> 8) & 0xFF;
    f[9]  = type;
    f[10] = payloadByte;
    // FCS placeholder
    f[11] = 0; f[12] = 0;
    return f;
}

function init() {
    if (mote.id === PINGER_ID) {
        mote.log("pinger: starting (interval=" +
                 (PING_INTERVAL_NS / NS_PER_MS) + " ms)");
        mote.scheduleWakeup(NS_PER_S / 2); // first ping at t=0.5s
    } else {
        mote.log("responder: ready");
    }
}

function execute(t_ns) {
    if (mote.id !== PINGER_ID) return;

    pendingSeq  = seq;
    pendingTxNs = t_ns;
    mote.send(buildFrame(TYPE_PING, seq & 0xFF));
    mote.log("ping seq=" + seq + " sent");
    seq++;
    stats.sent++;

    if (stats.received > 0) {
        const avgMs = stats.totalRttMs / stats.received;
        mote.log("stats: sent=" + stats.sent +
                 " received=" + stats.received +
                 " avg-rtt=" + avgMs.toFixed(3) + " ms");
    }

    mote.scheduleWakeup(PING_INTERVAL_NS);
}

function receivedPacket(bytes) {
    if (bytes.length < 11) return;
    const src  = bytes[7] | (bytes[8] << 8);
    const type = bytes[9];
    const sn   = bytes[10];

    if (type === TYPE_PING && mote.id !== PINGER_ID) {
        // Echo back as PONG with the same seq.
        mote.log("ping from " + src + " seq=" + sn + " -> pong");
        mote.send(buildFrame(TYPE_PONG, sn));
        seq++;
        return;
    }

    if (type === TYPE_PONG && mote.id === PINGER_ID && sn === (pendingSeq & 0xFF)) {
        const rttNs = mote.now() - pendingTxNs;
        const rttMs = rttNs / NS_PER_MS;
        stats.received++;
        stats.totalRttMs += rttMs;
        mote.log("pong from " + src + " seq=" + sn +
                 " rtt=" + rttMs.toFixed(3) + " ms");
        pendingSeq = -1;
    }
}
