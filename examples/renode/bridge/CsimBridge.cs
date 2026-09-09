//
// CsimBridge — joins Renode's 802.15.4 medium to csim's radio medium.
//
// Renode models the radio chip; csim models the air. This class is the seam:
// a Renode radio that owns no hardware, sitting on Renode's wireless medium,
// forwarding every frame it hears into csim and re-emitting every frame csim
// delivers back.
//
// It carries the frames over the co-simulation register window csim already
// exposes (include/native/renode_dev.h) rather than a socket of its own, so
// there is no second protocol: the same peripheral that keeps the two clocks
// in lockstep also carries the traffic. Nothing in csim changes.
//
// The bridge also has to supply the one thing the two models disagree about:
// AIR TIME. Renode's radios are frame-level -- a frame is delivered the
// instant it is sent -- while csim models the PHY at 32 us per byte, so a
// 100-byte frame occupies 3.4 ms of its medium. A bridge that forwarded
// frames as fast as Renode emitted them would start a second transmission on
// top of one still on the air; measured, that loses the second frame AND the
// first frame's acknowledgement (auto_ack drops from 1 to 0), which is enough
// to stop RPL from ever completing a DAO. So outgoing frames are queued and
// released no faster than the air can carry them.
//
//   Renode CC2538 ── wireless medium ── CsimBridge ── sysbus ── csim window
//                                                                  │
//                                                        csim's UDGM medium
//                                                          (Sky, nRF, …)
//
// Load it with `include @examples/renode/bridge/CsimBridge.cs`, then declare
//   bridge: Wireless.CsimBridge @ sysbus <0x50000000, +0x10> { csimBase: 0x40100000 }
// and `connector Connect bridge wireless`.
//
// See docs/design/renode-cosim-plan.md.
//
using System;
using System.Collections.Generic;
using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Peripherals.Timers;
using Antmicro.Renode.Time;

namespace Antmicro.Renode.Peripherals.Wireless
{
    public class CsimBridge : IDoubleWordPeripheral, IKnownSize, IRadio
    {
        public CsimBridge(IMachine machine, ulong csimBase = 0x40100000,
                          uint pollMicroseconds = 100, int channel = 26,
                          bool logFrames = false, bool appendFcs = true)
        {
            this.machine = machine;
            this.csimBase = csimBase;
            this.pollMicroseconds = pollMicroseconds;
            this.logFrames = logFrames;
            this.appendFcs = appendFcs;
            this.channel = channel;

            // A LimitTimer on the machine's clock, not machine.ScheduleAction:
            // the action scheduled from Reset() never fires, because Reset runs
            // before the machine's clock exists. This is the same mechanism
            // Renode's own co-simulation connection uses to pace itself.
            pollTimer = new LimitTimer(machine.ClockSource, 1000000, this, "csim-poll",
                                       pollMicroseconds, enabled: true,
                                       eventEnabled: true, autoUpdate: true);
            pollTimer.LimitReached += OnPollTick;
        }

        public long Size => 0x20;

        // --- IRadio ---------------------------------------------------------

        public event Action<IRadio, byte[]> FrameSent;

        public int Channel
        {
            get { return channel; }
            set { channel = value; }
        }

        // A Renode radio transmitted and the medium delivered it here. Hand it
        // straight to csim, which owns propagation from this point on: csim's
        // medium decides who hears it, at what RSSI, and when.
        public void ReceiveFrame(byte[] frame, IRadio sender)
        {
            if(frame == null || frame.Length == 0)
            {
                return;
            }

            // Renode's radios transmit the frame with its FCS; csim's medium
            // appends its own, so strip the trailing two bytes. Length is
            // checked rather than assumed -- a frame too short to carry an FCS
            // is passed through untouched instead of being truncated to junk.
            var length = frame.Length;
            if(appendFcs && length > 2)
            {
                length -= 2;
            }
            if(length > MaxFrame)
            {
                this.Log(LogLevel.Warning, "Frame of {0} B exceeds csim's {1} B limit, dropping",
                         length, MaxFrame);
                return;
            }

            // One-shot self-check: recompute the FCS Renode just transmitted.
            // If it disagrees, the frames this bridge injects the other way are
            // being rejected for the same reason, which is otherwise invisible --
            // they simply never arrive.
            if(!fcsChecked && appendFcs && frame.Length > 2)
            {
                fcsChecked = true;
                var expected = Crc16(frame, length);
                var actual = (ushort)(frame[length] | (frame[length + 1] << 8));
                if(expected != actual)
                {
                    this.Log(LogLevel.Warning,
                             "FCS mismatch: Renode sent 0x{0:X4}, this bridge computes 0x{1:X4} "
                             + "-- injected frames will be dropped by the receiver",
                             actual, expected);
                }
                else
                {
                    this.Log(LogLevel.Info, "FCS check OK (0x{0:X4})", actual);
                }
            }

            if(logFrames)
            {
                this.Log(LogLevel.Info, "renode -> csim  {0} B  {1}", length, Hex(frame, length));
            }

            // An acknowledgement is a reply to a frame csim just delivered, so it
            // is not paced like data -- the medium is free by definition -- but
            // it must not go on the air before the transmitter has turned
            // around to receive. 802.15.4 gives that 192 us; csim's own auto-ACK
            // path waits exactly that (sim_radio_bus.c), and the Sky's CC2420
            // drops every byte that arrives before it is back in RX. Renode's
            // radio has no air time, so its ACK is ready the same instant the
            // frame is; hold it in its own slot until the turnaround has passed.
            var isAck = (frame[0] & 0x7) == 2;
            if(isAck)
            {
                pendingAck = new byte[length];
                Array.Copy(frame, pendingAck, length);
                pendingAckNotBeforeNs = lastRxEndNs + AckTurnaroundNs;
                TrySend();
                return;
            }

            // Queue rather than inject: the air may still be busy with a frame
            // Renode considers long finished. TrySend() releases it.
            if(outgoing.Count >= MaxBacklog)
            {
                // Renode is producing faster than the medium can ever carry.
                // Dropping the newest keeps the queue a prefix of the truth,
                // and a real radio would have failed its CCA here anyway.
                droppedBacklog++;
                return;
            }
            var payload = new byte[length];
            Array.Copy(frame, payload, length);
            outgoing.Enqueue(payload);
            TrySend();
        }

        // On-air duration csim will spend on a MAC frame of `length` bytes:
        // 4 preamble + SFD + PHR + payload + 2 FCS, at 32 us per byte
        // (250 kbps O-QPSK). Plus, when the frame asks for one, the
        // acknowledgement that must follow it: 192 us turnaround and an
        // 11-byte ACK. Releasing the next frame before that lands is what
        // silently destroys the acknowledgement.
        private static ulong AirTimeNs(byte[] frame)
        {
            ulong bytes = (ulong)frame.Length + 8;
            ulong ns = bytes * ByteNs;
            var ackRequested = frame.Length > 0 && (frame[0] & 0x20) != 0;
            if(ackRequested)
            {
                ns += AckTurnaroundNs + 11 * ByteNs;
            }
            return ns;
        }

        private ulong CsimNowNs()
        {
            return ((ulong)Read(RegSimTimeHi) << 32) | Read(RegSimTimeLo);
        }

        // Release one queued frame if the air is clear. Called from the poll
        // tick, so a backlog drains at the medium's own rate.
        private void TrySend()
        {
            var now = CsimNowNs();

            if(pendingAck != null)
            {
                if(now < pendingAckNotBeforeNs)
                {
                    deferredAck++;
                    return;
                }
                WriteFrame(pendingAck);
                // The acknowledgement occupies the air like any frame -- 11
                // bytes at 32 us -- and nothing else keeps queued data off it:
                // the CCA register walks *neighbours*, so this bridge's own
                // transmission never reads as busy. Reserve the ACK's air time
                // and let the next poll release the data behind it.
                mediumFreeNs = now + AirTimeNs(pendingAck);
                pendingAck = null;
                sentToCsim++;
                return;
            }

            if(outgoing.Count == 0)
            {
                return;
            }
            if(now < mediumFreeNs)
            {
                deferred++;
                return;
            }

            // Carrier sense. Renode's radio model has no air time, so its CSMA
            // ran CCA against Renode's own medium -- which holds only this
            // bridge -- and saw an idle channel while a csim node was mid-frame.
            // Injecting there does not read as a collision: the receiver's radio
            // is busy with the first frame, drops the opening bytes of the
            // second, then locks SFD partway through it and fails CRC. Ask
            // csim's medium instead.
            if(Read(RegCca) != 0)
            {
                deferredCca++;
                return;
            }

            var frame = outgoing.Dequeue();
            WriteFrame(frame);
            mediumFreeNs = now + AirTimeNs(frame);
            sentToCsim++;
        }

        private void WriteFrame(byte[] frame)
        {
            Write(RegTxLen, (uint)frame.Length);
            for(var i = 0; i < frame.Length; i++)
            {
                machine.SystemBus.WriteByte(csimBase + RegTxData, frame[i]);
            }
            Write(RegTxCtrl, 1);
        }

        // --- polling --------------------------------------------------------

        // Every access to the window is a synchronous round trip to the csim
        // process, so this reads one register and stops when the queue is
        // empty. The poll period is the latency budget for an acknowledgement
        // crossing back, so it wants to be at or below the co-simulation
        // quantum -- see the plan's note on ACK timing.
        private void OnPollTick()
        {
            if(!started)
            {
                var id = Read(RegId);
                if(id != CsimMagic)
                {
                    this.Log(LogLevel.Error,
                             "No csim device at 0x{0:X}: read 0x{1:X8}, expected 0x{2:X8}. "
                             + "Is csim running with --renode?", csimBase, id, CsimMagic);
                    pollTimer.Enabled = false;
                    return;
                }
                started = true;
                this.Log(LogLevel.Info, "csim bridge up: node {0} of {1}, channel {2}",
                         (int)Read(RegSelfId), Read(RegNodeCount), channel);
                Write(RegChannel, (uint)channel);
            }

            // Release anything the air was too busy for last tick.
            TrySend();

            while(Read(RegRxCount) > 0)
            {
                var length = (int)Read(RegRxLen);
                if(length <= 0 || length > MaxFrame)
                {
                    Write(RegRxPop, 1);
                    continue;
                }

                var payload = new byte[appendFcs ? length + 2 : length];
                for(var i = 0; i < length; i++)
                {
                    payload[i] = machine.SystemBus.ReadByte(csimBase + RegRxData);
                }
                var from = (int)Read(RegRxFrom);
                var startNs = ((ulong)Read(RegRxTimeHi) << 32) | Read(RegRxTimeLo);
                Write(RegRxPop, 1);
                // On-air end of this frame: what an acknowledgement of it must
                // be timed against. 4 preamble + SFD + PHR + payload + 2 FCS.
                lastRxEndNs = startNs + ((ulong)length + 8) * ByteNs;

                // csim hands over the MAC frame without an FCS; Renode's radios
                // expect one and the CC2538 model checks it, so recompute rather
                // than pad with zeroes.
                if(appendFcs)
                {
                    var crc = Crc16(payload, length);
                    payload[length] = (byte)(crc & 0xFF);
                    payload[length + 1] = (byte)(crc >> 8);
                }

                if(logFrames)
                {
                    this.Log(LogLevel.Info, "csim -> renode  {0} B  from node {1}  {2}",
                             length, from, Hex(payload, payload.Length));
                }

                receivedFromCsim++;
                var handler = FrameSent;
                if(handler != null)
                {
                    // Emitting as if this bridge had transmitted is what puts the
                    // frame on Renode's medium; the medium then delivers it to
                    // every other radio, which is exactly what a csim node's
                    // transmission should look like from Renode's side.
                    handler(this, payload);
                }
            }
        }

        // --- IPeripheral ----------------------------------------------------

        public void Reset()
        {
            sentToCsim = 0;
            receivedFromCsim = 0;
            outgoing.Clear();
            mediumFreeNs = 0;
            deferred = 0;
            deferredCca = 0;
            deferredAck = 0;
            pendingAck = null;
            droppedBacklog = 0;
        }

        public uint ReadDoubleWord(long offset)
        {
            switch(offset)
            {
                case 0x0: return (uint)sentToCsim;
                case 0x4: return (uint)receivedFromCsim;
                case 0x8: return (uint)deferred;
                case 0x10: return (uint)deferredCca;
                case 0x14: return (uint)deferredAck;
                case 0xC: return (uint)droppedBacklog;
                default: return 0;
            }
        }

        public void WriteDoubleWord(long offset, uint value)
        {
        }

        // --- helpers --------------------------------------------------------

        private uint Read(ulong offset)
        {
            return machine.SystemBus.ReadDoubleWord(csimBase + offset);
        }

        private void Write(ulong offset, uint value)
        {
            machine.SystemBus.WriteDoubleWord(csimBase + offset, value);
        }

        // 802.15.4 FCS: CRC-16/KERMIT -- polynomial x^16+x^12+x^5+1 processed
        // least-significant-bit first (reflected 0x8408), initial value 0. Not
        // the MSB-first CCITT variant, which is the easy mistake here and
        // produces a checksum every receiver silently rejects.
        private static ushort Crc16(byte[] data, int length)
        {
            ushort crc = 0;
            for(var i = 0; i < length; i++)
            {
                crc ^= data[i];
                for(var bit = 0; bit < 8; bit++)
                {
                    crc = (ushort)(((crc & 1) != 0) ? ((crc >> 1) ^ 0x8408) : (crc >> 1));
                }
            }
            return crc;
        }

        private static string Hex(byte[] data, int length)
        {
            var chars = new char[length * 2];
            for(var i = 0; i < length; i++)
            {
                chars[i * 2] = HexDigits[data[i] >> 4];
                chars[i * 2 + 1] = HexDigits[data[i] & 0xF];
            }
            return new string(chars);
        }

        private readonly IMachine machine;
        private readonly ulong csimBase;
        private readonly uint pollMicroseconds;
        private readonly bool logFrames;
        private readonly bool appendFcs;
        // Frames from Renode waiting for the air to clear, and the csim time
        // at which it does.
        private readonly Queue<byte[]> outgoing = new Queue<byte[]>();
        private ulong mediumFreeNs;
        private int deferred;
        private int deferredCca;
        private int deferredAck;
        private byte[] pendingAck;
        private ulong pendingAckNotBeforeNs;
        private ulong lastRxEndNs;
        private int droppedBacklog;

        private readonly LimitTimer pollTimer;
        private int channel;
        private bool started;
        private bool fcsChecked;
        private int sentToCsim;
        private int receivedFromCsim;

        private const int MaxFrame = 152;
        // 802.15.4 at 250 kbps: 32 us per byte, 192 us (12 symbols) turnaround.
        private const ulong ByteNs = 32000;
        private const ulong AckTurnaroundNs = 192000;
        private const int MaxBacklog = 8;
        private const uint CsimMagic = 0x4353494D;   // "CSIM"

        private const ulong RegId = 0x00;
        private const ulong RegChannel = 0x10;
        private const ulong RegTxLen = 0x18;
        private const ulong RegTxData = 0x1C;
        private const ulong RegTxCtrl = 0x20;
        private const ulong RegRxCount = 0x28;
        private const ulong RegRxLen = 0x2C;
        private const ulong RegRxData = 0x30;
        private const ulong RegRxPop = 0x34;
        private const ulong RegRxFrom = 0x3C;
        private const ulong RegRxTimeLo = 0x44;
        private const ulong RegRxTimeHi = 0x48;
        private const ulong RegSelfId = 0x5C;
        private const ulong RegSimTimeLo = 0x50;
        private const ulong RegSimTimeHi = 0x54;
        private const ulong RegCca = 0x70;
        private const ulong RegNodeCount = 0x58;

        private const string HexDigits = "0123456789abcdef";
    }
}
