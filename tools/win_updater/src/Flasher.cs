// Flasher.cs -- the TECHART HID bootloader protocol, as tools/ea9flash.py
// implements it (that script is the reference; its docstring has the
// derivation).  All frames are 64-byte reports.
//
//   host -> device                         device -> host
//   95 27 68 18 02 02            CHECK     95 27 68 18 02 02 vv vv vv ...
//   95 27 68 18 01 01 vv vv vv   START     95 27 68 18 01 01 ...
//   <64 raw firmware bytes>      DATA      any report = ACK, send next
//
// DATA is ACK-clocked, with no address, erase or CRC: the bootloader owns the
// destination.  The host pads with zeros up to the next 1 KiB boundary plus
// one more KiB, and stops after the chunk that reaches that limit.

using System;

namespace EA9Updater
{
    sealed class Flasher
    {
        public const ushort Vid = 0x0483;
        public const ushort Pid = 0x575A;

        const int Chunk = 64;
        const int TimeoutMs = 5000;

        static readonly byte[] Magic = { 0x95, 0x27, 0x68, 0x18 };
        static readonly byte[] CmdStart = { 0x01, 0x01 };
        static readonly byte[] CmdCheck = { 0x02, 0x02 };

        readonly IReportChannel dev;

        public Flasher(IReportChannel dev)
        {
            this.dev = dev;
        }

        // CHECK: the version triplet the bootloader has on record.
        public byte[] Check()
        {
            dev.Send(Command(CmdCheck, null));
            byte[] rep = Expect(CmdCheck);
            return new[] { rep[6], rep[7], rep[8] };
        }

        // START: arm the bootloader for a sequential write.  `version` is
        // what it records as installed -- bootloader bookkeeping, not the
        // version the camera reads (that is in message 0x07 of the image).
        public void Start(byte[] version)
        {
            dev.Send(Command(CmdStart, version));
            Expect(CmdStart);
        }

        // DATA: stream the image, one 64-byte chunk per ACK.
        // progress(sent, total) is called after every acknowledged chunk.
        public void Write(byte[] image, Action<int, int> progress)
        {
            int limit = image.Length / 1024 * 1024 + 1024;
            int total = limit + Chunk;               // the loop runs while pos <= limit
            for (int pos = 0; pos <= limit; pos += Chunk) {
                var chunk = new byte[Chunk];
                if (pos < image.Length) {
                    Array.Copy(image, pos, chunk, 0, Math.Min(Chunk, image.Length - pos));
                }
                dev.Send(chunk);
                if (dev.Receive(TimeoutMs) == null) {
                    throw new TimeoutException(string.Format(
                        "the device stopped answering at byte {0} of {1}", pos + Chunk, total));
                }
                if (progress != null) {
                    progress(pos + Chunk, total);
                }
            }
        }

        static byte[] Command(byte[] cmd, byte[] args)
        {
            int n = Magic.Length + cmd.Length + (args == null ? 0 : args.Length);
            var b = new byte[n];
            Magic.CopyTo(b, 0);
            cmd.CopyTo(b, Magic.Length);
            if (args != null) {
                args.CopyTo(b, Magic.Length + cmd.Length);
            }
            return b;
        }

        byte[] Expect(byte[] cmd)
        {
            byte[] rep = dev.Receive(TimeoutMs);
            if (rep == null) {
                throw new TimeoutException("no reply from the device");
            }
            for (int i = 0; i < Magic.Length; i++) {
                if (rep[i] != Magic[i]) {
                    throw new InvalidOperationException(
                        "unexpected reply from the device: " + Hex(rep, 8));
                }
            }
            if (rep[4] != cmd[0] || rep[5] != cmd[1]) {
                throw new InvalidOperationException(string.Format(
                    "reply for command {0:x2}{1:x2}, expected {2:x2}{3:x2}",
                    rep[4], rep[5], cmd[0], cmd[1]));
            }
            return rep;
        }

        static string Hex(byte[] b, int n)
        {
            return BitConverter.ToString(b, 0, Math.Min(n, b.Length)).Replace('-', ' ');
        }
    }
}
