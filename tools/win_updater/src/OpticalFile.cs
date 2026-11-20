// OpticalFile.cs -- the message 0x3B optical file (LF) of one slot, built
// from its focal length and the exit pupil the user chose.
//
// A port of build() in tools/gen_optical.py, which is the reference: the
// same 296 bytes for the same inputs (cfgblock.py checks the release image's
// defaults against it).  The layout is the firmware's optical_file[]: one
// zoom point, 8 focus points, aperture 0 plus the end marker, two focus rows
// per block (index 0 = closest, index 7 = infinity).
//
//   focus axis  index i at extension 4.5 mm * (7-i)/7, index 7 = 0x700;
//               distance codes exactly as src/focus_dist.c computes them
//   type 0      all six columns: XPD * 15 at infinity, (XPD + 4.5) * 15 closest
//   type 1      f/1.8 at the centre, f/1.8 * (6 - 0.06 f) at the edge, f
//               clamped to 30..70 mm; the shape of the Yongnuo 50 DF row
//   type 2      the same shape, rising a quarter as much
//   check word  XOR of every u32 LE word before it

using System;
using System.IO;

namespace EA9Updater
{
    static class OpticalFile
    {
        public const int Size = 296;

        // The exit pupil drop-down, mm, and the one preselected for a focal
        // length (gen_optical.py XPD_CHOICES, auto_xpd).
        public static readonly int[] XpdChoices = { 25, 30, 35, 40, 45, 50, 55, 60, 70 };

        public static int AutoXpd(int focalMm)
        {
            if (focalMm <= 30) return 30;
            if (focalMm <= 35) return 35;
            if (focalMm <= 40) return 40;
            if (focalMm <= 50) return 45;
            if (focalMm <= 60) return 50;
            return 60;
        }

        const int TravelUm = 4500;              // src/focus_dist.h FOCUS_TRAVEL_UM
        const int Nf = 8;
        const int Inf = 0x700;
        const int ApEnd = 2645;
        const int VigMin = 30, VigMax = 70;
        static readonly int Lc3 = (int)Math.Round(512 * Log2(1.8));         // 434 = f/1.8
        static readonly int[] YnT1 = { 382, 440, 528, 645, 803, 1008 };
        static readonly int[] YnT2 = { 382, 384, 397, 426, 473, 541 };

        static double Log2(double x) { return Math.Log(x, 2); }

        // src/focus_dist.c focus_log2_q6, bit for bit.
        static int Log2Q6(uint x)
        {
            if (x == 0) return 0;
            int e = 0;
            for (uint t = x; (t >>= 1) != 0; ) e++;
            uint m = e <= 14 ? x << (14 - e) : x >> (e - 14);
            int frac = 0;
            for (int i = 0; i < 7; i++) {
                m = (m * m) >> 14;
                frac <<= 1;
                if (m >= (2u << 14)) {
                    m >>= 1;
                    frac |= 1;
                }
            }
            return ((e << 7) + frac + 1) >> 1;
        }

        // src/focus_dist.c focus_distance_code, from an extension in um.
        static int DistCode(int xUm, int focalMm)
        {
            if (xUm == 0) return Inf;
            uint fUm = (uint)focalMm * 1000u;
            uint d = (uint)focalMm * (fUm + (uint)xUm) / (uint)xUm;
            int code = 384 + Log2Q6(d) - 638;
            return Math.Min(Math.Max(code, 0), Inf);
        }

        public static byte[] Build(int focalMm, int xpdMm)
        {
            if (focalMm <= 0 || xpdMm <= 0) throw new ArgumentOutOfRangeException();
            var t0Near = Fill((int)Math.Round((xpdMm + TravelUm / 1000.0) * 15));
            var t0Inf = Fill((int)Math.Round(xpdMm * 15.0));
            double k = 6 - 0.06 * Math.Min(Math.Max(focalMm, VigMin), VigMax);
            double rise = 512 * Log2(k);
            double span = YnT1[5] - YnT1[0];
            var t1 = new int[6];
            var t2 = new int[6];
            for (int c = 0; c < 6; c++) {
                t1[c] = (int)Math.Round(Lc3 + rise * ((YnT1[c] - YnT1[0]) / span));
                t2[c] = (int)Math.Round(Lc3 + rise * ((YnT2[c] - YnT2[0]) / span));
            }
            var endT = new[] { Fill(2646), Fill(2572) };

            var ms = new MemoryStream();
            var w = new BinaryWriter(ms);       // little-endian
            // LF header; its length is patched below.
            w.Write((byte)'L'); w.Write((byte)'F'); w.Write((ushort)16);
            w.Write(0u); w.Write(0x0bu); w.Write(0x9140u);
            // ED
            Chunk(w, "ED", 8 + 6 * 34);
            U16(w, 6, 1, Nf, 2);
            Block(w, t0Near, t0Inf); Block(w, t1, t1); Block(w, t2, t2);
            Block(w, t0Near, t0Inf); Block(w, endT[0], endT[1]); Block(w, endT[0], endT[1]);
            // CS
            Chunk(w, "CS", 8 + 2 * Nf + 4);
            U16(w, 1, Nf, 2, 0);
            for (int i = 0; i < Nf; i++) {
                int x = (TravelUm * (Nf - 1 - i) + (Nf - 1) / 2) / (Nf - 1);
                U16(w, DistCode(x, focalMm));
            }
            U16(w, 0, ApEnd);
            // LC
            Chunk(w, "LC", 12);
            U16(w, 1, 848, 1008, Lc3, 3195, 0);
            w.Write(0u);                        // check word, below
            byte[] f = ms.ToArray();
            if (f.Length != Size) throw new InvalidDataException("optical file size " + f.Length);
            BitConverter.GetBytes((uint)(Size - 16)).CopyTo(f, 4);
            uint cw = 0;
            for (int i = 0; i < Size - 4; i += 4) cw ^= BitConverter.ToUInt32(f, i);
            BitConverter.GetBytes(cw).CopyTo(f, Size - 4);
            return f;
        }

        static int[] Fill(int v) { return new[] { v, v, v, v, v, v }; }

        static void U16(BinaryWriter w, params int[] v)
        {
            foreach (int x in v) w.Write((ushort)x);
        }

        static void Chunk(BinaryWriter w, string tag, int len)
        {
            w.Write((byte)tag[0]); w.Write((byte)tag[1]); w.Write((ushort)8); w.Write((uint)len);
        }

        // One zoom point (0), two focus rows: index 0 (closest) and 7 (infinity).
        static void Block(BinaryWriter w, int[] near, int[] inf)
        {
            U16(w, 1, 2, 0, 0, Nf - 1);
            U16(w, near);
            U16(w, inf);
        }
    }
}
