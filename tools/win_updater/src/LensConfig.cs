// LensConfig.cs -- the lens configuration block the firmware reads at boot,
// built from what the user entered.  spec.md section 5.
//
// The layout is struct lens_cfg in src/lens_cfg.h (2908 bytes, little-endian):
//
//   +0    magic[10]     "CfgAnchor" + NUL
//   +10   hw_version    1 or 2
//   +11   pad           0
//   +12   lens[8]       362 bytes each: focal_mm u16 (0 = unset), name[64],
//                       optical[296] (OpticalFile.Build; all zero when unset)
//
// Encode() must stay identical to pack() in cfgblock.py.

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace EA9Updater
{
    sealed class LensSlot
    {
        public int FocalMm;     // 0 = unset
        public string Name;     // "" when unset
        public int XpdMm;       // exit pupil, mm; 0 when unset
    }

    sealed class LensConfig
    {
        public static readonly byte[] Magic = Encoding.ASCII.GetBytes("CfgAnchor\0");
        public const int SlotCount = 8;
        public const int NameField = 64;        // 63 characters + NUL
        public const int NameMax = NameField - 1;
        public const int SlotSize = 2 + NameField + OpticalFile.Size;   // 362
        public const int HeaderSize = 12;
        public const int Size = HeaderSize + SlotCount * SlotSize;   // 2908

        public const int FocalMinMm = 8;
        public const int FocalMaxMm = 600;
        public const string DefaultName = "Community EA9 Adapter";
        public const int DefaultSingleFocalMm = 50;
        public static readonly int[] HwVersions = { 2, 1 };          // the first is the default

        // The aperture that selects each slot on the camera (lens_cfg.c AP_SEL).
        public static readonly string[] SlotAperture =
            { "f2.5", "f2.8", "f3.2", "f3.5", "f4.0", "f4.5", "f5.0", "f5.6" };

        public int HwVersion;
        public readonly LensSlot[] Slots;

        LensConfig(int hwVersion, LensSlot[] slots)
        {
            HwVersion = hwVersion;
            Slots = slots;
        }

        // What one row of the form holds: the focal text as typed, the name,
        // and the exit pupil chosen (mm, one of OpticalFile.XpdChoices).
        public struct Entry
        {
            public string Focal;
            public string Name;
            public int Xpd;
            public Entry(string focal, string name, int xpd) { Focal = focal; Name = name; Xpd = xpd; }
        }

        // Builds the block from the form, or returns null and says why.
        //   single: entries has one element, which becomes slot 0.
        //   empty focal -> 0 and no name; focal set but no name -> DefaultName;
        //   exactly one slot set -> it moves to slot 0; none set -> error.
        //   Several set: they stay where they are, slot 0 may be empty.
        public static LensConfig FromForm(int hwVersion, bool single, IList<Entry> entries,
                                          out string error)
        {
            error = null;
            if (!HwVersions.Contains(hwVersion)) {
                error = S.ChooseHwVersion;
                return null;
            }
            if (single ? entries.Count != 1 : entries.Count != SlotCount) {
                throw new ArgumentException("entries");
            }

            var slots = new LensSlot[SlotCount];
            for (int i = 0; i < SlotCount; i++) {
                slots[i] = new LensSlot { FocalMm = 0, Name = "", XpdMm = 0 };
            }
            for (int i = 0; i < entries.Count; i++) {
                string where = single ? "" : string.Format(S.InSlot, SlotAperture[i]);
                string ft = (entries[i].Focal ?? "").Trim();
                if (ft.Length == 0) {
                    continue;
                }
                int mm;
                if (!int.TryParse(ft, out mm) || mm < 0) {
                    error = string.Format(S.FocalWhole, where);
                    return null;
                }
                if (mm == 0) {
                    continue;
                }
                if (mm < FocalMinMm || mm > FocalMaxMm) {
                    error = string.Format(S.FocalRange, where, FocalMinMm, FocalMaxMm);
                    return null;
                }
                string name = (entries[i].Name ?? "").Trim();
                if (name.Length == 0) {
                    name = DefaultName;
                }
                error = CheckName(name, where);
                if (error != null) {
                    return null;
                }
                if (Array.IndexOf(OpticalFile.XpdChoices, entries[i].Xpd) < 0) {
                    error = string.Format(S.ChooseXpd, where);
                    return null;
                }
                slots[i].FocalMm = mm;
                slots[i].Name = name;
                slots[i].XpdMm = entries[i].Xpd;
            }

            var set = Enumerable.Range(0, SlotCount).Where(i => slots[i].FocalMm != 0).ToList();
            if (set.Count == 0) {
                error = single
                    ? S.NoLensSingle
                    : S.NoLensMultiple;
                return null;
            }
            // With several set, slot 0 may be empty: the firmware falls back
            // to the first set slot (lens_cfg.h).
            if (set.Count == 1 && set[0] != 0) {
                slots[0] = slots[set[0]];
                slots[set[0]] = new LensSlot { FocalMm = 0, Name = "", XpdMm = 0 };
            }
            return new LensConfig(hwVersion, slots);
        }

        // Null if the name is acceptable, else why not.  where: S.InSlot or "".
        public static string CheckName(string name, string where)
        {
            if (name.Length > NameMax) {
                return string.Format(S.NameTooLong, where, NameMax);
            }
            if (name.Any(c => c < 0x20 || c > 0x7E)) {
                return string.Format(S.NameAscii, where);
            }
            return null;
        }

        public byte[] Encode()
        {
            var b = new byte[Size];
            Magic.CopyTo(b, 0);
            b[10] = (byte)HwVersion;
            b[11] = 0;
            for (int i = 0; i < SlotCount; i++) {
                int o = HeaderSize + i * SlotSize;
                b[o] = (byte)(Slots[i].FocalMm & 0xFF);
                b[o + 1] = (byte)(Slots[i].FocalMm >> 8);
                Encoding.ASCII.GetBytes(Slots[i].Name).CopyTo(b, o + 2);   // NUL-padded
                if (Slots[i].FocalMm != 0) {
                    OpticalFile.Build(Slots[i].FocalMm, Slots[i].XpdMm).CopyTo(b, o + 2 + NameField);
                }
            }
            return b;
        }

        // One line per slot, for the confirmation dialog.
        public string Describe()
        {
            var sb = new StringBuilder();
            sb.AppendFormat(S.DescAdapter + "\n", HwVersion);
            var set = Slots.Where(s => s.FocalMm != 0).ToList();
            if (set.Count == 1) {
                sb.AppendFormat(S.DescSingle + "\n", Slots[0].FocalMm, Slots[0].XpdMm, Slots[0].Name);
            } else {
                for (int i = 0; i < SlotCount; i++) {
                    if (Slots[i].FocalMm != 0) {
                        sb.AppendFormat(S.DescSlot + "\n",
                                        i + 1, SlotAperture[i], Slots[i].FocalMm, Slots[i].XpdMm,
                                        Slots[i].Name);
                    }
                }
            }
            return sb.ToString();
        }
    }
}
