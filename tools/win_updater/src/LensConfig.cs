// LensConfig.cs -- the lens configuration block the firmware reads at boot,
// built from what the user entered.  spec.md section 5.
//
// The layout is struct lens_cfg in src/lens_cfg.h (540 bytes, little-endian):
//
//   +0    magic[10]     "CfgAnchor" + NUL
//   +10   hw_version    1 or 2
//   +11   pad           0
//   +12   lens[8]       66 bytes each: focal_mm u16 (0 = unset), name[64]
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
    }

    sealed class LensConfig
    {
        public static readonly byte[] Magic = Encoding.ASCII.GetBytes("CfgAnchor\0");
        public const int SlotCount = 8;
        public const int NameField = 64;        // 63 characters + NUL
        public const int NameMax = NameField - 1;
        public const int SlotSize = 2 + NameField;
        public const int HeaderSize = 12;
        public const int Size = HeaderSize + SlotCount * SlotSize;   // 540

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

        // What one row of the form holds: the focal text as typed, and the name.
        public struct Entry
        {
            public string Focal;
            public string Name;
            public Entry(string focal, string name) { Focal = focal; Name = name; }
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
                error = "Choose a hardware version.";
                return null;
            }
            if (single ? entries.Count != 1 : entries.Count != SlotCount) {
                throw new ArgumentException("entries");
            }

            var slots = new LensSlot[SlotCount];
            for (int i = 0; i < SlotCount; i++) {
                slots[i] = new LensSlot { FocalMm = 0, Name = "" };
            }
            for (int i = 0; i < entries.Count; i++) {
                string where = single ? "" : string.Format(" in slot {0}", i + 1);
                string ft = (entries[i].Focal ?? "").Trim();
                if (ft.Length == 0) {
                    continue;
                }
                int mm;
                if (!int.TryParse(ft, out mm) || mm < 0) {
                    error = "The focal length" + where + " must be a whole number of millimetres.";
                    return null;
                }
                if (mm == 0) {
                    continue;
                }
                if (mm < FocalMinMm || mm > FocalMaxMm) {
                    error = string.Format("The focal length{0} must be from {1} to {2} mm.",
                                          where, FocalMinMm, FocalMaxMm);
                    return null;
                }
                string name = (entries[i].Name ?? "").Trim();
                if (name.Length == 0) {
                    name = DefaultName;
                }
                string why = CheckName(name);
                if (why != null) {
                    error = why.Replace("The lens name", "The lens name" + where);
                    return null;
                }
                slots[i].FocalMm = mm;
                slots[i].Name = name;
            }

            var set = Enumerable.Range(0, SlotCount).Where(i => slots[i].FocalMm != 0).ToList();
            if (set.Count == 0) {
                error = single
                    ? "No lens is configured: enter a focal length."
                    : "No lens is configured: enter the focal length of at least one slot.";
                return null;
            }
            // With several set, slot 0 may be empty: the firmware falls back
            // to the first set slot (lens_cfg.h).
            if (set.Count == 1 && set[0] != 0) {
                slots[0] = slots[set[0]];
                slots[set[0]] = new LensSlot { FocalMm = 0, Name = "" };
            }
            return new LensConfig(hwVersion, slots);
        }

        // Null if the name is acceptable, else why not.
        public static string CheckName(string name)
        {
            if (name.Length > NameMax) {
                return string.Format("The lens name can be at most {0} characters.", NameMax);
            }
            if (name.Any(c => c < 0x20 || c > 0x7E)) {
                return "The lens name can only use English letters, digits, spaces and common symbols.";
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
            }
            return b;
        }

        // One line per slot, for the confirmation dialog.
        public string Describe()
        {
            var sb = new StringBuilder();
            sb.AppendFormat("Adapter version: {0}\n", HwVersion);
            var set = Slots.Where(s => s.FocalMm != 0).ToList();
            if (set.Count == 1) {
                sb.AppendFormat("Single lens: {0} mm, \"{1}\"\n", Slots[0].FocalMm, Slots[0].Name);
            } else {
                for (int i = 0; i < SlotCount; i++) {
                    if (Slots[i].FocalMm != 0) {
                        sb.AppendFormat("Slot {0} ({1}): {2} mm, \"{3}\"\n",
                                        i + 1, SlotAperture[i], Slots[i].FocalMm, Slots[i].Name);
                    }
                }
            }
            return sb.ToString();
        }
    }
}
