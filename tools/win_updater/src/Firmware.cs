// Firmware.cs -- the embedded release image and the lens configuration block
// in it.
//
// The firmware keeps its whole configuration in one struct that starts with
// the "CfgAnchor" magic (src/lens_cfg.h).  The updater finds it by that magic
// and overwrites it in place; nothing else in the image is touched.  spec.md
// section 5.

using System;
using System.IO;
using System.Reflection;
using System.Text;

namespace EA9Updater
{
    sealed class Firmware
    {
        // The version START declares to the bootloader (what ea9flash.py
        // --check reports afterwards).  Keep it in step with message 0x07.
        public static readonly byte[] DeclaredVersion = { 2, 4, 0 };

        readonly byte[] image;
        readonly int cfgOffset;

        // Finds the configuration block and checks it looks like one.  Throws
        // if not.
        public Firmware(byte[] image)
        {
            this.image = image;
            cfgOffset = FindConfig(image);
        }

        public static Firmware LoadEmbedded()
        {
            return new Firmware(Resource("firmware.bin"));
        }

        // The release notes in the current language (S.Zh).
        public static string LoadChangelog()
        {
            return Encoding.UTF8.GetString(Resource(S.Zh ? "changelog.zh.txt" : "changelog.txt"));
        }

        // A copy of the image with `cfg` written over the block.
        public byte[] Patch(LensConfig cfg)
        {
            byte[] block = cfg.Encode();
            var outImage = (byte[])image.Clone();
            block.CopyTo(outImage, cfgOffset);
            return outImage;
        }

        // The offset of the one block whose magic, version and padding are
        // right.  The magic must occur exactly once.
        static int FindConfig(byte[] image)
        {
            byte[] m = LensConfig.Magic;
            int found = -1;
            for (int i = 0; i + m.Length <= image.Length; i++) {
                int k = 0;
                while (k < m.Length && image[i + k] == m[k]) {
                    k++;
                }
                if (k < m.Length) {
                    continue;
                }
                if (found >= 0) {
                    throw new InvalidDataException("the firmware holds the configuration magic more than once");
                }
                found = i;
            }
            if (found < 0) {
                throw new InvalidDataException("the firmware has no lens configuration block");
            }
            if (found + LensConfig.Size > image.Length) {
                throw new InvalidDataException("the lens configuration block runs past the end of the firmware");
            }
            byte hw = image[found + 10];
            if (Array.IndexOf(LensConfig.HwVersions, (int)hw) < 0 || image[found + 11] != 0) {
                throw new InvalidDataException(string.Format(
                    "the lens configuration block at 0x{0:x} has an unexpected header", found));
            }
            for (int i = 0; i < LensConfig.SlotCount; i++) {
                int name = found + LensConfig.HeaderSize + i * LensConfig.SlotSize + 2;
                if (image[name + LensConfig.NameField - 1] != 0) {
                    throw new InvalidDataException(string.Format(
                        "the lens configuration block at 0x{0:x} has an unterminated name", found));
                }
            }
            return found;
        }

        public static byte[] Resource(string name)
        {
            using (var s = Assembly.GetExecutingAssembly().GetManifestResourceStream(name)) {
                if (s == null) {
                    throw new InvalidDataException("missing embedded resource: " + name);
                }
                using (var ms = new MemoryStream()) {
                    s.CopyTo(ms);
                    return ms.ToArray();
                }
            }
        }
    }
}
