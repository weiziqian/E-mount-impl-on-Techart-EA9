// Firmware.cs -- the embedded release image, its patch table, and the patch.
//
// locate.py builds the release image three times and records every place the
// focal length and the lens name land in it (patchmap.json, spec.md section
// 5).  The encodings below must stay identical to encode() in locate.py.

using System;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Runtime.Serialization;
using System.Runtime.Serialization.Json;
using System.Security.Cryptography;
using System.Text;

namespace EA9Updater
{
    // Filled in by the JSON serializer, which the compiler cannot see.
#pragma warning disable 0649
    [DataContract]
    sealed class PatchField
    {
        [DataMember(Name = "offset")] public int Offset;
        [DataMember(Name = "kind")] public string Kind;
        [DataMember(Name = "len")] public int Len;
    }

    [DataContract]
    sealed class PatchDefaults
    {
        [DataMember(Name = "focal_mm")] public int FocalMm;
        [DataMember(Name = "name")] public string Name;
    }

    [DataContract]
    sealed class PatchMap
    {
        [DataMember(Name = "image_len")] public int ImageLen;
        [DataMember(Name = "image_sha256")] public string ImageSha256;
        [DataMember(Name = "default")] public PatchDefaults Default;
        [DataMember(Name = "fields")] public PatchField[] Fields;
    }
#pragma warning restore 0649

    sealed class Firmware
    {
        public const int FocalMinMm = 8;
        public const int FocalMaxMm = 600;

        // The version START declares to the bootloader (what ea9flash.py
        // --check reports afterwards).  Keep it in step with message 0x07.
        public static readonly byte[] DeclaredVersion = { 2, 1, 0 };

        readonly byte[] image;
        readonly PatchMap map;

        public int NameMax { get; private set; }
        public int DefaultFocalMm { get { return map.Default.FocalMm; } }
        public string DefaultName { get { return map.Default.Name; } }

        // Checks that the table belongs to this image and that every field
        // still holds the default it was found with.  Throws if not.
        public Firmware(byte[] image, string patchMapJson)
        {
            this.image = image;
            using (var s = new MemoryStream(Encoding.UTF8.GetBytes(patchMapJson))) {
                map = (PatchMap)new DataContractJsonSerializer(typeof(PatchMap)).ReadObject(s);
            }
            if (map.Fields == null || map.Default == null) {
                throw new InvalidDataException("patchmap.json is incomplete");
            }
            if (image.Length != map.ImageLen || Sha256(image) != map.ImageSha256.ToLowerInvariant()) {
                throw new InvalidDataException(
                    "the embedded firmware does not match patchmap.json -- they come from different builds");
            }
            NameMax = map.Fields.Where(f => f.Kind == "name").Select(f => f.Len).DefaultIfEmpty(0).Min();
            if (NameMax == 0 || !map.Fields.Any(f => f.Kind == "u16le_mm10")) {
                throw new InvalidDataException("patchmap.json lacks the focal or name fields");
            }
            foreach (var f in map.Fields) {
                byte[] want = Encode(f, map.Default.FocalMm, map.Default.Name);
                for (int i = 0; i < want.Length; i++) {
                    if (image[f.Offset + i] != want[i]) {
                        throw new InvalidDataException(string.Format(
                            "the firmware does not hold the default at 0x{0:x}", f.Offset));
                    }
                }
            }
        }

        public static Firmware LoadEmbedded()
        {
            return new Firmware(Resource("firmware.bin"),
                                Encoding.UTF8.GetString(Resource("patchmap.json")));
        }

        public static string LoadChangelog()
        {
            return Encoding.UTF8.GetString(Resource("changelog.txt"));
        }

        // Null if the name is acceptable, else why not.
        public string CheckName(string name)
        {
            if (name.Length == 0) {
                return "Enter a lens name.";
            }
            if (name.Length > NameMax) {
                return string.Format("The lens name can be at most {0} characters.", NameMax);
            }
            if (name.Any(c => c < 0x20 || c > 0x7E)) {
                return "The lens name can only use English letters, digits, spaces and common symbols.";
            }
            return null;
        }

        // A copy of the image with the user's values written in.
        public byte[] Patch(int focalMm, string name)
        {
            if (focalMm < FocalMinMm || focalMm > FocalMaxMm) {
                throw new ArgumentOutOfRangeException("focalMm");
            }
            string why = CheckName(name);
            if (why != null) {
                throw new ArgumentException(why);
            }
            var outImage = (byte[])image.Clone();
            foreach (var f in map.Fields) {
                Encode(f, focalMm, name).CopyTo(outImage, f.Offset);
            }
            return outImage;
        }

        static byte[] Encode(PatchField f, int focalMm, string name)
        {
            switch (f.Kind) {
            case "u16le_mm10": {
                int v = focalMm * 10;
                return new[] { (byte)(v & 0xFF), (byte)(v >> 8) };
            }
            case "bcd_mm":
                return new[] { Bcd(focalMm / 100), Bcd(focalMm) };
            case "name": {
                var b = new byte[f.Len];              // NUL-padded
                Encoding.ASCII.GetBytes(name).CopyTo(b, 0);
                return b;
            }
            default:
                throw new InvalidDataException("unknown patch field kind: " + f.Kind);
            }
        }

        static byte Bcd(int n)
        {
            return (byte)(((n / 10 % 10) << 4) | (n % 10));
        }

        static string Sha256(byte[] b)
        {
            using (var h = SHA256.Create()) {
                return string.Concat(h.ComputeHash(b).Select(x => x.ToString("x2")));
            }
        }

        static byte[] Resource(string name)
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
