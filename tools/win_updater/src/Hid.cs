// Hid.cs -- just enough of the Windows HID API to talk to the bootloader:
// find a device by VID:PID, open it, and move one report at a time with a
// timeout.  Only the system DLLs (hid, setupapi, kernel32) are used.
//
// Windows differs from Linux hidraw in one way that matters here: the device
// uses unnumbered reports, yet every report on Windows carries the report ID
// byte (0) in front.  A write is 1 + 64 bytes, and a read RETURNS 1 + 64
// bytes, so the leading byte is stripped before the protocol sees it.

using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Runtime.InteropServices;
using System.Threading;
using Microsoft.Win32.SafeHandles;

namespace EA9Updater
{
    // What the protocol needs from a transport.  The real one is HidDevice;
    // a test can substitute a simulated bootloader.
    interface IReportChannel
    {
        // Send one report payload (at most 64 bytes, zero padded).
        void Send(byte[] payload);

        // One report payload, or null if none arrives within timeoutMs.
        byte[] Receive(int timeoutMs);
    }

    sealed class HidDevice : IReportChannel, IDisposable
    {
        public const int ReportLen = 64;
        const int WriteTimeoutMs = 5000;

        readonly SafeFileHandle handle;
        readonly int inLen, outLen;
        readonly IntPtr inBuf, outBuf, overlapped;

        public string Path { get; private set; }

        HidDevice(string path, SafeFileHandle h)
        {
            Path = path;
            handle = h;

            IntPtr pp;
            if (!HidD_GetPreparsedData(h, out pp)) {
                throw new Win32Exception(Marshal.GetLastWin32Error());
            }
            try {
                HIDP_CAPS caps;
                if (HidP_GetCaps(pp, out caps) != HIDP_STATUS_SUCCESS) {
                    throw new InvalidOperationException("HidP_GetCaps failed");
                }
                inLen = caps.InputReportByteLength;
                outLen = caps.OutputReportByteLength;
            } finally {
                HidD_FreePreparsedData(pp);
            }
            if (inLen < 2 || outLen < 2) {
                throw new InvalidOperationException(
                    "the device has no input or output report");
            }

            inBuf = Marshal.AllocHGlobal(inLen);
            outBuf = Marshal.AllocHGlobal(outLen);
            overlapped = Marshal.AllocHGlobal(Marshal.SizeOf(typeof(NativeOverlapped)));
        }

        public void Dispose()
        {
            handle.Dispose();
            Marshal.FreeHGlobal(inBuf);
            Marshal.FreeHGlobal(outBuf);
            Marshal.FreeHGlobal(overlapped);
        }

        // Device paths of every present HID device with this VID:PID.
        public static List<string> Find(ushort vid, ushort pid)
        {
            var found = new List<string>();
            Guid g;
            HidD_GetHidGuid(out g);
            IntPtr set = SetupDiGetClassDevs(ref g, IntPtr.Zero, IntPtr.Zero,
                                             DIGCF_PRESENT | DIGCF_DEVICEINTERFACE);
            if (set == INVALID_HANDLE_VALUE) {
                throw new Win32Exception(Marshal.GetLastWin32Error());
            }
            try {
                var data = new SP_DEVICE_INTERFACE_DATA();
                data.cbSize = Marshal.SizeOf(data);
                for (int i = 0; SetupDiEnumDeviceInterfaces(set, IntPtr.Zero, ref g, i, ref data); i++) {
                    string path = DetailPath(set, ref data);
                    if (path == null) {
                        continue;
                    }
                    // Access 0: enough for the attributes, and works even on
                    // devices the system holds open (keyboards, mice).
                    using (var h = CreateFile(path, 0, FILE_SHARE_READ | FILE_SHARE_WRITE,
                                              IntPtr.Zero, OPEN_EXISTING, 0, IntPtr.Zero)) {
                        if (h.IsInvalid) {
                            continue;
                        }
                        var a = new HIDD_ATTRIBUTES();
                        a.Size = Marshal.SizeOf(a);
                        if (HidD_GetAttributes(h, ref a) && a.VendorID == vid && a.ProductID == pid) {
                            found.Add(path);
                        }
                    }
                }
            } finally {
                SetupDiDestroyDeviceInfoList(set);
            }
            return found;
        }

        public static HidDevice Open(string path)
        {
            var h = CreateFile(path, GENERIC_READ | GENERIC_WRITE,
                               FILE_SHARE_READ | FILE_SHARE_WRITE, IntPtr.Zero,
                               OPEN_EXISTING, FILE_FLAG_OVERLAPPED, IntPtr.Zero);
            if (h.IsInvalid) {
                throw new Win32Exception(Marshal.GetLastWin32Error());
            }
            try {
                return new HidDevice(path, h);
            } catch {
                h.Dispose();
                throw;
            }
        }

        public void Send(byte[] payload)
        {
            if (payload.Length > ReportLen) {
                throw new ArgumentException("payload too long");
            }
            var buf = new byte[outLen];          // [0] = report ID 0
            Array.Copy(payload, 0, buf, 1, Math.Min(payload.Length, outLen - 1));
            Marshal.Copy(buf, 0, outBuf, outLen);
            if (Transfer(true, outBuf, outLen, WriteTimeoutMs) < 0) {
                throw new TimeoutException("the device did not accept a report");
            }
        }

        public byte[] Receive(int timeoutMs)
        {
            int n = Transfer(false, inBuf, inLen, timeoutMs);
            if (n < 0) {
                return null;
            }
            var buf = new byte[inLen];
            Marshal.Copy(inBuf, buf, 0, inLen);
            var rep = new byte[ReportLen];        // drop the report ID byte
            Array.Copy(buf, 1, rep, 0, Math.Min(ReportLen, Math.Max(0, n - 1)));
            return rep;
        }

        // One overlapped read or write.  Returns the byte count, or -1 on
        // timeout -- in which case the request is cancelled and fully retired
        // before returning, so a late report cannot land in a reused buffer.
        int Transfer(bool write, IntPtr buf, int len, int timeoutMs)
        {
            using (var ev = new ManualResetEvent(false)) {
                var ov = new NativeOverlapped();
                ov.EventHandle = ev.SafeWaitHandle.DangerousGetHandle();
                Marshal.StructureToPtr(ov, overlapped, false);

                bool ok = write
                    ? WriteFile(handle, buf, len, IntPtr.Zero, overlapped)
                    : ReadFile(handle, buf, len, IntPtr.Zero, overlapped);
                int n;
                if (!ok) {
                    int err = Marshal.GetLastWin32Error();
                    if (err != ERROR_IO_PENDING) {
                        throw new Win32Exception(err);
                    }
                    if (!ev.WaitOne(timeoutMs)) {
                        CancelIo(handle);
                        // Completed in the gap between the wait and the
                        // cancel?  Then it is a real transfer, not a timeout.
                        return GetOverlappedResult(handle, overlapped, out n, true) ? n : -1;
                    }
                }
                if (!GetOverlappedResult(handle, overlapped, out n, false)) {
                    throw new Win32Exception(Marshal.GetLastWin32Error());
                }
                return n;
            }
        }

        static string DetailPath(IntPtr set, ref SP_DEVICE_INTERFACE_DATA data)
        {
            int size;
            SetupDiGetDeviceInterfaceDetail(set, ref data, IntPtr.Zero, 0, out size, IntPtr.Zero);
            if (size <= 0) {
                return null;
            }
            IntPtr detail = Marshal.AllocHGlobal(size);
            try {
                // SP_DEVICE_INTERFACE_DETAIL_DATA_W.cbSize: the size of the
                // fixed part, which the packing makes 8 on x64 and 6 on x86.
                Marshal.WriteInt32(detail, IntPtr.Size == 8 ? 8 : 6);
                if (!SetupDiGetDeviceInterfaceDetail(set, ref data, detail, size, out size, IntPtr.Zero)) {
                    return null;
                }
                return Marshal.PtrToStringUni(new IntPtr(detail.ToInt64() + 4));
            } finally {
                Marshal.FreeHGlobal(detail);
            }
        }

        // ---- P/Invoke --------------------------------------------------

        const int DIGCF_PRESENT = 0x02, DIGCF_DEVICEINTERFACE = 0x10;
        const uint GENERIC_READ = 0x80000000, GENERIC_WRITE = 0x40000000;
        const uint FILE_SHARE_READ = 1, FILE_SHARE_WRITE = 2, OPEN_EXISTING = 3;
        const uint FILE_FLAG_OVERLAPPED = 0x40000000;
        const int ERROR_IO_PENDING = 997;
        const int HIDP_STATUS_SUCCESS = 0x00110000;
        static readonly IntPtr INVALID_HANDLE_VALUE = new IntPtr(-1);

        [StructLayout(LayoutKind.Sequential)]
        struct SP_DEVICE_INTERFACE_DATA
        {
            public int cbSize;
            public Guid InterfaceClassGuid;
            public int Flags;
            public IntPtr Reserved;
        }

        [StructLayout(LayoutKind.Sequential)]
        struct HIDD_ATTRIBUTES
        {
            public int Size;
            public ushort VendorID;
            public ushort ProductID;
            public ushort VersionNumber;
        }

        [StructLayout(LayoutKind.Sequential)]
        struct HIDP_CAPS
        {
            public ushort Usage;
            public ushort UsagePage;
            public ushort InputReportByteLength;
            public ushort OutputReportByteLength;
            public ushort FeatureReportByteLength;
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 17)]
            public ushort[] Reserved;
            public ushort NumberLinkCollectionNodes;
            public ushort NumberInputButtonCaps;
            public ushort NumberInputValueCaps;
            public ushort NumberInputDataIndices;
            public ushort NumberOutputButtonCaps;
            public ushort NumberOutputValueCaps;
            public ushort NumberOutputDataIndices;
            public ushort NumberFeatureButtonCaps;
            public ushort NumberFeatureValueCaps;
            public ushort NumberFeatureDataIndices;
        }

        [DllImport("hid.dll")]
        static extern void HidD_GetHidGuid(out Guid hidGuid);

        [DllImport("hid.dll", SetLastError = true)]
        static extern bool HidD_GetAttributes(SafeFileHandle h, ref HIDD_ATTRIBUTES attributes);

        [DllImport("hid.dll", SetLastError = true)]
        static extern bool HidD_GetPreparsedData(SafeFileHandle h, out IntPtr preparsed);

        [DllImport("hid.dll")]
        static extern bool HidD_FreePreparsedData(IntPtr preparsed);

        [DllImport("hid.dll")]
        static extern int HidP_GetCaps(IntPtr preparsed, out HIDP_CAPS caps);

        [DllImport("setupapi.dll", SetLastError = true, CharSet = CharSet.Unicode)]
        static extern IntPtr SetupDiGetClassDevs(ref Guid classGuid, IntPtr enumerator,
                                                 IntPtr hwndParent, int flags);

        [DllImport("setupapi.dll", SetLastError = true)]
        static extern bool SetupDiEnumDeviceInterfaces(IntPtr set, IntPtr devInfo, ref Guid classGuid,
                                                       int index, ref SP_DEVICE_INTERFACE_DATA data);

        [DllImport("setupapi.dll", SetLastError = true, CharSet = CharSet.Unicode)]
        static extern bool SetupDiGetDeviceInterfaceDetail(IntPtr set, ref SP_DEVICE_INTERFACE_DATA data,
                                                           IntPtr detail, int detailSize,
                                                           out int requiredSize, IntPtr devInfo);

        [DllImport("setupapi.dll")]
        static extern bool SetupDiDestroyDeviceInfoList(IntPtr set);

        [DllImport("kernel32.dll", SetLastError = true, CharSet = CharSet.Unicode)]
        static extern SafeFileHandle CreateFile(string name, uint access, uint share, IntPtr security,
                                                uint disposition, uint flags, IntPtr template);

        [DllImport("kernel32.dll", SetLastError = true)]
        static extern bool ReadFile(SafeFileHandle h, IntPtr buf, int len, IntPtr read, IntPtr overlapped);

        [DllImport("kernel32.dll", SetLastError = true)]
        static extern bool WriteFile(SafeFileHandle h, IntPtr buf, int len, IntPtr written, IntPtr overlapped);

        [DllImport("kernel32.dll", SetLastError = true)]
        static extern bool GetOverlappedResult(SafeFileHandle h, IntPtr overlapped,
                                               out int transferred, bool wait);

        [DllImport("kernel32.dll", SetLastError = true)]
        static extern bool CancelIo(SafeFileHandle h);
    }
}
