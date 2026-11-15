#!/usr/bin/env python3
"""
ea9flash.py -- flash a local EA9 .bin over its HID bootloader.

Reimplementation of the protocol used by TECHART_Updater(USB).exe, recovered from
the .NET assembly (USBHIDControl.Form1 / USBHIDControl.USBHID).  Unlike the
official updater this reads the image from a FILE, so no HTTP server, no DNS
redirect and no binary patching are needed.

PROTOCOL (all frames are 64-byte HID reports, report ID 0)
----------------------------------------------------------
magic = 95 27 68 18   (both directions)

  host -> device                                  device -> host
  95 27 68 18 02 02                    CHECK      95 27 68 18 02 02 vv vv vv ...
  95 27 68 18 01 01 vv vv vv           START      95 27 68 18 01 01 ...
  <64 raw firmware bytes>              DATA       any report = ACK, send next
  95 27 68 18 03 03 <u32 addr LE>      READ       <64 raw flash bytes>

vv vv vv is a version triplet in plain decimal bytes (1.9.0 -> 01 09 00).

VERSION REPORTING
-----------------
CHECK returns a USB bootloader version triplet. START carries the version the
host declares, matching the official updater's protocol. Persistence of that
triplet is not established: an Apple Silicon user reported 0.0.0 after a fully
acknowledged v2.2.0 transfer and reported that the firmware appeared to work.
The camera reads its version from the application's E-mount protocol replies.
Use a read-back comparison to verify firmware bytes; CHECK is not verification.

DATA transfer is ACK-clocked: every inbound report triggers exactly one outbound
64-byte chunk.  There is no address, no erase command and no CRC in the stream --
the bootloader owns the destination address.  The upstream loop includes the
report starting at limit = (len(image) // 1024) * 1024 + 1024, so the total is
limit + 64 bytes.  Bytes beyond the image are zero padded.

There is no signature and no hash anywhere in the official update path, so an
arbitrary image is accepted.

Linux uses /dev/hidraw directly. macOS uses the PyPI hidapi package (import hid).
See tools/install_macos.sh for setup, including on Apple Silicon.
"""

import argparse
import glob
import os
import select
import struct
import sys
import time

# Loaded only on macOS; Linux continues to use the standard library.
hid = None


def require_hidapi():
    global hid
    if hid is None:
        try:
            import hid as backend
        except ImportError as error:
            raise RuntimeError("Run tools/install_macos.sh to install hidapi") from error
        hid = backend
    return hid


MAGIC = bytes((0x95, 0x27, 0x68, 0x18))
CMD_START = bytes((0x01, 0x01))
CMD_CHECK = bytes((0x02, 0x02))
CMD_READ = bytes((0x03, 0x03))

REPORT_LEN = 64
CHUNK = 64

DEFAULT_VID = 0x0483
DEFAULT_PID = 0x575A

# SAM D21E17A: 128 KiB flash, 16 KiB SRAM, application at 0x5000.
FLASH_SIZE = 0x20000
APP_BASE = 0x5000
RAM_BASE = 0x20000000
RAM_SIZE = 0x4000
CORE_VECTOR_BYTES = 16 * 4
MAX_IMAGE_SIZE = 0x1A800  # Retain headroom for the upstream padding sequence.


def transfer_size(image_size):
    """Full upstream DATA stream, including its final all-zero report."""
    return (image_size // 1024) * 1024 + 1024 + CHUNK


def validate_image(image):
    """Reject structurally invalid application images before any USB command.

    These checks cannot authenticate firmware or detect every corruption; a
    raw image has no length/checksum header. Only flash a trusted EA9 build.
    """
    if len(image) < CORE_VECTOR_BYTES + 2:
        raise ValueError("Image is too short for a Cortex-M core vector table and code")
    if (len(image) > MAX_IMAGE_SIZE or
            APP_BASE + transfer_size(len(image)) > FLASH_SIZE):
        raise ValueError("Image and transfer padding are too large for the application region")

    stack, reset = struct.unpack_from("<II", image)
    # A descending stack may start one byte past SRAM, but must be word aligned.
    if not RAM_BASE < stack <= RAM_BASE + RAM_SIZE or stack % 4:
        raise ValueError("Invalid initial stack pointer 0x%08x; expected a word-aligned "
                         "address in EA9 SRAM" % stack)
    if not reset & 1:
        raise ValueError("Invalid reset vector: Cortex-M requires a Thumb address (bit 0 set)")
    entry = reset & ~1
    if not APP_BASE + CORE_VECTOR_BYTES <= entry <= APP_BASE + len(image) - 2:
        raise ValueError("Reset vector 0x%08x is outside this application's code; "
                         "expected a raw image linked at 0x5000" % reset)


def validate_flash_range(addr, length):
    if not 0 <= addr < FLASH_SIZE:
        raise ValueError("Flash address must be between 0x0 and 0x1ffff")
    if length <= 0 or length > FLASH_SIZE - addr:
        raise ValueError("Flash length must be positive and the range must end at or before 0x20000")


def parse_dump_range(value):
    try:
        address, length = value.split(":")
        address, length = int(address, 0), int(length, 0)
        validate_flash_range(address, length)
    except ValueError as error:
        raise argparse.ArgumentTypeError("Invalid dump range %r: %s" % (value, error)) from error
    return address, length


def parse_probe_address(value):
    try:
        address = int(value, 0)
        validate_flash_range(address, CHUNK)
    except ValueError as error:
        raise argparse.ArgumentTypeError("Invalid 64-byte probe address: %s" % error) from error
    return address


def read_flash(dev, addr, length):
    """Read a validated byte range using complete, in-bounds 64-byte reports."""
    validate_flash_range(addr, length)
    first = addr // CHUNK * CHUNK
    end = ((addr + length + CHUNK - 1) // CHUNK) * CHUNK
    out = bytearray()
    for block in range(first, end, CHUNK):
        reply = dev.read_at(block)
        if len(reply) != CHUNK:
            raise IOError("Incomplete flash read at 0x%08x" % block)
        out.extend(reply)
    offset = addr - first
    return bytes(out[offset:offset + length])


class Adapter:
    def __enter__(self):
        return self

    def __exit__(self, *a):
        self.close()

    # -- commands ---------------------------------------------------------

    def check(self):
        """CHECK: returns the device's firmware version triplet."""
        self.send(MAGIC + CMD_CHECK)
        rep = self._expect(CMD_CHECK)
        return rep[6], rep[7], rep[8]

    def start(self, version):
        """START: arm the bootloader for a sequential app write."""
        self.send(MAGIC + CMD_START + bytes(version))
        self._expect(CMD_START)

    def read_at(self, addr):
        """READ ('spy'): flash contents at addr.

        CONFIRMED on LM-EA9 hardware 2026-09-16: the reply is **64 raw flash
        bytes with no header at all** -- no magic, no echoed command, no length.
        A probe at 0x5000 returned all 64 bytes of the app image's vector table
        byte-for-byte.  So the reply is NOT a protocol frame and must not be
        validated as one; payload offset is 0 and the stride is 64.

        The command works at arbitrary addresses, including the 20 KB
        bootloader region below 0x5000, which no other channel exposes.
        """
        validate_flash_range(addr, CHUNK)
        self.send(MAGIC + CMD_READ + struct.pack("<I", addr))
        rep = self.recv()
        if rep is None:
            raise TimeoutError("no reply to READ at 0x%08x" % addr)
        return rep

    def _expect(self, cmd):
        rep = self.recv()
        if rep is None:
            raise TimeoutError("no reply from device")
        if rep[0:4] != MAGIC:
            raise IOError("bad magic in reply: %s" % rep[:8].hex(" "))
        if rep[4:6] != cmd:
            raise IOError(
                "reply for %s, expected %s" % (rep[4:6].hex(), cmd.hex())
            )
        return rep


class HidapiAdapter(Adapter):
    def __init__(self, path, timeout=5.0):
        backend = require_hidapi()
        self.path = path
        self.timeout = timeout
        self.device = backend.device()
        try:
            self.device.open_path(path)
        except Exception:
            self.device.close()
            raise

    def close(self):
        self.device.close()

    def send(self, payload):
        """HIDAPI expects report ID zero followed by the 64-byte payload."""
        if len(payload) > REPORT_LEN:
            raise ValueError("payload too long")
        buf = bytes(1) + payload + bytes(REPORT_LEN - len(payload))
        written = self.device.write(buf)
        if written != len(buf):
            raise IOError("short HID write: %s of %d bytes" % (written, len(buf)))

    def recv(self, timeout=None):
        t = self.timeout if timeout is None else timeout
        reply = bytes(self.device.read(REPORT_LEN, max(1, int(t * 1000))))
        if not reply:
            return None
        if len(reply) != REPORT_LEN:
            raise IOError("short HID report: %d of %d bytes" %
                          (len(reply), REPORT_LEN))
        return reply


class HidrawAdapter(Adapter):
    def __init__(self, path, timeout=5.0):
        self.path = path
        self.timeout = timeout
        self.fd = os.open(path, os.O_RDWR)

    def close(self):
        os.close(self.fd)

    def send(self, payload):
        """Send one 64-byte report (zero padded), prefixed with report ID 0."""
        if len(payload) > REPORT_LEN:
            raise ValueError("payload too long")
        buf = b"\x00" + payload + b"\x00" * (REPORT_LEN - len(payload))
        written = os.write(self.fd, buf)
        if written != len(buf):
            raise IOError("short HID write: %d of %d bytes" % (written, len(buf)))

    def recv(self, timeout=None):
        """Read one report, or None on timeout."""
        t = self.timeout if timeout is None else timeout
        r, _, _ = select.select([self.fd], [], [], t)
        if not r:
            return None
        reply = os.read(self.fd, REPORT_LEN)
        if len(reply) != REPORT_LEN:
            raise IOError("short HID report: %d of %d bytes" % (len(reply), REPORT_LEN))
        return reply


def open_adapter(path):
    if sys.platform == "darwin":
        return HidapiAdapter(path)
    return HidrawAdapter(path)


def list_hid():
    """Every HID device the kernel knows about, as (path, vid, pid, name)."""
    if sys.platform == "darwin":
        return [(item["path"], item["vendor_id"], item["product_id"],
                 item.get("product_string") or "?")
                for item in require_hidapi().enumerate()]
    out = []
    for node in sorted(glob.glob("/sys/class/hidraw/hidraw*")):
        uevent = os.path.join(node, "device", "uevent")
        try:
            with open(uevent) as f:
                info = f.read()
        except OSError:
            continue
        vid = pid = None
        name = "?"
        for line in info.splitlines():
            if line.startswith("HID_ID="):
                parts = line.split("=", 1)[1].split(":")
                if len(parts) == 3:
                    vid, pid = int(parts[1], 16), int(parts[2], 16)
            elif line.startswith("HID_NAME="):
                name = line.split("=", 1)[1]
        if vid is not None:
            out.append(("/dev/" + os.path.basename(node), vid, pid, name))
    return out


def no_device_help(vid, pid):
    """Say what was actually found, not just what was not.

    'No device found' is the same message for four different problems -- not
    plugged in, still on the camera, a permissions issue, or a wrong VID/PID --
    and the fix differs for each.  Listing what IS present separates them.
    """
    if sys.platform == "darwin":
        return (
            "No %04x:%04x HID device found. Connect the EA9 USB update connection "
            "directly to this Mac, with no camera or lens attached. Use a USB data "
            "cable, and release the device from any virtual machine or other updater."
        ) % (vid, pid)
    lines = ["no %04x:%04x HID device found." % (vid, pid)]
    others = list_hid()
    if others:
        lines.append("HID devices that ARE present:")
        for path, v, p, name in others:
            lines.append("  %-14s %04x:%04x  %s" % (path, v, p, name))
    else:
        lines.append("No HID devices at all -- the kernel sees nothing.")
    lines += [
        "",
        "Check, in order:",
        "  1. The adapter is on USB and NOT mounted on the camera.  It takes",
        "     power from whichever it is attached to, and only the USB side",
        "     runs the bootloader that speaks this protocol.",
        "  2. It enumerated:  lsusb | grep -i %04x" % vid,
        "  3. Permission.  /dev/hidraw* is root-only by default, and this fails",
        "     as 'not found' only when enumeration itself is empty -- but if you",
        "     see the device in lsusb and not here, run with sudo or add:",
        "       SUBSYSTEM==\"hidraw\", ATTRS{idVendor}==\"%04x\", "
        "ATTRS{idProduct}==\"%04x\", MODE=\"0660\", TAG+=\"uaccess\""
        % (vid, pid),
        "     to /etc/udev/rules.d/99-techart.rules, then replug.",
        "  4. --wait N polls for up to N seconds, so you can start the command",
        "     first and plug in after.",
    ]
    return "\n".join(lines)


def find_device(vid=DEFAULT_VID, pid=DEFAULT_PID, wait=0):
    """Return matching OS device paths, optionally polling for the adapter."""
    deadline = time.monotonic() + wait
    while True:
        found = [path for path, v, p, _ in list_hid() if v == vid and p == pid]
        if found or time.monotonic() >= deadline:
            return found
        time.sleep(0.25)


def parse_version(s):
    """"1.9.0" -> (1, 9, 0).  Decimal bytes, as Convert.ToByte does."""
    parts = s.split(".")
    if len(parts) != 3:
        raise ValueError("version must be M.m.p, e.g. 1.9.0")
    out = tuple(int(p, 10) for p in parts)
    if not all(0 <= b <= 255 for b in out):
        raise ValueError("each version component must fit in a byte")
    return out


def flash(dev, image, declare=None, progress=True):
    validate_image(image)
    total = transfer_size(len(image))
    ver = dev.check()
    print("device reports version %d.%d.%d" % ver)

    # START declares the version; CHECK need not return it after flashing.
    if declare is None:
        declare = ver
        print("declaring %d.%d.%d (echoing device; pass --declare-version to "
              "declare the intended version)" % declare)
    else:
        print("declaring version %d.%d.%d" % declare)

    dev.start(declare)
    print("bootloader armed, streaming %d bytes" % len(image))

    sent = 0
    for pos in range(0, total, CHUNK):
        chunk = image[pos:pos + CHUNK]
        dev.send(chunk)
        if dev.recv() is None:
            raise TimeoutError("device stopped acking at offset %d" % (pos + CHUNK))
        sent += 1
        acknowledged = pos + CHUNK
        if progress and (sent % 32 == 0 or acknowledged == total):
            pct = 100 * acknowledged // total
            print("\r  %3d%%  (%d/%d bytes)" % (pct, acknowledged, total),
                  end="\n" if acknowledged == total else "", flush=True)

    print("done -- %d chunks sent. Unplug the USB connection." % sent)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("firmware", nargs="?", help="path to the .bin to flash")
    ap.add_argument("--vid", type=lambda s: int(s, 16), default=DEFAULT_VID)
    ap.add_argument("--pid", type=lambda s: int(s, 16), default=DEFAULT_PID)
    ap.add_argument("-d", "--device", help="explicit device path (hidraw on Linux, HIDAPI on macOS)")
    ap.add_argument("--check", action="store_true",
                    help="query the device version and exit (safe, read-only)")
    ap.add_argument("--declare-version", metavar="M.m.p", type=parse_version,
                    help="version to send in START; default: echo CHECK. "
                         "The bootloader may not report it afterwards. This "
                         "does not change the camera-visible application version.")
    ap.add_argument("--probe", metavar="ADDR", type=parse_probe_address,
                    help="send ONE READ (03 03) at ADDR and print the raw reply "
                         "verbatim, without validating it (safe, read-only). "
                         "Use this before --dump: it is the only way to learn "
                         "whether the bootloader implements 03 03 at all, and "
                         "what its reply actually looks like.")
    ap.add_argument("--dump", metavar="ADDR:LEN", type=parse_dump_range,
                    help="read a positive byte range within the 128 KiB flash, "
                         "e.g. 0x0:0x5000")
    ap.add_argument("-o", "--out", help="file to write --dump output to")
    ap.add_argument("-y", "--yes", action="store_true", help="skip confirmation")
    ap.add_argument("--wait", type=float, default=0, metavar="SECONDS",
                    help="poll for the adapter for this long before giving up, "
                         "so the command can be started before plugging in")
    args = ap.parse_args()

    image = None
    if not args.check and args.probe is None and args.dump is None:
        if not args.firmware:
            ap.error("a firmware path is required unless --check/--probe/--dump is used")
        try:
            with open(args.firmware, "rb") as f:
                image = f.read(MAX_IMAGE_SIZE + 1)
            validate_image(image)
        except (OSError, ValueError) as error:
            ap.error(str(error))

    if sys.platform not in ("darwin", "linux"):
        ap.error("This tool supports macOS and Linux")
    if sys.platform == "darwin":
        try:
            require_hidapi()
        except RuntimeError as error:
            ap.error(str(error))
    path = args.device
    if path and sys.platform == "darwin":
        path = os.fsencode(path)
    if not path:
        cands = find_device(args.vid, args.pid, wait=args.wait)
        if not cands:
            sys.exit(no_device_help(args.vid, args.pid))
        if len(cands) > 1:
            ap.error("Multiple matching HID interfaces/devices; disconnect extra "
                     "adapters or select one with --device: %r" % (cands,))
        path = cands[0]
    print("using %s" % path)

    if sys.platform == "linux" and not os.access(path, os.R_OK | os.W_OK):
        sys.exit(
            "%s exists but is not readable/writable by this user.\n"
            "\n"
            "/dev/hidraw* is root-only on most distributions.  Either:\n"
            "  sudo %s ...\n"
            "or install a udev rule once and replug:\n"
            "  echo 'SUBSYSTEM==\"hidraw\", ATTRS{idVendor}==\"%04x\", "
            "ATTRS{idProduct}==\"%04x\", MODE=\"0660\", TAG+=\"uaccess\"' \\\n"
            "    | sudo tee /etc/udev/rules.d/99-techart.rules\n"
            "  sudo udevadm control --reload-rules"
            % (path, sys.argv[0], args.vid, args.pid))

    with open_adapter(path) as dev:
        if args.check:
            print("version: %d.%d.%d" % dev.check())
            print("(USB bootloader reply; not verification of the installed firmware. "
                  "The camera reads its version from the application.)")
            return

        if args.probe is not None:
            print("READ probe: 03 03 at 0x%08x" % args.probe)
            dev.send(MAGIC + CMD_READ + struct.pack("<I", args.probe))
            rep = dev.recv()
            if rep is None:
                print("no reply within %.1fs." % dev.timeout)
                print("=> the bootloader most likely does NOT implement 03 03.")
                print("   (the device is not wedged -- this was a read; unplug "
                      "and replug to be sure it still answers --check)")
                return
            print("raw reply, %d bytes:" % len(rep))
            for i in range(0, len(rep), 16):
                print("  +%02x  %s" % (i, rep[i:i + 16].hex(" ")))
            if rep[0:4] == MAGIC and rep[4:6] == CMD_READ:
                print("=> framed reply (unexpected -- the LM-EA9 sends raw data)")
            else:
                print("=> raw flash data, no header: this is what the LM-EA9 "
                      "returns.  offset 0, stride 64.  --dump handles it.")
            return

        if args.dump is not None:
            addr, length = args.dump
            out = read_flash(dev, addr, length)
            if args.out:
                with open(args.out, "wb") as f:
                    f.write(out)
                print("wrote %d bytes to %s" % (length, args.out))
            else:
                print(out.hex(" "))
            return

        print("image: %s, %d bytes" % (args.firmware, len(image)))

        if not args.yes:
            reply = input("Replace the adapter application with this image? [y/N] ")
            if reply.strip().lower() not in ("y", "yes"):
                sys.exit("aborted")

        flash(dev, image, declare=args.declare_version)


if __name__ == "__main__":
    main()
