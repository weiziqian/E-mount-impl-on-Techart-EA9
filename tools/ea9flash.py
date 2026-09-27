#!/usr/bin/env python3
"""
ea9flash.py -- flash a local .bin to a TECHART adapter over its HID bootloader.

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

TWO INDEPENDENT VERSION RECORDS -- do not confuse them
------------------------------------------------------
  * the BCD byte in message 0x07 of the image (file offset 0x4a44, 0x18 = 1.8).
    This is what the CAMERA reads.  Patch it with tools/mkfw.py --version.
  * the triplet the BOOTLOADER stores and returns in the CHECK reply.  This is
    what --check prints.  It lives nowhere in the app image -- no image contains
    the bytes it reports -- so flashing cannot change it.

The only channel that carries a version to the device is START[6..8].  The
official updater fills that in from the MANIFEST line for the firmware the user
selected (Form1::On_ListBoxEX_Click splits the version text on '.' and
Convert.ToByte's the three parts into version[0..2]; btn_send_Click and
BinReadCallback then copy version[0..2] into start_tab[6..8]).  So it declares
the version being installed.

It does NOT echo the CHECK reply back, despite sending CHECK first:
btn_send_Click sets TypeOfData = isUpdate before that CHECK and only switches to
isNone just before START, and usbHID_DataReceived has no isUpdate branch -- so
the CHECK reply is discarded.  The handler that would assign
version[] = reply[6..8] runs only for the standalone version display
(img_Title_Click / btn_cfg_Click).

This script echoed the CHECK reply into START until 2026-08-14, which pinned the
reported version forever.  Use --declare-version to set it.  (That the
bootloader *stores* the declared triplet is inference -- strong, since it is the
only version-bearing channel and the value exists in no image -- but it is
confirmed the moment a --declare-version flash changes what --check returns.)

DATA transfer is ACK-clocked: every inbound report triggers exactly one outbound
64-byte chunk.  There is no address, no erase command and no CRC in the stream --
the bootloader owns the destination address.  The host stops after
   limit = (len(image) // 1024) * 1024 + 1024
i.e. it pads with zeros up to the next 1 KiB boundary plus one extra KiB.

There is no signature and no hash anywhere in the official update path, so an
arbitrary image is accepted.

Linux note: this talks to /dev/hidraw* directly, no hidapi/libusb needed.  The
device uses unnumbered reports, so writes carry a leading 0x00 (stripped by the
kernel) and reads return the bare 64-byte payload.
"""

import argparse
import glob
import os
import select
import struct
import sys
import time

MAGIC = bytes((0x95, 0x27, 0x68, 0x18))
CMD_START = bytes((0x01, 0x01))
CMD_CHECK = bytes((0x02, 0x02))
CMD_READ = bytes((0x03, 0x03))

REPORT_LEN = 64
CHUNK = 64

DEFAULT_VID = 0x0483
DEFAULT_PID = 0x575A


class Adapter:
    def __init__(self, path, timeout=5.0):
        self.path = path
        self.timeout = timeout
        self.fd = os.open(path, os.O_RDWR)

    def close(self):
        os.close(self.fd)

    def __enter__(self):
        return self

    def __exit__(self, *a):
        self.close()

    def send(self, payload):
        """Send one 64-byte report (zero padded), prefixed with report ID 0."""
        if len(payload) > REPORT_LEN:
            raise ValueError("payload too long")
        buf = b"\x00" + payload + b"\x00" * (REPORT_LEN - len(payload))
        os.write(self.fd, buf)

    def recv(self, timeout=None):
        """Read one report, or None on timeout."""
        t = self.timeout if timeout is None else timeout
        r, _, _ = select.select([self.fd], [], [], t)
        if not r:
            return None
        return os.read(self.fd, REPORT_LEN)

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


def list_hid():
    """Every HID device the kernel knows about, as (path, vid, pid, name)."""
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
    """Return /dev/hidraw* paths matching vid:pid, optionally polling for it."""
    deadline = time.time() + wait
    while True:
        found = [path for path, v, p, _ in list_hid() if v == vid and p == pid]
        if found or time.time() >= deadline:
            return found
        time.sleep(0.25)


def _find_device_unused(vid=DEFAULT_VID, pid=DEFAULT_PID):
    """Return /dev/hidraw* paths matching vid:pid."""
    found = []
    for node in sorted(glob.glob("/sys/class/hidraw/hidraw*")):
        uevent = os.path.join(node, "device", "uevent")
        try:
            with open(uevent) as f:
                info = f.read()
        except OSError:
            continue
        for line in info.splitlines():
            if not line.startswith("HID_ID="):
                continue
            # HID_ID=<bus>:<VID>:<PID>, 8 hex digits each
            parts = line.split("=", 1)[1].split(":")
            if len(parts) != 3:
                continue
            if int(parts[1], 16) == vid and int(parts[2], 16) == pid:
                found.append("/dev/" + os.path.basename(node))
    return found


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
    ver = dev.check()
    print("device reports version %d.%d.%d" % ver)

    # START[6..8] is what the bootloader records as "the version now installed";
    # it is the ONLY channel that carries a version to the device, and it is
    # independent of the BCD byte in the image's message 0x07.  The official
    # updater declares the version parsed from the manifest line here -- it does
    # NOT echo the CHECK reply back (that reply is discarded: btn_send_Click
    # sends CHECK while TypeOfData == isUpdate, for which the dispatcher has no
    # branch).  Echoing, as this script used to do, freezes the reported version
    # forever.  Default to echoing only because it is the conservative choice.
    if declare is None:
        declare = ver
        print("declaring %d.%d.%d (echoing device; pass --declare-version to "
              "change what --check reports)" % declare)
    else:
        print("declaring version %d.%d.%d" % declare)

    dev.start(declare)
    print("bootloader armed, streaming %d bytes" % len(image))

    limit = (len(image) // 1024) * 1024 + 1024
    pos = 0
    sent = 0
    while pos <= limit:
        chunk = image[pos:pos + CHUNK]
        dev.send(chunk)
        pos += CHUNK
        sent += 1
        if progress and sent % 32 == 0:
            pct = min(100, int(100 * pos / limit))
            print("\r  %3d%%  (%d/%d bytes)" % (pct, min(pos, limit), limit),
                  end="", flush=True)
        if dev.recv() is None:
            raise TimeoutError("device stopped acking at offset %d" % pos)
    if progress:
        print("\r  100%%  (%d/%d bytes)" % (limit, limit))

    print("done -- %d chunks sent. Unplug the USB connection." % sent)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("firmware", nargs="?", help="path to the .bin to flash")
    ap.add_argument("--vid", type=lambda s: int(s, 16), default=DEFAULT_VID)
    ap.add_argument("--pid", type=lambda s: int(s, 16), default=DEFAULT_PID)
    ap.add_argument("-d", "--device", help="explicit /dev/hidrawN")
    ap.add_argument("--check", action="store_true",
                    help="query the device version and exit (safe, read-only)")
    ap.add_argument("--declare-version", metavar="M.m.p", type=parse_version,
                    help="version to record in START[6..8], i.e. what --check "
                         "will report afterwards. This is bootloader-side "
                         "bookkeeping and is NOT the version the camera sees "
                         "(that is the BCD byte in message 0x07 of the image). "
                         "Default: echo back whatever the device already "
                         "reports, which leaves it unchanged.")
    ap.add_argument("--probe", metavar="ADDR", type=lambda x: int(x, 0),
                    help="send ONE READ (03 03) at ADDR and print the raw reply "
                         "verbatim, without validating it (safe, read-only). "
                         "Use this before --dump: it is the only way to learn "
                         "whether the bootloader implements 03 03 at all, and "
                         "what its reply actually looks like.")
    ap.add_argument("--dump", metavar="ADDR:LEN",
                    help="try the READ command over a range, e.g. 0x0:0x5000 "
                         "(unproven on the LM-EA9)")
    ap.add_argument("-o", "--out", help="file to write --dump output to")
    ap.add_argument("-y", "--yes", action="store_true", help="skip confirmation")
    ap.add_argument("--wait", type=float, default=0, metavar="SECONDS",
                    help="poll for the adapter for this long before giving up, "
                         "so the command can be started before plugging in")
    args = ap.parse_args()

    path = args.device
    if not path:
        cands = find_device(args.vid, args.pid, wait=args.wait)
        if not cands:
            sys.exit(no_device_help(args.vid, args.pid))
        if len(cands) > 1:
            print("multiple matches: %s -- using %s" % (cands, cands[0]))
        path = cands[0]
    print("using %s" % path)

    if not os.access(path, os.R_OK | os.W_OK):
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

    with Adapter(path) as dev:
        if args.check:
            print("version: %d.%d.%d" % dev.check())
            print("(bootloader-recorded, from the last START; the camera reads "
                  "the BCD byte in message 0x07 instead)")
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

        if args.dump:
            a, l = args.dump.split(":")
            addr, length = int(a, 0), int(l, 0)
            out = bytearray()
            while len(out) < length:
                rep = dev.read_at(addr + len(out))
                out += rep[:CHUNK]            # raw flash, no header (see read_at)
                if len(out) % 4096 == 0:
                    print("  read %d/%d bytes" % (len(out), length), flush=True)
            if args.out:
                with open(args.out, "wb") as f:
                    f.write(bytes(out[:length]))
                print("wrote %d bytes to %s" % (length, args.out))
            else:
                print(bytes(out[:length]).hex(" "))
            return

        if not args.firmware:
            ap.error("a firmware path is required unless --check/--dump is used")
        with open(args.firmware, "rb") as f:
            image = f.read()
        print("image: %s, %d bytes" % (args.firmware, len(image)))

        if not args.yes:
            reply = input("flash this image? there is no going back without SWD [y/N] ")
            if reply.strip().lower() not in ("y", "yes"):
                sys.exit("aborted")

        flash(dev, image, declare=args.declare_version)


if __name__ == "__main__":
    main()
