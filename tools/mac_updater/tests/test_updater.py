import hashlib
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import cfgblock
from firmware import EMPTY_SLOT, Firmware, normalize_slots
from prepare import firmware_version

with patch.dict(sys.modules, {"hid": SimpleNamespace()}):
    import ea9_usb as usb


class FirmwareTests(unittest.TestCase):
    def setUp(self):
        resources = tempfile.TemporaryDirectory()
        self.addCleanup(resources.cleanup)
        self.path = Path(resources.name)
        slots = [(50, "50 mm", 45)] + [EMPTY_SLOT] * 7
        self.image = b"prefix" + cfgblock.pack(2, slots) + b"suffix"
        (self.path / "ea9.bin").write_bytes(self.image)
        (self.path / "firmware.json").write_text(json.dumps({
            "version": [2, 4, 0], "sha256": hashlib.sha256(self.image).hexdigest(),
        }))

    def test_multiple_lenses_patch_only_the_config_block(self):
        firmware = Firmware(self.path)
        rows = [("", "", "")] * 8
        rows[2] = (" 28 ", "Wide", "35")
        rows[6] = ("90", "Portrait", "60")
        slots = normalize_slots(rows, multiple=True)
        patched = firmware.patch(1, slots)
        # Check a sparse configuration keeps the camera's aperture mapping.
        self.assertEqual(cfgblock.parse(patched[6:-6]), (1, [
            EMPTY_SLOT, EMPTY_SLOT, (28, "Wide", 35), EMPTY_SLOT,
            EMPTY_SLOT, EMPTY_SLOT, (90, "Portrait", 60), EMPTY_SLOT,
        ]))
        self.assertEqual(patched[:6] + patched[-6:], b"prefixsuffix")
        self.assertEqual(firmware.image, self.image)

    def test_one_lens_uses_slot_zero(self):
        rows = [("", "", "")] * 7 + [("35", " ", "35")]
        slots = normalize_slots(rows, multiple=True)
        self.assertEqual(slots, [(35, "Community EA9 Adapter", 35)] + [EMPTY_SLOT] * 7)
        for row in [("0", "", ""), ("601", "Lens", "45"), ("50", "镜头", "45"),
                    ("50", "Lens", "47"), ("50", "x" * 64, "45")]:
            with self.subTest(row=row), self.assertRaises(ValueError):
                normalize_slots([row], multiple=False)

    def test_corrupt_bundle_rejected(self):
        (self.path / "ea9.bin").write_bytes(self.image + b"\0")
        with self.assertRaisesRegex(ValueError, "manifest"):
            Firmware(self.path)

    def test_release_version_must_match_camera_response(self):
        frame = b"\xf0\x2b\x00\x02\x00\x07" + bytes(6) + b"\x24" + bytes(27)
        frame += sum(frame[1:]).to_bytes(2, "little") + b"\x55"
        self.assertEqual(firmware_version(frame, "DeclaredVersion = {2, 4, 0}"), [2, 4, 0])
        with self.assertRaises(ValueError):
            firmware_version(frame, "DeclaredVersion = {2, 3, 0}")


class Device:
    def __init__(self, drop_ack=None, check_reply=None):
        self.writes = []
        self.path = None
        self.closed = 0
        self.drop_ack = drop_ack
        self.check_reply = check_reply

    def open_path(self, path):
        self.path = path

    def close(self):
        self.closed += 1

    def write(self, report):
        self.writes.append(report)
        return len(report)

    def read(self, size, timeout_ms):
        self.last_read = (size, timeout_ms)
        if len(self.writes) == self.drop_ack:
            return []
        command = self.writes[-1][1:7]
        if command == bytes.fromhex("95 27 68 18 02 02") and self.check_reply is not None:
            return self.check_reply
        if command[:4] == bytes.fromhex("95 27 68 18"):
            return command.ljust(64, b"\0")
        return bytes(64)


class TransferTests(unittest.TestCase):
    def set_device(self, device):
        self.paths = {b"LM-EA9"}
        connection = usb.Connection()
        backend = SimpleNamespace(
            enumerate=lambda vid, pid: [{"path": path} for path in self.paths]
            if (vid, pid) == (0x0483, 0x575A) else [],
            device=lambda: device,
        )
        fake = patch.object(usb, "hid", backend)
        fake.start()
        self.addCleanup(fake.stop)
        return connection

    def test_check_start_padding_and_replug(self):
        device = Device()
        connection = self.set_device(device)
        path, status = connection.poll()
        self.assertIn("0.0.0", status)
        self.assertEqual(connection.poll(), (path, status))
        self.assertEqual(len(device.writes), 1)
        image = b"\xff" * 1024
        progress = []
        connection.flash(path, image, (2, 4, 0), lambda sent, total: progress.append((sent, total)))
        self.assertEqual(device.writes[1], bytes.fromhex("00 95 27 68 18 02 02").ljust(65, b"\0"))
        self.assertEqual(device.writes[2], bytes.fromhex("00 95 27 68 18 01 01 02 04 00").ljust(65, b"\0"))
        self.assertTrue(all(report[0] == 0 and len(report) == 65 for report in device.writes))
        self.assertEqual(b"".join(w[1:] for w in device.writes[3:]), image + bytes(1088))
        self.assertEqual(progress[-1], (2112, 2112))
        self.assertEqual(device.closed, 2)
        self.assertIsNone(connection.poll()[0])
        self.paths.clear()
        self.assertIsNone(connection.poll()[0])
        self.paths.add(path)
        self.assertEqual(connection.poll()[0], path)

    def test_drop_ack_stops_transfer_and_blocks_retry(self):
        device = Device(drop_ack=5)
        connection = self.set_device(device)
        path, _ = connection.poll()
        progress = []
        with self.assertRaisesRegex(OSError, "after 64 of 1088"):
            connection.flash(path, bytes(128), (2, 4, 0), lambda n, _: progress.append(n))
        self.assertEqual(progress, [64])
        self.assertEqual(len(device.writes), 5)
        self.assertEqual(device.closed, 2)
        self.assertIsNone(connection.poll()[0])

    def test_wrong_or_ambiguous_device_cannot_start(self):
        device = Device()
        connection = self.set_device(device)
        path, _ = connection.poll()
        self.paths.add(b"second")
        self.assertIsNone(connection.poll()[0])
        with self.assertRaisesRegex(OSError, "connection changed"):
            connection.flash(path, bytes(128), (2, 4, 0), lambda *_: None)
        self.assertEqual(len(device.writes), 1)

    def test_short_check_reply_never_enables_write(self):
        device = Device(check_reply=b"\x95\x27\x68\x18\x02\x02")
        connection = self.set_device(device)
        path, status = connection.poll()
        self.assertIsNone(path)
        self.assertIn("short report", status)
        self.assertEqual(device.last_read, (64, 5000))
        self.assertEqual(device.closed, 1)


if __name__ == "__main__":
    unittest.main()
