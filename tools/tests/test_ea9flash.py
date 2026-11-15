import contextlib
import io
import struct
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import Mock, patch
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import ea9flash as f


def make_image(size=256, stack=0x20004000, reset=0x50C1):
    """Synthetic vector table and code bytes; never use this on hardware."""
    image = bytearray(size)
    struct.pack_into('<II', image, 0, stack, reset)
    if size >= 194:
        image[192:194] = b'\xfe\xe7'  # Thumb branch to itself.
    return bytes(image)


class FakeHID:
    def __init__(self):
        self.writes = []
        self.replies = []
        self.closed = False

    def open_path(self, path):
        self.path = path

    def close(self):
        self.closed = True

    def write(self, data):
        self.writes.append(data)
        return len(data)

    def read(self, length, timeout):
        return self.replies.pop(0) if self.replies else []


class Tests(unittest.TestCase):
    def setUp(self):
        platform = patch.object(sys, 'platform', 'darwin')
        platform.start()
        self.addCleanup(platform.stop)
        self.dev = FakeHID()
        self.patcher = patch.object(f, 'hid')
        self.hid = self.patcher.start()
        self.hid.device.return_value = self.dev
        self.addCleanup(self.patcher.stop)

    def test_check_and_close(self):
        self.dev.replies = [f.MAGIC + f.CMD_CHECK + bytes([1, 8, 0]) + bytes(55)]
        with f.HidapiAdapter(b'path') as adapter:
            self.assertEqual(adapter.check(), (1, 8, 0))
        self.assertTrue(self.dev.closed)
        self.assertEqual(self.dev.writes[0], b'\x00' + f.MAGIC + f.CMD_CHECK + bytes(58))

    def test_timeout_and_short_report(self):
        with f.HidapiAdapter(b'path') as adapter:
            self.assertIsNone(adapter.recv())
            self.dev.replies = [b'x']
            with self.assertRaises(IOError):
                adapter.recv()

    def test_short_write(self):
        with f.HidapiAdapter(b'path') as adapter:
            with patch.object(self.dev, 'write', return_value=1):
                with self.assertRaises(IOError):
                    adapter.send(b'x')

    def test_raw_read(self):
        self.dev.replies = [bytes(range(64))]
        with f.HidapiAdapter(b'path') as adapter:
            self.assertEqual(adapter.read_at(0x5000), bytes(range(64)))
        self.assertEqual(self.dev.writes[0][1:11], f.MAGIC + f.CMD_READ + b'\x00\x50\x00\x00')

    def test_flash_sequence(self):
        for length, total in [(256, 1088), (1023, 1088), (1024, 2112),
                              (1025, 2112), (0x1A800, 0x1AC40)]:
            with self.subTest(length=length):
                image = make_image(length)
                self.dev.writes = []
                self.dev.replies = [f.MAGIC + f.CMD_CHECK + bytes([1, 8, 0]) + bytes(55),
                                    f.MAGIC + f.CMD_START + bytes(58)] + [bytes(64)] * (total // 64)
                with f.HidapiAdapter(b'path') as adapter, contextlib.redirect_stdout(io.StringIO()):
                    f.flash(adapter, image, progress=False)
                self.assertEqual(len(self.dev.writes), 2 + total // 64)
                self.assertTrue(all(len(w) == 65 and w[0] == 0 for w in self.dev.writes))
                stream = b''.join(w[1:] for w in self.dev.writes[2:])
                self.assertEqual(stream, image + bytes(total - length))
                self.assertEqual(self.dev.replies, [])

    def test_enumeration(self):
        self.hid.enumerate.return_value = [dict(path=b'ea9', vendor_id=0x0483,
                                               product_id=0x575A)]
        self.assertEqual(f.find_device(), [b'ea9'])


class ImageValidationTests(unittest.TestCase):
    def test_invalid_images_do_not_issue_any_usb_command(self):
        bad_images = [b'', b'x', bytes(64), bytes(256), b'\xff' * 256,
                      make_image(0x1A801), make_image(128),
                      make_image(stack=0x20000000), make_image(stack=0x1FFFFFFC),
                      make_image(stack=0x20004004), make_image(stack=0x20003FFF),
                      make_image(reset=0x50C0), make_image(reset=0x101),
                      make_image(reset=0x5001), make_image(reset=0x5101)]
        for image in bad_images:
            with self.subTest(length=len(image), header=image[:8].hex()):
                dev = Mock()
                with self.assertRaises(ValueError):
                    f.flash(dev, image)
                self.assertEqual(dev.mock_calls, [])

    def test_valid_stack_and_reset_boundaries(self):
        for image in [make_image(), make_image(stack=0x20000004),
                      make_image(reset=0x50FF), make_image(66, reset=0x5041)]:
            with self.subTest(header=image[:8].hex()):
                f.validate_image(image)

    def test_cli_rejects_invalid_image_before_opening_device(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'bad.bin'
            path.write_bytes(b'x')
            with patch.object(sys, 'argv', ['ea9flash_macos.py', '--device', 'mock',
                                           '--yes', str(path)]), patch.object(f, 'open_adapter') as adapter, \
                    contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit) as error:
                    f.main()
            self.assertEqual(error.exception.code, 2)
            adapter.assert_not_called()


class DumpValidationTests(unittest.TestCase):
    def test_invalid_ranges_leave_backup_untouched_and_do_not_open_device(self):
        ranges = ['0x0:0', '0x0:-1', '-1:64', '0x20000:1', '0x1ffff:2',
                  '0:0x20001', '0:0x100000000', 'garbage', '0:1:2', '0:', '']
        with tempfile.TemporaryDirectory() as directory:
            backup = Path(directory) / 'backup.bin'
            backup.write_bytes(b'existing backup contents')
            for value in ranges:
                with self.subTest(value=value), patch.object(sys, 'argv',
                        ['ea9flash_macos.py', '--device', 'mock', '--dump=' + value,
                         '-o', str(backup)]), patch.object(f, 'open_adapter') as adapter, \
                        contextlib.redirect_stderr(io.StringIO()):
                    with self.assertRaises(SystemExit) as error:
                        f.main()
                    self.assertEqual(error.exception.code, 2)
                    self.assertEqual(backup.read_bytes(), b'existing backup contents')
                    adapter.assert_not_called()

    def test_dump_full_flash_and_unaligned_edges(self):
        memory = bytes(range(256)) * 512
        for address, length in [(0, 0x20000), (63, 3), (0x1FFFF, 1), (0x1FF81, 127)]:
            with self.subTest(address=address, length=length):
                dev = Mock()
                dev.read_at.side_effect = lambda addr: memory[addr:addr + 64]
                result = f.read_flash(dev, address, length)
                self.assertEqual(result, memory[address:address + length])
                for call in dev.read_at.call_args_list:
                    addr = call.args[0]
                    self.assertEqual(addr % 64, 0)
                    self.assertGreaterEqual(addr, 0)
                    self.assertLessEqual(addr + 64, 0x20000)

    def test_read_flash_rejects_invalid_ranges_without_reads(self):
        for address, length in [(0, 0), (0, -1), (-1, 1), (0x1FFFF, 2)]:
            dev = Mock()
            with self.subTest(address=address, length=length), self.assertRaises(ValueError):
                f.read_flash(dev, address, length)
            dev.read_at.assert_not_called()

    def test_cli_backup_is_not_replaced_when_read_fails(self):
        with tempfile.TemporaryDirectory() as directory:
            backup = Path(directory) / 'backup.bin'
            backup.write_bytes(b'existing backup contents')
            with patch.object(sys, 'argv', ['ea9flash_macos.py', '--device', 'mock',
                                           '--dump', '0:128', '-o', str(backup)]), \
                    patch.object(sys, 'platform', 'darwin'), \
                    patch.object(f, 'hid'), patch.object(f, 'open_adapter') as adapter, \
                    contextlib.redirect_stdout(io.StringIO()):
                adapter.return_value.__enter__.return_value.read_at.side_effect = [
                    bytes(64), TimeoutError('simulated disconnect')]
                with self.assertRaises(TimeoutError):
                    f.main()
            self.assertEqual(backup.read_bytes(), b'existing backup contents')

    def test_cli_writes_exact_requested_bytes(self):
        with tempfile.TemporaryDirectory() as directory:
            backup = Path(directory) / 'backup.bin'
            with patch.object(sys, 'argv', ['ea9flash_macos.py', '--device', 'mock',
                                           '--dump', '0x1ffff:1', '-o', str(backup)]), \
                    patch.object(sys, 'platform', 'darwin'), \
                    patch.object(f, 'hid'), patch.object(f, 'open_adapter') as adapter, \
                    contextlib.redirect_stdout(io.StringIO()):
                dev = adapter.return_value.__enter__.return_value
                dev.read_at.return_value = bytes(range(64))
                f.main()
                dev.read_at.assert_called_once_with(0x1FFC0)
            self.assertEqual(backup.read_bytes(), b'\x3f')

    def test_probe_rejects_out_of_bounds_report_before_opening_device(self):
        with patch.object(sys, 'argv', ['ea9flash_macos.py', '--probe', '0x1ffff']), \
                patch.object(f, 'open_adapter') as adapter, contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit) as error:
                f.main()
        self.assertEqual(error.exception.code, 2)
        adapter.assert_not_called()


class ProgressTests(unittest.TestCase):
    def run_transfer(self, fail_at=None):
        output = io.StringIO()
        dev = Mock()
        dev.check.return_value = (1, 8, 0)
        acknowledgments = []

        def receive():
            # Observe progress while the next ACK is still pending.
            self.assertNotIn('100%', output.getvalue())
            acknowledgments.append(dev.send.call_count)
            return None if dev.send.call_count == fail_at else bytes(64)

        dev.recv.side_effect = receive
        with contextlib.redirect_stdout(output):
            if fail_at:
                with self.assertRaises(TimeoutError):
                    f.flash(dev, make_image(1024))
            else:
                f.flash(dev, make_image(1024))
        return output.getvalue(), acknowledgments, dev

    def test_success_reports_complete_total_only_after_final_ack(self):
        output, acknowledgments, dev = self.run_transfer()
        self.assertEqual(acknowledgments, list(range(1, 34)))
        self.assertEqual(dev.send.call_count, 33)
        self.assertIn('96%  (2048/2112 bytes)', output)
        self.assertIn('100%  (2112/2112 bytes)', output)
        self.assertEqual(output.count('100%'), 1)
        self.assertIn('done -- 33 chunks sent', output)

    def test_timeout_never_reports_completion_or_sends_more_data(self):
        for fail_at in [1, 32, 33]:
            with self.subTest(fail_at=fail_at):
                output, acknowledgments, dev = self.run_transfer(fail_at)
                self.assertNotIn('100%', output)
                self.assertNotIn('done --', output)
                self.assertEqual(dev.send.call_count, fail_at)
                self.assertEqual(len(acknowledgments), fail_at)


class PlatformTests(unittest.TestCase):
    def test_linux_transport_uses_hidraw_without_loading_hidapi(self):
        reply = f.MAGIC + f.CMD_CHECK + bytes([1, 8, 0]) + bytes(55)
        with patch.object(sys, 'platform', 'linux'), \
                patch.object(f, 'require_hidapi') as backend, \
                patch.object(f.os, 'open', return_value=17) as opened, \
                patch.object(f.os, 'close') as closed, \
                patch.object(f.os, 'write', return_value=65) as write, \
                patch.object(f.os, 'read', return_value=reply), \
                patch.object(f.select, 'select', return_value=([17], [], [])):
            with f.open_adapter('/dev/hidraw3') as dev:
                self.assertEqual(dev.check(), (1, 8, 0))
            opened.assert_called_once_with('/dev/hidraw3', f.os.O_RDWR)
            write.assert_called_once_with(17, b'\0' + f.MAGIC + f.CMD_CHECK + bytes(58))
            closed.assert_called_once_with(17)
            backend.assert_not_called()

    def test_linux_short_write_and_timeout(self):
        with patch.object(f.os, 'open', return_value=17), \
                patch.object(f.os, 'close'), patch.object(f.os, 'write', return_value=1), \
                patch.object(f.select, 'select', return_value=([], [], [])):
            with f.HidrawAdapter('/dev/hidraw3') as dev:
                with self.assertRaises(IOError):
                    dev.send(b'x')
                self.assertIsNone(dev.recv())

    def test_macos_zero_version_is_reported_without_installation_claim(self):
        device = FakeHID()
        device.replies = [f.MAGIC + f.CMD_CHECK + bytes(58)]
        output = io.StringIO()
        with patch.object(sys, 'platform', 'darwin'), patch.object(f, 'hid') as backend, \
                patch.object(sys, 'argv', ['ea9flash.py', '--device', 'DevSrvsID:1', '--check']), \
                contextlib.redirect_stdout(output):
            backend.device.return_value = device
            f.main()
        self.assertEqual(device.path, b'DevSrvsID:1')
        self.assertIn('version: 0.0.0', output.getvalue())
        self.assertIn('not verification', output.getvalue())
        self.assertNotIn('from the last START', output.getvalue())
        self.assertEqual(len(device.writes), 1)


if __name__ == '__main__':
    unittest.main()
