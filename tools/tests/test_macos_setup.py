import copy
import contextlib
import hashlib
import io
import json
import os
from pathlib import Path
import shutil
import struct
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import extract_release_firmware as extractor


def release_resources():
    image = bytearray(256)
    struct.pack_into('<II', image, 0, 0x20004000, 0x50C1)
    struct.pack_into('<H', image, 80, 500)
    image[84:116] = b'TECHART LM-EA9'.ljust(32, b'\0')
    metadata = {'image_len': len(image), 'image_sha256': hashlib.sha256(image).hexdigest(),
                'default': {'focal_mm': 50, 'name': 'TECHART LM-EA9'},
                'fields': [{'offset': 80, 'len': 2, 'kind': 'u16le_mm10'},
                           {'offset': 84, 'len': 32, 'kind': 'name'}]}
    return {'firmware.bin': bytes(image), 'patchmap.json': json.dumps(metadata).encode()}


class ExtractionTests(unittest.TestCase):
    def test_valid_resources_are_returned_unchanged(self):
        resources = release_resources()
        image, defaults = extractor.validate_resources(resources)
        self.assertEqual(image, resources['firmware.bin'])
        self.assertEqual(defaults, {'focal_mm': 50, 'name': 'TECHART LM-EA9'})

    def test_corruption_length_and_default_mismatches_are_rejected(self):
        resources = release_resources()
        metadata = json.loads(resources['patchmap.json'])
        variants = []
        for key, value in [('image_len', 255), ('image_sha256', '0' * 64),
                           ('fields', []), ('default', {'focal_mm': 51, 'name': 'TECHART LM-EA9'})]:
            altered = copy.deepcopy(metadata)
            altered[key] = value
            variants.append({'firmware.bin': resources['firmware.bin'],
                             'patchmap.json': json.dumps(altered).encode()})
        variants += [{}, {'firmware.bin': b'x', 'patchmap.json': b'[]'},
                     {'firmware.bin': resources['firmware.bin'], 'patchmap.json': b'bad json'}]
        for variant in variants:
            with self.subTest(variant=variant), self.assertRaises(ValueError):
                extractor.validate_resources(variant)

    def test_matching_hash_does_not_bypass_image_validation(self):
        resources = release_resources()
        metadata = json.loads(resources['patchmap.json'])
        image = bytes(256)
        metadata['image_sha256'] = hashlib.sha256(image).hexdigest()
        with self.assertRaises(ValueError):
            extractor.validate_resources({'firmware.bin': image,
                                          'patchmap.json': json.dumps(metadata).encode()})

    def test_existing_output_is_preserved(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'firmware.bin'
            path.write_bytes(b'keep me')
            with patch.object(sys, 'argv', ['extract', 'release.exe', '-o', str(path)]), \
                    patch.object(extractor, 'extract', return_value=(b'new', {})), \
                    contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit):
                    extractor.main()
            self.assertEqual(path.read_bytes(), b'keep me')


class InstallerTests(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory(prefix='ea9 setup ')
        self.addCleanup(directory.cleanup)
        self.root = Path(directory.name)
        scripts = self.root / 'repo with spaces' / 'tools'
        scripts.mkdir(parents=True)
        real_tools = Path(__file__).resolve().parents[1]
        for name in ['install_macos.sh', 'requirements-macos.txt']:
            shutil.copyfile(real_tools / name, scripts / name)
        self.script = scripts / 'install_macos.sh'
        commands = self.root / 'commands'
        commands.mkdir()
        uname = commands / 'uname'
        uname.write_text('#!/bin/sh\nprintf "%s\\n" "${EA9_TEST_OS:-Darwin}"\n')
        uname.chmod(0o755)
        python = self.root / 'fake python'
        python.write_text('#!' + sys.executable + '\n' + '''
import json, os, pathlib, shutil, sys
with open(os.environ['EA9_TEST_LOG'], 'a') as log:
    log.write(json.dumps(sys.argv[1:]) + '\\n')
if sys.argv[1:3] == ['-m', 'venv']:
    target = pathlib.Path(sys.argv[3]) / 'bin' / 'python'
    target.parent.mkdir(parents=True)
    shutil.copyfile(__file__, target)
    target.chmod(0o755)
if sys.argv[1:3] == ['-m', 'pip'] and os.environ.get('EA9_TEST_FAIL_PIP'):
    sys.exit(1)
''')
        python.chmod(0o755)
        self.log = self.root / 'calls.jsonl'
        self.env = dict(os.environ, PATH=str(commands) + os.pathsep + os.environ['PATH'],
                        EA9_PYTHON=str(python), EA9_TEST_LOG=str(self.log))

    def run_setup(self):
        return subprocess.run(['bash', str(self.script)], env=self.env, text=True,
                              capture_output=True, timeout=15)

    def calls(self):
        return [json.loads(line) for line in self.log.read_text().splitlines()]

    def test_paths_with_spaces_and_repeat_setup(self):
        for _ in range(2):
            result = self.run_setup()
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertIn('Setup complete', result.stdout)
        calls = self.calls()
        self.assertEqual(sum(c[:2] == ['-m', 'venv'] for c in calls), 1)
        self.assertEqual(sum(c[:2] == ['-m', 'pip'] for c in calls), 2)
        self.assertFalse(any('tools/ea9flash.py' in argument for c in calls for argument in c))

    def test_failed_dependency_install_does_not_report_success(self):
        self.env['EA9_TEST_FAIL_PIP'] = '1'
        result = self.run_setup()
        self.assertNotEqual(result.returncode, 0)
        self.assertNotIn('Setup complete', result.stdout)
        self.assertEqual(self.calls()[-1][:2], ['-m', 'pip'])

    def test_wrong_platform_does_not_create_environment(self):
        self.env['EA9_TEST_OS'] = 'Linux'
        result = self.run_setup()
        self.assertNotEqual(result.returncode, 0)
        self.assertFalse(self.log.exists())

    def test_missing_python_explains_prerequisite(self):
        self.env['EA9_PYTHON'] = str(self.root / 'missing python')
        result = self.run_setup()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('Install Python 3.9+', result.stderr)
        self.assertFalse(self.log.exists())


if __name__ == '__main__':
    unittest.main()
