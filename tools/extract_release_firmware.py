#!/usr/bin/env python3
"""Extract an unchanged EA9 application from a downloaded release updater.

The EXE is parsed as data, never executed. Its embedded patch map supplies
the expected length, SHA-256 and default lens settings. Obtain the updater
from this project's GitHub releases; an embedded hash is not authentication.
"""
import argparse
import hashlib
import json
from pathlib import Path
import struct

from ea9flash import validate_image


def validate_resources(resources):
    try:
        image = resources['firmware.bin']
        metadata = json.loads(resources['patchmap.json'])
        if not isinstance(metadata, dict):
            raise ValueError('patchmap.json must contain an object')
        if len(image) != metadata['image_len']:
            raise ValueError('firmware length does not match patchmap.json')
        if hashlib.sha256(image).hexdigest() != metadata['image_sha256'].lower():
            raise ValueError('firmware SHA-256 does not match patchmap.json')
        validate_image(image)

        defaults = metadata['default']
        kinds = set()
        for field in metadata['fields']:
            offset, length, kind = field['offset'], field['len'], field['kind']
            if offset < 0 or length <= 0 or offset + length > len(image):
                raise ValueError('patch field lies outside the image')
            if kind == 'u16le_mm10':
                expected = struct.pack('<H', defaults['focal_mm'] * 10)
            elif kind == 'bcd_mm':
                focal = defaults['focal_mm']
                bcd = lambda n: ((n // 10 % 10) << 4) | (n % 10)
                expected = bytes((bcd(focal // 100), bcd(focal)))
            elif kind == 'name':
                expected = defaults['name'].encode('ascii').ljust(length, b'\0')
            else:
                raise ValueError('unknown patch field kind: %s' % kind)
            if len(expected) != length or image[offset:offset + length] != expected:
                raise ValueError('firmware default does not match patch field at 0x%x' % offset)
            kinds.add(kind)
        if not {'name', 'u16le_mm10'} <= kinds:
            raise ValueError('patch map lacks the focal length or lens name')
    except (KeyError, TypeError, AttributeError, UnicodeError, struct.error) as error:
        raise ValueError('incomplete or malformed release resources: %s' % error) from error
    return image, defaults


def extract(updater):
    try:
        import dnfile
    except ImportError as error:
        raise ValueError('Run tools/install_macos.sh to install dnfile') from error
    if updater.stat().st_size > 64 * 1024 * 1024:
        raise ValueError('updater is unexpectedly large (limit: 64 MiB)')
    try:
        pe = dnfile.dnPE(str(updater))
    except dnfile.PEFormatError as error:
        raise ValueError('not a valid release updater executable') from error
    try:
        if pe.net is None:
            raise ValueError('not a .NET release updater')
        resources = {}
        for resource in pe.net.resources:
            name = str(resource.name)
            if name in ('firmware.bin', 'patchmap.json'):
                if name in resources or not isinstance(resource.data, bytes):
                    raise ValueError('duplicate or non-binary resource: %s' % name)
                resources[name] = resource.data
        return validate_resources(resources)
    finally:
        pe.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('updater', type=Path, help='downloaded Community EA9 Firmware Updater .exe')
    parser.add_argument('-o', '--out', type=Path, required=True, help='new application .bin file')
    args = parser.parse_args()
    try:
        image, defaults = extract(args.updater)
        args.out.parent.mkdir(parents=True, exist_ok=True)
        # Never overwrite a firmware file or the input updater accidentally.
        with args.out.open('xb') as output:
            output.write(image)
    except (OSError, ValueError) as error:
        parser.exit(1, 'error: %s\n' % error)
    print('Extracted %d bytes to %s' % (len(image), args.out))
    print('SHA-256: %s' % hashlib.sha256(image).hexdigest())
    print('Default lens: %s mm, %s (unchanged)' % (defaults['focal_mm'], defaults['name']))
    print('Extraction only; no adapter was accessed.')


if __name__ == '__main__':
    main()
