"""Validate and configure the application image bundled with the updater."""

import hashlib
import json
from pathlib import Path

import cfgblock
import gen_optical

APP_BASE = 0x5000
LOG_BASE = 0x16000
REPORT_SIZE = 64
APERTURES = ("f/2.5", "f/2.8", "f/3.2", "f/3.5", "f/4.0", "f/4.5", "f/5.0", "f/5.6")
FOCAL_PRESETS = (21, 24, 25, 28, 32, 35, 40, 45, 50, 56, 75, 90, 135)
DEFAULT_NAME = "Community EA9 Adapter"
EMPTY_SLOT = (0, "", 0)


def transfer_size(image_size):
    # Same inclusive final chunk as tools/ea9flash.py and the Windows updater.
    return (image_size // 1024 + 1) * 1024 + REPORT_SIZE


def check_size(image):
    if not image or APP_BASE + transfer_size(len(image)) > LOG_BASE:
        raise ValueError("Firmware and its transfer padding must fit in the application area.")


def inspect(image):
    """Use the shared block schema, and turn checker errors into UI errors."""
    check_size(image)
    try:
        offset = cfgblock.find(image)
        block = image[offset:offset + cfgblock.SIZE]
        hardware, slots = cfgblock.parse(block)
    except SystemExit as error:
        raise ValueError(str(error)) from error
    if cfgblock.pack(hardware, slots) != block:
        raise ValueError("The bundled lens configuration has invalid padding.")
    return offset, hardware, slots


def normalize_slots(rows, multiple):
    if len(rows) != (cfgblock.SLOTS if multiple else 1):
        raise ValueError("Incorrect number of lens slots.")
    slots = []
    for index, (focal_text, name, pupil_text) in enumerate(rows):
        label = APERTURES[index] if multiple else "Lens"
        focal_text = focal_text.strip()
        if focal_text and not (focal_text.isascii() and focal_text.isdecimal()):
            raise ValueError(f"{label} needs a whole focal length in millimeters.")
        # Trim leading zeroes before the length check and integer conversion.
        digits = focal_text.lstrip("0")
        if not digits:
            slots.append(EMPTY_SLOT)
            continue
        if len(digits) > 3 or not 8 <= int(digits) <= 600:
            raise ValueError(f"{label} needs a focal length from 8 to 600 mm.")
        if pupil_text not in {str(value) for value in gen_optical.XPD_CHOICES}:
            raise ValueError(f"Choose an exit pupil for {label}.")
        name = name.strip() or DEFAULT_NAME
        if len(name) >= cfgblock.NAME_FIELD or any(not 32 <= ord(c) <= 126 for c in name):
            raise ValueError(f"{label} needs a name of at most 63 printable ASCII characters.")
        slots.append((int(digits), name, int(pupil_text)))
    configured = [slot for slot in slots if slot[0]]
    if not configured:
        raise ValueError("Configure at least one lens.")
    if len(configured) == 1:
        return configured + [EMPTY_SLOT] * (cfgblock.SLOTS - 1)
    return slots


class Firmware:
    def __init__(self, resources):
        self.image = (resources / "ea9.bin").read_bytes()
        manifest = json.loads((resources / "firmware.json").read_text(encoding="utf-8"))
        if hashlib.sha256(self.image).hexdigest() != manifest["sha256"]:
            raise ValueError("The bundled firmware does not match its build manifest.")
        version = manifest["version"]
        if (not isinstance(version, list) or len(version) != 3
                or any(type(n) is not int or not 0 <= n <= 255 for n in version)):
            raise ValueError("The bundled firmware version is invalid.")
        self.version = tuple(version)
        self.offset, self.hardware, self.slots = inspect(self.image)

    @property
    def version_text(self):
        return ".".join(map(str, self.version))

    def patch(self, hardware, slots):
        if hardware not in (1, 2):
            raise ValueError("Choose adapter hardware version 1 or 2.")
        block = cfgblock.pack(hardware, slots)
        image = self.image[:self.offset] + block + self.image[self.offset + len(block):]
        if inspect(image) != (self.offset, hardware, slots):
            raise ValueError("The patched lens configuration did not validate.")
        return image


def resource_dir():
    return Path(__file__).resolve().parent
