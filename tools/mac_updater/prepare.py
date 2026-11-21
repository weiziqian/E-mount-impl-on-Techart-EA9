"""Prepare the Mac app resources from the repository's release build."""

import hashlib
import json
from pathlib import Path
import re
import shutil
import sys

from firmware import inspect
from verify_image import check


def firmware_version(image, windows_source):
    # The Windows updater supplies all three START bytes. Confirm the major
    # and minor agree with the actual 0x07 camera response in the built image.
    match = re.search(r"DeclaredVersion\s*=\s*\{\s*(\d+)\s*,\s*(\d+)\s*,\s*(\d+)\s*\}",
                      windows_source)
    if not match:
        raise ValueError("Cannot locate DeclaredVersion in the Windows updater source.")
    version = list(map(int, match.groups()))
    major, minor, patch = version
    if not (0 <= major <= 9 and 0 <= minor <= 9 and 0 <= patch <= 255):
        raise ValueError("DeclaredVersion is outside the EA9 version format.")
    frame = re.compile(rb"\xf0\x2b\x00\x02.\x07.{37}", re.DOTALL)
    packets = [p for p in frame.findall(image)
               if p[-1] == 0x55 and sum(p[1:-3]) == int.from_bytes(p[-3:-1], "little")]
    if len(packets) != 1 or packets[0][12] != (major << 4 | minor):
        raise ValueError("The release image's camera version does not match DeclaredVersion.")
    return version


def main(repo, output):
    windows = repo / "tools" / "win_updater"
    source = repo / "build" / "ea9-rebuild-release.bin"
    image = source.read_bytes()
    offset, hardware, slots = inspect(image)
    if len(image) < 176 or not check(image, str(source)):
        raise ValueError("The application image failed static checks.")
    version = firmware_version(image, (windows / "src/Firmware.cs").read_text(encoding="utf-8"))
    output.mkdir(parents=True, exist_ok=True)
    (output / "ea9.bin").write_bytes(image)
    (output / "firmware.json").write_text(
        json.dumps({"version": version, "sha256": hashlib.sha256(image).hexdigest()}) + "\n",
        encoding="utf-8",
    )
    shutil.copyfile(windows / "changelog.txt", output / "changelog.txt")
    print(f"Lens configuration at 0x{offset:x}, hardware version {hardware}")
    for index, (mm, name, pupil) in enumerate(slots):
        if mm:
            print(f"  Slot {index + 1}: {mm} mm, exit pupil {pupil} mm, {name}")
    print(f"Prepared firmware {'.'.join(map(str, version))} in {output}")


if __name__ == "__main__":
    try:
        main(Path(sys.argv[1]), Path(sys.argv[2]))
    except (ValueError, OSError) as error:
        sys.exit(str(error))
