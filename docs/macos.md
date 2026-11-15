# Flashing from macOS

The Python flasher supports Apple Silicon through HIDAPI's native macOS
backend. Linux continues to use `/dev/hidraw` with no third-party dependencies.

## Install the host tools

Install Python 3.9 or later from [python.org](https://www.python.org/downloads/macos/)
if `python3` is unavailable. Download or clone this repository, open Terminal
in its directory, then run:

```bash
./tools/install_macos.sh
```

The script creates `.venv-macos` inside the repository and installs `hidapi`
and `dnfile`. It can be rerun. It does not access the adapter or flash anything,
and does not need `sudo`. All commands below run from the repository directory;
there is no need to activate the virtual environment. To select another Python,
use `EA9_PYTHON=/path/to/python3 ./tools/install_macos.sh`.

## Obtain application firmware

You can use an application `.bin` built according to the main README. To use
a published release, download the updater `.exe` from this project's
[Releases](https://github.com/weiziqian/E-mount-impl-on-Techart-EA9/releases).
For example, for v2.2.0:

```bash
.venv-macos/bin/python tools/extract_release_firmware.py \
  ~/Downloads/Community.EA9.Firmware.Updater.v2.2.0.exe \
  -o build/ea9-release.bin
```

The helper parses the EXE as data; it never runs Windows code. It checks the
firmware's length, SHA-256 and default lens fields against the embedded patch
map, and validates the image's initial stack and reset vector. It refuses to
overwrite an existing output file. The embedded hash detects a mismatch between
resources, but does not authenticate the publisher: download from the project.

The extracted image retains the release's default lens settings. For v2.2.0
these are **50 mm** and **TECHART LM-EA9**. That release supports AF-C; AF-S still
misses focus, and autofocus is tuned for 50 mm lenses. Read the selected
release's notes before installing it. Lens customization in the Windows UI is
not performed by this extraction helper.

## Check and back up

Connect the adapter through its USB update connection with **no camera or lens
attached**, using a data cable. Release it from any VM or other updater first.

```bash
.venv-macos/bin/python tools/ea9flash.py --check
.venv-macos/bin/python tools/ea9flash.py --dump 0x0:0x20000 -o build/ea9-before-flash.bin
```

The backup should contain **131072 bytes**. Keep it and a known-good stock
application firmware. The backup includes the bootloader and is **not** an
application image; do not pass it to the flashing command. Use a new backup
filename for later sessions, as a successful dump replaces its output file.

`--check` reports the USB bootloader's reply, not the camera's application
version. **It may report `0.0.0` after an update**, even when the user reports
working firmware. Sending `--declare-version` does not guarantee that CHECK
will return that version. Verify the application bytes by reading them back.

## Flash and verify

For v2.2.0, using the extracted file above:

```bash
.venv-macos/bin/python tools/ea9flash.py --declare-version 2.2.0 build/ea9-release.bin
```

Review the filename and size, then answer `y`. Leave USB connected until the
`done` message. Progress reaches 100% only after the final acknowledgment.
Unplug and reconnect USB, then compare the application against the input file:

```bash
ea9_bytes=$(wc -c < build/ea9-release.bin | tr -d '[:space:]')
.venv-macos/bin/python tools/ea9flash.py --dump "0x5000:$ea9_bytes" -o build/ea9-readback.bin
cmp build/ea9-release.bin build/ea9-readback.bin
```

Run `cmp` only after the dump completes successfully. No output and exit status
zero means the files match. A difference or read error needs investigation;
do not use the version reply alone to decide whether to flash again.

The host validates image structure and padded transfer length. The device's
bootloader controls the actual destination address; the host assumes the
documented application base `0x5000` and does not configure or verify hardware
bootloader protection. Structural checks do not detect every firmware defect.

## Validation

An Apple Silicon user exercised the HIDAPI transport with an LM-EA9: CHECK
returned `1.8.0`, the v2.2.0 transfer acknowledged all 385 data reports, and the
user reported working firmware. A subsequent CHECK returned `0.0.0`. A matching
read-back hash was not supplied, so this is not a recorded byte-for-byte
hardware verification. Other macOS and adapter combinations need testing.

The automated tests exercise both transports, malformed image/range rejection,
backup preservation, transfer padding, and progress on timeouts without hardware:

```bash
python3 -m unittest discover -s tools/tests -v
```
