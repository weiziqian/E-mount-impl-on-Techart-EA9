# Community EA9 Firmware Updater for macOS

The Mac updater packages the firmware built by `make release` into a standalone
macOS app. It writes the same lens configuration as the Windows updater,
including adapter hardware version, lens names, focal lengths, and exit pupils.
It supports one lens or up to eight lenses selectable from the camera.

## Download the Apple silicon app

In the repository's **Actions** tab, open a successful
**Build Apple silicon updater** run.
Under **Artifacts**, download **Community-EA9-Firmware-Updater-macOS-arm64**
and unzip it to get **Community EA9 Firmware Updater.app**. GitHub may ask you
to sign in, and Actions artifacts expire; this is a development build, not a
signed or notarized release.

## Use the updater

1. Open **Community EA9 Firmware Updater.app**. Connect the LM-EA9 by USB with
   the camera and lens removed.
2. Select the hardware version. Use **Single lens** for a fixed configuration
   or **Multiple lenses** to assign lenses to camera aperture settings. Choose
   a focal length preset or enter a value from 8 to 600 mm. The suggested exit
   pupil updates when the focal length changes.
3. Click **Write Firmware** and review the settings. Keep USB connected until
   the app reports that all chunks were acknowledged, then unplug the adapter.
4. In multiple lens mode, select a configured aperture on the camera and take
   a picture to switch lenses. Return the camera to its widest aperture.

The connection status shows the bootloader's CHECK reply separately from the
bundled firmware version. Some adapters return `0.0.0` after a successful
transfer. The updater uses the individual USB acknowledgements to report
completion. The camera reads the version from the installed application.

## Build on Apple silicon

Use a native arm64 terminal and Python 3.12 with Tk, such as the
[python.org macOS installer](https://www.python.org/downloads/macos/). Install the
Xcode command line tools, an arm64 macOS build of Arm GNU Toolchain, and the
SAM D21 ASF4 headers from the [repository build instructions](../../README.md#41-prerequisites).
Set `TOOLCHAIN` to the compiler path prefix and `ASF4` to the header directory.
When the toolchain is on `PATH`, use `TOOLCHAIN=arm-none-eabi-`.

Run from the repository root:

```sh
python3.12 -m venv tools/mac_updater/.venv
. tools/mac_updater/.venv/bin/activate
python -m pip install -r tools/mac_updater/requirements-build.txt
TOOLCHAIN=arm-none-eabi- ASF4="/path/to/asf4/samd21" tools/mac_updater/build.sh
open "tools/mac_updater/out/Community EA9 Firmware Updater.app"
```

The app bundles Python, Tk, HIDAPI, and the release firmware. These dependencies
are needed only on the build machine. PyInstaller ad hoc signs local arm64
builds. Distributing the built app to other Macs is a separate signing and
notarization step.

The build validates the application vectors and the lens configuration block
and checks that the bundled version matches both the Windows updater's START
version and the camera's version byte in the image. The updater accepts the
embedded image and replaces its configuration block in memory before writing.

## Checks without an adapter

```sh
# Build and validate the firmware resources only. This also runs on Linux.
TOOLCHAIN=arm-none-eabi- ASF4="/path/to/asf4/samd21" tools/mac_updater/build.sh --firmware-only

# Run configuration and simulated USB tests.
PYTHONPATH=tools/mac_updater:tools/win_updater:tools \
    python3 -m unittest discover -s tools/mac_updater/tests
```

The development flasher remains [tools/ea9flash.py](../ea9flash.py).
