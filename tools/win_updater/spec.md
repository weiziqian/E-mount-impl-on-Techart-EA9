# EA9 Windows Firmware Updater — Requirements

A small Windows GUI program: the user plugs in USB, clicks one button, and the firmware is written to the LM-EA9.

## 1. Technology

- **C# + WinForms, targeting .NET Framework 4.8.** Windows 10/11 ship with it, so the user installs nothing.
- The output is a **single exe**: no extra DLLs, no installer.
- **Must build on Ubuntu:**
  - Use `dotnet build` (.NET SDK).
  - SDK-style `.csproj` with `<TargetFramework>net48</TargetFramework>`, referencing the NuGet package `Microsoft.NETFramework.ReferenceAssemblies`.
  - Do not depend on the Visual Studio designer; lay out the UI in code.
- **USB HID via P/Invoke** to the Windows system DLLs `hid.dll`, `setupapi.dll` and `kernel32.dll`. No third-party libraries.
- Running and testing happen on Windows.

## 2. Function: write EA9 firmware over USB

- The protocol is exactly the one in `tools/ea9flash.py`, which is the reference:
  - Device VID:PID = `0483:575A`, 64-byte HID reports.
  - Sequence: CHECK → START (with a version triplet) → send 64-byte chunks, one per ACK → pad with zeros up to `(len // 1024) * 1024 + 1024`.
- Windows I/O details:
  - Write: 65 bytes = report ID `0x00` + 64 bytes.
  - Read: 65 bytes arrive; the first byte is the report ID and must be stripped.
- If no device is found, tell the user that the adapter must be plugged into the PC's USB and **must not be mounted on the camera**.
- Writing must not freeze the UI and must show progress. Disable the button while writing, so it cannot be clicked twice.

## 3. Firmware bundled in the exe

- The `.bin` firmware is built from source by `build.sh` (release configuration) and compiled into the exe as an embedded resource (EmbeddedResource). The generated `firmware/` directory is not committed.
- The user does not need to, and cannot, choose an external .bin file.
- The version declared in START is also stored in the project, next to the firmware. The user does not enter it.
- The embedded image is the release build (section 5.4).

## 4. UI

**Language.** English and Chinese, from `src/Strings.cs`. At start the window is Chinese if the Windows display language is Chinese (`zh`, simplified or traditional), English otherwise; the choice is not remembered. A drop-down at the top right (English / 中文) switches at once and keeps what the user typed. The change log follows it (`changelog.txt` / `changelog.zh.txt`). Errors from the HID and firmware layers stay in English. Lens names stay ASCII (5.3), so the Chinese hint reads "镜头名称只允许输入英文字母、数字和符号".

Top to bottom:

1. **Release notes**: multi-line, read-only text box showing the change log.
   - The change log is a separate file in the repo, e.g. `changelog.txt`, also embedded in the exe at build time.
2. **Adapter version** (the hardware version): drop-down, choices `2` and `1`, default `2`. A "How to choose?" link next to it opens a help page (`src/HelpPage.cs`):
   - a picture telling the two apart (`assets/hw_version.jpg`, embedded): version 1 has a single continuous metal spring strip, version 2 a separate triangular block;
   - **What is the difference**: version 1 has less room past infinity, so to focus at infinity it must retract to its end stop; version 2 need not, which is easier on the mechanism;
   - **How to choose**: version 2, unless the adapter is version 1 hardware and cannot focus at infinity.
3. **Focal length**: radio buttons, **Multiple Lenses** (default) or **Single Lens**, and, in **Single** only, a "How to choose?" link that opens a help page (`src/HelpPage.cs`):
   - **Single Lens**: for one lens; the aperture is fixed while shooting, so it cannot be changed by mistake;
   - **Multiple Lenses**: enter each lens in the row of the aperture chosen for it; to change lenses, set that aperture and press the shutter, and the adapter restarts with that lens; then set the aperture back to its widest. Only the inputs of the chosen mode are shown:
   - **Single**: a focal length box (default `50`), an exit pupil drop-down and a lens name box (default `Community EA9 Adapter`).
   - **Multiple**: a table with one row per slot:

     | Column | Content |
     |---|---|
     | Aperture | `f2.5`, `f2.8`, `f3.2`, `f3.5`, `f4.0`, `f4.5`, `f5.0`, `f5.6`, read-only: the aperture that selects the slot on the camera (row 1 is entry index 0); errors and the confirmation name a row by it |
     | Focal Length (mm) | input, empty by default, digits only |
     | Lens Name | input, empty by default; hidden while the slot's focal length is empty or 0 |
     | Exit Pupil (mm) | drop-down, `25 30 35 40 45 50 55 60 70`; hidden while the slot's focal length is empty or 0 |

   Below the table, on its own line and right-aligned, a "How to choose the exit pupil?" link opens a help page (what the exit pupil is for; keep the preselected value unless the lens's is known; try the neighbours if off-centre focus is consistently off). In **Single** the same page opens from a plain "How to choose?" link next to the exit pupil drop-down.

   **Exit pupil preselection.** Whenever a focal length changes (in either mode), its exit pupil drop-down is set from it: ≤ 30 mm → 30, ≤ 35 → 35, ≤ 40 → 40, ≤ 50 → 45, ≤ 60 → 50, otherwise 60 (`OpticalFile.AutoXpd`, `gen_optical.py auto_xpd`). The user may then pick another value; it is kept until the focal length changes again.

   Input rules are in section 5.3.
4. **Adapter**: connection state and bootloader version (see "Adapter detection" below).
5. **Write button**: checks the configuration (section 5.3), shows it in a confirmation dialog, then writes the firmware.
   - Enabled only while an adapter that answered CHECK is connected.
   - A progress bar shows write progress.
6. **When writing finishes**: a dialog tells the user to **unplug the USB**.
   - If writing fails, a dialog says why.
   - After a successful write the button stays disabled until the adapter is unplugged and plugged in again.

### Adapter detection

Every 2 seconds the updater looks for the adapter on USB. A poll only enumerates USB devices and does not talk to the adapter. When an adapter appears, the updater sends CHECK (read-only) **once** to read its version. This runs in the background and does not freeze the UI. Polling pauses while writing.

| State | Adapter row | Status line | Write button |
|---|---|---|---|
| Not found | `Not connected` | `Please connect the adapter to this computer by USB.` | disabled |
| Found, answered CHECK | `Connected, bootloader version x.y.z` | ready | enabled |
| Found, no valid answer | `Not responding` | asks the user to unplug and plug it in again | disabled |
| Written successfully | `Firmware written` | `Done. Unplug the USB cable.` | disabled until the adapter is unplugged and plugged in again |

If writing fails, the next poll checks the adapter again.

The bootloader version is the triplet the bootloader recorded at the last START, i.e. what the last updater declared. It is **not** the firmware version the camera shows (message 0x07). This updater declares `Firmware.DeclaredVersion` (2.4.0) in START. There is no UI to change it.

## 5. The lens configuration

### 5.1 Firmware side

The firmware keeps its whole configuration in one struct, `struct lens_cfg` in `src/lens_cfg.h` (`src/lens_cfg.c` holds the default). It starts with a magic, so the updater finds it by searching the image and overwrites it in place. Nothing else in the image is patched, and no addresses are looked up at build time.

Layout (2908 bytes, little-endian):

| Offset | Field | Content |
|---|---|---|
| +0 | `magic[10]` | `"CfgAnchor"` + NUL |
| +10 | `hw_version` | 1 or 2 |
| +11 | `pad` | 0 |
| +12 | `lens[8]` | 362 bytes each: `focal_mm` u16 (0 = slot not set), `name[64]` ASCII, NUL-padded, at most 63 characters, `optical[296]` the slot's message 0x3B file (all zero when the slot is not set) |

The optical file is built by the updater from the slot's focal length and exit pupil (`src/OpticalFile.cs`, a port of `tools/gen_optical.py`, which is the reference; the two give identical bytes for every focal length 8–600 mm and every exit pupil choice):

- focus axis: closest focus at the full 4.5 mm extension, 8 points equally spaced in extension, distance codes computed as `src/focus_dist.c` does;
- type 0: exit pupil × 15 at infinity, (exit pupil + 4.5 mm) × 15 at the closest focus, the same in all six columns;
- type 1: f/1.8 at the centre, f/1.8 × (6 − 0.06 × focal) at the edge, with the focal length clamped to 30–70 mm in this formula; type 2 the same shape, a quarter of the rise.

The firmware serves the file of the slot in use through messages 0x3A/0x3B (`lens_cfg_apply` sets `em_optical_file`); a slot whose file does not start with `LF` falls back to the built-in `optical_file[]`.

At boot the firmware reads the block (`lens_cfg_boot`):
- If any slot 1..7 is set, it is a multiple-focal-length lens: on the camera, setting the aperture of a slot (f/2.5 … f/5.6) and taking a picture switches to that slot. If the saved slot is missing or not set, the firmware uses the first set slot, searching up from slot 0. So slot 0 may be empty when several slots are set.
- If only slot 0 is set, it is a single lens at f/1.8.

### 5.2 What the updater does

1. At start-up, find the magic in the embedded image. It must occur exactly once, the block must fit in the image, `hw_version` must be 1 or 2, `pad` 0 and every name NUL-terminated. Otherwise the updater says it is damaged and refuses to write.
2. When **Write** is clicked, build the block from the form (5.3). If that fails, a dialog says why and nothing is written.
3. Copy the image, write the 2908-byte block over the old one, and write that copy to the device.

No other checksum needs fixing: the firmware recomputes frame checksums on every send, and the image has no CRC or signature. The optical file's own check word is computed by `OpticalFile.Build`.

### 5.3 Input rules and normalisation

Applied in this order (`LensConfig.FromForm` in `src/LensConfig.cs`):

1. **Single Lens** is slot 0; slots 1..7 are written as unset.
2. A slot whose focal length is empty or 0 is written as `focal_mm = 0` with an empty name, whatever its name box holds.
3. A set focal length must be 8–600 mm.
4. A slot with a focal length but an empty name gets the name `Community EA9 Adapter`.
5. A name is printable ASCII (0x20–0x7E), at most 63 characters; leading and trailing spaces are trimmed.
6. **Multiple Lenses** with exactly one slot set: that slot is moved to slot 0 (the firmware then treats it as a single lens).
7. **Multiple Lenses** with two or more slots set: they stay where they are; slot 1 (f2.5) may be empty.
8. A set slot must have an exit pupil chosen from the list (it always has one once a focal length is typed).
9. No slot set, in either mode: error, "No lens is configured".

### 5.4 Release

- **Releases are built with the release configuration only.** `build.sh` runs `make release` and embeds that image. Never ship a debug build: it writes a diagnostic trail to flash.
- `cfgblock.py` checks the image before it is embedded: the magic occurs exactly once, the block parses, and every set slot's optical file equals `gen_optical.build()` for its focal length and exit pupil (read back from type 0). It prints the default block, and `pack()` in it is the reference encoding that `LensConfig.Encode()` must match. The firmware's default slots get their files from `src/lens_cfg_optical.h`, generated by `tools/gen_optical.py --defaults`.

## 6. Building

```
tools/win_updater/build.sh        # -> tools/win_updater/out/Community EA9 Firmware Updater v<ver>.exe
```

1. `make release` builds the firmware; `cfgblock.py` checks it and copies it to `firmware/ea9.bin`.
2. `dotnet build -c Release` compiles the exe with `firmware/ea9.bin` and `changelog.txt` embedded. The build fails if the image is missing.

Needs the ARM toolchain and ASF4 (README section 4) and the .NET SDK (8.x; `DOTNET=` overrides the path). The output is one file, `Community EA9 Firmware Updater v<ver>.exe` (`<ver>` = `Firmware.DeclaredVersion`); there is no `.exe.config`.

Source layout:

| File | Content |
|---|---|
| `src/Hid.cs` | Windows HID via P/Invoke; overlapped I/O with timeouts |
| `src/Flasher.cs` | the bootloader protocol, mirroring `tools/ea9flash.py` |
| `src/Firmware.cs` | embedded resources, finding and checking the configuration block, the patch |
| `src/LensConfig.cs` | the configuration block: form → normalised slots → 2908 bytes |
| `src/OpticalFile.cs` | a slot's message 0x3B optical file from its focal length and exit pupil; port of `tools/gen_optical.py` |
| `src/HelpPage.cs`, `assets/hw_version.jpg` | the "How to choose?" help pages: adapter version, focal length, exit pupil |
| `cfgblock.py` | release check of the image; reference encoding |
| `src/MainForm.cs` | the window, laid out in code |
| `src/Strings.cs` | every text in the window, English and Chinese; UTF-8 with BOM |
| `changelog.txt`, `changelog.zh.txt` | the release notes shown in the window, English and Chinese |

## 7. Open questions

None at the moment.
