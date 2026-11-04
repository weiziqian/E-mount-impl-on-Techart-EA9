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
- The embedded image is the release build, together with its `patchmap.json` (section 5).

## 4. UI

Top to bottom:

1. **Release notes**: multi-line, read-only text box showing the change log.
   - The change log is a separate file in the repo, e.g. `changelog.txt`, also embedded in the exe at build time.
2. **Focal length**: integer input, in mm, default `50`, range 8–600 (section 5.4).
3. **Lens name**: input box, default `TECHART LM-EA9`, at most 63 ASCII characters. Display only.
4. **Adapter**: connection state and bootloader version (see "Adapter detection" below).
5. **Write button**: starts writing the firmware.
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

The bootloader version is the triplet the bootloader recorded at the last START, i.e. what the last updater declared. It is **not** the firmware version the camera shows (message 0x07). This updater declares `Firmware.DeclaredVersion` (2.0.0) in START. There is no UI to change it.

## 5. Patching focal length and lens name

### 5.1 Firmware side (the only source change)

- The focal length lives in one word, `extern const uint16_t em_focal_mm10`, defined in `src/emount_packets.c` from the `EM_FOCAL_MM10` macro. Nothing else uses the macro. The packet templates hold zeros where the focal length goes, and `em_init` (`put_focal` in `src/emount.c`) writes it from that word into 0x05 pl[24..27], 0x08 pl[16..17] (BCD), 0x28 pl[4..7] and 0x35 pl[4..7]. The code that computes with it (`src/main.c`) reads the same word. Because the value is in another file, the compiler cannot fold it into instructions; the word in flash is the only copy. This only holds without LTO, and `locate.py` fails if it stops holding.
- The lens name stays in the `em_msg_3F` template. It is initialised data, so it is plain bytes in the image.

### 5.2 Release step: `locate.py`

`tools/win_updater/locate.py -o patchmap.json` builds the release image three times:

| Build | Source | Focal | Name |
|---|---|---|---|
| A | the repo as it is: **this is the image that ships** | 50 | `TECHART LM-EA9` |
| B | temporary copy | 137 | `Z` × 63 |
| C | temporary copy | 85 | `Summicron-M 35mm f/2 ASPH` |

B and C are built in a scratch copy of `src/` and the `Makefile`; the repo is never edited.

1. A and B must have the same length. Every byte that differs must be one of: focal u16 LE (mm × 10), focal BCD (whole mm), or the 63-byte name field. Anything else stops the script.
2. The number of locations must match the current source: 1 × u16, 0 × BCD, 1 × name. A different count means the source changed and must be looked at.
3. A is patched to B's and C's values with the table, and each result must equal the real build **byte for byte**.
4. It writes `patchmap.json`: image length, image SHA-256, defaults, and the list of `{offset, kind, len}`.

Currently found (file offsets in `build/ea9-rebuild-release.bin`):

| Offset | Kind | Where |
|---|---|---|
| 0x04278 | u16le_mm10 | `em_focal_mm10` |
| 0x04f7b | name (63 B) | 0x3F pl[1..63] |

The offsets move with every rebuild; always use the `patchmap.json` produced for the image being embedded.

### 5.3 What the updater does

1. Check that the embedded image's length and SHA-256 match `patchmap.json`. Otherwise refuse to start.
2. For each field, write the encoding of the user's values (same as `encode()` in `locate.py`):
   - `u16le_mm10`: focal mm × 10, 2 bytes little-endian
   - `bcd_mm`: two BCD bytes, `bcd(mm / 100)`, `bcd(mm % 100)` (50 → `00 50`, 135 → `01 35`)
   - `name`: ASCII, NUL-padded to 63 bytes
3. Write the patched image to the device.

No checksum needs fixing: the firmware recomputes frame checksums on every send (`transmit()` in `src/emount.c`), and the image has no CRC or signature.

### 5.4 Input rules

- **Focal length**: integer mm, default 50, minimum 8, maximum 600.
- **Lens name**: printable ASCII (0x20–0x7E), 1–63 characters, default `TECHART LM-EA9`. It only affects what the camera displays.

### 5.5 Release

- **Releases are built with the release configuration only.** `locate.py` runs `make release` itself and embeds nothing else. Never ship a debug build: it writes a diagnostic trail to flash.
- Embed exactly the `build/ea9-rebuild-release.bin` and `patchmap.json` from the same `locate.py` run.

## 6. Building

```
tools/win_updater/build.sh        # -> tools/win_updater/out/EA9Updater.exe
```

1. `locate.py` runs `make release`, finds the patch fields, and writes `firmware/ea9.bin` + `firmware/patchmap.json` from the same run.
2. `dotnet build -c Release` compiles the exe with both files and `changelog.txt` embedded. The build fails if `firmware/` is missing.

Needs the ARM toolchain and ASF4 (README section 4) and the .NET SDK (8.x; `DOTNET=` overrides the path). The output is one file, `EA9Updater.exe`; there is no `.exe.config`.

Source layout:

| File | Content |
|---|---|
| `src/Hid.cs` | Windows HID via P/Invoke; overlapped I/O with timeouts |
| `src/Flasher.cs` | the bootloader protocol, mirroring `tools/ea9flash.py` |
| `src/Firmware.cs` | embedded resources, `patchmap.json` checks, the patch |
| `src/MainForm.cs` | the window, laid out in code |
| `changelog.txt` | the release notes shown in the window |

## 7. Open questions

None at the moment.
