#!/bin/sh
set -eu

here=$(cd "$(dirname "$0")" && pwd)
repo=$(cd "$here/../.." && pwd)
python=${PYTHON:-python3}
build="$repo/build/mac_updater"
out="$here/out"

case "${1:-}" in
    "") mode=all ;;
    --firmware-only) mode=firmware ;;
    --package-resources) mode=package ;;
    *) echo "Usage: $0 [--firmware-only|--package-resources]" >&2; exit 2 ;;
esac

if [ "$mode" != firmware ]; then
    if [ "$(uname -s)" != Darwin ]; then
        echo "Building the .app requires macOS. Use --firmware-only to build and validate its firmware." >&2
        exit 1
    fi
    "$python" -c "import PyInstaller, hid, tkinter; print('Tk', tkinter.TkVersion)"
fi

if [ "$mode" != package ]; then
    make -C "$repo" -B release
    PYTHONPATH="$repo/tools:$repo/tools/win_updater" \
        "$python" "$here/prepare.py" "$repo" "$build/resources"
fi
[ "$mode" != firmware ] || exit 0

PYTHONPATH="$repo/tools:$repo/tools/win_updater:$here" \
    "$python" -c 'from pathlib import Path; from firmware import Firmware; import sys; Firmware(Path(sys.argv[1]))' "$build/resources"

app="Community EA9 Firmware Updater"
"$python" -m PyInstaller --noconfirm --windowed --onedir \
    --target-arch "$(uname -m)" --name "$app" \
    --osx-bundle-identifier org.ea9community.updater \
    --distpath "$out" --workpath "$build/pyinstaller" --specpath "$build" \
    --paths "$repo/tools" --paths "$repo/tools/win_updater" \
    --add-data "$build/resources/ea9.bin:." \
    --add-data "$build/resources/firmware.json:." \
    --add-data "$build/resources/changelog.txt:." \
    "$here/app.py"
echo "Built $out/$app.app"
