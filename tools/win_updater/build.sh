#!/bin/sh
# build.sh -- the release firmware, its patch table, and the updater exe.
#
#   tools/win_updater/build.sh  -> tools/win_updater/out/Community EA9 Firmware Updater v<ver>.exe
#
# <ver> is Firmware.DeclaredVersion in src/Firmware.cs.
#
# The image embedded is the RELEASE build (make release); cfgblock.py checks
# that it holds exactly one lens configuration block before it is copied in.
set -e
here=$(cd "$(dirname "$0")" && pwd)
dotnet=${DOTNET:-$(command -v dotnet || echo "$HOME/.dotnet/dotnet")}

# Text with Chinese in it must be UTF-8, or the window shows mojibake.
# Strings.cs must also carry the BOM, so no editor can take it for GBK.
python3 -I - "$here/src/Strings.cs" "$here/changelog.txt" "$here/changelog.zh.txt" <<'PY'
import sys
for p in sys.argv[1:]:
    b = open(p, 'rb').read()
    try:
        b.decode('utf-8')
    except UnicodeDecodeError as e:
        sys.exit('%s is not valid UTF-8: %s' % (p, e))
    if p.endswith('Strings.cs') and not b.startswith(b'\xef\xbb\xbf'):
        sys.exit('%s must be saved as UTF-8 with a BOM' % p)
PY

mkdir -p "$here/firmware"
repo=$(cd "$here/../.." && pwd)
make -C "$repo" release
rm -f "$here/firmware/patchmap.json"
python3 -I "$here/cfgblock.py" "$repo/build/ea9-rebuild-release.bin" --out "$here/firmware/ea9.bin"
"$dotnet" build "$here/EA9Updater.csproj" -c Release -nologo -v quiet -o "$here/out"
# The shipped name.  Windows file names cannot hold ':', so it is dropped here
# (the window title has it).
ver=$(sed -n 's/.*DeclaredVersion = { *\([0-9]*\), *\([0-9]*\), *\([0-9]*\) *}.*/\1.\2.\3/p' "$here/src/Firmware.cs")
[ -n "$ver" ] || { echo "cannot read DeclaredVersion from src/Firmware.cs" >&2; exit 1; }
exe="$here/out/Community EA9 Firmware Updater v$ver.exe"
rm -f "$here/out/Community EA9 Firmware Updater v"*.exe
mv "$here/out/EA9Updater.exe" "$exe"
echo "built $exe"
