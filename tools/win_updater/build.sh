#!/bin/sh
# build.sh -- the release firmware, its patch table, and the updater exe.
#
#   tools/win_updater/build.sh  -> tools/win_updater/out/Community EA9 Firmware Updater v<ver>.exe
#
# <ver> is Firmware.DeclaredVersion in src/Firmware.cs.
#
# locate.py builds the RELEASE image (make release) and finds the focal and
# name fields; the image and the table it writes are what the exe embeds.
set -e
here=$(cd "$(dirname "$0")" && pwd)
dotnet=${DOTNET:-$(command -v dotnet || echo "$HOME/.dotnet/dotnet")}

mkdir -p "$here/firmware"
python3 -I "$here/locate.py" -o "$here/firmware/patchmap.json" --bin "$here/firmware/ea9.bin"
"$dotnet" build "$here/EA9Updater.csproj" -c Release -nologo -v quiet -o "$here/out"
# The shipped name.  Windows file names cannot hold ':', so it is dropped here
# (the window title has it).
ver=$(sed -n 's/.*DeclaredVersion = { *\([0-9]*\), *\([0-9]*\), *\([0-9]*\) *}.*/\1.\2.\3/p' "$here/src/Firmware.cs")
[ -n "$ver" ] || { echo "cannot read DeclaredVersion from src/Firmware.cs" >&2; exit 1; }
exe="$here/out/Community EA9 Firmware Updater v$ver.exe"
rm -f "$here/out/Community EA9 Firmware Updater v"*.exe
mv "$here/out/EA9Updater.exe" "$exe"
echo "built $exe"
