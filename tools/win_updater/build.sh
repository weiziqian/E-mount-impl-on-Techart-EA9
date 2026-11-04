#!/bin/sh
# build.sh -- the release firmware, its patch table, and the updater exe.
#
#   tools/win_updater/build.sh        -> tools/win_updater/out/EA9Updater.exe
#
# locate.py builds the RELEASE image (make release) and finds the focal and
# name fields; the image and the table it writes are what the exe embeds.
set -e
here=$(cd "$(dirname "$0")" && pwd)
dotnet=${DOTNET:-$(command -v dotnet || echo "$HOME/.dotnet/dotnet")}

mkdir -p "$here/firmware"
python3 -I "$here/locate.py" -o "$here/firmware/patchmap.json" --bin "$here/firmware/ea9.bin"
"$dotnet" build "$here/EA9Updater.csproj" -c Release -nologo -v quiet -o "$here/out"
echo "built $here/out/EA9Updater.exe"
