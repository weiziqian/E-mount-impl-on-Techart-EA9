#!/bin/bash
# Set up the host tools only. Firmware writes are separate, explicit commands.
set -euo pipefail

if [[ "${1:-}" == "--help" ]]; then
    printf '%s\n' 'Usage: tools/install_macos.sh' \
        'Requires Python 3.9+. Set EA9_PYTHON to select a Python executable.' \
        'Creates .venv-macos in the repository and installs the host dependencies.'
    exit 0
fi
if [[ $# -ne 0 ]]; then
    printf '%s\n' 'Usage: tools/install_macos.sh' >&2
    exit 2
fi
if [[ "$(uname -s)" != "Darwin" ]]; then
    printf '%s\n' 'This setup script is for macOS. Linux uses python3 tools/ea9flash.py directly.' >&2
    exit 1
fi

ea9_script_dir="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ea9_repo_root="$(CDPATH= cd -- "$ea9_script_dir/.." && pwd)"
ea9_python="${EA9_PYTHON:-python3}"
ea9_venv="$ea9_repo_root/.venv-macos"

if ! command -v "$ea9_python" >/dev/null 2>&1 || \
   ! "$ea9_python" -c 'import sys; sys.exit(sys.version_info < (3, 9))'; then
    printf '%s\n' 'Install Python 3.9+ from https://www.python.org/downloads/macos/ and rerun.' >&2
    exit 1
fi

if [[ ! -x "$ea9_venv/bin/python" ]]; then
    "$ea9_python" -m venv "$ea9_venv"
fi
"$ea9_venv/bin/python" -m pip install --disable-pip-version-check \
    -r "$ea9_script_dir/requirements-macos.txt"
"$ea9_venv/bin/python" -c 'import hid, dnfile; assert callable(hid.device) and callable(hid.enumerate)'

printf '\nSetup complete. Run commands from:\n  cd %q\n' "$ea9_repo_root"
printf '%s\n' \
    '' \
    'Read-only connection check (adapter on USB, no camera or lens):' \
    '  .venv-macos/bin/python tools/ea9flash.py --check' \
    '' \
    'See docs/macos.md for release extraction, backup, flashing and verification.'
