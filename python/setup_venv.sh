#!/bin/sh
# (c) Jan Zwiener (jan@zwiener.org)
# One-time setup of a venv for python/replay.py (PyYAML + pymavlink) and
# the INSLIB package itself (editable install, needs `make pylib` first).
# POSIX shell (Linux/macOS, or git-bash/MSYS on Windows) -- for a native
# cmd.exe prompt use setup_venv.bat instead.
#
# Usage:
#   sh python/setup_venv.sh              # creates python/.venv
#   sh python/setup_venv.sh /path/to/env # custom venv location
#
# Then:
#   . python/.venv/bin/activate          # (or: python/.venv/Scripts/activate on Windows)
#   python3 python/replay.py datasets/fog --realtime
set -e
cd "$(dirname "$0")/.."   # repo root

VENV="${1:-python/.venv}"

if [ ! -f python/INSLIB/libINSLIB.so ] && [ ! -f python/INSLIB/libINSLIB.dylib ] \
   && [ ! -f python/INSLIB/libINSLIB.dll ]; then
    echo "building libINSLIB (make pylib)..."
    make pylib
fi

echo "creating venv at $VENV ..."
python3 -m venv "$VENV"

# shellcheck disable=SC1091
. "$VENV/bin/activate" 2>/dev/null || . "$VENV/Scripts/activate"

pip install --upgrade pip >/dev/null
pip install -r python/requirements.txt
pip install -e python/

echo
echo "done. activate with:"
printf '  . %s/bin/activate   (Windows: %s\\Scripts\\activate)\n' "$VENV" "$VENV"
echo "then e.g.:"
echo "  python3 python/replay.py datasets/fog --realtime"
