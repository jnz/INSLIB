# (c) Jan Zwiener (jan@zwiener.org)
# Source this to activate the INSLIB python venv (python/.venv).
# POSIX shell (Linux/macOS) and git-bash/MSYS on Windows.
#
# Usage:
#   . env.sh              # from repo root
#   . /path/to/INSLIB/env.sh   # from elsewhere
#
# If python/.venv does not exist yet, run python/setup_venv.sh first
# (native cmd.exe: python\setup_venv.bat).

_inslib_root="$(cd "$(dirname "${BASH_SOURCE:-$0}")" && pwd)"
_inslib_venv="$_inslib_root/python/.venv"

if [ -f "$_inslib_venv/bin/activate" ]; then
    . "$_inslib_venv/bin/activate"
elif [ -f "$_inslib_venv/Scripts/activate" ]; then
    . "$_inslib_venv/Scripts/activate"
else
    echo "env.sh: no venv found at $_inslib_venv" >&2
    echo "env.sh: run 'python/setup_venv.sh' first (native cmd.exe: python\\setup_venv.bat)" >&2
    unset _inslib_root _inslib_venv
    return 1 2>/dev/null || exit 1
fi

unset _inslib_root _inslib_venv
