#!/bin/sh
# (c) Jan Zwiener (jan@zwiener.org)
# One-time repo setup: init the KFCore submodule, build libINSLIB and
# create the python/.venv (PyYAML + pymavlink + editable INSLIB install).
# POSIX shell (Linux/macOS, or git-bash/MSYS on Windows).
#
# Usage:
#   sh setup.sh
#
# Then:
#   . env.sh                             # activate the venv
#   python3 python/replay.py datasets/fog --realtime
set -e
cd "$(dirname "$0")"   # repo root

if [ ! -f KFCore/c/kalman_udu.c ]; then
    echo "initializing KFCore submodule..."
    git submodule update --init --recursive
fi

sh python/setup_venv.sh

echo
echo "setup done. activate with:"
echo "  . env.sh"
