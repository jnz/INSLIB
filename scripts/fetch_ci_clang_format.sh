#!/usr/bin/env bash
# (c) Jan Zwiener (jan@zwiener.org)
#
# Vendor the exact clang-format that `make format`/`make format-check`
# are judged with into .tools/clang-format/, without touching any system
# package database. .github/workflows/ci.yml runs this same script, so
# this file is the one place the version is pinned.
#
# Why: clang-format's output depends on its exact version - releases
# disagree on ambiguous constructs like `(MACRO_PARAM)*x` vs
# `(MACRO_PARAM) * x`. `make format`/`make format-check` only reproduce
# CI's verdict when run with the same build. The Makefile picks up the
# vendored binary automatically when it exists, falling back to the
# `clang-format` on PATH otherwise.
#
# Usage:
#   scripts/fetch_ci_clang_format.sh
#
# Installs the `clang-format` wheel from PyPI (official LLVM release
# binaries, Linux/macOS/Windows), so it works the same on a CI runner and
# on any developer machine. A file on PyPI can never be replaced, so a
# pinned version always yields the same binary. To move to a newer
# clang-format, bump VERSION, run `make format` and commit both together.
set -euo pipefail
cd "$(dirname "$0")/.." # repo root

VERSION="19.1.7"
DEST=".tools/clang-format"
PYTHON="${PYTHON:-python3}"
BIN="$DEST/clang_format/data/bin/clang-format"

if [ -x "$BIN" ] && "$BIN" --version | grep -q "version $VERSION\$"; then
    echo "already installed: $("$BIN" --version)"
    exit 0
fi

rm -rf "$DEST"
"$PYTHON" -m pip install --quiet --disable-pip-version-check --no-deps \
    --target "$DEST" "clang-format==$VERSION"

echo "installed: $("$BIN" --version)"
