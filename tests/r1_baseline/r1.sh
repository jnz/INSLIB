#!/usr/bin/env bash
# R1 regression check (REQ-VER-051): proves that the working tree computes
# exactly what a reference version computed, bit for bit.
#
#   tests/r1_baseline/r1.sh [REF]     REF: any git commit or tag, default HEAD
#
# Builds tools/replay.c twice with this machine's compiler and the Makefile's
# flags: once from REF (src/, tools/, Makefile and its KFCore commit, taken
# with git archive, nothing is checked out) and once from the working tree.
# state_dump_shim.c is linked into both with -Wl,--wrap=nav_suite_update and
# dumps the state after every epoch, every float in C99 hex. Every dataset
# replay of `make test` (read from the working tree's Makefile) then runs
# through both binaries on the working tree's datasets, and the dumps are
# compared with cmp. Nothing is stored in the repository: both sides are built
# on the same machine, so the comparison holds on any toolchain.
set -euo pipefail

cd "$(dirname "$0")/../.."
ROOT=$PWD
REF=${1:-HEAD}
SHIM="$ROOT/tests/r1_baseline/state_dump_shim.c"
CC=${CC:-cc}

TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT

# The reference sources without touching the working tree.
mkdir -p "$TMP/ref/KFCore"
git archive "$REF" src tools Makefile | tar -x -C "$TMP/ref"
git -C KFCore archive "$(git rev-parse "$REF:KFCore")" | tar -x -C "$TMP/ref/KFCore"

make_var() {
    make -s --no-print-directory -C "$1" --eval "r1-var: ; @echo \$($2)" r1-var
}

build() {
    local dir=$1 out=$2
    # shellcheck disable=SC2046
    (cd "$dir" && $CC $(make_var . HARNESS_CFLAGS) $(make_var . INCLUDES) \
        $(make_var . SUITE_SRC) $(make_var . KFCORE_SRC) tools/replay.c "$SHIM" \
        -Wl,--wrap=nav_suite_update -lm -o "$out")
}

echo "R1: building replay of $REF ($(git rev-parse --short "$REF")) and of the working tree with $($CC --version | head -1)"
build "$TMP/ref" "$TMP/replay_ref"
build "$ROOT" "$TMP/replay_cur"

list_runs() {
    make -n --no-print-directory datasets 2>/dev/null |
        grep -o 'build/replay [^ ;]*' | awk '$2 !~ /\$/ {print $2}'
    make_var . 'PEDESTRIAN_DATASETS) $(SIMULATED_DATASETS) $(SIMULATED_C_ONLY) $(CRAZYFLIE_DATASETS' | tr ' ' '\n'
}

fails=0
runs=0
while IFS= read -r run; do
    [ -n "$run" ] || continue
    runs=$((runs + 1))
    rc_ref=0
    rc_cur=0
    INSLIB_STATE_DUMP="$TMP/ref.state" "$TMP/replay_ref" "$run" >"$TMP/ref.log" 2>&1 </dev/null &
    pid=$!
    INSLIB_STATE_DUMP="$TMP/cur.state" "$TMP/replay_cur" "$run" >"$TMP/cur.log" 2>&1 </dev/null || rc_cur=$?
    wait "$pid" || rc_ref=$?
    epochs=$(($(wc -l <"$TMP/cur.state") - 1))
    if diff_at=$(cmp "$TMP/ref.state" "$TMP/cur.state" 2>&1) && [ "$rc_ref" = "$rc_cur" ]; then
        printf '  same  %-56s %8d epochs\n' "$run" "$epochs"
    else
        fails=$((fails + 1))
        printf '  DIFF  %-56s exit %d vs %d, %s\n' "$run" "$rc_ref" "$rc_cur" "$diff_at"
    fi
    rm -f "$TMP/ref.state" "$TMP/cur.state"
done < <(list_runs)

if [ "$fails" -eq 0 ]; then
    echo "R1: all $runs runs bit-identical to $REF"
else
    echo "R1: FAIL, $fails of $runs runs differ from $REF" >&2
    exit 1
fi
