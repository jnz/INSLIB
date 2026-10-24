#!/usr/bin/env bash
# (c) Jan Zwiener (jan@zwiener.org)
#
# Regenerate the "Test coverage" block near the top of readme.md from
# coverage/coverage-core.info (produced by `make coverage`), so the
# published C0/C1/MC-DC numbers come out of lcov's own machine-readable
# .info fields (LF/LH, BRF/BRH, MCF/MCH), not a hand-typed table someone
# forgot to refresh. No commit hash is stamped: this repo's history gets
# re-exported into a public repo with different commit hashes, so a
# stamped hash here would go stale/misleading immediately.
# See CLAUDE.md's release process, step 4.
#
# Usage:
#   make coverage                       # regenerates coverage-core.info
#   scripts/update_readme_coverage.sh
set -euo pipefail
cd "$(dirname "$0")/.." # repo root

INFO="coverage/coverage-core.info"
README="readme.md"
START="<!-- COVERAGE:START -->"
END="<!-- COVERAGE:END -->"

[ -f "$INFO" ] || {
    echo "update_readme_coverage.sh: $INFO not found - run 'make coverage' first." >&2
    exit 1
}
grep -qF "$START" "$README" && grep -qF "$END" "$README" || {
    echo "update_readme_coverage.sh: $START/$END markers not found in $README." >&2
    exit 1
}

if ! git diff --quiet -- src/ tests/ || ! git diff --cached --quiet -- src/ tests/; then
    echo "update_readme_coverage.sh: warning: src/ or tests/ has uncommitted" \
         "changes - these coverage numbers won't reflect what actually ships" \
         "until you commit them too." >&2
fi

read -r lf lh bf bh mf mh <<<"$(awk -F: '
    /^LF:/  { lf += $2 }
    /^LH:/  { lh += $2 }
    /^BRF:/ { bf += $2 }
    /^BRH:/ { bh += $2 }
    /^MCF:/ { mf += $2 }
    /^MCH:/ { mh += $2 }
    END     { print lf, lh, bf, bh, mf, mh }
' "$INFO")"

pct() { awk -v h="$1" -v t="$2" 'BEGIN { printf "%.1f", (t > 0 ? 100.0 * h / t : 0) }'; }

c0="$(pct "$lh" "$lf")"
c1="$(pct "$bh" "$bf")"
mcdc="$(pct "$mh" "$mf")"

block="$(cat <<EOF
$START
**Test coverage** of the core library (\`src/\`):

| Metric | Coverage |
|---|---|
| C0 (Line) | ${c0}% ($lh/$lf) |
| C1 (Branch) | ${c1}% ($bh/$bf) |
| MC/DC | ${mcdc}% ($mh/$mf) |
$END
EOF
)"

awk -v block="$block" -v start="$START" -v end="$END" '
    $0 == start { print block; skip = 1; next }
    $0 == end   { skip = 0; next }
    !skip       { print }
' "$README" >"$README.tmp" && mv "$README.tmp" "$README"

echo "readme.md coverage block updated: C0 ${c0}%, C1 ${c1}%, MC/DC ${mcdc}%"
