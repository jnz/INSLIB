#!/usr/bin/env bash
# Export the current, clean, tagged state of this (private) repo into a
# local clone of the public INSLIB repo, as one squashed release commit.
#
# Usage:
#   scripts/release_public.sh <path-to-public-repo-clone> <version-tag>
#
# Example:
#   scripts/release_public.sh ../INSLIB-public v1.2.0
#
# Preconditions:
#   - Both repos have a clean working tree.
#   - You already tagged the private repo at the commit you want to ship
#     (git tag <version-tag>) and are checked out there.
#   - <path-to-public-repo-clone> is a local clone of the public repo with
#     its own git history - this script never touches the private repo's
#     git history or remotes.
#
# The script does NOT push. Review the diff and push yourself.
#
# Verifies readme.md's coverage block is current for this exact commit
# (see CLAUDE.md release process step 4), then runs `make check-all`
# against the exported tree before committing - this needs a POSIX
# environment with the toolchain from coding_style.md /
# .github/workflows/ci.yml (WSL, not native Windows: `make check-all`
# itself refuses to run outside POSIX).
#
# Note that the coverage check only runs for real on gcc >= 14.2 with
# lcov >= 2, the toolchain that can produce MC/DC numbers. Ubuntu 22.04,
# which the static analysis gates are pinned to, cannot. On such a
# toolchain the check aborts with instructions, and
# INSLIB_COVERAGE_VERIFIED_ELSEWHERE=1 skips it after you confirmed the
# block on the newer box. See the long comment at the check itself.

set -euo pipefail

PUBLIC_REPO_PATH="${1:?usage: release_public.sh <path-to-public-repo-clone> <version-tag>}"
VERSION="${2:?usage: release_public.sh <path-to-public-repo-clone> <version-tag>}"

PRIVATE_ROOT="$(git rev-parse --show-toplevel)"
EXCLUDE_FILE="$PRIVATE_ROOT/scripts/public_export.exclude"
SYNC_MARKER="$PRIVATE_ROOT/.public_sync_head"

if [[ ! -d "$PUBLIC_REPO_PATH/.git" ]]; then
    echo "error: $PUBLIC_REPO_PATH is not a git repository" >&2
    exit 1
fi

# --untracked-files=no: untracked files are fine here (e.g. the local
# config.yaml or datasets/baroalt_mag01/ that public_export.exclude already
# keeps out of the export) - only uncommitted changes to tracked files
# should block a release.
if [[ -n "$(git -C "$PRIVATE_ROOT" status --porcelain --untracked-files=no)" ]]; then
    echo "error: private repo working tree has uncommitted changes to tracked files" >&2
    exit 1
fi

# readme.md's <!-- COVERAGE:START/END --> block must reflect this exact
# commit (CLAUDE.md release process step 4). Re-run the generator and
# diff instead of trusting that step was done: it deterministically
# reproduces the block from coverage-core.info's LF/LH/BRF/BRH/MCF/MCH
# fields, so any diff here means it's stale (skipped, or run before the
# last src/tests/ change) - same pattern as `format-check` vs. `format`.
#
# That only holds on the toolchain the block was generated with. MC/DC
# needs gcc >= 14.2 (-fcondition-coverage) and lcov >= 2
# (--mcdc-coverage), see the MCDC_SUPPORTED block in the Makefile. An
# older toolchain writes no MCF/MCH fields at all, so MC/DC regenerates
# as "0.0% (/)", and even the branch counts come out different, because
# gcc's branch accounting changes between releases. Regenerating there
# would report a stale block for every release no matter how current it
# is, so detect that case and say what is actually wrong instead.
#
# This is not a corner case: the environment the rest of this script
# wants (Ubuntu 22.04, matching the pinned cppcheck/clang-tidy/doxygen
# versions in .github/workflows/ci.yml) ships gcc 11, which cannot do
# MC/DC at all, so the coverage block has to come from a separate, newer
# box. Running the whole export from that box is no fix either, it just
# moves the failure to the static analysis gates, whose versions are
# pinned to 22.04 and not vendored the way clang-format is.
CC_FOR_COVERAGE="${CC:-cc}"
GCC_VERSION_NUM="$("$CC_FOR_COVERAGE" -dumpfullversion 2>/dev/null |
    awk -F. '{ printf "%d%02d%02d", $1, $2, $3 }')"
LCOV_VERSION_MAJOR="$(lcov --version 2>/dev/null |
    sed -n 's/.*version \([0-9]*\).*/\1/p')"
TOOLCHAIN_DESC="gcc $("$CC_FOR_COVERAGE" -dumpfullversion 2>/dev/null || echo '?'), lcov ${LCOV_VERSION_MAJOR:-?}.x"

MCDC_CAPABLE=0
if [[ -n "$GCC_VERSION_NUM" && "$GCC_VERSION_NUM" -ge 140200 ]] \
   && [[ -n "$LCOV_VERSION_MAJOR" && "$LCOV_VERSION_MAJOR" -ge 2 ]]; then
    MCDC_CAPABLE=1
fi

if [[ "$MCDC_CAPABLE" == "1" ]]; then
    echo "Checking readme.md coverage block is current ($TOOLCHAIN_DESC) ..."
    ( cd "$PRIVATE_ROOT" && make coverage >/dev/null && scripts/update_readme_coverage.sh )
    if [[ -n "$(git -C "$PRIVATE_ROOT" status --porcelain -- readme.md)" ]]; then
        echo "error: readme.md's coverage block was stale for this commit:" >&2
        git -C "$PRIVATE_ROOT" diff -- readme.md >&2
        git -C "$PRIVATE_ROOT" checkout -- readme.md
        echo "run 'make readme-coverage', commit the change, re-tag, and re-run this script." >&2
        exit 1
    fi
elif [[ "${INSLIB_COVERAGE_VERIFIED_ELSEWHERE:-}" == "1" ]]; then
    echo "note: skipping the readme.md coverage check, this toolchain" \
         "($TOOLCHAIN_DESC) cannot reproduce MC/DC."
    echo "      INSLIB_COVERAGE_VERIFIED_ELSEWHERE=1 is set, so the block is taken" \
         "as verified on the gcc >= 14.2 machine."
else
    echo "error: readme.md's coverage block cannot be verified on this toolchain." >&2
    echo "  found:  $TOOLCHAIN_DESC" >&2
    echo "  needed: gcc >= 14.2 and lcov >= 2, otherwise MC/DC regenerates as" >&2
    echo "          \"0.0% (/)\" and the branch counts differ from the committed ones." >&2
    echo >&2
    echo "  Either run 'make readme-coverage' on the gcc >= 14.2 machine, confirm" >&2
    echo "  readme.md is unchanged there, and re-run this script with" >&2
    echo "    INSLIB_COVERAGE_VERIFIED_ELSEWHERE=1 scripts/release_public.sh ..." >&2
    echo "  or run this script on that machine, where the check runs for real." >&2
    exit 1
fi

if [[ -n "$(git -C "$PUBLIC_REPO_PATH" status --porcelain)" ]]; then
    echo "error: public repo working tree ($PUBLIC_REPO_PATH) is not clean" >&2
    exit 1
fi

# Warn if the public repo moved since the last export (e.g. someone merged
# an issue fix or you edited something directly there) - overwriting that
# silently would look like an unfriendly revert to anyone watching the repo.
# A brand-new clone has no HEAD yet (first-ever release), which is fine.
# --verify (not plain rev-parse): on an unborn branch, plain "rev-parse
# HEAD" can print the literal string "HEAD" and exit 0 instead of failing,
# which would defeat the "|| echo ''" fallback below.
CURRENT_PUBLIC_HEAD="$(git -C "$PUBLIC_REPO_PATH" rev-parse --verify -q HEAD 2>/dev/null || echo "")"
if [[ -f "$SYNC_MARKER" && -n "$CURRENT_PUBLIC_HEAD" ]]; then
    LAST_SYNC_HEAD="$(cat "$SYNC_MARKER")"
    if [[ "$LAST_SYNC_HEAD" != "$CURRENT_PUBLIC_HEAD" ]]; then
        echo "warning: public repo HEAD changed since the last export."
        echo "  last exported public HEAD: $LAST_SYNC_HEAD"
        echo "  current public HEAD:       $CURRENT_PUBLIC_HEAD"
        echo "  inspect with: git -C \"$PUBLIC_REPO_PATH\" log $LAST_SYNC_HEAD..HEAD"
        read -rp "Continue and overwrite the working tree anyway? [y/N] " ans
        [[ "$ans" == "y" || "$ans" == "Y" ]] || exit 1
    fi
else
    echo "note: no previous export recorded, this looks like the first release."
fi

echo "Syncing $PRIVATE_ROOT -> $PUBLIC_REPO_PATH ..."
rsync -a --delete \
    --exclude-from="$EXCLUDE_FILE" \
    --filter='P /.git' \
    "$PRIVATE_ROOT"/ "$PUBLIC_REPO_PATH"/

# Sanity check: nothing private-only leaked through despite the exclude list.
if find "$PUBLIC_REPO_PATH" -iname '*embedded*' -not -path '*/.git/*' | grep -q .; then
    echo "error: a path matching 'embedded' is present in the export, aborting" >&2
    echo "review $EXCLUDE_FILE before retrying" >&2
    exit 1
fi

# KFCore is excluded from the rsync above (see public_export.exclude) so it
# can be wired up as a real git submodule instead of plain copied files.
# `git submodule add` only runs on the very first export, but the pin has
# to be re-synced on every export: rsync cannot do it (it skips KFCore)
# and nothing else would, so the public repo would keep shipping whatever
# KFCore commit the previous release left behind, while sbom.cdx.json and
# the changelog describe the current one.
KFCORE_SHA="$(git -C "$PRIVATE_ROOT" rev-parse HEAD:KFCore)"

if [[ ! -d "$PUBLIC_REPO_PATH/KFCore" ]]; then
    echo "Adding KFCore as a git submodule ..."
    ( cd "$PUBLIC_REPO_PATH" && git submodule add https://github.com/jnz/KFCore.git KFCore )
fi

PUBLIC_KFCORE_SHA="$(git -C "$PUBLIC_REPO_PATH/KFCore" rev-parse HEAD)"
if [[ "$PUBLIC_KFCORE_SHA" != "$KFCORE_SHA" ]]; then
    echo "Updating the KFCore pin in the export:"
    echo "  $PUBLIC_KFCORE_SHA (public) -> $KFCORE_SHA (private)"
    ( cd "$PUBLIC_REPO_PATH/KFCore" && git fetch --quiet origin \
      && git checkout --quiet --detach "$KFCORE_SHA" )
    if [[ "$(git -C "$PUBLIC_REPO_PATH/KFCore" rev-parse HEAD)" != "$KFCORE_SHA" ]]; then
        echo "error: could not check out KFCore $KFCORE_SHA in the export" >&2
        exit 1
    fi
fi

# sbom.cdx.json names the KFCore commit this release ships (release process
# step 2). It is hand-edited, so cross-check it against the actual pin
# instead of letting the two drift apart unnoticed.
if ! grep -q "$KFCORE_SHA" "$PRIVATE_ROOT/sbom.cdx.json"; then
    echo "error: sbom.cdx.json does not mention the pinned KFCore commit" >&2
    echo "  submodule pin: $KFCORE_SHA" >&2
    echo "  update sbom.cdx.json (release process step 2), commit, re-tag." >&2
    exit 1
fi

echo "Running make check-all on the exported tree ..."
# Run this from an Ubuntu 22.04 environment (e.g. WSL) to match
# .github/workflows/ci.yml's pinned clang-format/cppcheck/doxygen
# versions - see the CI workflow comment for why 22.04, not 24.04.
if ! (cd "$PUBLIC_REPO_PATH" && make check-all); then
    echo "error: check-all failed on the exported tree, release aborted" >&2
    echo "the working tree in $PUBLIC_REPO_PATH is left as-is for inspection" >&2
    exit 1
fi

cd "$PUBLIC_REPO_PATH"
git add -A

if git diff --cached --quiet; then
    echo "note: export is identical to the current public HEAD, nothing to commit."
    exit 0
fi

git commit -m "Release ${VERSION}"
git tag -a "$VERSION" -m "Release ${VERSION}"

git -C "$PRIVATE_ROOT" rev-parse HEAD > /dev/null # sanity: private repo still reachable
git rev-parse HEAD > "$SYNC_MARKER"

echo
echo "Committed and tagged ${VERSION} in $PUBLIC_REPO_PATH."
echo "Review the commit, then push yourself:"
echo "  git -C \"$PUBLIC_REPO_PATH\" push origin main --follow-tags"
