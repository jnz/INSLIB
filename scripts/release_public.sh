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
# Runs `make check-all` against the exported tree before committing - this
# needs a POSIX environment with the toolchain from coding_style.md /
# .github/workflows/ci.yml (WSL, not native Windows: `make check-all` itself
# refuses to run outside POSIX).

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
# Only on the first export: once added, later runs leave it alone (rsync
# doesn't touch it, and it's already a proper submodule in the public
# repo's own history from here on).
if [[ ! -d "$PUBLIC_REPO_PATH/KFCore" ]]; then
    echo "Adding KFCore as a git submodule ..."
    ( cd "$PUBLIC_REPO_PATH" && git submodule add https://github.com/jnz/KFCore.git KFCore )
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
