#!/usr/bin/env python3
# (c) Jan Zwiener (jan@zwiener.org)
"""Regenerate the stack usage block in readme.md from make stack's JSON.

Reads build/stack/stack_usage.json (written by `make stack`, the x86_64
host analysis) and rewrites the block between <!-- STACK:START --> and
<!-- STACK:END --> in readme.md with the worst case of the main entry
points. Host-only, on purpose: the readme is public and only needs to
show that stack usage is under control, not the actual embedded
target's numbers - the STM32F429 cross-compiled figures from
embedded/stm32f429's own `make stack` stay an embedded/ concern. No
commit hash is stamped: this repo's history gets re-exported into a
public repo with different commit hashes, so a stamped hash here would
go stale/misleading immediately. No hand-typed numbers, see CLAUDE.md's
release process, step 4.

Usage:
  make stack                        # regenerates build/stack/stack_usage.json
  python3 scripts/update_readme_stack.py
"""

import json
import os
import re
import subprocess
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
README = "readme.md"
START = "<!-- STACK:START -->"
END = "<!-- STACK:END -->"
RESULTS = ["build/stack/stack_usage.json"]
ENTRY_POINTS = ["nav_suite_update", "ins_update", "ahrs_update", "baro_alt_update",
                "nav_suite_init", "ins_init"]
INPUTS = ["src", "KFCore/c", "scripts/stack_usage.cfg", "scripts/stack_usage.py"]


def newest_input():
    newest = 0.0
    for p in INPUTS:
        if os.path.isfile(p):
            newest = max(newest, os.path.getmtime(p))
            continue
        for root, _, files in os.walk(p):
            for f in files:
                if f.endswith((".c", ".h")):
                    newest = max(newest, os.path.getmtime(os.path.join(root, f)))
    return newest


def main():
    os.chdir(REPO)
    if not os.path.isfile(RESULTS[0]):
        print("update_readme_stack.py: %s not found - run 'make stack' first." % RESULTS[0])
        return 1
    with open(README, "r", encoding="utf-8") as f:
        text = f.read()
    if START not in text or END not in text:
        print("update_readme_stack.py: %s/%s markers not found in %s." % (START, END, README))
        return 1

    newest = newest_input()
    columns = []
    for path in RESULTS:
        if not os.path.isfile(path):
            continue
        if os.path.getmtime(path) < newest:
            print("update_readme_stack.py: warning: %s is older than the library "
                  "sources, rerun make stack." % path, file=sys.stderr)
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        version = re.search(r"\d+\.\d+\.\d+", data["compiler"])
        columns.append(("%s, GCC %s" % (data["machine"], version.group(0) if version else "?"),
                        data["entry_points"]))

    dirty = subprocess.run(["git", "status", "--porcelain", "--", "src", "KFCore",
                            "scripts/stack_usage.cfg"],
                           stdout=subprocess.PIPE, universal_newlines=True).stdout.strip()
    if dirty:
        print("update_readme_stack.py: warning: the library has uncommitted changes - "
              "these figures won't reflect what actually ships until you commit them too.",
              file=sys.stderr)

    lines = [START,
             "**Worst-case stack usage** in bytes, deepest call chain including KFCore "
             "and the C library, from static analysis (`make stack`):",
             "",
             "| Entry point | " + " | ".join(name for name, _ in columns) + " |",
             "|---|" + "---:|" * len(columns)]
    for ep in ENTRY_POINTS:
        cells = [str(eps[ep]["worst"]) if ep in eps else "-" for _, eps in columns]
        lines.append("| `%s()` | %s |" % (ep, " | ".join(cells)))
    lines.append(END)
    block = "\n".join(lines)

    head, rest = text.split(START, 1)
    _, tail = rest.split(END, 1)
    with open(README, "w", encoding="utf-8") as f:
        f.write(head + block + tail)

    print("readme.md stack block updated: %s" % (
        ", ".join("%s %s" % (name, eps.get("nav_suite_update", {}).get("worst", "-"))
                  for name, eps in columns)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
