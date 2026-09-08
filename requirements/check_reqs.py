#!/usr/bin/env python3
"""Requirements database consistency checker (see requirements/README.md).

Checks all requirements/req_*.md files:
  - requirement IDs are well-formed and unique, file prefix matches
  - mandatory fields (Status, Verification) present, Status value valid
  - Parent references point to existing requirement IDs
  - every `Test: <file>:<function>` verification entry resolves: the
    file exists (relative to the repo root) and contains the function

Code traceability (`@satisfies REQ-...` tags in src/):
  - every tag must reference an existing, non-deleted requirement
  - every component requirement (REQ-NAV/-AHRS/-BARO/-SUITE) with
    status implemented or verified must be tagged somewhere in src/

Run with --matrix to print the full requirement -> code/test matrix.

Exit code 0 if consistent; 1 on any violation. Requirements whose
verification is `Open` are reported but do not fail the check.

(c) Jan Zwiener (jan@zwiener.org)
"""
import glob
import os
import re
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REQ_DIR = os.path.join(REPO_ROOT, "requirements")

ID_RE = re.compile(r"^## (REQ-([A-Z]+)-(\d{3})) — (.+)$")
FIELD_RE = re.compile(r"^- \*\*(\w+):\*\* (.+)$")
STATUSES = {"draft", "implemented", "verified", "deleted"}
METHODS = {"Test", "Analysis", "Inspection", "Demonstration"}

# Sources scanned for @satisfies tags, and the requirement prefixes that
# MUST be tagged in code (component level; system/verification-level
# requirements have no single code location).
SRC_GLOBS = ["src/*.c", "src/*.h"]
CODE_TRACED_PREFIXES = {"NAV", "AHRS", "SUITE", "BARO"}
TAG_RE = re.compile(r"@satisfies\s+((?:REQ-[A-Z]+-\d{3}[ \t]*)+)")

FILE_PREFIX = {
    "req_sys.md": "SYS",
    "req_ins.md": "NAV",
    "req_ahrs.md": "AHRS",
    "req_baro.md": "BARO",
    "req_nav_suite.md": "SUITE",
    "req_verification.md": "VER",
}


def parse_file(path, errors):
    """Return list of dicts: id, title, fields, file, line."""
    reqs = []
    cur = None
    fname = os.path.basename(path)
    with open(path, encoding="utf-8") as f:
        for lineno, line in enumerate(f, 1):
            line = line.rstrip("\n")
            m = ID_RE.match(line)
            if m:
                cur = {"id": m.group(1), "prefix": m.group(2),
                       "title": m.group(4), "fields": {},
                       "file": fname, "line": lineno}
                reqs.append(cur)
                want = FILE_PREFIX.get(fname)
                if want and m.group(2) != want:
                    errors.append("%s:%d: %s has prefix %s, expected %s"
                                  % (fname, lineno, m.group(1),
                                     m.group(2), want))
                continue
            if line.startswith("## "):
                errors.append("%s:%d: malformed requirement heading: %r"
                              % (fname, lineno, line))
                cur = None
                continue
            if cur is not None:
                fm = FIELD_RE.match(line)
                if fm:
                    cur["fields"][fm.group(1)] = fm.group(2).strip()
    return reqs


def check_verification(req, errors, open_reqs):
    ver = req["fields"].get("Verification", "")
    if ver == "Open":
        open_reqs.append(req)
        return
    for entry in [e.strip() for e in ver.split(";") if e.strip()]:
        if ":" not in entry:
            errors.append("%s: %s: malformed verification entry: %r"
                          % (req["file"], req["id"], entry))
            continue
        method, arg = entry.split(":", 1)
        method = method.strip()
        arg = arg.strip()
        if method not in METHODS:
            errors.append("%s: %s: unknown verification method: %r"
                          % (req["file"], req["id"], method))
            continue
        if method != "Test":
            continue
        if ":" not in arg:
            errors.append("%s: %s: Test entry needs <file>:<function>: %r"
                          % (req["file"], req["id"], arg))
            continue
        tfile, tfunc = arg.rsplit(":", 1)
        tpath = os.path.join(REPO_ROOT, tfile.strip())
        if not os.path.isfile(tpath):
            errors.append("%s: %s: test file not found: %s"
                          % (req["file"], req["id"], tfile))
            continue
        with open(tpath, encoding="utf-8", errors="replace") as f:
            if tfunc.strip() not in f.read():
                errors.append("%s: %s: function %r not found in %s"
                              % (req["file"], req["id"], tfunc, tfile))


def scan_code_tags(errors, known_ids):
    """Return dict req_id -> ["file:line", ...] from @satisfies tags."""
    tags = {}
    for pattern in SRC_GLOBS:
        for path in sorted(glob.glob(os.path.join(REPO_ROOT, pattern))):
            rel = os.path.relpath(path, REPO_ROOT)
            with open(path, encoding="utf-8", errors="replace") as f:
                for lineno, line in enumerate(f, 1):
                    m = TAG_RE.search(line)
                    if not m:
                        continue
                    for rid in m.group(1).split():
                        loc = "%s:%d" % (rel, lineno)
                        if rid not in known_ids:
                            errors.append("%s: @satisfies references "
                                          "unknown requirement %s"
                                          % (loc, rid))
                            continue
                        tags.setdefault(rid, []).append(loc)
    return tags


def main():
    errors = []
    reqs = []
    for path in sorted(glob.glob(os.path.join(REQ_DIR, "req_*.md"))):
        reqs.extend(parse_file(path, errors))

    if not reqs:
        print("no requirements found in %s" % REQ_DIR)
        return 1

    seen = {}
    for r in reqs:
        if r["id"] in seen:
            errors.append("%s:%d: duplicate ID %s (first in %s)"
                          % (r["file"], r["line"], r["id"], seen[r["id"]]))
        seen[r["id"]] = r["file"]

    open_reqs = []
    by_status = {}
    for r in reqs:
        status = r["fields"].get("Status")
        if status is None:
            errors.append("%s: %s: missing Status field"
                          % (r["file"], r["id"]))
        elif status not in STATUSES:
            errors.append("%s: %s: invalid Status %r"
                          % (r["file"], r["id"], status))
        by_status[status] = by_status.get(status, 0) + 1

        if "Verification" not in r["fields"]:
            errors.append("%s: %s: missing Verification field"
                          % (r["file"], r["id"]))
        else:
            check_verification(r, errors, open_reqs)

        parent = r["fields"].get("Parent")
        if parent is not None and parent not in seen and \
           parent not in [x["id"] for x in reqs]:
            errors.append("%s: %s: parent %s does not exist"
                          % (r["file"], r["id"], parent))

    # Code traceability: @satisfies tags in src/.
    tags = scan_code_tags(errors, set(seen))
    n_locs = sum(len(v) for v in tags.values())
    for r in reqs:
        if r["prefix"] not in CODE_TRACED_PREFIXES:
            continue
        if r["fields"].get("Status") not in ("implemented", "verified"):
            continue
        if r["id"] not in tags:
            errors.append("%s: %s: no @satisfies tag in src/ (add one at "
                          "the implementing code)" % (r["file"], r["id"]))
    for rid in tags:
        r = next(x for x in reqs if x["id"] == rid)
        if r["fields"].get("Status") == "deleted":
            errors.append("%s: tagged in code (%s) but Status is deleted"
                          % (rid, ", ".join(tags[rid])))

    print("requirements: %d total (%s)" %
          (len(reqs), ", ".join("%s: %d" % kv
                                for kv in sorted(by_status.items()
                                                 , key=lambda kv: str(kv[0])))))
    print("code trace: %d @satisfies locations covering %d requirements"
          % (n_locs, len(tags)))
    if open_reqs:
        print("open verification (%d):" % len(open_reqs))
        for r in open_reqs:
            print("  %-14s %s (%s)" % (r["id"], r["title"], r["file"]))

    if "--matrix" in sys.argv:
        print("\ntraceability matrix:")
        for r in reqs:
            print("  %-14s [%-11s] %s" % (r["id"],
                                          r["fields"].get("Status", "?"),
                                          r["title"]))
            for loc in tags.get(r["id"], []):
                print("      code: %s" % loc)
            ver = r["fields"].get("Verification", "")
            for entry in [e.strip() for e in ver.split(";") if e.strip()]:
                print("      %s" % entry)

    if errors:
        print("\n%d error(s):" % len(errors))
        for e in errors:
            print("  " + e)
        return 1
    print("requirements database consistent.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
