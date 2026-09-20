#!/usr/bin/env python3
"""Worst-case stack usage analysis for C code built with GCC.

Compiles the given C sources with GCC's -fcallgraph-info=su, which writes
one VCG call graph per translation unit (.ci) carrying both the frame size
of every function the compiler emitted and its call edges, taken after
optimization, so inlining, partial inlining (.part) and constant propagation
clones (.constprop) are already resolved. Static functions are qualified with
their file name, so equal names in different files stay apart.

The worst case of a function is its own frame plus the largest worst case
among its callees, i.e. the deepest call chain. It is computed for every
entry point (config: root, task, interrupt) and checked against the budgets
(config: budget). With --stack-size, the whole stack of a bare-metal
program is checked as well: the worst task plus, per interrupt nesting
level, the worst interrupt handler and the exception frame the core pushes.

The run fails (exit code 1) on everything that would make the number a
guess rather than a bound:
  - recursion (a cycle in the call graph)
  - a variable length array or alloca (-Werror=vla, -Werror=alloca), and any
    frame GCC reports as dynamic without a bound
  - a call through a function pointer the config does not resolve (indirect)
  - a call to a function that is neither compiled here nor given an assumed
    worst case in the config (extern)
  - a worst case above its budget, a budget line matching no function, or a
    program stack above --stack-size

What it cannot see: calls the compiler emits itself after the call graph is
taken (libgcc helpers in older GCC releases, see config: margin) and link
time optimization.

Config files (--config, may be given more than once, read in order), one
directive per line, # starts a comment:
  root <glob> ...                entry points to report
  indirect <caller> <target>...  possible targets of the function pointer
                                 calls in <caller>
  extern <symbol> <bytes>        assumed worst case of a function that is
                                 not compiled by the analysis (libc, libm)
  margin <bytes>                 added to every worst case
  budget <machine> <root> <bytes>
                                 upper limit for the entry points matching
                                 <root>, for compilers whose -dumpmachine
                                 matches <machine> (globs), the first
                                 matching line applies
  task <glob> ...                entry points of the program (--stack-size)
  interrupt <glob> ...           interrupt and exception handlers
  nesting <levels>               how many handlers can be active at once
  exception_frame <bytes>        what the core pushes per exception

Static functions are named <file>:<function>, the file relative to the
repository root. A glob starting with ! excludes (root, task, interrupt), a
glob as an indirect target stands for every function it matches.
"""

import argparse
import fnmatch
import json
import os
import re
import shlex
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor

INDIRECT = "__indirect_call"
REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

NODE_RE = re.compile(r'^node: \{ title: "([^"]*)" label: "(.*)"( shape : ellipse)? \}$')
EDGE_RE = re.compile(r'^edge: \{ sourcename: "([^"]*)" targetname: "([^"]*)"')
SIZE_RE = re.compile(r'\\n(\d+) bytes \(([^)]*)\)$')


def repo_path(path):
    """Path relative to the repository root, whatever directory GCC ran in."""
    try:
        rel = os.path.relpath(os.path.abspath(path), REPO)
    except ValueError:
        return path.replace("\\", "/")
    return (path if rel.startswith("..") else rel).replace("\\", "/")


def norm_title(title):
    if ":" not in title:
        return title
    f, name = title.rsplit(":", 1)
    return repo_path(f) + ":" + name


def norm_loc(loc):
    parts = loc.rsplit(":", 2)
    if len(parts) != 3:
        return loc
    return "%s:%s:%s" % (repo_path(parts[0]), parts[1], parts[2])


class Config:
    def __init__(self):
        self.externs = {}          # symbol -> assumed worst case [bytes]
        self.indirect = {}         # caller -> list of possible targets
        self.roots = []            # glob patterns selecting the entry points
        self.budgets = []          # (machine glob, root glob, bytes, where)
        self.margin = 0            # added to every worst case [bytes]
        self.tasks = []            # glob patterns, program entry points
        self.interrupts = []       # glob patterns, exception handlers
        self.nesting = 1           # handlers active at once
        self.exception_frame = 0   # pushed by the core per exception [bytes]
        self.files = []


def parse_config(paths):
    cfg = Config()
    errors = []
    for path in paths:
        cfg.files.append(path)
        with open(path, "r", encoding="utf-8") as f:
            for lineno, raw in enumerate(f, 1):
                tok = raw.split("#", 1)[0].split()
                if not tok:
                    continue
                where = "%s:%d" % (path, lineno)
                key, n = tok[0], len(tok)
                try:
                    if key == "extern" and n == 3:
                        cfg.externs[tok[1]] = int(tok[2])
                    elif key == "indirect" and n >= 3:
                        cfg.indirect[tok[1]] = tok[2:]
                    elif key == "root" and n >= 2:
                        cfg.roots.extend(tok[1:])
                    elif key == "budget" and n == 4:
                        cfg.budgets.append((tok[1], tok[2], int(tok[3]), where))
                    elif key == "margin" and n == 2:
                        cfg.margin = int(tok[1])
                    elif key == "task" and n >= 2:
                        cfg.tasks.extend(tok[1:])
                    elif key == "interrupt" and n >= 2:
                        cfg.interrupts.extend(tok[1:])
                    elif key == "nesting" and n == 2:
                        cfg.nesting = int(tok[1])
                    elif key == "exception_frame" and n == 2:
                        cfg.exception_frame = int(tok[1])
                    else:
                        errors.append("%s: cannot parse: %s" % (where, raw.strip()))
                except ValueError:
                    errors.append("%s: expected a number: %s" % (where, raw.strip()))
    return cfg, errors


class Graph:
    def __init__(self):
        self.frame = {}  # function -> own frame [bytes]
        self.kind = {}   # function -> "static", "dynamic", "dynamic,bounded"
        self.loc = {}    # function -> "file:line:column"
        self.calls = {}  # function -> set of callees


def parse_ci(path, g, errors):
    with open(path, "r", encoding="utf-8", errors="replace") as f:
        for line in f:
            line = line.rstrip("\r\n")
            m = NODE_RE.match(line)
            if m:
                title, label, external = m.groups()
                if external:
                    continue
                title = norm_title(title)
                s = SIZE_RE.search(label)
                if not s:
                    errors.append("%s: no stack size for %s" % (path, title))
                    continue
                if title in g.frame:
                    errors.append("%s: %s is defined twice" % (path, title))
                g.frame[title] = int(s.group(1))
                g.kind[title] = s.group(2)
                parts = label.split("\\n")
                g.loc[title] = norm_loc(parts[1]) if len(parts) > 2 else "?"
                g.calls.setdefault(title, set())
                continue
            m = EDGE_RE.match(line)
            if m:
                src, dst = norm_title(m.group(1)), norm_title(m.group(2))
                g.calls.setdefault(src, set()).add(dst)


def object_name(src):
    return repo_path(os.path.splitext(src)[0]).replace("/", "_").replace(".", "_") + ".o"


def compile_all(cc, cflags, sources, build_dir):
    os.makedirs(build_dir, exist_ok=True)
    flags = shlex.split(cflags) + ["-fcallgraph-info=su", "-Werror=vla", "-Werror=alloca"]

    def one(src):
        obj = os.path.join(build_dir, object_name(src)).replace("\\", "/")
        ci = obj[:-2] + ".ci"
        if os.path.exists(ci):
            os.remove(ci)
        cmd = [cc] + flags + ["-c", src, "-o", obj]
        p = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                           universal_newlines=True)
        return src, ci, p.returncode, p.stdout, cmd

    with ThreadPoolExecutor(max_workers=os.cpu_count() or 2) as pool:
        return list(pool.map(one, sources))


class Analysis:
    def __init__(self, g, cfg):
        self.g = g
        self.cfg = cfg
        self.memo = {}              # function -> (worst case, callee on worst path)
        self.on_path = []
        self.recursion = set()
        self.unknown = {}           # unknown external -> set of callers
        self.unresolved = set()     # callers with an unresolved indirect call
        self.qualified = {}         # function name -> file qualified definitions
        for title in g.frame:
            if ":" in title:
                self.qualified.setdefault(title.rsplit(":", 1)[1], []).append(title)

    def callees(self, fn):
        for c in sorted(self.g.calls.get(fn, ())):
            if c != INDIRECT:
                if c not in self.g.frame and c not in self.cfg.externs and c in self.qualified:
                    # GCC qualifies a weak definition with its file, like a
                    # static one, while callers use the plain name. Every
                    # definition of that name is a candidate.
                    for q in self.qualified[c]:
                        yield q
                else:
                    yield c
            elif fn in self.cfg.indirect:
                for t in self.cfg.indirect[fn]:
                    if any(ch in t for ch in "*?["):
                        # A glob stands for every analysed function it matches.
                        hit = [f for f in sorted(self.g.frame) if fnmatch.fnmatchcase(f, t)]
                        if not hit:
                            self.unknown.setdefault(t, set()).add(fn)
                        for h in hit:
                            yield h
                    else:
                        yield t
            else:
                self.unresolved.add(fn)

    def worst(self, fn, caller=None):
        if fn not in self.g.frame:
            if fn in self.cfg.externs:
                return self.cfg.externs[fn]
            self.unknown.setdefault(fn, set()).add(caller)
            return 0
        if fn in self.memo:
            return self.memo[fn][0]
        if fn in self.on_path:
            cycle = self.on_path[self.on_path.index(fn):] + [fn]
            self.recursion.add(" -> ".join(cycle))
            return 0
        self.on_path.append(fn)
        best, best_callee = 0, None
        for c in self.callees(fn):
            w = self.worst(c, fn)
            if best_callee is None or w > best:
                best, best_callee = w, c
        self.on_path.pop()
        self.memo[fn] = (self.g.frame[fn] + best, best_callee)
        return self.memo[fn][0]

    def path(self, fn):
        out = []
        seen = set()
        while fn is not None and fn not in seen:
            seen.add(fn)
            if fn in self.g.frame:
                out.append((fn, self.g.frame[fn], self.g.loc[fn]))
                fn = self.memo[fn][1]
            else:
                out.append((fn, self.cfg.externs.get(fn, 0), "assumed (extern)"))
                fn = None
        return out


def tool_output(cmd):
    try:
        p = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                           universal_newlines=True)
    except OSError:
        return None
    return p.stdout.strip() if p.returncode == 0 else None


def matching(names, globs):
    """Names matching one of the globs and none of the !globs."""
    pos = [p for p in globs if not p.startswith("!")]
    neg = [p[1:] for p in globs if p.startswith("!")]
    return [f for f in names
            if any(fnmatch.fnmatchcase(f, p) for p in pos)
            and not any(fnmatch.fnmatchcase(f, p) for p in neg)]


def print_path(a, fn, total):
    print()
    print("worst path of %s (%d bytes):" % (fn, total))
    for name, size, loc in a.path(fn):
        print("  %8d  %s  [%s]" % (size, name, loc))


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--config", action="append", required=True,
                    help="externs, indirect calls, roots, budgets (repeatable)")
    ap.add_argument("--cc", default="gcc", help="GCC (>= 10) or a GCC cross compiler")
    ap.add_argument("--cflags", default="-O2", help="compiler flags, as for the real build")
    ap.add_argument("--build-dir", default="build/stack", help="object and .ci output")
    ap.add_argument("--stack-size", type=lambda s: int(s, 0),
                    help="stack reserved for the program, checked against task and interrupts")
    ap.add_argument("--json", help="also write the result as JSON")
    ap.add_argument("--top", type=int, default=15, help="entry points listed in the table")
    ap.add_argument("sources", nargs="+")
    args = ap.parse_args()
    cfg_names = ", ".join(args.config)

    cfg, errors = parse_config(args.config)
    machine = tool_output([args.cc, "-dumpmachine"])
    if machine is None and not args.cc.endswith(".exe"):
        # Under WSL a Windows toolchain is only reachable with its suffix.
        machine = tool_output([args.cc + ".exe", "-dumpmachine"])
        if machine is not None:
            args.cc += ".exe"
    if machine is None:
        print("stack: cannot run the compiler %r (needs GCC >= 10)" % args.cc)
        return 1
    version = (tool_output([args.cc, "--version"]) or "?").splitlines()[0]

    g = Graph()
    for src, ci, rc, out, cmd in compile_all(args.cc, args.cflags, args.sources, args.build_dir):
        if rc != 0 or not os.path.exists(ci):
            sys.stdout.write(out)
            errors.append("compile failed: %s" % " ".join(cmd))
            continue
        parse_ci(ci, g, errors)
    if errors:
        print("\n".join("stack: " + e for e in errors))
        return 1

    a = Analysis(g, cfg)
    globals_ = sorted(f for f in g.frame if ":" not in f)
    tasks = matching(globals_, cfg.tasks)
    isrs = matching(globals_, cfg.interrupts)
    roots = sorted(set(matching(globals_, cfg.roots) if cfg.roots else globals_) | set(tasks) | set(isrs))
    for r in roots:
        a.worst(r)

    budget = {}
    for mglob, rglob, limit, where in cfg.budgets:
        if not fnmatch.fnmatchcase(machine, mglob):
            continue
        hit = [r for r in roots if fnmatch.fnmatchcase(r, rglob)]
        if not hit:
            errors.append("%s: budget %s matches no entry point" % (where, rglob))
        for r in hit:
            budget.setdefault(r, limit)

    for fn in sorted(g.frame):
        if "dynamic" in g.kind[fn] and "bounded" not in g.kind[fn]:
            errors.append("unbounded dynamic stack frame: %s (%s)" % (fn, g.loc[fn]))
    for cyc in sorted(a.recursion):
        errors.append("recursion: %s" % cyc)
    for fn in sorted(a.unresolved):
        errors.append("call through a function pointer in %s (%s), add an 'indirect' line "
                      "to %s" % (fn, g.loc[fn], cfg_names))
    for fn in sorted(a.unknown):
        errors.append("no stack figure for external %s (called by %s), add an 'extern' line "
                      "to %s" % (fn, ", ".join(sorted(a.unknown[fn])), cfg_names))

    total = {r: a.memo[r][0] + cfg.margin for r in roots}
    ranked = sorted(roots, key=lambda r: (-total[r], r))
    over = [r for r in ranked if r in budget and total[r] > budget[r]]
    shown = sorted(set(ranked[:args.top] + over), key=lambda r: (-total[r], r))
    for r in over:
        errors.append("%s needs %d bytes, budget %d" % (r, total[r], budget[r]))

    print("stack: %s (%s), %d translation units, %d functions, %d entry points"
          % (version, machine, len(args.sources), len(g.frame), len(roots)))
    print("stack: worst case includes a margin of %d bytes for compiler generated calls"
          % cfg.margin)
    print()
    print("  %8s  %8s  %s" % ("worst", "budget", "entry point"))
    for r in shown:
        limit = "%8d" % budget[r] if r in budget else "%8s" % "-"
        print("  %8d  %s  %s%s" % (total[r], limit, r, "  OVER" if r in over else ""))
    if len(roots) > len(shown):
        print("  (%d more entry points below, see --json)" % (len(roots) - len(shown)))

    for r in ranked[:1] + [r for r in over if r not in ranked[:1]]:
        print_path(a, r, total[r])

    program = None
    if args.stack_size is not None:
        if not tasks:
            errors.append("--stack-size needs a 'task' line matching a function")
        if cfg.interrupts and not isrs:
            errors.append("no function matches the 'interrupt' lines")
    if args.stack_size is not None and tasks:
        task = max(tasks, key=lambda r: (a.memo[r][0], r))
        isr = max(isrs, key=lambda r: (a.memo[r][0], r)) if isrs else None
        task_ws = a.memo[task][0]
        isr_ws = a.memo[isr][0] if isr else 0
        levels = cfg.nesting if isr else 0
        need = task_ws + levels * (isr_ws + cfg.exception_frame) + cfg.margin
        program = {"task": task, "task_worst": task_ws, "interrupt": isr,
                   "interrupt_worst": isr_ws, "nesting": levels,
                   "exception_frame": cfg.exception_frame, "margin": cfg.margin,
                   "needed": need, "stack_size": args.stack_size}
        if task != ranked[0]:
            print_path(a, task, task_ws)
        if isr:
            print_path(a, isr, isr_ws)
        print()
        print("program stack: %s %d + %d x (%s %d + exception frame %d) + margin %d"
              % (task, task_ws, levels, isr or "-", isr_ws, cfg.exception_frame, cfg.margin))
        print("             = %d bytes of %d reserved (%d left)"
              % (need, args.stack_size, args.stack_size - need))
        if need > args.stack_size:
            errors.append("program stack needs %d bytes, only %d reserved"
                          % (need, args.stack_size))

    if args.json:
        with open(args.json, "w", encoding="utf-8") as f:
            json.dump({
                "compiler": version,
                "machine": machine,
                "margin": cfg.margin,
                "program": program,
                "entry_points": {
                    r: {
                        "worst": total[r],
                        "budget": budget.get(r),
                        "path": [{"function": fn, "bytes": s, "where": loc}
                                 for fn, s, loc in a.path(r)],
                    } for r in ranked
                },
            }, f, indent=2)

    print()
    if errors:
        print("\n".join("stack: FAIL " + e for e in errors))
        return 1
    print("stack: ok")
    return 0


if __name__ == "__main__":
    sys.exit(main())
