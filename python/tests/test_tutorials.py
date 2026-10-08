#!/usr/bin/env python3
"""The tutorials in tutorial/*.md have to keep working.

The code a reader copies out of a tutorial is built and run exactly as the
text says, and its output is compared with the "Expected output" block next
to it. A change to the library that breaks a tutorial (a renamed function, a
tightened start-up gate, a different default) fails here instead of in front
of a new user.

* The C programs are compiled with the very command line printed in the
  tutorial (read out of the markdown, only the source and output path are
  swapped), so a missing source file in that command is caught as well.
* The Python scripts are run in a subprocess against the built libINSLIB.
* Every other Python snippet is parsed, and the names it uses (imports,
  Config keywords, Navigator/Ins methods, Solution fields) must exist.
* Every `ins_*`/`ahrs_*`/... function and `opt.*` field the prose names must
  still exist in src/.

The C part needs a C compiler (CC, default gcc) and is skipped without one,
the Python part needs the library from `make pylib`.

Runs under pytest or standalone:

    python3 python/tests/test_tutorials.py
"""

import ast
import dataclasses
import glob
import os
import re
import shlex
import shutil
import subprocess
import sys
import tempfile

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
TUTORIAL_DIR = os.path.join(REPO, "tutorial")
sys.path.insert(0, os.path.join(REPO, "python"))

try:
    import pytest
    _skip = pytest.skip
except ImportError:                     # standalone runner
    class _Skip(Exception):
        pass

    def _skip(msg):
        raise _Skip(msg)

_FENCE = re.compile(r"^```([a-z]*)\n(.*?)^```$", re.S | re.M)


def _read(name):
    with open(os.path.join(TUTORIAL_DIR, name), encoding="utf-8") as f:
        return f.read()


def _blocks(text):
    """All fenced code blocks as (lang, body, text_before)."""
    out = []
    for m in _FENCE.finditer(text):
        out.append((m.group(1), m.group(2), text[:m.start()]))
    return out


def _block(name, lang, startswith):
    """The one fenced block of that language whose body starts with a text."""
    hits = [b for (lg, b, _) in _blocks(_read(name))
            if lg == lang and b.lstrip().startswith(startswith)]
    assert len(hits) == 1, f"{name}: {len(hits)} {lang} blocks start with {startswith!r}"
    return hits[0]


def _block_after(name, marker):
    """The fenced block that follows the first occurrence of marker."""
    text = _read(name)
    at = text.index(marker)
    m = _FENCE.search(text, at)
    assert m is not None, f"{name}: no code block after {marker!r}"
    return m.group(2)


# ---------------------------------------------------------------------------
# Output comparison
# ---------------------------------------------------------------------------

_NUM = re.compile(r"-?\d+\.\d+|-?\d+")


def _compare_output(got, expected, what):
    """Same text, numbers equal to the printed precision.

    Angles that sit at zero (roll -0.00 / yaw 0.02 deg of a stationary sensor)
    are Earth-rate and rounding noise, a hundredth of a degree is not
    something a tutorial should pin, so those lines get 0.05 deg."""
    got_lines = [ln.strip() for ln in got.strip().splitlines()
                 if ln.strip() and not ln.startswith("[")]   # drop log lines
    exp_lines = [ln.strip() for ln in expected.strip().splitlines() if ln.strip()]
    assert len(got_lines) == len(exp_lines), (
        f"{what}: {len(got_lines)} output lines, tutorial shows {len(exp_lines)}\n"
        f"--- got\n{got}\n--- expected\n{expected}")
    for g, e in zip(got_lines, exp_lines):
        gt, et = _NUM.split(g), _NUM.split(e)
        assert gt == et, f"{what}: text differs\n  got      {g}\n  expected {e}"
        angle_line = any(w in e for w in ("roll", "rpy"))
        for gn, en in zip(_NUM.findall(g), _NUM.findall(e)):
            decimals = len(en.split(".")[1]) if "." in en else 0
            tol = 0.05 if angle_line else 2.0 * 10.0 ** (-decimals)
            if angle_line and gn == _NUM.findall(g)[0] and "t =" in e:
                tol = 1e-6                       # the time stamp itself
            assert abs(float(gn) - float(en)) <= tol + 1e-9, (
                f"{what}: number differs\n  got      {g}\n  expected {e}")


# ---------------------------------------------------------------------------
# C programs
# ---------------------------------------------------------------------------

def _compiler():
    cc = os.environ.get("CC", "gcc")
    exe = shutil.which(cc.split()[0])
    if exe is None:
        _skip(f"no C compiler ({cc}) on PATH")
    return cc


def _build_command(sh_block, source, out):
    """The tutorial's cc command line with our compiler, source and output."""
    joined = sh_block.replace("\\\n", " ")
    cmd = shlex.split(joined.splitlines()[0])
    assert cmd[0] == "cc", f"unexpected build command: {joined}"
    cmd[0:1] = shlex.split(_compiler())
    for i, tok in enumerate(cmd):
        if tok.endswith(".c") and not tok.startswith("src/") and "/" not in tok:
            cmd[i] = source                      # hello_*.c
        if tok == "-o":
            cmd[i + 1] = out
    return cmd


def _build_and_run(source_text, sh_block, tmp):
    src = os.path.join(tmp, "prog.c")
    exe = os.path.join(tmp, "prog.exe" if os.name == "nt" else "prog")
    with open(src, "w", encoding="utf-8", newline="\n") as f:
        f.write(source_text)
    cmd = _build_command(sh_block, src, exe)
    cmd.insert(1, "-Wall")
    cmd.insert(2, "-Werror")                     # a tutorial must build warning-free
    b = subprocess.run(cmd, cwd=REPO, capture_output=True, text=True)
    assert b.returncode == 0, f"tutorial build failed:\n{' '.join(cmd)}\n{b.stderr}"
    r = subprocess.run([exe], cwd=REPO, capture_output=True, text=True, timeout=60)
    assert r.returncode == 0, f"tutorial program exited {r.returncode}\n{r.stdout}{r.stderr}"
    return r.stdout + r.stderr


def _c_tutorial_program():
    return _block("c_tutorial.md", "c", "/* hello_ins.c")


def _c_tutorial_build():
    return _block("c_tutorial.md", "sh", "cc -std=c11")


def test_c_tutorial_first_program():
    prog = _c_tutorial_program()
    expected = _block_after("c_tutorial.md", "Expected output")
    with tempfile.TemporaryDirectory() as tmp:
        out = _build_and_run(prog, _c_tutorial_build(), tmp)
    _compare_output(out, expected, "c_tutorial first program")


def test_c_tutorial_lighthouse_snippet():
    """The GNSS-free snippet replaces the GNSS block of the first program."""
    prog = _c_tutorial_program()
    snippet = _block("c_tutorial.md", "c", "/* A 10 Hz indoor tracker")
    gnss = re.search(r"^        if \(k % 100 == 0\) \{\n.*?^        \}\n", prog,
                     re.S | re.M)
    assert gnss, "GNSS block of the first program not found"
    indented = "".join("        " + ln if ln.strip() else ln
                       for ln in snippet.splitlines(True))
    variant = prog[:gnss.start()] + indented + prog[gnss.end():]
    with tempfile.TemporaryDirectory() as tmp:
        out = _build_and_run(variant, _c_tutorial_build(), tmp)
    assert "warming up" not in out, f"lighthouse variant never became ready\n{out}"
    m = re.search(r"position : lat ([\d.]+) deg", out)
    assert m, out
    # 1.5 m north of the anchor at 48.1372 deg is +1.35e-5 deg
    assert abs(float(m.group(1)) - 48.137213) < 3e-6, out


def test_attitude_tutorial_program():
    prog = _block("c_attitude_tutorial.md", "c", "/* hello_ars.c")
    build = _block("c_attitude_tutorial.md", "sh", "cc -std=c11")
    expected = _block_after("c_attitude_tutorial.md", "Expected output")
    with tempfile.TemporaryDirectory() as tmp:
        out = _build_and_run(prog, build, tmp)
    _compare_output(out, expected, "c_attitude_tutorial program")


# ---------------------------------------------------------------------------
# Python scripts
# ---------------------------------------------------------------------------

def _run_python(script):
    try:
        import INSLIB  # noqa: F401
    except OSError as e:                         # library not built
        _skip(f"libINSLIB not available (make pylib): {e}")
    env = dict(os.environ)
    env["PYTHONPATH"] = os.path.join(REPO, "python") + os.pathsep + env.get("PYTHONPATH", "")
    # The library's log lines go through C stdio to the same pipe and are
    # flushed in blocks, in the middle of the script's own lines. The script
    # therefore prints into a file, which the log cannot reach.
    with tempfile.TemporaryDirectory() as tmp:
        out_path = os.path.join(tmp, "out.txt")
        redirect = f"import sys\nsys.stdout = open({out_path!r}, 'w', encoding='utf-8')\n"
        r = subprocess.run([sys.executable, "-"], input=redirect + script, cwd=REPO,
                           env=env, capture_output=True, text=True, timeout=120)
        assert r.returncode == 0, f"tutorial script failed:\n{r.stdout}{r.stderr}"
        with open(out_path, encoding="utf-8") as f:
            return f.read()


def test_python_tutorial_first_script():
    blocks = [b for (lg, b, _) in _blocks(_read("python_tutorial.md"))
              if lg == "python" and "nav.gnss_pos_llh((lat, lon, h)" in b
              and b.startswith("import math")]
    assert len(blocks) == 1
    script = blocks[0]
    expected = _block_after("python_tutorial.md", "Expected output")
    _compare_output(_run_python(script), expected, "python_tutorial first script")


def test_python_tutorial_lighthouse_script():
    blocks = [b for (lg, b, _) in _blocks(_read("python_tutorial.md"))
              if lg == "python" and "local_pos((1.5" in b]
    assert len(blocks) == 1
    script = blocks[0]
    expected = re.search(r"^# -> (.*)$", script, re.M).group(1)
    out = _run_python(script)
    got = [ln for ln in out.splitlines() if ln.strip() and not ln.startswith("[")][-1]
    _compare_output(got, expected, "python_tutorial lighthouse script")


# ---------------------------------------------------------------------------
# Every other snippet: the names have to exist
# ---------------------------------------------------------------------------

def _python_snippets():
    out = []
    for (lg, body, _) in _blocks(_read("python_tutorial.md")):
        if lg == "python":
            out.append(body)
    return out


def test_python_snippets_use_existing_names():
    try:
        import INSLIB
        from INSLIB import Config, Ins, Navigator, Solution
    except OSError as e:
        _skip(f"libINSLIB not available (make pylib): {e}")
    config_fields = {f.name for f in dataclasses.fields(Config)}
    solution_fields = {f.name for f in dataclasses.fields(Solution)}
    nav_attrs = set(dir(Navigator)) | set(dir(Ins))
    problems = []
    for body in _python_snippets():
        tree = ast.parse(body)                    # a syntax error fails here
        for node in ast.walk(tree):
            if isinstance(node, ast.ImportFrom) and node.module == "INSLIB":
                for a in node.names:
                    if not hasattr(INSLIB, a.name):
                        problems.append(f"INSLIB has no {a.name}")
            if isinstance(node, ast.Call):
                fn = node.func
                if isinstance(fn, ast.Name) and fn.id == "Config":
                    for kw in node.keywords:
                        if kw.arg and kw.arg not in config_fields:
                            problems.append(f"Config has no field {kw.arg}")
                if (isinstance(fn, ast.Attribute) and isinstance(fn.value, ast.Name)
                        and fn.value.id == "nav" and fn.attr not in nav_attrs):
                    problems.append(f"nav.{fn.attr}() does not exist")
            if (isinstance(node, ast.Attribute) and isinstance(node.value, ast.Name)
                    and node.value.id == "sol" and node.attr not in solution_fields):
                problems.append(f"Solution has no field {node.attr}")
    assert not problems, "\n".join(sorted(set(problems)))


# ---------------------------------------------------------------------------
# Names in the prose
# ---------------------------------------------------------------------------

def _src_text():
    parts = []
    for p in sorted(glob.glob(os.path.join(REPO, "src", "*.[ch]"))):
        with open(p, encoding="utf-8", errors="replace") as f:
            parts.append(f.read())
    return "\n".join(parts)


def test_c_names_in_tutorials_exist():
    """`ins_foo(` style functions and `opt.field` names must exist in src/."""
    src = _src_text()
    with open(os.path.join(REPO, "src", "ins.h"), encoding="utf-8") as f:
        ins_h = f.read()
    problems = []
    for name in ("c_tutorial.md", "c_attitude_tutorial.md"):
        text = _read(name)
        for m in re.finditer(r"\b((?:ins|ahrs|nav_suite|baro_alt)_[a-z0-9_]+)\b", text):
            ident = m.group(1)
            if not re.search(r"\b" + re.escape(ident) + r"\b", src):
                problems.append(f"{name}: {ident} is not in src/")
        for m in re.finditer(r"\bopt\.([a-z0-9_]+)(\*?)", text):
            field, star = m.group(1), m.group(2)
            pat = (r"\b" + re.escape(field)) if star else (r"\b" + re.escape(field) + r"\b")
            if not re.search(pat, ins_h):
                problems.append(f"{name}: opt.{field}{star} is not in ins.h")
    assert not problems, "\n".join(sorted(set(problems)))


_TESTS = [
    test_c_tutorial_first_program,
    test_c_tutorial_lighthouse_snippet,
    test_attitude_tutorial_program,
    test_python_tutorial_first_script,
    test_python_tutorial_lighthouse_script,
    test_python_snippets_use_existing_names,
    test_c_names_in_tutorials_exist,
]


def _main():
    fails = 0
    for t in _TESTS:
        try:
            t()
            print(f"ok    {t.__name__}")
        except Exception as e:  # noqa: BLE001
            if type(e).__name__ == "_Skip":
                print(f"skip  {t.__name__}: {e}")
                continue
            fails += 1
            print(f"FAIL  {t.__name__}: {e}")
    print(f"\n{fails} failures")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(_main())
