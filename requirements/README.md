# Requirements database

Plain-markdown requirements tracking for ins, aerospace-style:
stable IDs, explicit verification, machine-checked traceability.
`make reqs` runs `check_reqs.py`, which enforces the format below and
verifies that every referenced test actually exists in the code.

## Files

| File | Prefix | Scope |
|---|---|---|
| `req_sys.md` | REQ-SYS | System-level requirements |
| `req_ins.md` | REQ-NAV | ins 15-state navigation filter |
| `req_ahrs.md` | REQ-AHRS | AHRS/ARS attitude filter |
| `req_baro.md` | REQ-BARO | baro_alt vertical channel filter |
| `req_nav_suite.md` | REQ-SUITE | nav_suite wrapper |
| `req_verification.md` | REQ-VER | Verification environment (tests, replay) |

## Requirement format

Each requirement is one `##` section:

```markdown
## REQ-NAV-015 — Short title

- **Status:** verified
- **Parent:** REQ-SYS-006
- **Verification:** Test: tests/test_ins_core.c:scenario_auto_zaru_vibration

The zero-rotation update shall ... (normative "shall" text, atomic and
testable; optional rationale below the shall-statement.)
```

Rules (enforced by `check_reqs.py`):

- **ID**: `REQ-<PREFIX>-<NNN>`, unique across all files, never reused.
  Obsolete requirements are marked `Status: deleted` and kept.
- **Status**: `draft` (not agreed), `implemented` (code exists,
  verification incomplete), `verified` (verification passes),
  `deleted` (kept for ID stability).
- **Parent** (optional): upward trace; the referenced ID must exist.
- **Verification**: one or more `;`-separated entries:
  - `Test: <file>:<function>` — the function must exist in that file
    (checked). Passing means the requirement is verified by the
    regular `make test` / `make datasets` runs.
  - `Analysis: <text>` — argued in the text/rationale.
  - `Inspection: <text>` — verified by code review.
  - `Demonstration: <text>` — shown by using the system.
  - `Open` — verification still missing (reported by the checker,
    does not fail the run).

## Code traceability (`@satisfies`)

Downward trace from requirement to implementation lives in the source
code as comment tags, one line above the implementing function (or
inline for block-level code):

```c
/* @satisfies REQ-NAV-005 REQ-NAV-006 REQ-NAV-008 */
static void ins_fuse_gnss(ins_t* f, const ins_measurements_t* m)
```

Enforced by `check_reqs.py` in both directions:

- every `@satisfies` tag must reference an existing, non-deleted
  requirement;
- every component requirement (REQ-NAV / REQ-AHRS / REQ-BARO /
  REQ-SUITE) with status `implemented` or `verified` must be tagged at
  least once in `src/`.

System- and verification-level requirements (REQ-SYS, REQ-VER) have no
single code location and are exempt from the coverage rule (tagging
them is allowed where it helps, e.g. REQ-SYS-005 at the health check).

The trace from requirement to test is NOT duplicated in the test code:
it lives only in the `Verification:` lines here and is checked against
the test sources. `check_reqs.py --matrix` prints the full requirement
-> code/test matrix.

## Workflow

- New behaviour first gets a requirement (or an update to one), then
  code + verification; the `Verification:` line closes the loop.
- Never renumber. New requirements take the next free number of their
  prefix.
- `make reqs` must pass before a change is considered integrated; it
  is cheap enough to run with every test run.
