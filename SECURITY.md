# Security Policy

## Supported Versions

Only the latest released minor version of INSLIB receives security fixes.

| Version | Supported |
| ------- | --------- |
| 1.1.x   | yes       |
| < 1.1   | no        |

## Reporting a Vulnerability

Please do **not** open a public GitHub issue for security vulnerabilities.

Report privately via one of:

- GitHub's [private vulnerability reporting](https://github.com/jnz/INSLIB/security/advisories/new)
  for this repository.
- Email: jan.zwiener@h-da.de or jan@zwiener.org

Include the affected version/commit, the code path and steps involved and a
description of the impact (e.g. memory safety issue, integer overflow, etc.).
A minimal reproducing input file is helpful.

## Scope

The core library (`src/`, `KFCore/`) is the security-relevant surface: it has
no third-party dependencies other than the `KFCore` submodule (see `sbom.cdx.json`).
The Python tooling under `python/` and `tools/` is developer/post-processing
tooling, not part of the embedded deliverable.

