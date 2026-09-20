# C Coding Style Guidelines (C11)

## 1. File Structure and Headers

 * **Include Guards:** Every header file must be protected by include guards (`#ifndef MODULE_H`, `#define MODULE_H`, `#endif /* MODULE_H */`).
 * **Doxygen File Header:** Every file must begin with a Doxygen comment block containing at least `@file`, `@author`, and a detailed `@brief` description.
 * **Section Separators:** Logical sections within the code (Includes, Defines, Typedefs, Function Prototypes) must be separated by comment blocks:
    ```c
    /******************************************************************************
     * SECTION NAME
     ******************************************************************************/
    ```
 * **Section Names**: "SYSTEM INCLUDE FILES", "PROJECT INCLUDE FILES", "DEFINES", "TYPEDEFS", "LOCAL DATA DEFINITIONS", "LOCAL FUNCTION PROTOTYPES", "FUNCTION PROTOTYPES"
 * **C++ Compatibility:** Function declarations in headers must be enclosed in an `extern "C"` block, guarded by `#ifdef __cplusplus`.

## 2. Naming Conventions

 * **Types (Structs, Enums, Typedefs):** `snake_case` with a `_t` suffix (e.g., `ahrs_time_us_t`, `ahrs_config_t`).
 * **Functions:** `snake_case` prefixed with the module name (e.g., `ahrs_init`, `ahrs_update`).
 * **Variables:** `snake_case`.
 * **Physical Units:** Variables representing physical quantities must append their unit as a suffix (e.g., `_rad` for radians, `_rps` for rad/s, `_sec` for seconds, `_mps2` for m/s², `_us` for microseconds, `_hz` for Hertz).
 * **Macros & Enum Values:** `UPPER_SNAKE_CASE`. Enum values must start with the prefix of the enum type (e.g., `AHRS_MODE_ARS`).

## 3. Formatting and Indentation

 * **Indentation:** 4 spaces (no tabs).
 * **Column width:** 100 characters
 * **Brace Style:** Allman/BSD style. The opening curly brace `{` must be placed on a new, dedicated line.
    ```c
    typedef struct
    {
        int member;
    } name_t;
    ```
 * **Alignment:** Struct members and their trailing inline comments (`/**< ... */`) should be vertically aligned to improve readability.
 * **Array Parameters:** When the size of an array parameter is fixed, it should be explicitly specified in the function declaration for better documentation (e.g., `const float acc_mps2[3]`).
 * **Enforcement:** `make format`/`make format-check` run `.clang-format`.
   Its verdict is version-sensitive (e.g. `(MACRO_PARAM)*x` vs.
   `(MACRO_PARAM) * x` differs between clang-format releases), so CI pins
   clang-format via `ubuntu-22.04` (see `.github/workflows/ci.yml`). On a
   machine whose system clang-format is newer (Debian, Ubuntu 24.04, ...),
   run `scripts/fetch_ci_clang_format.sh` once to vendor a copy matching
   CI's exact version into `.tools/` - `make format`/`format-check` pick it
   up automatically when present.

## 4. Documentation (Doxygen)

 * **Functions:** Documented using `/** @brief ... */`. Parameters must be explicitly specified using `@param[in]`, `@param[out]`, or `@param[in,out]`. Return values must use `@return`.
 * **Struct/Enum Members:** Use trailing Doxygen comments `/**< ... */` on the same line as the member definition.
 * **Value Ranges/Defaults:** For configuration parameters, document reasonable default values directly within the comment (e.g., `(0 -> 5)`).


## 5. Safety-Critical & Deterministic Design

To ensure highly predictable runtime behavior and avoid common embedded systems failures, the following architectural rules apply:

 * **No Dynamic Memory Allocation:** The use of the heap (`malloc`, `calloc`, `realloc`, `free`) is not allowed. All data structures and memory buffers must be allocated statically or on the stack with deterministic, fixed bounds. All operational state must reside inside context structures.
 * **No Recursion:** Functions must not call themselves, either directly or indirectly.
 * **Bounded Loops:** Every loop (`for`, `while`) must have a deterministically bounded number of iterations. Unbounded loops or infinite polling mechanisms inside computational blocks are forbidden.
 * **Defensive Input Validation:** * Pointer arguments must be verified against `NULL` before dereferencing, or guaranteed safe by design with explicit documentation.
 * Computational algorithms (such as Kalman filters) must actively check input samples for non-finite values (`NaN`, `Inf`) to prevent state corruption (e.g., dropping invalid IMU epochs).
 * **Safe Floating-Point Comparisons:** Floating-point numbers must never be compared directly using exact equality (`==` or `!=`). Always use relational operators (`<`, `>`, `<=`, `>=`) or an epsilon-based tolerance window (`fabs(a - b) < EPSILON`).
 * **Mandatory Error Handling:** Functions that can fail due to invalid configurations or bad state must return an explicit status code (e.g., `0` for success, `-1` for failure). The calling function is responsible for capturing and handling this return value.
 * Errors and irregular events must be tracked in low-overhead status variables.

## 6. Domain & Numerical Conventions

 * **Language Standard:** C11, must compile clean with `-Wall -Wextra`. Comments and identifiers are in English.
 * **Floating-Point Precision:** The hot path uses 32-bit `float` throughout. `double` is reserved for absolute-position anchors (ECEF / lat-lon), where 32-bit precision is insufficient.
 * **Matrices:** Column-major.
 * **Quaternions:** Hamilton convention, `q[0]` is the scalar/real part (`q = [w, x, y, z]`).
 * **Body Frame:** FRD (x forward, y right, z down).
 * **Navigation Frame:** NED (North-East-Down).
 * **Euler Angles:** Tait-Bryan ZYX (roll, pitch, yaw).
 * **Timestamps:** `int64_t`, microseconds.
 * **Comments:** Avoid the characters `;` and `--` inside comments.

## 7. Testing Conventions

 * Tests are plain C with no external test framework: `scenario_*` functions using `CHECK` macros, registered in `main()`. The test binary exits with a non-zero status on any failure.

