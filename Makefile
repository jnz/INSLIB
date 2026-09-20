# Makefile for INSLIB
#
# Builds/runs the sanity + integration tests against the KFCore
# submodule (KFCore/c/*.c).
#
# Cross-platform: detects Windows vs. POSIX (Linux/macOS) and picks the
# right executable suffix, run prefix and clean command automatically.
#
# Usage:
#   make               build the test binaries
#   make test          build, then run the unit/integration test binaries
#                      plus the replay gates (simulated, crazyflie, datasets)
#   make test-asan     (POSIX) rebuild + run the tests under ASan/UBSan
#                      (turns memory errors/UB into hard failures)
#   make datasets      run the real-world dataset replay tests on their own
#                      (`make test` runs them too)
#   make simulated     gate replay.c + check_simulated.py on the committed
#                      synthetic datasets
#   make reqs          check the requirements database (requirements/)
#   make format        apply clang-format (style: .clang-format) to all
#                      project C sources (src/, tests/, datasets/,
#                      python/csrc/ -- not the KFCore submodule, not
#                      the generated src/wmm_*.h tables)
#   make format-check  same file set, but only verify: exits non-zero if
#                      anything is not clang-format clean (CI gate)
#   make cppcheck      static analysis (cppcheck) over the same file set,
#                      not part of `make test`, run on demand or in CI
#   make clang-tidy    static analysis (clang-tidy, config: .clang-tidy)
#                      over the same file set, also not part of `make test`
#   make stack         worst-case stack usage of every public API function,
#                      gated against the budgets in scripts/stack_usage.cfg
#                      (needs gcc >= 10), not part of `make test`
#   make readme-stack  refresh readme.md's stack usage block from the JSON
#                      of `make stack` (release process step 4)
#   make coverage     (Linux only) build with gcov instrumentation, run
#                      every host test binary (incl. KFCore's own
#                      KFCore/tests/test.c and the firmware config/UBX
#                      tests, so embedded/stm32f429 shows up in the
#                      report too) and render an lcov HTML report
#   make check-all     (POSIX) run everything above that's a quality gate
#                      (test, reqs, format-check, cppcheck, clang-tidy,
#                      stack, doxygen -- treating any doc warning as failure --
#                      and coverage), prints a pass/fail summary, exits 1
#                      if anything failed. Pre-push / CI use.
#   make clean         remove the built binaries and (on POSIX) the
#                      coverage build/report
#   make coverage-clean  remove only the coverage build/report
#
# All build output (test binaries, replay, insrcv) lands in build/, kept
# out of the source tree; `make clean` removes it wholesale.
#
# On Windows use mingw32-make in place of make.

# --- Platform detection -----------------------------------------------------
ifeq ($(OS),Windows_NT)
    EXE := .exe
    SO  := .dll
    # How a recipe invokes a freshly built binary, and which delete command
    # works, both depend on which shell make actually settled on, not just
    # on $(OS): cmd.exe searches "." implicitly and only understands its own
    # builtins (del, with "/" args parsed as switches), while make prefers
    # sh.exe when one is on PATH (Git Bash, MSYS2) -- sh does NOT search ".",
    # so a bare "replay.exe" fails with "command not found" there, and sh
    # cannot CreateProcess "del" at all since it is not a real executable,
    # only a cmd.exe builtin. Detect once and pick both from that.
    ifneq (,$(findstring sh,$(notdir $(SHELL))))
        RUN := ./
        RM  := rm -f
    else
        RUN :=
        RM  := del /q
    endif
else
    EXE :=
    SO  := .so
    RM  := rm -f
    RUN := ./
    UNAME_S := $(shell uname -s)
    ifeq ($(UNAME_S),Darwin)
        SO := .dylib
    endif
endif

BUILD_DIR := build

CC       := gcc
CFLAGS   := -Wall -Wextra -std=c11 -D_GNU_SOURCE -g \
            -Wshadow -Wfloat-equal -Wconversion -Wsign-conversion \
            -Wdouble-promotion -Wcast-align -Wformat=2 -Wundef \
            -Wmissing-prototypes -Wstrict-prototypes -Wredundant-decls \
            -Wpointer-arith -Wwrite-strings -Wnull-dereference \
            -Wduplicated-cond -Wduplicated-branches -Wlogical-op \
            -Wbad-function-cast -Wswitch-enum

# KFCore's Kalman backend dimensions its scratch matrices from these two
# compile-time limits, which default to 32 each (KFCore/c/kalman_udu.c).
# INSLIB's largest filter is the 18-state INS, whose process noise input
# matrix G has 12 + 18 - 6 = 24 columns (INS_NOISE_COLS_MAX), so the
# defaults leave several kB of stack unused on every predict step. src/ins.c
# static-asserts both values against INS_UNKNOWNS_MAX/INS_NOISE_COLS_MAX: a
# value too small for the filters is a build error, not a silent overflow.
KFCORE_LIMITS := -DKALMAN_MAX_STATE_SIZE=18 -DKALMAN_MAX_NOISE_SIZE=24
CFLAGS   += $(KFCORE_LIMITS)

HARNESS_CFLAGS := $(CFLAGS) -Wno-double-promotion
INCLUDES := -Isrc -IKFCore/c -IKFCore/c/navigation_tools -IKFCore/tests
LDLIBS   := -lm

NAV_SRC          := src/ins.c src/geodetic_toolbox.c src/magnetic_model.c src/log.c
MATH_SRC         := src/geodetic_toolbox.c src/magnetic_model.c
SUITE_SRC        := src/ins.c src/geodetic_toolbox.c src/magnetic_model.c src/ahrs.c src/baro_alt.c src/nav_suite.c src/log.c
KFCORE_SRC       := KFCore/c/linalg.c KFCore/c/kalman_udu.c KFCore/c/miniblas.c
KFCORE_SRC_MATH  := KFCore/c/linalg.c KFCore/c/miniblas.c

# KFCore is a git submodule: a plain `git clone` (without --recursive)
# leaves it as an empty directory. Every target below lists a KFCore file
# as a prerequisite, so this rule fires automatically the first time one
# of them is actually needed, instead of everyone hitting the same
# confusing "No rule to make target 'KFCore/c/linalg.c'" on a fresh clone.
KFCore/c/linalg.c:
	@echo "KFCore submodule not initialized, running git submodule update --init --recursive ..."
	git submodule update --init --recursive

# Every binary is built in one compiler invocation from the .c files
# above, so make has no object files from which to notice a changed
# header. Without this, editing a header (a tuning constant in
# sensor_defaults.h, a struct field) leaves a STALE test binary that
# still passes -- silently reporting a green run for code that was never
# compiled. Listed as an order-independent prerequisite on every target.
NAV_HDR          := $(wildcard src/*.h) $(wildcard KFCore/c/*.h)
# KFCore's own unit tests (KFCore/tests/test.c), exercising KFCore/c
# directly, added to `make coverage` only (see COV_TEST_KFCORE below).
KFCORE_TEST_SRC  := KFCore/c/linalg.c KFCore/c/miniblas.c KFCore/c/kalman_udu.c \
                     KFCore/c/kalman_takasu.c KFCore/tests/test.c \
                     tests/kfcore_benchmark_stub.c

# Firmware configuration store, sensor calibration and config protocol
# (embedded/stm32f429). Everything platform specific in them sits behind
# cfg_port.h, so they build and run on the host against the RAM backed
# flash in tests/test_cfg.c. Without that, the slot rotation and the
# torn-write recovery would only ever be exercised on the target, where
# a power cut in the middle of a save is not something one can arrange
# on demand.
FW_DIR      := embedded/stm32f429/Core
FW_CFG_SRC  := $(FW_DIR)/Src/inslib_sensor/cfg.c \
               $(FW_DIR)/Src/inslib_sensor/cal.c \
               $(FW_DIR)/Src/inslib_sensor/cfg_ubx.c \
               $(FW_DIR)/Src/inslib_sensor/ubx.c \
               $(FW_DIR)/Src/inslib_sensor/imu_saturation.c
FW_INCLUDES := -I$(FW_DIR)/Inc
FW_HDR      := $(wildcard $(FW_DIR)/Inc/inslib/*.h)

TEST_CORE := $(BUILD_DIR)/test_core$(EXE)
TEST_MATH := $(BUILD_DIR)/test_math$(EXE)
TEST_AHRS := $(BUILD_DIR)/test_ahrs$(EXE)
TEST_BARO := $(BUILD_DIR)/test_baro$(EXE)
TEST_LOG  := $(BUILD_DIR)/test_log$(EXE)
TEST_YAML := $(BUILD_DIR)/test_yaml$(EXE)
TEST_CFG  := $(BUILD_DIR)/test_cfg$(EXE)
REPLAY    := $(BUILD_DIR)/replay$(EXE)
INSRCV    := $(BUILD_DIR)/insrcv$(EXE)

# embedded/ only exists in the source (internal) repo - the public repo and
# its CI build without it. test_cfg needs it directly (it exercises the
# firmware's config store on the host, see the comment above), so it is
# built/run only when embedded/ is actually present, rather than failing
# the whole build for everyone else.
HAVE_EMBEDDED := $(wildcard $(FW_DIR)/Src/inslib_sensor/cfg.c)
ifneq ($(HAVE_EMBEDDED),)
ALL_TEST_CFG := $(TEST_CFG)
RUN_TEST_CFG := $(RUN)$(TEST_CFG)
else
ALL_TEST_CFG :=
# No leading @: Make's @-silencing is resolved on the literal recipe text
# before variable expansion, so an @ hidden inside this variable would
# reach the shell literally instead of silencing the line.
RUN_TEST_CFG := echo "test_cfg: skipped, embedded/ not present (public repo)"
endif

# --- Toolchain check (Windows) ----------------------------------------------
# Without this, a missing compiler shows up as a bare "gcc: command not
# found" from inside a recipe, halfway down a wall of build output. Catch it
# before anything runs and say what to do about it. Deliberately no search
# of "likely" install directories: those guesses are wrong on most machines,
# and the fix is the same wherever the toolchain lives.
ifeq ($(OS),Windows_NT)

# "gcc -dumpmachine" prints ONE token (x86_64-w64-mingw32) when it works,
# while every shell's "not found" message is several words. Comparing the
# word count keeps the probe independent of which shell make picked (sh or
# cmd) and of its language -- a German cmd says "nicht gefunden", so
# searching for "not" would not do.
ifneq (1,$(words $(shell gcc -dumpmachine 2>&1)))

# Goals that actually compile. clean/reqs/format/doc must keep working
# without a compiler, so only abort for these (no goal at all means the
# default target, which builds).
CC_GOALS := all test test-asan check-all coverage readme-coverage \
            $(TEST_CORE) $(TEST_MATH) $(TEST_AHRS) $(TEST_BARO) $(TEST_LOG) \
            test_core test_math test_ahrs test_baro test_log \
            $(REPLAY) replay $(INSRCV) insrcv \
            pylib pytest datasets datasets-fog datasets-kfgins \
            datasets-tunnel datasets-tunnel-nhc datasets-tunnel-odometry \
            datasets-pedestrian simulated crazyflie

define CC_MISSING_MSG

  gcc was not found on PATH.

  Windows has no compiler on PATH by default, and MinGW-w64 / MSYS2
  installers usually do not add one either. Put your toolchain's bin
  directory on PATH for this shell and re-run make:

      cmd:       set "PATH=<toolchain>\bin;%PATH%"
      Git Bash:  export PATH="/c/<toolchain>/bin:$$PATH"

  Some MinGW-w64 distributions ship a mingwvars.bat / mingwvars.sh in the
  install root that does exactly this -- run (or source) it instead. MSYS2
  users can start its "MSYS2 MinGW 64-bit" shell, which comes preset.

  For VS Code's build tasks and the debugger, add that bin directory to
  your user PATH permanently (Windows Settings > "Edit environment
  variables for your account") and restart VS Code -- they inherit PATH at
  startup, so a per-shell setup does not reach them.

  No toolchain installed yet? Get MinGW-w64 or MSYS2. To build just the
  Python binding without gcc, use build_pylib_msvc.bat (MSVC).

endef

ifneq (,$(filter $(CC_GOALS),$(or $(MAKECMDGOALS),all)))
$(error $(CC_MISSING_MSG))
endif

endif
endif

.PHONY: all test clean coverage coverage-clean readme-coverage datasets datasets-kfgins \
        datasets-fog datasets-tunnel datasets-tunnel-nhc datasets-tunnel-odometry \
        datasets-pedestrian \
        simulated crazyflie reqs pylib pytest wmm doc doxygen test-asan \
        format format-check cppcheck clang-tidy stack readme-stack check-all insrcv \
        test_core test_math test_ahrs test_baro test_log test_cfg test_yaml \
        replay

# --- Build output directory --------------------------------------------------
# Order-only prerequisite (the "|" below) on every binary rule: it only
# needs to exist before linking, its own timestamp must never force a
# relink of an otherwise up-to-date binary.
$(BUILD_DIR):
ifneq (,$(findstring del,$(RM)))
	@if not exist $(BUILD_DIR) mkdir $(BUILD_DIR)
else
	@mkdir -p $(BUILD_DIR)
endif

# Bare-name convenience aliases (e.g. `make test_core`, matching the
# .vscode/tasks.json labels) for the $(EXE)-suffixed real targets above.
test_core: $(TEST_CORE)
test_math: $(TEST_MATH)
test_ahrs: $(TEST_AHRS)
test_baro: $(TEST_BARO)
test_log:  $(TEST_LOG)
test_yaml: $(TEST_YAML)
ifneq ($(HAVE_EMBEDDED),)
test_cfg:  $(TEST_CFG)
else
test_cfg:
	@echo "test_cfg: skipped, embedded/ not present (public repo)"
endif
replay:    $(REPLAY)

# Suppress the optional debug-log facility (src/log.h) in the filter
# unit/integration tests: they deliberately drive the filters through
# thousands of injected faults/outliers, and every LOG_WARN would
# otherwise drown the terse ok/FAIL output in noise (verified: over
# 2000 chi2-downweight lines in one run without this). test_log itself
# is excluded - it specifically exercises level filtering across the
# full range and needs log.h's default (LOG_LEVEL_INFO) ceiling intact.
# Not applied to $(CFLAGS)/pylib/replay/insrcv: consumers of the
# library get full logging by default, as documented in log.h - only
# this repo's own noisy fault-injection tests opt out.
TEST_LOG_LEVEL_CFLAGS := -DLOG_LEVEL=LOG_LEVEL_NONE

all: $(TEST_CORE) $(TEST_MATH) $(TEST_AHRS) $(TEST_BARO) $(TEST_LOG) $(TEST_YAML) \
     $(ALL_TEST_CFG)

$(TEST_CORE): $(NAV_SRC) $(KFCORE_SRC) tests/test_ins_core.c $(NAV_HDR) | $(BUILD_DIR)
	$(CC) $(HARNESS_CFLAGS) $(TEST_LOG_LEVEL_CFLAGS) $(INCLUDES) $(filter %.c,$^) $(LDLIBS) -o $@

$(TEST_MATH): $(MATH_SRC) $(KFCORE_SRC_MATH) tests/test_ins_math.c $(NAV_HDR) | $(BUILD_DIR)
	$(CC) $(HARNESS_CFLAGS) $(INCLUDES) $(filter %.c,$^) $(LDLIBS) -o $@

$(TEST_AHRS): $(SUITE_SRC) $(KFCORE_SRC) tests/test_ahrs.c $(NAV_HDR) | $(BUILD_DIR)
	$(CC) $(HARNESS_CFLAGS) $(TEST_LOG_LEVEL_CFLAGS) $(INCLUDES) $(filter %.c,$^) $(LDLIBS) -o $@

$(TEST_BARO): $(SUITE_SRC) $(KFCORE_SRC) tests/test_baro.c $(NAV_HDR) | $(BUILD_DIR)
	$(CC) $(HARNESS_CFLAGS) $(TEST_LOG_LEVEL_CFLAGS) $(INCLUDES) $(filter %.c,$^) $(LDLIBS) -o $@

$(TEST_LOG): src/log.c tests/test_log.c $(NAV_HDR) | $(BUILD_DIR)
	$(CC) $(HARNESS_CFLAGS) $(INCLUDES) $(filter %.c,$^) $(LDLIBS) -o $@

# The YAML subset reader the two harnesses share. Header only, so this
# needs nothing but -Itools and the test file itself.
$(TEST_YAML): tests/test_yaml.c tools/mini_yaml.h | $(BUILD_DIR)
	$(CC) $(HARNESS_CFLAGS) -Itools $(filter %.c,$^) $(LDLIBS) -o $@

$(TEST_CFG): $(FW_CFG_SRC) tests/test_cfg.c $(FW_HDR) | $(BUILD_DIR)
	$(CC) $(HARNESS_CFLAGS) $(FW_INCLUDES) $(filter %.c,$^) $(LDLIBS) -o $@

test: all
	$(RUN)$(TEST_CORE)
	$(RUN)$(TEST_MATH)
	$(RUN)$(TEST_AHRS)
	$(RUN)$(TEST_BARO)
	$(RUN)$(TEST_LOG)
	$(RUN)$(TEST_YAML)
	$(RUN_TEST_CFG)
	@$(MAKE) --no-print-directory simulated
	@$(MAKE) --no-print-directory crazyflie
	@$(MAKE) --no-print-directory datasets

tests: test

# --- Sanitizer test run (ASan + UBSan, POSIX/gcc only) ------------------------
# Separate binaries so the normal build stays uninstrumented. Findings are
# non-recoverable: any memory error or UB aborts the run -> non-zero exit.
# Rationale: the corrupted-covariance fail-safe tests (REQ-SYS-005) run the
# filter math on broken UDU factors; "does not crash" is only meaningful if
# out-of-bounds accesses and UB are detected (REQ-VER-007).
SAN_FLAGS := -fsanitize=address,undefined -fno-sanitize-recover=all -O1

ifeq ($(OS),Windows_NT)
test-asan:
	@echo "make test-asan is POSIX-only (needs gcc ASan/UBSan)." && exit 1
else
test-asan: | $(BUILD_DIR)
	$(CC) $(HARNESS_CFLAGS) $(TEST_LOG_LEVEL_CFLAGS) $(SAN_FLAGS) $(INCLUDES) $(NAV_SRC) $(KFCORE_SRC) tests/test_ins_core.c $(LDLIBS) -o $(BUILD_DIR)/test_core_asan
	$(CC) $(HARNESS_CFLAGS) $(SAN_FLAGS) $(INCLUDES) $(MATH_SRC) $(KFCORE_SRC_MATH) tests/test_ins_math.c $(LDLIBS) -o $(BUILD_DIR)/test_math_asan
	$(CC) $(HARNESS_CFLAGS) $(TEST_LOG_LEVEL_CFLAGS) $(SAN_FLAGS) $(INCLUDES) $(SUITE_SRC) $(KFCORE_SRC) tests/test_ahrs.c $(LDLIBS) -o $(BUILD_DIR)/test_ahrs_asan
	$(CC) $(HARNESS_CFLAGS) $(TEST_LOG_LEVEL_CFLAGS) $(SAN_FLAGS) $(INCLUDES) $(SUITE_SRC) $(KFCORE_SRC) tests/test_baro.c $(LDLIBS) -o $(BUILD_DIR)/test_baro_asan
	$(CC) $(HARNESS_CFLAGS) $(SAN_FLAGS) $(INCLUDES) src/log.c tests/test_log.c $(LDLIBS) -o $(BUILD_DIR)/test_log_asan
ifneq ($(HAVE_EMBEDDED),)
	$(CC) $(HARNESS_CFLAGS) $(SAN_FLAGS) $(FW_INCLUDES) $(FW_CFG_SRC) tests/test_cfg.c $(LDLIBS) -o $(BUILD_DIR)/test_cfg_asan
endif
	./$(BUILD_DIR)/test_core_asan
	./$(BUILD_DIR)/test_math_asan
	./$(BUILD_DIR)/test_ahrs_asan
	./$(BUILD_DIR)/test_baro_asan
	./$(BUILD_DIR)/test_log_asan
ifneq ($(HAVE_EMBEDDED),)
	./$(BUILD_DIR)/test_cfg_asan
else
	@echo "test_cfg_asan: skipped, embedded/ not present (public repo)"
endif
endif

# --- Worst-case stack usage (scripts/stack_usage.py) --------------------------
# Compiles the library with the compiler at hand and takes GCC's
# -fcallgraph-info=su output (frame size of every function plus the call
# graph, after inlining) to compute the deepest call chain of every public
# API function. Fails on recursion, VLA/alloca, a call through a function
# pointer or into a library function without a stack figure, and on a
# budget overrun. Figures for library functions, function pointer targets
# and the budgets live in scripts/stack_usage.cfg, the budgets there are for
# x86_64. Needs GCC >= 10.
# Fortify-source and stack-protector are distro-injected hardening a
# from-source embedded build does not carry, they add calls to libc
# helpers (__memcpy_chk, __stack_chk_fail, ...) under names the cfg does
# not track, so both are turned off to measure the library's own frames.
STACK_CC     ?= $(CC)
STACK_CFLAGS := -std=c11 -O2 -D_GNU_SOURCE -U_FORTIFY_SOURCE -fno-stack-protector \
                $(KFCORE_LIMITS) $(INCLUDES)

stack: $(SUITE_SRC) $(KFCORE_SRC) $(NAV_HDR) scripts/stack_usage.cfg scripts/stack_usage.py | $(BUILD_DIR)
	python3 scripts/stack_usage.py --config scripts/stack_usage.cfg --cc $(STACK_CC) \
	    --cflags="$(STACK_CFLAGS)" --build-dir $(BUILD_DIR)/stack \
	    --json $(BUILD_DIR)/stack/stack_usage.json $(SUITE_SRC) $(KFCORE_SRC)

# Refresh the "Worst-case stack usage" block in readme.md from the JSON of
# `make stack` (scripts/update_readme_stack.py). Host (x86_64) figures only,
# on purpose - the readme is public and only needs to show stack usage is
# under control; embedded/stm32f429's own cross-compiled numbers from its
# `make stack` stay an embedded/ concern. Release process step 4 in
# CLAUDE.md.
readme-stack: stack
	python3 scripts/update_readme_stack.py

# --- Requirements database (see requirements/README.md) ----------------------
# requirements/ is internal-only (see scripts/public_export.exclude in the
# source repo) - the public repo and its CI build without it, so this gate
# is skipped there rather than failed.
reqs:
ifeq ($(wildcard requirements/check_reqs.py),)
	@echo "reqs: skipped, requirements/ not present (public repo)"
else
	python3 requirements/check_reqs.py
endif

# --- Code formatting (clang-format, style rules in .clang-format) ------------
# Covers all C sources we own; excludes the KFCore submodule
# (never edited here) and the generated WMM lookup tables (make wmm
# rewrites them, so formatting would just churn).
#
# clang-format's verdict depends on its exact version (ci.yml pins
# ubuntu-22.04 for this reason). If scripts/fetch_ci_clang_format.sh has
# vendored a copy matching that pinned version, use it instead of whatever
# is on PATH, so `make format`/`format-check` agree with CI even on a
# machine whose system clang-format is newer (e.g. Debian, Ubuntu 24.04).
CLANG_FORMAT := $(if $(wildcard .tools/clang-format-14/bin/clang-format), \
                 .tools/clang-format-14/bin/clang-format,clang-format)
FORMAT_SRC   := $(filter-out src/wmm_lut.h src/wmm_test_vectors.h, \
                $(wildcard src/*.c src/*.h tests/*.c tests/*.h \
                           datasets/*.c datasets/*.h \
                           python/csrc/*.c python/csrc/*.h))

format:
	@echo "using $$($(CLANG_FORMAT) --version)"
	$(CLANG_FORMAT) -i $(FORMAT_SRC)

format-check:
	@echo "using $$($(CLANG_FORMAT) --version)"
	$(CLANG_FORMAT) --dry-run -Werror $(FORMAT_SRC)

# --- Static analysis (cppcheck) -----------------------------------------------
# Production code only (src/, datasets/, python/csrc/ - not tests/, not
# the KFCore submodule, not the generated WMM tables, narrower
# than `format`'s file set, which does cover tests/). Catches bug classes
# -Wall -Wextra don't (NULL derefs, dead stores, suspicious patterns) -
# complements, doesn't replace, the compiler warnings.
# Not part of `make test` (like test-asan): run on demand or in CI.
# False positives get an inline `// cppcheck-suppress <id>` right above
# the flagged line (see src/ahrs.c) rather than a blanket --suppress.
CPPCHECK     := cppcheck
CPPCHECK_SRC := $(filter-out tests/%, $(filter %.c, $(FORMAT_SRC)))

cppcheck:
	$(CPPCHECK) --enable=warning,style,performance,portability \
	            --std=c11 --language=c --inline-suppr --error-exitcode=1 \
	            --suppress=missingIncludeSystem \
	            --suppress=normalCheckLevelMaxBranches \
	            $(INCLUDES) $(CPPCHECK_SRC)

# --- Static analysis (clang-tidy) ----------------------------------------------
# Same (production-only) file set as cppcheck, checks/exclusions/
# header-filter live in .clang-tidy (auto-discovered) rather than on the
# command line. Different bug classes than cppcheck (deeper dataflow via
# clang's static analyzer), not part of `make test`, run on demand or in CI.
CLANG_TIDY := clang-tidy

clang-tidy:
	$(CLANG_TIDY) $(CPPCHECK_SRC) -- -std=c11 -D_GNU_SOURCE $(INCLUDES)

# --- Documentation (see doc/) -------------------------------------------------
# `make doc` builds two independent artifacts: doc/INSLIB_manual.pdf
# (architecture, design decisions, math models, needs latexmk or pdflatex)
# and the Doxygen API reference under doc/doxygen/ (needs doxygen, see
# doc/Doxyfile). Neither is part of `make test`. `make doxygen` builds
# only the latter.
doc:
	$(MAKE) -C doc

doxygen:
	$(MAKE) -C doc doxygen

# --- World Magnetic Model lookup tables --------------------------------------
# Regenerates the committed src/wmm_lut.h and src/wmm_test_vectors.h from
# the WMM coefficients. Run only when the WMM epoch changes (e.g. 2030) or
# the model is updated - the generated headers are committed, so this is
# deliberately NOT part of the normal build/test path. Needs pygeomag:
#   pip install -r magneticmodel/requirements.txt
#
# After regenerating, run the error analysis and read its numbers before
# committing the new tables:
#   python3 magneticmodel/wmm_error_analysis.py --step 1 --years 5
# It re-measures the interpolated table against the exact spherical-harmonics
# model and reports the residual error outside the dip pole exclusion zone,
# which is what REQ-SYS-018 and MAGNETIC_DIP_POLE_EXCLUSION_DEG rest on. The
# dip poles drift (the northern one by ~33 km per year), so the radius wants a
# fresh look at every epoch. It reports rather than passes or fails, hence a
# separate manual step and not a recipe line here.
wmm:
	python3 magneticmodel/generate_wmm_grid.py
	python3 magneticmodel/generate_test_vectors.py

# --- Real-world replay tests -------------------------------------------------
# Dataset bundle contract: doc/INSLIB_manual.tex, section "config.yaml"
# (schema) and datasets/replay_format.py (CSV column contract).
# `make datasets` runs one sub-target per dataset, add new datasets to
# the list below. Every dataset here is committed and deterministic, so
# `make test` runs the whole list as a regression gate -- a new entry
# below is picked up by `make test` without a second edit.
datasets: datasets-fog datasets-kfgins datasets-tunnel datasets-tunnel-nhc \
          datasets-tunnel-odometry datasets-pedestrian

$(REPLAY): $(SUITE_SRC) $(KFCORE_SRC) tools/replay.c $(NAV_HDR) | $(BUILD_DIR)
	$(CC) $(HARNESS_CFLAGS) $(INCLUDES) $(filter %.c,$^) $(LDLIBS) -o $@

# --- Live UBX receiver (UDP in, nav_suite, PlotJuggler out) -----------------
# Reads the sensor board's UBX stream off a serial port, drives nav_suite
# and publishes to PlotJuggler over UDP/JSON. Needs the socket library on
# Windows (ws2_32); POSIX has sockets and termios in libc.
ifeq ($(OS),Windows_NT)
INSRCV_LDLIBS := $(LDLIBS) -lws2_32
else
INSRCV_LDLIBS := $(LDLIBS)
endif

insrcv: $(INSRCV)

$(INSRCV): $(SUITE_SRC) $(KFCORE_SRC) tools/insrcv.c tools/mini_yaml.h tools/mini_mavlink.h \
           $(NAV_HDR) | $(BUILD_DIR)
	$(CC) $(HARNESS_CFLAGS) $(INCLUDES) $(filter %.c,$^) $(INSRCV_LDLIBS) -o $@

# --- Python binding: shared library loaded by the INSLIB package (ctypes) ----
PYLIB     := python/INSLIB/libINSLIB$(SO)
PYLIB_SRC := $(SUITE_SRC) $(KFCORE_SRC) python/csrc/ins_capi.c

pylib: $(PYLIB)

# $(NAV_HDR) and the capi header are prerequisites like everywhere else:
# without them a header-only change (a tuning constant in sensor_defaults.h,
# say) leaves a stale .dll behind, and every Python consumer -
# replay.py, check_simulated.py, the pytest suite - silently keeps
# reporting the OLD behaviour while the C harness reports the new one.
$(PYLIB): $(PYLIB_SRC) $(NAV_HDR) python/csrc/ins_capi.h
	$(CC) $(CFLAGS) -fPIC -shared -Ipython/csrc $(INCLUDES) $(filter %.c,$^) $(LDLIBS) -o $@

# Smoke-test the Python binding (uses pytest if present, else standalone).
# Not part of `make test`: the Python side is outside the C test gate.
# The fallback globs the same test_*.py set pytest would collect (each file
# has a __main__ runner exiting non-zero on failure), so a new test file
# cannot silently go unrun on a machine without pytest. It keeps going after
# a failure and reports every failing file, then exits non-zero.
pytest: $(PYLIB)
	@if python3 -c "import pytest" 2>/dev/null; then \
	    python3 -m pytest python/tests -q; \
	else \
	    echo "(pytest not installed - running standalone runner)"; \
	    failed=""; \
	    for t in python/tests/test_*.py; do \
	        echo "--> $$t"; \
	        python3 "$$t" || failed="$$failed $$t"; \
	    done; \
	    if [ -n "$$failed" ]; then \
	        echo "standalone runner failures:$$failed"; \
	        exit 1; \
	    fi; \
	    echo "standalone runner: all test files passed"; \
	fi

# Real MEMS ADAHRS vs. an independent FOG strapdown attitude reference, real
# GNSS position/velocity aiding (see datasets/fog/config.yaml).
datasets-fog: $(REPLAY)
	$(RUN)$(REPLAY) datasets/fog

# KF-GINS Dataset: Leador-A15 FOG + RTK-GNSS (https://github.com/i2Nav-WHU/KF-GINS/dataset).
datasets-kfgins: $(REPLAY)
	$(RUN)$(REPLAY) datasets/kfgins/config.yaml

# Car through a road tunnel: real u-blox fixes across a 94 s total GNSS
# blackout, the only recording here that drives the coasting expiry, the
# quality-loss re-arm and the origin carry on real receiver behaviour.
# Attitude is not scored (no attitude reference), see the header of
# datasets/tunnel/config.yaml for what the numbers do and do not mean.
datasets-tunnel: $(REPLAY)
	$(RUN)$(REPLAY) datasets/tunnel

# The same tunnel again, but the branch where the filter keeps its 3D
# solution for the whole outage instead of giving it up: gated on the
# coasting re-acquisition error (REQ-VER-029, REQ-VER-031), not on the
# whole-run RMS, which on such a dataset rewards abandoning the coast. Also
# the only committed dataset running the non-holonomic lateral constraint
# (REQ-NAV-077). Same tunnel as datasets/tunnel, driven the other way, and
# therefore not independent evidence about tunnels, see REQ-VER-031.
datasets-tunnel-nhc: $(REPLAY)
	$(RUN)$(REPLAY) datasets/tunnel_nhc

# The same Wattkopf tunnel once more, in the direction of datasets/tunnel, but
# with OBD2 wheel speed: the only real-data check of the absolute speed aiding
# (REQ-NAV-068). Gated tight enough that the exit error fails when the
# odometry stops helping, see REQ-VER-032.
datasets-tunnel-odometry: $(REPLAY)
	$(RUN)$(REPLAY) datasets/tunnel_odometry

# Hand-carried pedestrian, three recordings on one route: with corrections,
# without corrections, and outdoors only. The reference is a second estimator
# fed the same measurements, so the scored numbers are the difference between
# two filters, not an accuracy. See each info_*.txt.
PEDESTRIAN_DATASETS := datasets/pedestrian/05_outdoor_to_indoor_no_spartn \
                       datasets/pedestrian/06_outdoor_to_indoor \
                       datasets/pedestrian/07_outdoor_only

datasets-pedestrian: $(REPLAY)
	@for d in $(PEDESTRIAN_DATASETS); do \
	    echo "== $$d"; $(RUN)$(REPLAY) $$d || exit 1; \
	done

# --- Simulated regression datasets --------
# `make simulated` gates BOTH harnesses on the committed synthetic datasets
# (REQ-VER-016): the C harness (replay.c) scores ins + the ARS/AHRS
# sub-filters vs the true reference. The Python harness
# (datasets/check_simulated.py) re-scores ins through the ctypes binding
# AND checks it stays within a factor of the Groves textbook LC filter's own
# accuracy. That harness drives python/replay.py and owns the pass/fail gates itself.
# Deterministic synthetic data.
SIMULATED_DATASETS := datasets/simulated/profile_1_car \
                      datasets/simulated/profile_3_aircraft

# Scored by the C harness only. check_simulated.py benchmarks against the
# Groves textbook LC filter, which the profiles above are built for and
# these are not.
#
# B_drone/config_coasting.yaml replays the B_drone flight with a coasting
# window (4 s) SHORTER than its 62-72 s GNSS outage, so the filter goes
# inert mid-gap and re-acquires at the far end (REQ-NAV-064, REQ-NAV-065,
# REQ-NAV-066). Its own config.yaml deliberately keeps the whole outage
# inside the window, so without this second config nothing gates the
# freeze on data with an independent truth.
SIMULATED_C_ONLY := datasets/simulated/B_drone/config_coasting.yaml

# Part of `make test` (the default stability gate): the C harness always
# runs (pure C, committed data). check_simulated.py runs additionally when
# python3 + PyYAML + numpy are available (gracefully skipped otherwise, like
# the pytest target). numpy is pulled in transitively via replay.py ->
# geodetic_toolbox.py, so it is checked here too, not just yaml. The Python
# side stays optional for the C-only workflow.
simulated: $(REPLAY)
	@set -e; \
	for d in $(SIMULATED_DATASETS) $(SIMULATED_C_ONLY); do \
	    echo "=== replay.c: $$d ==="; \
	    $(RUN)$(REPLAY) $$d; \
	done; \
	if python3 -c "import yaml, numpy" >/dev/null 2>&1; then \
	    $(MAKE) --no-print-directory pylib; \
	    for d in $(SIMULATED_DATASETS); do \
	        echo "=== check_simulated.py: $$d ==="; \
	        python3 datasets/check_simulated.py $$d; \
	    done; \
	    echo "simulated: all datasets passed (replay.c + check_simulated.py)"; \
	else \
	    echo "simulated: replay.c passed; check_simulated.py SKIPPED (no python3/PyYAML/numpy)"; \
	fi

# --- Crazyflie regression datasets (Lighthouse ground truth) -----
# `make crazyflie` gates ins on real Crazyflie captures whose ground truth
# comes from an Lighthouse system (cm-accurate reference)
CRAZYFLIE_DATASETS := datasets/crazyflie/dead_reckoning \
                      datasets/crazyflie/motors_off_idle_ground \
                      datasets/crazyflie/motors_off_meandering \
                      datasets/crazyflie/motors_off_automotive_slow \
                      datasets/crazyflie/motors_off_aggressive \
                      datasets/crazyflie/motors_off_time_delay_200_ms \
                      datasets/crazyflie/baro_alt_test

crazyflie: $(REPLAY)
	@set -e; \
	for d in $(CRAZYFLIE_DATASETS); do \
	    echo "=== replay.c: $$d ==="; \
	    $(RUN)$(REPLAY) $$d; \
	done; \
	echo "crazyflie: all datasets passed (replay.c)"

# Every platform's shared-library extension the binding could have been
# built as (.dll/.so/.dylib), not just this platform's $(PYLIB)/$(SO): a
# tree built natively on Windows and also via WSL/Linux (or macOS) leaves
# the other platform's artifact sitting next to it, and a plain $(PYLIB)
# clean would never touch it.
PYLIB_ALL_SO := python/INSLIB/libINSLIB.dll python/INSLIB/libINSLIB.so python/INSLIB/libINSLIB.dylib

clean:
ifneq (,$(findstring del,$(RM)))
	-if exist $(BUILD_DIR) rmdir /s /q $(BUILD_DIR)
# del treats "/" as a switch marker, so "python/INSLIB/lib...dll" is parsed
# as an (unrecognized) option -- convert to backslashes first. (Only when
# $(RM) is actually cmd.exe's del; rm -f under Git Bash's sh is fine as-is.)
	-$(RM) $(subst /,\,$(PYLIB_ALL_SO))
else
	-$(RM) -r $(BUILD_DIR)
	-$(RM) $(PYLIB_ALL_SO)
endif
ifneq ($(OS),Windows_NT)
	-$(RM) -r $(COV_DIR)
endif

# --- Coverage (gcov + lcov, Linux only) -------------------------------------
#
# Builds separate, instrumented copies of the test binaries under coverage/
# (object files mirror the source tree under coverage/obj/, so files aren't
# built into the normal $(TEST_CORE)/$(TEST_MATH) above), runs them, then
# turns the resulting .gcda/.gcno files into an lcov HTML report.
#
# Requires gcov, lcov and genhtml (Ubuntu/Debian: apt install lcov).

COV_DIR    := coverage
COV_OBJDIR := $(COV_DIR)/obj
# KFCore's own tests build the same KFCore/c sources as the INSLIB tests,
# but with different flags (see $(COV_KFCORE_OBJ) below), so they need
# their own object tree -- a shared object would silently take whichever
# flags make happened to build it with first.
COV_KFCORE_OBJDIR := $(COV_DIR)/obj-kfcore
COV_CFLAGS := $(HARNESS_CFLAGS) -O0 -g --coverage

# Branch coverage: the --rc option was renamed in lcov 2.x
# (1.x: lcov_branch_coverage, 2.x: branch_coverage) -- with the wrong
# name the setting is silently ignored and the report has no branch
# data. Pick by installed major version.
#
# coverage is POSIX/Linux-only (see `make coverage` doc above), but these
# $(shell ...) calls are plain variable assignments, so GNU make evaluates
# them while *parsing* the Makefile -- for every goal, including `clean` on
# Windows, where awk/sed/[ are not on PATH. Guard the whole block so a
# Windows invocation never shells out to POSIX-only tools.
ifneq ($(OS),Windows_NT)
LCOV_VERSION_MAJOR := $(shell lcov --version 2>/dev/null | sed -n 's/.*version \([0-9]*\).*/\1/p')
ifeq ($(LCOV_VERSION_MAJOR),1)
LCOV_BRANCH_RC := --rc lcov_branch_coverage=1
else
LCOV_BRANCH_RC := --rc branch_coverage=1
endif

# MC/DC (modified condition/decision coverage) needs gcc's
# -fcondition-coverage. Only enable it for gcc >= 14.2, so older gcc
# builds still produce a plain line/branch coverage report.
GCC_VERSION_NUM := $(shell $(CC) -dumpfullversion | awk -F. '{printf "%d%02d%02d", $$1, $$2, $$3}')
MCDC_SUPPORTED  := $(shell [ $(GCC_VERSION_NUM) -ge 140200 ] && echo 1)
ifeq ($(MCDC_SUPPORTED),1)
COV_CFLAGS  += -fcondition-coverage
LCOV_MCDC   := --mcdc-coverage
else
LCOV_MCDC   :=
endif
endif

COV_TEST_CORE   := $(COV_DIR)/test_core
COV_TEST_MATH   := $(COV_DIR)/test_math
COV_TEST_AHRS   := $(COV_DIR)/test_ahrs
COV_TEST_BARO   := $(COV_DIR)/test_baro
COV_TEST_LOG    := $(COV_DIR)/test_log
COV_TEST_CFG    := $(COV_DIR)/test_cfg
COV_TEST_KFCORE := $(COV_DIR)/test_kfcore

# See HAVE_EMBEDDED above: coverage for test_cfg needs embedded/, which is
# internal-only and absent in the public repo/CI.
ifneq ($(HAVE_EMBEDDED),)
COV_ALL_TEST_CFG := $(COV_TEST_CFG)
else
COV_ALL_TEST_CFG :=
endif

COV_CORE_OBJ   := $(addprefix $(COV_OBJDIR)/,$(NAV_SRC:.c=.o) $(KFCORE_SRC:.c=.o) \
                                             tests/test_ins_core.o)
COV_MATH_OBJ   := $(addprefix $(COV_OBJDIR)/,$(MATH_SRC:.c=.o) $(KFCORE_SRC_MATH:.c=.o) \
                                             tests/test_ins_math.o)
COV_AHRS_OBJ   := $(addprefix $(COV_OBJDIR)/,$(SUITE_SRC:.c=.o) $(KFCORE_SRC:.c=.o) \
                                             tests/test_ahrs.o)
COV_BARO_OBJ   := $(addprefix $(COV_OBJDIR)/,$(SUITE_SRC:.c=.o) $(KFCORE_SRC:.c=.o) \
                                             tests/test_baro.o)
COV_LOG_OBJ    := $(addprefix $(COV_OBJDIR)/,src/log.o tests/test_log.o)
COV_CFG_OBJ    := $(addprefix $(COV_OBJDIR)/,$(FW_CFG_SRC:.c=.o) tests/test_cfg.o)
COV_KFCORE_OBJ := $(addprefix $(COV_KFCORE_OBJDIR)/,$(KFCORE_TEST_SRC:.c=.o))

# The firmware config store builds against embedded/stm32f429/Core/Inc,
# not against src/ - see $(TEST_CFG).
$(COV_CFG_OBJ): INCLUDES := $(FW_INCLUDES)

# KFCore/c/kalman_takasu.c and KFCore/tests/test.c is submodule code.
# Relax flags for this coverage-only build instead of fixing upstream code.
# The -U also drops $(KFCORE_LIMITS): KFCore's own tests exercise the backend
# beyond what INSLIB's filters need (15 states with 27 noise columns), so
# they run against KFCore's larger defaults. The -U wins over the -D because
# it comes later on the command line.
$(COV_KFCORE_OBJ): COV_CFLAGS += -Wno-missing-prototypes -Wno-sign-conversion \
                                 -Wno-conversion -Wno-float-conversion \
                                 -Wno-unused-parameter -Wno-format-nonliteral \
                                 -UKALMAN_MAX_STATE_SIZE -UKALMAN_MAX_NOISE_SIZE

$(COV_OBJDIR)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(COV_CFLAGS) $(INCLUDES) -c $< -o $@

$(COV_KFCORE_OBJDIR)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(COV_CFLAGS) $(INCLUDES) -c $< -o $@

# Link from the instrumented objects above. Note $^ (not
# $(filter %.c,$^) as in the non-coverage rules): the prerequisites here
# are .o files, so filtering for %.c leaves the linker with no input at
# all ("undefined reference to `main'").
$(COV_TEST_CORE): $(COV_CORE_OBJ)
	$(CC) $(COV_CFLAGS) $^ $(LDLIBS) -o $@

$(COV_TEST_MATH): $(COV_MATH_OBJ)
	$(CC) $(COV_CFLAGS) $^ $(LDLIBS) -o $@

$(COV_TEST_AHRS): $(COV_AHRS_OBJ)
	$(CC) $(COV_CFLAGS) $^ $(LDLIBS) -o $@

$(COV_TEST_BARO): $(COV_BARO_OBJ)
	$(CC) $(COV_CFLAGS) $^ $(LDLIBS) -o $@

$(COV_TEST_LOG): $(COV_LOG_OBJ)
	$(CC) $(COV_CFLAGS) $^ $(LDLIBS) -o $@

$(COV_TEST_CFG): $(COV_CFG_OBJ)
	$(CC) $(COV_CFLAGS) $^ $(LDLIBS) -o $@

$(COV_TEST_KFCORE): $(COV_KFCORE_OBJ)
	$(CC) $(COV_CFLAGS) $^ $(LDLIBS) -o $@

ifeq ($(OS),Windows_NT)
coverage coverage-clean:
	@echo "make coverage is Linux-only (needs gcov/lcov/genhtml)." && exit 1
else
coverage: $(COV_TEST_CORE) $(COV_TEST_MATH) $(COV_TEST_AHRS) $(COV_TEST_BARO) \
          $(COV_TEST_LOG) $(COV_ALL_TEST_CFG) $(COV_TEST_KFCORE)
	@# Drop coverage artifacts of renamed/deleted sources: their stale
	@# .gcno/.gcda would otherwise feed lcov/genhtml dead source paths
	@# ("genhtml: ERROR: cannot read ...").
	@for d in $(COV_OBJDIR) $(COV_KFCORE_OBJDIR); do \
	    find "$$d" -name '*.gcno' | while read -r g; do \
	        rel=$${g#$$d/}; \
	        test -f "$${rel%.gcno}.c" || \
	            rm -f "$${g%.gcno}.gcno" "$${g%.gcno}.gcda" "$${g%.gcno}.o"; \
	    done; \
	done
	./$(COV_TEST_CORE)
	./$(COV_TEST_MATH)
	./$(COV_TEST_AHRS)
	./$(COV_TEST_BARO)
	./$(COV_TEST_LOG)
ifneq ($(HAVE_EMBEDDED),)
	./$(COV_TEST_CFG)
else
	@echo "test_cfg coverage: skipped, embedded/ not present (public repo)"
endif
	./$(COV_TEST_KFCORE)
	lcov --capture --directory $(COV_OBJDIR) --directory $(COV_KFCORE_OBJDIR) \
	     --output-file $(COV_DIR)/coverage.info \
	     $(LCOV_BRANCH_RC) $(LCOV_MCDC)
	@# Exclude test/benchmark harness sources themselves, no coverage.
	lcov --remove $(COV_DIR)/coverage.info '/usr/*' '*/tests/*' \
	     '*/kalman_takasu.c' \
	     --ignore-errors unused \
	     --output-file $(COV_DIR)/coverage.info $(LCOV_BRANCH_RC) $(LCOV_MCDC)
	genhtml $(COV_DIR)/coverage.info --output-directory $(COV_DIR)/html \
	        --branch-coverage $(LCOV_MCDC)
	@echo "Coverage report: $(COV_DIR)/html/index.html"
	@# Core library only (src/): the >90% coverage claim (readme.md) is
	@# about the released library, not KFCore/ (separately versioned
	@# submodule) or embedded/ (reference board firmware, not part of the
	@# public release) - both drag the combined number above from the
	@# genhtml summary. Report src/ on its own so the two don't get
	@# conflated.
	lcov --extract $(COV_DIR)/coverage.info '*/src/*.c' \
	     --ignore-errors unused \
	     --output-file $(COV_DIR)/coverage-core.info $(LCOV_BRANCH_RC) $(LCOV_MCDC)
	@echo "==== src/ (core library) coverage ===="
	lcov --summary $(COV_DIR)/coverage-core.info $(LCOV_BRANCH_RC) $(LCOV_MCDC)

coverage-clean:
	$(RM) -r $(COV_DIR)

# Refresh the "Test coverage" block in readme.md from coverage-core.info's
# machine-readable LF/LH/BRF/BRH/MCF/MCH fields (scripts/
# update_readme_coverage.sh) - release process step 4 in CLAUDE.md.
readme-coverage: coverage
	scripts/update_readme_coverage.sh
endif

# --- Aggregate check (tests + reqs + format + static analysis + stack +
#     doxygen + coverage) -----------------------------------------------------
# Runs every gate independently, one failure doesn't stop the others.
# Prints a pass/fail summary at the end. exits 1 if anything failed.
# POSIX only (coverage is Linux-only).
ifeq ($(OS),Windows_NT)
check-all:
	@echo "make check-all is POSIX-only (coverage needs gcov/lcov/genhtml)." && exit 1
else
check-all:
	@log=$$(mktemp); \
	failed=0; \
	run() { \
	    name="$$1"; shift; \
	    echo "=== $$name ==="; \
	    if "$$@"; then \
	        echo "  ok    $$name" >> "$$log"; \
	    else \
	        echo "  FAIL  $$name" >> "$$log"; \
	        failed=1; \
	    fi; \
	    echo; \
	}; \
	run "tests"        $(MAKE) test; \
	run "requirements" $(MAKE) reqs; \
	run "clang-format" $(MAKE) format-check; \
	run "cppcheck"     $(MAKE) cppcheck; \
	run "clang-tidy"   $(MAKE) clang-tidy; \
	run "stack"        $(MAKE) stack; \
	run "doxygen"      bash -c 'set -o pipefail; $(MAKE) doxygen 2>&1 | tee /tmp/ins-check-all-doxygen.log; mst=$$?; ! grep -q "warning:" /tmp/ins-check-all-doxygen.log; gst=$$?; [ "$$mst" -eq 0 ] && [ "$$gst" -eq 0 ]'; \
	run "coverage"     $(MAKE) coverage; \
	echo "==== check-all summary ===="; \
	cat "$$log"; \
	rm -f "$$log"; \
	exit $$failed
endif

# --- Everything check-all doesn't cover (release process step 3) -----------
# check-all's "tests" gate already runs `make test`, whose last step
# replays the real datasets (datasets-fog/kfgins/tunnel/tunnel-nhc/
# tunnel-odometry/pedestrian) against ground truth, so this does not call
# `datasets` again. Adds the sanitizer build (ASan/UBSan) and the Python
# binding's test suite, both deliberately outside check-all/`make test` (see
# their own target comments), plus the STM32 target's cross-compiled stack
# analysis, best-effort: skipped without embedded/ (public repo) or an ARM
# toolchain (embedded/stm32f429/config.mk). Still leaves out `make doc`
# (needs LaTeX, only required when doc/*.tex changed) and the public-repo
# export (release process steps 5 and 7) - those stay manual.
# POSIX only, same reason as check-all/test-asan.
ifeq ($(OS),Windows_NT)
release-check:
	@echo "make release-check is POSIX-only (see check-all/test-asan)." && exit 1
else
release-check:
	@log=$$(mktemp); \
	failed=0; \
	run() { \
	    name="$$1"; shift; \
	    echo "=== $$name ==="; \
	    if "$$@"; then \
	        echo "  ok    $$name" >> "$$log"; \
	    else \
	        echo "  FAIL  $$name" >> "$$log"; \
	        failed=1; \
	    fi; \
	    echo; \
	}; \
	run "check-all" $(MAKE) check-all; \
	run "test-asan" $(MAKE) test-asan; \
	run "pytest"    $(MAKE) pytest; \
	if [ -n "$(HAVE_EMBEDDED)" ] && [ -f embedded/stm32f429/config.mk ]; then \
	    run "arm-stack" $(MAKE) -C embedded/stm32f429 stack; \
	else \
	    echo "=== arm-stack ==="; \
	    echo "skipped, embedded/ or embedded/stm32f429/config.mk not present"; \
	    echo "  skip  arm-stack" >> "$$log"; \
	    echo; \
	fi; \
	echo "==== release-check summary ===="; \
	cat "$$log"; \
	rm -f "$$log"; \
	exit $$failed
endif
