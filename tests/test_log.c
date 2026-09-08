/** @file test_log.c
 * @author Jan Zwiener (jan@zwiener.org)
 *
 * Tests for the debug logging facility (src/log.c, src/log.h).
 *
 * Scenarios:
 *   1. default level: the runtime threshold starts at the compile-time
 *                      LOG_LEVEL ceiling (LOG_LEVEL_INFO for this file,
 *                      the header default -- this file does not
 *                      override it).
 *   2. runtime filter: log_set_level() gates which LOG_* calls reach
 *                      the sink; all four macros are compiled in here
 *                      (LOG_LEVEL defaults to LOG_LEVEL_INFO), so this
 *                      exercises the runtime half of the filter only.
 *   3. clamp:          log_set_level() clamps to [NONE, LOG_LEVEL].
 *   4. formatting:     the message reaching the sink matches a plain
 *                      vsnprintf of the same format/args, and the file
 *                      argument is a bare basename (no path).
 *   5. sink redirect:  log_set_sink()/NULL swaps the active sink and
 *                      restores the default one.
 *   6. default sink:   log_default_sink() writes "[LEVEL] file:line: msg"
 *                      to stdout for every level name, including an
 *                      unknown one.
 */
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#include "log.h"

/* Capturing stdout needs the fd duplication calls, which Windows spells
   with a leading underscore and declares in <io.h> rather than
   <unistd.h>. Only used by scenario_default_sink below. */
#if defined(_WIN32)
#include <io.h>
#define TEST_DUP    _dup
#define TEST_DUP2   _dup2
#define TEST_FILENO _fileno
#define TEST_CLOSE  _close
#else
#include <unistd.h>
#define TEST_DUP    dup
#define TEST_DUP2   dup2
#define TEST_FILENO fileno
#define TEST_CLOSE  close
#endif

static int fails = 0;

#define CHECK_TRUE(cond, msg)                    \
    do {                                         \
        if (!(cond))                             \
        {                                        \
            printf("  FAIL  %-40s\n", msg);      \
            fails++;                             \
        }                                        \
        else { printf("  ok    %-40s\n", msg); } \
    } while (0)

/* ---------------------------------------------------------------------------
 * Capturing sink: records the last call instead of printing it.
 * ---------------------------------------------------------------------------
 */

static int  cap_calls = 0;
static int  cap_level = -1;
static char cap_file[128];
static int  cap_line = -1;
static char cap_msg[256];

static void capture_sink(int level, const char* file, int line, const char* fmt,
                         va_list args) LOG_VPRINTF_ATTR;

static void capture_sink(int level, const char* file, int line, const char* fmt, va_list args)
{
    cap_calls++;
    cap_level = level;
    snprintf(cap_file, sizeof(cap_file), "%s", file);
    cap_line = line;
    vsnprintf(cap_msg, sizeof(cap_msg), fmt, args);
}

static void reset_capture(void)
{
    cap_calls   = 0;
    cap_level   = -1;
    cap_file[0] = '\0';
    cap_line    = -1;
    cap_msg[0]  = '\0';
}

/* ---------------------------------------------------------------------------
 * Scenario 1: default runtime level
 * ---------------------------------------------------------------------------
 */

static void scenario_default_level(void)
{
    printf("\n-- scenario: default runtime level --\n");
    CHECK_TRUE(log_get_level() == LOG_LEVEL_INFO,
               "starts at this file's compiled LOG_LEVEL (INFO)");
}

/* ---------------------------------------------------------------------------
 * Scenario 2: runtime filtering
 * ---------------------------------------------------------------------------
 */

static void scenario_runtime_filter(void)
{
    printf("\n-- scenario: runtime filtering --\n");
    log_set_sink(capture_sink);

    log_set_level(LOG_LEVEL_WARN);
    reset_capture();
    LOG_INFO("suppressed info %d", 1);
    CHECK_TRUE(cap_calls == 0, "INFO suppressed at WARN threshold");
    LOG_WARN("visible warn %d", 2);
    CHECK_TRUE(cap_calls == 1 && cap_level == LOG_LEVEL_WARN, "WARN passes at WARN threshold");
    LOG_ERROR("visible error %d", 3);
    CHECK_TRUE(cap_calls == 2 && cap_level == LOG_LEVEL_ERROR, "ERROR passes at WARN threshold");
    LOG_FATAL("visible fatal %d", 4);
    CHECK_TRUE(cap_calls == 3 && cap_level == LOG_LEVEL_FATAL, "FATAL passes at WARN threshold");

    log_set_level(LOG_LEVEL_NONE);
    reset_capture();
    LOG_FATAL("suppressed fatal");
    CHECK_TRUE(cap_calls == 0, "even FATAL suppressed at NONE threshold");

    log_set_level(LOG_LEVEL_INFO);
    reset_capture();
    LOG_INFO("visible info again");
    CHECK_TRUE(cap_calls == 1 && cap_level == LOG_LEVEL_INFO,
               "INFO passes again at INFO threshold");

    log_set_sink(NULL);
}

/* ---------------------------------------------------------------------------
 * Scenario 3: threshold clamping
 * ---------------------------------------------------------------------------
 */

static void scenario_clamp(void)
{
    printf("\n-- scenario: threshold clamping --\n");

    log_set_level(LOG_LEVEL_INFO + 100);
    CHECK_TRUE(log_get_level() == LOG_LEVEL_INFO,
               "over-range clamps to this file's compiled ceiling");

    log_set_level(-100);
    CHECK_TRUE(log_get_level() == LOG_LEVEL_NONE, "under-range clamps to LOG_LEVEL_NONE");

    log_set_level(LOG_LEVEL_INFO); /* restore for later scenarios */
}

/* ---------------------------------------------------------------------------
 * Scenario 4: message formatting and file/line
 * ---------------------------------------------------------------------------
 */

static void scenario_formatting(void)
{
    printf("\n-- scenario: message formatting --\n");
    log_set_sink(capture_sink);
    log_set_level(LOG_LEVEL_INFO);

    reset_capture();
    int line_of_call = __LINE__ + 1;
    LOG_INFO("value=%d name=%s pi=%.2f", 42, "abc", 3.14159);
    CHECK_TRUE(cap_calls == 1, "sink invoked once");
    CHECK_TRUE(strcmp(cap_msg, "value=42 name=abc pi=3.14") == 0, "formatted message matches");
    CHECK_TRUE(cap_line == line_of_call, "line number matches the call site");
    CHECK_TRUE(strchr(cap_file, '/') == NULL && strchr(cap_file, '\\') == NULL,
               "file argument is a bare basename");
    CHECK_TRUE(strcmp(cap_file, "test_log.c") == 0, "file basename is test_log.c");

    /* __FILE__ only ever gives '/' on this build host, so the '\\'
       separator (a cross-compiled Windows build's convention) needs an
       explicit call through log_write() to exercise the other half of
       the basename scan. */
    reset_capture();
    log_write(LOG_LEVEL_INFO, "C:\\src\\dir\\file.c", 7, "msg");
    CHECK_TRUE(strcmp(cap_file, "file.c") == 0, "backslash separator is honored");

    log_set_sink(NULL);
}

/* ---------------------------------------------------------------------------
 * Scenario 5: sink redirection
 * ---------------------------------------------------------------------------
 */

static void scenario_sink_redirect(void)
{
    printf("\n-- scenario: sink redirection --\n");

    reset_capture();
    log_set_sink(capture_sink);
    LOG_INFO("through custom sink");
    CHECK_TRUE(cap_calls == 1, "custom sink receives the call");

    reset_capture();
    log_set_sink(NULL); /* restore default (stdout) sink */
    LOG_INFO("through default sink, not captured");
    CHECK_TRUE(cap_calls == 0,
               "capture sink no longer receives calls after NULL restores the default");
}

/* ---------------------------------------------------------------------------
 * Scenario 6: the default (stdout) sink
 *
 * The default sink is what every consumer gets until it installs its
 * own, and it is the only piece of log.c a capturing sink cannot
 * observe -- so stdout is redirected into a file for the duration and
 * read back.
 * ---------------------------------------------------------------------------
 */

static void call_default_sink(int level, const char* file, int line, const char* fmt,
                              ...) LOG_PRINTF_ATTR;

static void call_default_sink(int level, const char* file, int line, const char* fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    log_default_sink(level, file, line, fmt, args);
    va_end(args);
}

static void scenario_default_sink(void)
{
    printf("\n-- scenario: default (stdout) sink --\n");

    const char* const path = "test_log_default_sink.tmp";
    char              buf[1024];
    size_t            n = 0;

    fflush(stdout);
    const int saved = TEST_DUP(TEST_FILENO(stdout));
    if (saved < 0 || freopen(path, "w", stdout) == NULL)
    {
        printf("  FAIL  %-40s\n", "could not redirect stdout");
        fails++;
        return;
    }

    call_default_sink(LOG_LEVEL_FATAL, "log.c", 11, "fatal %d", 1);
    call_default_sink(LOG_LEVEL_ERROR, "log.c", 22, "error %d", 2);
    call_default_sink(LOG_LEVEL_WARN, "log.c", 33, "warn %d", 3);
    call_default_sink(LOG_LEVEL_INFO, "log.c", 44, "info %d", 4);
    /* A level outside the known set must still print something rather
       than index off the end of the name table. */
    call_default_sink(LOG_LEVEL_INFO + 99, "log.c", 55, "unknown %d", 5);
    /* The default sink is also what log_set_sink(NULL) restores, so the
       same path has to work through log_write(). */
    log_set_sink(NULL);
    log_set_level(LOG_LEVEL_INFO);
    LOG_INFO("through the restored default sink");

    fflush(stdout);
    TEST_DUP2(saved, TEST_FILENO(stdout));
    TEST_CLOSE(saved);
    clearerr(stdout);

    {
        FILE* fp = fopen(path, "r");
        if (fp != NULL)
        {
            n = fread(buf, 1, sizeof(buf) - 1, fp);
            fclose(fp);
        }
        remove(path);
    }
    buf[n] = '\0';

    CHECK_TRUE(n > 0, "the default sink wrote to stdout");
    CHECK_TRUE(strstr(buf, "[FATAL] log.c:11: fatal 1") != NULL, "FATAL line, padded level name");
    CHECK_TRUE(strstr(buf, "[ERROR] log.c:22: error 2") != NULL, "ERROR line");
    CHECK_TRUE(strstr(buf, "[WARN ] log.c:33: warn 3") != NULL, "WARN line, name padded to 5");
    CHECK_TRUE(strstr(buf, "[INFO ] log.c:44: info 4") != NULL, "INFO line");
    CHECK_TRUE(strstr(buf, "[?    ] log.c:55: unknown 5") != NULL, "an unknown level prints as ?");
    CHECK_TRUE(strstr(buf, "through the restored default sink") != NULL,
               "log_set_sink(NULL) routes back to the default sink");
}

/* ------------------------------------------------------------------------- */

int main(void)
{
    scenario_default_level();
    scenario_runtime_filter();
    scenario_clamp();
    scenario_formatting();
    scenario_sink_redirect();
    scenario_default_sink();
    printf("\n==== %d failures ====\n", fails);
    return fails == 0 ? 0 : 1;
}
