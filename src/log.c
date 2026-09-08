/** @file log.c
 * @author Jan Zwiener (jan@zwiener.org)
 *
 * @brief Optional debug logging facility for INSLIB.
 *
 * See log.h for the compile-time (LOG_LEVEL) and runtime
 * (log_set_level) filtering model and the sink redirection mechanism.
 * Everything below is the runtime half only: which calls that were NOT
 * already compiled out actually reach a sink, and where they end up by
 * default (stdout, via vprintf).
 */

#include <stdio.h>

#include "log.h"

/* ============================================================================
 * Local state
 * ============================================================================
 */

/* Runtime threshold: starts at the ceiling this file was compiled
   with, can only be lowered (or raised back up to it) at runtime, see
   log_set_level(). */
static int g_log_level = LOG_LEVEL;

static void default_sink(int level, const char* file, int line, const char* fmt,
                         va_list args) LOG_VPRINTF_ATTR;

static log_sink_fn g_log_sink = default_sink;

/* ============================================================================
 * Local functions
 * ============================================================================
 */

/* Last path component of file, so a full compiler-supplied __FILE__
   path does not dominate every log line. Both separators are checked:
   __FILE__ follows the build's path convention, which may be either on
   a cross-compile host. A forward scan (instead of two strrchr() calls
   compared with '>') avoids relying on relative-order pointer
   comparison across two independent searches. */
static const char* log_basename(const char* file)
{
    const char* last = file;
    const char* p;

    for (p = file; *p != '\0'; ++p)
    {
        if (*p == '/' || *p == '\\') { last = p + 1; }
    }
    return last;
}

static const char* level_name(int level)
{
    switch (level)
    {
    case LOG_LEVEL_FATAL: return "FATAL";
    case LOG_LEVEL_ERROR: return "ERROR";
    case LOG_LEVEL_WARN: return "WARN";
    case LOG_LEVEL_INFO: return "INFO";
    default: return "?";
    }
}

static void default_sink(int level, const char* file, int line, const char* fmt, va_list args)
{
    printf("[%-5s] %s:%d: ", level_name(level), file, line);
    vprintf(fmt, args);
    printf("\n");
}

/* ============================================================================
 * Global functions
 * ============================================================================
 */

void log_write(int level, const char* file, int line, const char* fmt, ...)
{
    va_list args;

    if (level > g_log_level) { return; }

    va_start(args, fmt);
    g_log_sink(level, log_basename(file), line, fmt, args);
    va_end(args);
}

void log_set_sink(log_sink_fn sink) { g_log_sink = (sink != NULL) ? sink : default_sink; }

void log_default_sink(int level, const char* file, int line, const char* fmt, va_list args)
{
    default_sink(level, file, line, fmt, args);
}

void log_set_level(int level)
{
    if (level < LOG_LEVEL_NONE) { level = LOG_LEVEL_NONE; }
    if (level > LOG_LEVEL) { level = LOG_LEVEL; }
    g_log_level = level;
}

int log_get_level(void) { return g_log_level; }
