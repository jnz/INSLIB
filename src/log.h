/** @file log.h
 * @author Jan Zwiener (jan@zwiener.org)
 *
 * @brief Optional debug logging facility for INSLIB.
 *
 * Four severity levels, most to least verbose: INFO, WARN, ERROR, FATAL.
 * A single macro, LOG_LEVEL, is the compile-time ceiling: any call whose level
 * is above it expands to nothing - no log_write() reference is even emitted, so
 * a build with logging off (LOG_LEVEL_NONE) carries no code size or call
 * overhead and does not need log.c linked in. Define LOG_LEVEL on the compiler
 * command line (e.g. -DLOG_LEVEL=LOG_LEVEL_WARN) for the project-wide ceiling,
 * or before including "log.h" in a single .c file to change it just there.
 * LOG_LEVEL defaults to LOG_LEVEL_INFO.
 *
 * On top of that ceiling, log.c keeps one runtime threshold
 * (log_set_level/log_get_level) so verbosity can be changed without a rebuild,
 * up to the compiled-in ceiling.
 *
 * Output is redirectable: log_set_sink() installs a callback receiving the
 * level, file, line and printf-style format/args of every call that passes both
 * filters. The default sink writes to stdout via vprintf(); an embedded target
 * can point it at a UART, RTT or ring buffer instead, and may still call
 * log_default_sink() from within its own sink.
 *
 * This is the one piece of global mutable state in the library (the runtime
 * level and the sink pointer): threading a log context through every function
 * signature just for diagnostics is not worth the intrusion. It carries no
 * filter state and affects nothing numerically.
 *
 * Logging is diagnostic-only and NOT part of the WCET budget of any filter
 * here: vsnprintf/the installed sink have no bound guaranteed by this file. Do
 * not call the LOG_* macros from a path with a hard real-time deadline in a
 * build where LOG_LEVEL might be enabled. A FATAL log call does not itself
 * abort or reset - the caller decides what to do next.
 *
 * Arguments to a call whose level is compiled out are NOT evaluated: do not
 * rely on side effects inside LOG_* arguments.
 */

/** @addtogroup log
 *  @{ */

#ifndef INS_LOG_H
#define INS_LOG_H

/******************************************************************************
 * SYSTEM INCLUDE FILES
 ******************************************************************************/

#include <stdarg.h>

/******************************************************************************
 * DEFINES
 ******************************************************************************/

/** Severity levels, most to least verbose. LOG_LEVEL_NONE is not a
 *  severity, only usable as a threshold ("nothing"). */
#define LOG_LEVEL_NONE  (0)
#define LOG_LEVEL_FATAL (1) /**< Unrecoverable error. */
#define LOG_LEVEL_ERROR (2) /**< Recoverable error. */
#define LOG_LEVEL_WARN  (3) /**< Warning, execution continues normally. */
#define LOG_LEVEL_INFO  (4) /**< Informational message. */

/** Compile-time verbosity ceiling: messages above this level expand to
 *  nothing at their call site. Override with -DLOG_LEVEL=... (project-
 *  wide) or define LOG_LEVEL before including "log.h" (single file). */
#ifndef LOG_LEVEL
#define LOG_LEVEL LOG_LEVEL_INFO
#endif

#if defined(__GNUC__) || defined(__clang__)
/** printf-style argument checking (format string is argument 4, the
 *  variadic args start at 5 - matches log_write's signature below). */
#define LOG_PRINTF_ATTR __attribute__((format(printf, 4, 5)))
/** Same, for the va_list-taking sink signature (format string is
 *  argument 4, 0 marks the rest as "already captured in a va_list"). */
#define LOG_VPRINTF_ATTR __attribute__((format(printf, 4, 0)))
#else
/** No-op on compilers without printf-style argument checking. */
#define LOG_PRINTF_ATTR
/** No-op on compilers without printf-style argument checking. */
#define LOG_VPRINTF_ATTR
#endif

#if LOG_LEVEL >= LOG_LEVEL_INFO
/** @brief Log an INFO-level message (compiles to nothing if LOG_LEVEL <
 *         LOG_LEVEL_INFO). */
#define LOG_INFO(...) log_write(LOG_LEVEL_INFO, __FILE__, __LINE__, __VA_ARGS__)
#else
#define LOG_INFO(...) ((void)0)
#endif

#if LOG_LEVEL >= LOG_LEVEL_WARN
/** @brief Log a WARN-level message (compiles to nothing if LOG_LEVEL <
 *         LOG_LEVEL_WARN). */
#define LOG_WARN(...) log_write(LOG_LEVEL_WARN, __FILE__, __LINE__, __VA_ARGS__)
#else
#define LOG_WARN(...) ((void)0)
#endif

#if LOG_LEVEL >= LOG_LEVEL_ERROR
/** @brief Log an ERROR-level message (compiles to nothing if LOG_LEVEL <
 *         LOG_LEVEL_ERROR). */
#define LOG_ERROR(...) log_write(LOG_LEVEL_ERROR, __FILE__, __LINE__, __VA_ARGS__)
#else
#define LOG_ERROR(...) ((void)0)
#endif

#if LOG_LEVEL >= LOG_LEVEL_FATAL
/** @brief Log a FATAL-level message (compiles to nothing if LOG_LEVEL <
 *         LOG_LEVEL_FATAL). */
#define LOG_FATAL(...) log_write(LOG_LEVEL_FATAL, __FILE__, __LINE__, __VA_ARGS__)
#else
#define LOG_FATAL(...) ((void)0)
#endif

/******************************************************************************
 * TYPEDEFS
 ******************************************************************************/

/** @brief Sink callback receiving one already-filtered log call.
 *  @param[in] level One of the LOG_LEVEL_* severities (never NONE).
 *  @param[in] file  __FILE__ of the call site (basename, see log.c).
 *  @param[in] line  __LINE__ of the call site.
 *  @param[in] fmt   printf-style format string.
 *  @param[in] args  Format arguments, consumed at most once, the same
 *                   way vprintf() consumes them. */
typedef void (*log_sink_fn)(int level, const char* file, int line, const char* fmt,
                            va_list args) LOG_VPRINTF_ATTR;

/******************************************************************************
 * FUNCTION PROTOTYPES
 ******************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

    /** @brief Entry point for the LOG_* macros, not normally called
     *  directly. No-op if level is above the current runtime threshold
     *  (log_get_level()).
     *  @param[in] level One of the LOG_LEVEL_* severities.
     *  @param[in] file  __FILE__ of the call site.
     *  @param[in] line  __LINE__ of the call site.
     *  @param[in] fmt   printf-style format string, followed by its
     *                   arguments. */
    void log_write(int level, const char* file, int line, const char* fmt, ...) LOG_PRINTF_ATTR;

    /** @brief Install a custom output sink, replacing the default
     *  stdout/vprintf one.
     *  @param[in] sink New sink, or NULL to reinstate the default. */
    void log_set_sink(log_sink_fn sink);

    /** @brief Default sink (stdout, "[LEVEL] file:line: " + vprintf(fmt,
     *  args) + newline). Exposed so a custom sink can fall back to it,
     *  e.g. to also print to stdout in addition to a UART.
     *  @param[in] level One of the LOG_LEVEL_* severities.
     *  @param[in] file  Call site file (basename).
     *  @param[in] line  Call site line.
     *  @param[in] fmt   printf-style format string.
     *  @param[in] args  Format arguments. */
    void log_default_sink(int level, const char* file, int line, const char* fmt,
                          va_list args) LOG_VPRINTF_ATTR;

    /** @brief Set the runtime verbosity threshold. Clamped to
     *  [LOG_LEVEL_NONE, LOG_LEVEL] - it can only ever be as loose as
     *  the compile-time ceiling log.c itself was built with, since
     *  anything above that was never compiled into a log_write() call
     *  in the first place.
     *  @param[in] level Desired threshold (one of the LOG_LEVEL_*
     *                   constants, including LOG_LEVEL_NONE to silence
     *                   everything at runtime). */
    void log_set_level(int level);

    /** @brief Current runtime verbosity threshold.
     *  @return One of the LOG_LEVEL_* constants. */
    int log_get_level(void);

#ifdef __cplusplus
}
#endif

#endif /* INS_LOG_H */
/** @} */
