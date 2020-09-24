
#ifndef _LVOLOG_H_
#define _LVOLOG_H_

#include <ctime>
#include <iostream>
#include <cstring>
#ifdef ANDROID
#include <android/log.h>
#endif

enum LogLevel {
    LVO_LOG_OFF = 0,
    LVO_LOG_TRACE = 1,
    LVO_LOG_DEBUG = 2,
    LVO_LOG_INFO = 3,
    LVO_LOG_WARN = 4,
    LVO_LOG_ERROR = 5,
};

class LoggingSupport {
  public:
    LoggingSupport();
    ~LoggingSupport();

    static size_t &level() {
        static size_t lvl;
        return lvl;
    }

    static void set_log_level(size_t t_lvl) {
        level() = t_lvl;
    }
};

// log_trace --------------------------------------------------------------
#ifndef log_trace
#ifdef ANDROID
#define log_trace(...)                                                                                                                \
    do {                                                                                                                              \
        if (LoggingSupport::level() >= LogLevel::LVO_LOG_TRACE) ((void)__android_log_print(ANDROID_LOG_DEBUG, "[LVO]", __VA_ARGS__)); \
    } while (0);
#else
#define log_trace(format, ...)                                                                                                         \
    do {                                                                                                                               \
        if (LoggingSupport::level() >= LogLevel::LVO_LOG_TRACE) printf("\033[0;35m[LVO][trace]\033[0;0m " format "\n", ##__VA_ARGS__); \
    } while (0);
#endif
#endif

// log_debug --------------------------------------------------------------
#ifndef log_debug
#ifdef ANDROID
#define log_debug(...)                                                                                                                \
    do {                                                                                                                              \
        if (LoggingSupport::level() >= LogLevel::LVO_LOG_DEBUG) ((void)__android_log_print(ANDROID_LOG_DEBUG, "[LVO]", __VA_ARGS__)); \
    } while (0);
#else
#define log_debug(format, ...)                                                                                                         \
    do {                                                                                                                               \
        if (LoggingSupport::level() >= LogLevel::LVO_LOG_DEBUG) printf("\033[0;35m[LVO][debug]\033[0;0m " format "\n", ##__VA_ARGS__); \
    } while (0);
#endif
#endif

// log_info --------------------------------------------------------------
#ifndef log_info
#ifdef ANDROID
#define log_info(...)                                                                                                               \
    do {                                                                                                                            \
        if (LoggingSupport::level() >= LogLevel::LVO_LOG_INFO) ((void)__android_log_print(ANDROID_LOG_INFO, "[LVO]", __VA_ARGS__)); \
    } while (0);
#else
#define log_info(format, ...)                                                                                                        \
    do {                                                                                                                             \
        if (LoggingSupport::level() >= LogLevel::LVO_LOG_INFO) printf("\033[0;32m[LVO][info]\033[0;0m " format "\n", ##__VA_ARGS__); \
    } while (0);
#endif
#endif

// log_warn --------------------------------------------------------------
#ifndef log_warn
#ifdef ANDROID
#define log_warn(...)                                                                                                               \
    do {                                                                                                                            \
        if (LoggingSupport::level() >= LogLevel::LVO_LOG_WARN) ((void)__android_log_print(ANDROID_LOG_WARN, "[LVO]", __VA_ARGS__)); \
    } while (0);
#else
#define log_warn(format, ...)                                                                                                        \
    do {                                                                                                                             \
        if (LoggingSupport::level() >= LogLevel::LVO_LOG_WARN) printf("\033[0;33m[LVO][warn]\033[0;0m " format "\n", ##__VA_ARGS__); \
    } while (0);
#endif
#endif

// log_error --------------------------------------------------------------
#ifndef log_error
#ifdef ANDROID
#define log_error(...)                                                                                                               \
    do {                                                                                                                             \
        if (LoggingSupport::level() > LogLevel::LVO_LOG_ERROR) ((void)__android_log_print(ANDROID_LOG_ERROR, "[LVO]", __VA_ARGS__)); \
    } while (0);
#else
#define log_error(format, ...)                                                                                                        \
    do {                                                                                                                              \
        if (LoggingSupport::level() > LogLevel::LVO_LOG_ERROR) printf("\033[0;31m[LVO][error]\033[0;0m " format "\n", ##__VA_ARGS__); \
    } while (0);
#endif
#endif

#ifdef LVO_DEBUG
#define runtime_assert(condition, message)                                                                                         \
    do {                                                                                                                           \
        if (!(condition)) {                                                                                                        \
            log_error("Assertion failed at " __FILE__ ":%d : %s\nWhen testing condition:\n    %s", __LINE__, message, #condition); \
            abort();                                                                                                               \
        }                                                                                                                          \
    } while (0)
#else
#define runtime_assert(...)
#endif

#endif // VO_LOG_H
