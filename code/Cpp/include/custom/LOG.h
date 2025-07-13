#ifndef LOG_H
#define LOG_H

#ifdef __cplusplus
extern "C" {
#endif

// Define log levels
typedef enum {
    LOG_TRACE,
    LOG_DEBUG,
    LOG_INFO,
    LOG_WARN,
    LOG_ERROR,
    LOG_CRITICAL,
    LOG_FATAL
} LogLevel;

// Set the current log level threshold (only messages at this level or higher will be printed)
void setLogLevel(LogLevel level);

// Print a log message at the given level
void printLog(LogLevel level, const char *message);

#ifdef __cplusplus
}
#endif

#endif // LOG_H
