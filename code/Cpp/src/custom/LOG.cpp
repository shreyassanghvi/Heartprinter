#include "../../include/custom/LOG.h"
#include <cstdio>

// Internal variable to store the current log level threshold
static LogLevel currentLogLevel = LOG_INFO;

// Convert log level enum to string
static const char *logLevelToString(LogLevel level) {
    switch (level) {
        case LOG_TRACE: return "TRACE";
        case LOG_DEBUG: return "DEBUG";
        case LOG_INFO: return "INFO";
        case LOG_WARN: return "WARN";
        case LOG_ERROR: return "ERROR";
        case LOG_CRITICAL: return "CRITICAL";
        case LOG_FATAL: return "FATAL";
        default: return "UNKNOWN";
    }
}

void setLogLevel(LogLevel level) {
    currentLogLevel = level;
}

void printLog(LogLevel level, const char *message) {
    if (level >= currentLogLevel) {
        printf("[%s] %s\n", logLevelToString(level), message);
    }
}
