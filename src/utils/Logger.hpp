#pragma once

#include <iostream>
#include <source_location>
#include <string>
#include <cstdint>

namespace fador::utils {

enum class LogLevel {
    Trace,
    Debug,
    Info,
    Warn,
    Error
};

enum LogCategory {
    CAT_GENERIC = 0x01,
    CAT_CPU     = 0x02,
    CAT_VIDEO   = 0x04,
    CAT_DOS     = 0x08
};

inline LogLevel currentLevel = LogLevel::Info;
inline uint32_t enabledCategories = CAT_GENERIC; // Default to generic only

// Cheap filter evaluated by the LOG_* macros *before* any argument is
// evaluated, so disabled log statements in hot paths cost one branch.
inline bool shouldLog(LogLevel level, LogCategory cat) {
    if (level < currentLevel) return false;
    // Warnings and errors are always logged regardless of category
    return (enabledCategories & cat) || level >= LogLevel::Warn;
}

template<typename... Args>
void log(LogLevel level, LogCategory cat, const std::source_location& loc, Args&&... args) {
    if (!shouldLog(level, cat)) return;

    const char* levelStr = "INFO";
    switch (level) {
        case LogLevel::Trace: levelStr = "TRACE"; break;
        case LogLevel::Debug: levelStr = "DEBUG"; break;
        case LogLevel::Info:  levelStr = "INFO";  break;
        case LogLevel::Warn:  levelStr = "WARN";  break;
        case LogLevel::Error: levelStr = "ERROR"; break;
    }

    const char* catStr = "";
    switch (cat) {
        case CAT_CPU:   catStr = "[CPU] "; break;
        case CAT_VIDEO: catStr = "[VID] "; break;
        case CAT_DOS:   catStr = "[DOS] "; break;
        default: break;
    }

    // Messages commonly switch the stream to std::hex; always print the
    // prefix in decimal and restore the default format afterwards so the
    // next message (and its line number) is not printed in hex.
    std::cerr.flags(std::ios_base::dec | std::ios_base::skipws);
    std::cerr << "[" << levelStr << "] " << catStr << loc.file_name() << ":" << loc.line() << " | ";
    (std::cerr << ... << std::forward<Args>(args)) << "\n";
    std::cerr.flags(std::ios_base::dec | std::ios_base::skipws);
}

} // namespace fador::utils

#define LOG_TRACE(...) do { if (fador::utils::shouldLog(fador::utils::LogLevel::Trace, fador::utils::CAT_GENERIC)) fador::utils::log(fador::utils::LogLevel::Trace, fador::utils::CAT_GENERIC, std::source_location::current(), __VA_ARGS__); } while (0)
#define LOG_DEBUG(...) do { if (fador::utils::shouldLog(fador::utils::LogLevel::Debug, fador::utils::CAT_GENERIC)) fador::utils::log(fador::utils::LogLevel::Debug, fador::utils::CAT_GENERIC, std::source_location::current(), __VA_ARGS__); } while (0)
#define LOG_INFO(...) do { if (fador::utils::shouldLog(fador::utils::LogLevel::Info, fador::utils::CAT_GENERIC)) fador::utils::log(fador::utils::LogLevel::Info, fador::utils::CAT_GENERIC, std::source_location::current(), __VA_ARGS__); } while (0)
#define LOG_WARN(...) do { if (fador::utils::shouldLog(fador::utils::LogLevel::Warn, fador::utils::CAT_GENERIC)) fador::utils::log(fador::utils::LogLevel::Warn, fador::utils::CAT_GENERIC, std::source_location::current(), __VA_ARGS__); } while (0)
#define LOG_ERROR(...) do { if (fador::utils::shouldLog(fador::utils::LogLevel::Error, fador::utils::CAT_GENERIC)) fador::utils::log(fador::utils::LogLevel::Error, fador::utils::CAT_GENERIC, std::source_location::current(), __VA_ARGS__); } while (0)

#define LOG_CPU(...) do { if (fador::utils::shouldLog(fador::utils::LogLevel::Debug, fador::utils::CAT_CPU)) fador::utils::log(fador::utils::LogLevel::Debug, fador::utils::CAT_CPU, std::source_location::current(), __VA_ARGS__); } while (0)
#define LOG_VIDEO(...) do { if (fador::utils::shouldLog(fador::utils::LogLevel::Debug, fador::utils::CAT_VIDEO)) fador::utils::log(fador::utils::LogLevel::Debug, fador::utils::CAT_VIDEO, std::source_location::current(), __VA_ARGS__); } while (0)
#define LOG_DOS(...) do { if (fador::utils::shouldLog(fador::utils::LogLevel::Debug, fador::utils::CAT_DOS)) fador::utils::log(fador::utils::LogLevel::Debug, fador::utils::CAT_DOS, std::source_location::current(), __VA_ARGS__); } while (0)
