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

template<typename... Args>
void log(LogLevel level, LogCategory cat, const std::source_location& loc, Args&&... args) {
    if (level < currentLevel) return;
    if (!(enabledCategories & cat) && level < LogLevel::Warn) return; // Always log warnings/errors

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

    std::cerr << "[" << levelStr << "] " << catStr << loc.file_name() << ":" << loc.line() << " | ";
    (std::cerr << ... << std::forward<Args>(args)) << "\n";
}

} // namespace fador::utils

#define LOG_TRACE(...) fador::utils::log(fador::utils::LogLevel::Trace, fador::utils::CAT_GENERIC, std::source_location::current(), __VA_ARGS__)
#define LOG_DEBUG(...) fador::utils::log(fador::utils::LogLevel::Debug, fador::utils::CAT_GENERIC, std::source_location::current(), __VA_ARGS__)
#define LOG_INFO(...)  fador::utils::log(fador::utils::LogLevel::Info,  fador::utils::CAT_GENERIC, std::source_location::current(), __VA_ARGS__)
#define LOG_WARN(...)  fador::utils::log(fador::utils::LogLevel::Warn,  fador::utils::CAT_GENERIC, std::source_location::current(), __VA_ARGS__)
#define LOG_ERROR(...) fador::utils::log(fador::utils::LogLevel::Error, fador::utils::CAT_GENERIC, std::source_location::current(), __VA_ARGS__)

#define LOG_CPU(...)   fador::utils::log(fador::utils::LogLevel::Debug, fador::utils::CAT_CPU,   std::source_location::current(), __VA_ARGS__)
#define LOG_VIDEO(...) fador::utils::log(fador::utils::LogLevel::Debug, fador::utils::CAT_VIDEO, std::source_location::current(), __VA_ARGS__)
#define LOG_DOS(...)   fador::utils::log(fador::utils::LogLevel::Debug, fador::utils::CAT_DOS,   std::source_location::current(), __VA_ARGS__)
