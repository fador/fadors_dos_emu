#pragma once

#include <iostream>
#include <source_location>

namespace fador::utils {

enum class LogLevel {
    Debug,
    Info,
    Warn,
    Error
};

inline LogLevel currentLevel = LogLevel::Info;

template<typename... Args>
void log(LogLevel level, const std::source_location& loc, Args&&... args) {
    if (level < currentLevel) return;

    const char* levelStr = "INFO";
    switch (level) {
        case LogLevel::Debug: levelStr = "DEBUG"; break;
        case LogLevel::Info:  levelStr = "INFO"; break;
        case LogLevel::Warn:  levelStr = "WARN"; break;
        case LogLevel::Error: levelStr = "ERROR"; break;
    }

    std::cerr << "[" << levelStr << "] " << loc.file_name() << ":" << loc.line() << " | ";
    (std::cerr << ... << std::forward<Args>(args)) << "\n";
}

} // namespace fador::utils

#define LOG_DEBUG(...) fador::utils::log(fador::utils::LogLevel::Debug, std::source_location::current(), __VA_ARGS__)
#define LOG_INFO(...)  fador::utils::log(fador::utils::LogLevel::Info, std::source_location::current(), __VA_ARGS__)
#define LOG_WARN(...)  fador::utils::log(fador::utils::LogLevel::Warn, std::source_location::current(), __VA_ARGS__)
#define LOG_ERROR(...) fador::utils::log(fador::utils::LogLevel::Error, std::source_location::current(), __VA_ARGS__)
