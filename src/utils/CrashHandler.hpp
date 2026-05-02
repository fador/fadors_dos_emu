#pragma once
#include <functional>
#include <string>

namespace fador::utils {

/// Install platform crash handlers (SEH on Windows, signals on POSIX).
/// @param onCrash  Called with a description string when a hard crash occurs.
/// @return true if handlers were installed.
bool installCrashHandler(std::function<void(const std::string &)> onCrash);

/// Set a synchronous callback that dumps emulator state (registers, VRAM, etc.)
/// before the crash handler terminates. Called from within the crash context.
void setCrashDumpCallback(std::function<void()> dumpFn);

} // namespace fador::utils
