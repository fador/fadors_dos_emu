#include "CrashHandler.hpp"
#include "Logger.hpp"
#include <cstdio>
#include <cstdlib>
#include <csignal>

namespace fador::utils {

static std::function<void(const std::string &)> g_crashCallback;
static std::function<void()> g_dumpCallback;

// ── Signal name helper ────────────────────────────────────────────────────
static const char *signalName(int sig) {
  switch (sig) {
  case SIGSEGV: return "SIGSEGV (Access Violation)";
  case SIGFPE:  return "SIGFPE (Arithmetic Exception)";
  case SIGILL:  return "SIGILL (Illegal Instruction)";
  case SIGABRT: return "SIGABRT (Abort)";
  case SIGTERM: return "SIGTERM (Termination request)";
  case SIGINT:  return "SIGINT (Interrupt)";
  default:      return "Unknown signal";
  }
}

// ── Common crash handler body ─────────────────────────────────────────────
[[noreturn]] static void handleCrash(const char *description) {
  // Run the state dump callback FIRST (before any logging that might crash)
  if (g_dumpCallback) {
    // Use a try-catch since the dump itself might crash
    try { g_dumpCallback(); } catch (...) {}
  }

  // Log the crash reason
  if (g_crashCallback) {
    g_crashCallback(description);
  }

  // Write emergency crash info to stderr directly (bypasses logger which may
  // not be safe in a signal/SEH context)
  std::fprintf(stderr, "\n========================================\n");
  std::fprintf(stderr, "FATAL CRASH: %s\n", description);
  std::fprintf(stderr, "The emulator encountered a fatal error.\n");
  std::fprintf(stderr, "========================================\n\n");
  std::fflush(stderr);

  // Exit with failure code
  std::_Exit(1);
}

// ═══════════════════════════════════════════════════════════════════════════
// Windows: Structured Exception Handling (SEH) + signals
// ═══════════════════════════════════════════════════════════════════════════
#ifdef _WIN32

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>

static const char *sehCodeName(DWORD code) {
  switch (code) {
  case EXCEPTION_ACCESS_VIOLATION:         return "Access Violation";
  case EXCEPTION_ARRAY_BOUNDS_EXCEEDED:    return "Array Bounds Exceeded";
  case EXCEPTION_BREAKPOINT:               return "Breakpoint";
  case EXCEPTION_DATATYPE_MISALIGNMENT:    return "Datatype Misalignment";
  case EXCEPTION_FLT_DENORMAL_OPERAND:     return "FP Denormal Operand";
  case EXCEPTION_FLT_DIVIDE_BY_ZERO:       return "FP Divide by Zero";
  case EXCEPTION_FLT_INEXACT_RESULT:       return "FP Inexact Result";
  case EXCEPTION_FLT_INVALID_OPERATION:    return "FP Invalid Operation";
  case EXCEPTION_FLT_OVERFLOW:             return "FP Overflow";
  case EXCEPTION_FLT_STACK_CHECK:          return "FP Stack Check";
  case EXCEPTION_FLT_UNDERFLOW:            return "FP Underflow";
  case EXCEPTION_ILLEGAL_INSTRUCTION:      return "Illegal Instruction";
  case EXCEPTION_IN_PAGE_ERROR:            return "In-Page Error";
  case EXCEPTION_INT_DIVIDE_BY_ZERO:       return "Integer Divide by Zero";
  case EXCEPTION_INT_OVERFLOW:             return "Integer Overflow";
  case EXCEPTION_INVALID_DISPOSITION:      return "Invalid Disposition";
  case EXCEPTION_NONCONTINUABLE_EXCEPTION: return "Noncontinuable Exception";
  case EXCEPTION_PRIV_INSTRUCTION:         return "Privileged Instruction";
  case EXCEPTION_SINGLE_STEP:              return "Single Step";
  case EXCEPTION_STACK_OVERFLOW:           return "Stack Overflow";
  default:                                 return "Unknown SEH Exception";
  }
}

static LONG WINAPI sehHandler(EXCEPTION_POINTERS *exInfo) {
  DWORD code = exInfo->ExceptionRecord->ExceptionCode;
  void *addr = exInfo->ExceptionRecord->ExceptionAddress;

  char buf[384];
  std::snprintf(buf, sizeof(buf),
                "%s (code 0x%08lX) at address %p",
                sehCodeName(code), (unsigned long)code, addr);

  // If it's an access violation, report the attempted address
  if (code == EXCEPTION_ACCESS_VIOLATION && exInfo->ExceptionRecord->NumberParameters >= 2) {
    std::snprintf(buf + std::strlen(buf), sizeof(buf) - std::strlen(buf),
                  "  [%s address 0x%p]",
                  exInfo->ExceptionRecord->ExceptionInformation[0] == 0 ? "read from" :
                  exInfo->ExceptionRecord->ExceptionInformation[0] == 1 ? "write to" : "execute at",
                  (void *)(uintptr_t)exInfo->ExceptionRecord->ExceptionInformation[1]);
  }

  handleCrash(buf);
  return EXCEPTION_EXECUTE_HANDLER;
}

static void signalHandlerWin(int sig) {
  char buf[192];
  std::snprintf(buf, sizeof(buf), "Signal: %s (%d)", signalName(sig), sig);
  handleCrash(buf);
}

// ── std::terminate handler ──────────────────────────────────────────────
static void terminateHandler() {
  handleCrash("std::terminate() called (unhandled C++ exception or noexcept violation)");
}

bool installCrashHandler(std::function<void(const std::string &)> onCrash) {
  g_crashCallback = std::move(onCrash);

  // SEH top-level filter
  SetUnhandledExceptionFilter(sehHandler);

  // C signal handlers
  std::signal(SIGSEGV, signalHandlerWin);
  std::signal(SIGFPE, signalHandlerWin);
  std::signal(SIGILL, signalHandlerWin);
  std::signal(SIGABRT, signalHandlerWin);
  std::signal(SIGTERM, signalHandlerWin);

  // C++ terminate handler
  std::set_terminate(terminateHandler);

  // Prevent Windows Error Reporting dialog from blocking
  SetErrorMode(SEM_FAILCRITICALERRORS | SEM_NOGPFAULTERRORBOX);

  return true;
}

// ═══════════════════════════════════════════════════════════════════════════
// POSIX (Linux / macOS): signal handlers
// ═══════════════════════════════════════════════════════════════════════════
#else

#include <cstring>
#include <unistd.h>

static void signalHandlerPosix(int sig, siginfo_t *info, void * /*ctx*/) {
  char buf[256];
  if (info && info->si_addr) {
    std::snprintf(buf, sizeof(buf),
                  "Signal: %s (%d) at address %p",
                  signalName(sig), sig, info->si_addr);
  } else {
    std::snprintf(buf, sizeof(buf),
                  "Signal: %s (%d)", signalName(sig), sig);
  }
  handleCrash(buf);
}

static void terminateHandlerPosix() {
  handleCrash("std::terminate() called (unhandled C++ exception or noexcept violation)");
}

bool installCrashHandler(std::function<void(const std::string &)> onCrash) {
  g_crashCallback = std::move(onCrash);

  // Install sigaction-based handlers for more detail
  struct sigaction sa {};
  sa.sa_sigaction = signalHandlerPosix;
  sa.sa_flags = SA_SIGINFO;
  sigemptyset(&sa.sa_mask);

  sigaction(SIGSEGV, &sa, nullptr);
  sigaction(SIGFPE, &sa, nullptr);
  sigaction(SIGILL, &sa, nullptr);
  sigaction(SIGABRT, &sa, nullptr);
  sigaction(SIGTERM, &sa, nullptr);

  // C++ terminate handler
  std::set_terminate(terminateHandlerPosix);

  return true;
}

#endif // _WIN32

void setCrashDumpCallback(std::function<void()> dumpFn) {
  g_dumpCallback = std::move(dumpFn);
}

} // namespace fador::utils
