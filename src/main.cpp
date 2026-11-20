#include "cpu/Assembler.hpp"
#include "cpu/InstructionDecoder.hpp"
#include "hw/BIOS.hpp"
#include "hw/CMOSRTC.hpp"
#include "hw/DMA8237.hpp"
#include "hw/DOS.hpp"
#include "hw/DPMI.hpp"
#include "hw/IOBus.hpp"
#include "hw/Joystick.hpp"
#include "hw/KeyboardController.hpp"
#include "hw/PIC8259.hpp"
#include "hw/PIT8254.hpp"
#include "hw/ParallelPort.hpp"
#include "hw/SerialBackends.hpp"
#include "hw/Uart16550.hpp"
#include "hw/ProgramLoader.hpp"
#include "hw/DriveManager.hpp"
#include "hw/DosShell.hpp"
#include "hw/VGAController.hpp"
#include "hw/VirtualClock.hpp"
#include "hw/MachineTime.hpp"
#include "hw/CivilTime.hpp"
#include "hw/audio/AdLib.hpp"
#include "hw/audio/AudioBackend.hpp"
#include "hw/audio/MidiDevice.hpp"
#include "hw/audio/MidiSynth.hpp"
#include "hw/audio/SoundBlaster.hpp"
#include "hw/audio/GravisUltrasound.hpp"
#include "hw/audio/WavWriter.hpp"
#include "hw/audio/PcSpeaker.hpp"
#include "ui/Debugger.hpp"
#include "ui/FrameRenderer.hpp"
#include "ui/HostSimdBench.hpp"
#include "ui/InputManager.hpp"
#include "ui/TerminalRenderer.hpp"
#include "utils/CrashHandler.hpp"
#include "utils/Logger.hpp"
#include "utils/simd/Simd.hpp"
#include <array>
#include <iostream>
#ifdef _WIN32
#include <crtdbg.h>
#include <cstdio>
#include <cstdlib>
#ifndef NOMINMAX
#define NOMINMAX // keep std::min / std::max usable
#endif
#include <windows.h>
#include <io.h>
#include <fcntl.h>
#endif
#ifdef HAVE_SDL2
#include "ui/SDLRenderer.hpp"
#endif

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <functional>
#include <memory>
#include <optional>
#include <sstream>
#include <thread>

namespace {

std::string toUpperAscii(const std::string &in) {
  std::string out = in;
  for (auto &ch : out) {
    ch = static_cast<char>(std::toupper(static_cast<unsigned char>(ch)));
  }
  return out;
}

std::string pathToUtf8String(const std::filesystem::path &p) {
  auto u8 = p.u8string();
  return std::string(reinterpret_cast<const char *>(u8.data()), u8.size());
}

bool isZipArchiveFile(const std::string &pathStr) {
  if (pathStr.empty()) {
    return false;
  }
  std::error_code ec;
  const std::filesystem::path p(pathStr);
  if (!std::filesystem::is_regular_file(p, ec) || ec) {
    return false;
  }
  return toUpperAscii(pathToUtf8String(p.extension())) == ".ZIP";
}

bool isDosLaunchExtension(const std::string &extUpper) {
  return extUpper == ".EXE" || extUpper == ".COM" || extUpper == ".BAT";
}

std::string normalizeAlnumUpper(const std::string &in) {
  std::string out;
  for (unsigned char ch : in) {
    if (std::isalnum(ch)) {
      out.push_back(static_cast<char>(std::toupper(ch)));
    }
  }
  return out;
}

int scoreZipExecutable(const std::string &stemUpper,
                       const std::string &extUpper,
                       const std::string &zipStemUpper) {
  constexpr int SCORE_EXE = 20;
  constexpr int SCORE_COM = 15;
  constexpr int SCORE_BAT = 5;
  constexpr int SCORE_EXACT_STEM = 100;
  constexpr int SCORE_PREFIX_STEM = 50;
  constexpr int PENALTY_DOS_EXTENDER = -100;
  constexpr int PENALTY_UTILITY = -80;

  int score = 0;
  if (extUpper == ".EXE") {
    score = SCORE_EXE;
  } else if (extUpper == ".COM") {
    score = SCORE_COM;
  } else if (extUpper == ".BAT") {
    score = SCORE_BAT;
  }

  const std::string stemNorm = normalizeAlnumUpper(stemUpper);
  const std::string zipNorm = normalizeAlnumUpper(zipStemUpper);
  if (!stemUpper.empty() && stemUpper == zipStemUpper) {
    score += SCORE_EXACT_STEM;
  } else if (stemNorm.size() >= 3 && zipNorm.size() >= 3 &&
             (zipNorm.find(stemNorm) == 0 || stemNorm.find(zipNorm) == 0)) {
    score += SCORE_PREFIX_STEM;
  }

  if (stemUpper == "DOS4GW" || stemUpper == "CWSDPMI" || stemUpper == "DOS32A") {
    score += PENALTY_DOS_EXTENDER;
  } else {
    static constexpr const char *UTILITY_KEYWORDS[] = {
        "SETUP", "INSTALL", "CONFIG", "SETSOUND", "SNDSETUP",
        "UNINST", "EDIT", "README", "PKUNZIP", "UNIVBE"};
    for (const char *kw : UTILITY_KEYWORDS) {
      if (stemUpper.find(kw) != std::string::npos) {
        score += PENALTY_UTILITY;
        break;
      }
    }
  }
  return score;
}

std::string selectZipLaunchTarget(fador::hw::DriveManager &driveManager,
                                  const std::string &mountRootStr,
                                  const std::string &zipStem,
                                  std::string &args) {
  namespace fs = std::filesystem;
  std::error_code ec;

  const size_t tokStart = args.find_first_not_of(" \t");
  if (tokStart != std::string::npos) {
    size_t tokEnd = args.find_first_of(" \t", tokStart);
    if (tokEnd == std::string::npos) {
      tokEnd = args.size();
    }
    const std::string token = args.substr(tokStart, tokEnd - tokStart);
    const std::string extUpper = toUpperAscii(pathToUtf8String(fs::path(token).extension()));
    if (isDosLaunchExtension(extUpper)) {
      const std::string resolved = driveManager.matchHostCase(mountRootStr, token);
      if (fs::is_regular_file(resolved, ec) && !ec) {
        const size_t restStart = args.find_first_not_of(" \t", tokEnd);
        args = (restStart == std::string::npos) ? std::string() : args.substr(restStart);
        return resolved;
      }
    }
  }

  struct Candidate {
    int score;
    std::string nameUpper;
    std::string hostPath;
  };
  std::vector<Candidate> candidates;
  const std::string zipStemUpper = toUpperAscii(zipStem);
  for (fs::directory_iterator it(mountRootStr, ec), end; !ec && it != end; it.increment(ec)) {
    if (!it->is_regular_file(ec) || ec) {
      continue;
    }
    const fs::path p = it->path();
    const std::string extUpper = toUpperAscii(pathToUtf8String(p.extension()));
    if (!isDosLaunchExtension(extUpper)) {
      continue;
    }
    const std::string stemUpper = toUpperAscii(pathToUtf8String(p.stem()));
    const std::string nameUpper = toUpperAscii(pathToUtf8String(p.filename()));
    candidates.push_back({scoreZipExecutable(stemUpper, extUpper, zipStemUpper),
                          nameUpper, pathToUtf8String(p)});
  }

  if (candidates.empty()) {
    return {};
  }
  std::sort(candidates.begin(), candidates.end(),
            [](const Candidate &a, const Candidate &b) {
              if (a.score != b.score) {
                return a.score > b.score;
              }
              if (a.nameUpper.size() != b.nameUpper.size()) {
                return a.nameUpper.size() < b.nameUpper.size();
              }
              return a.nameUpper < b.nameUpper;
            });
  return candidates.front().hostPath;
}

enum class BenchmarkKind {
  None,
  DecoderLoop,
  RepMovsb,
  RepMovsd,
  RepStosd,
};

BenchmarkKind parseBenchmarkKind(const std::string &name) {
  if (name == "decoder-loop")
    return BenchmarkKind::DecoderLoop;
  if (name == "rep-movsb")
    return BenchmarkKind::RepMovsb;
  if (name == "rep-movsd")
    return BenchmarkKind::RepMovsd;
  if (name == "rep-stosd")
    return BenchmarkKind::RepStosd;
  return BenchmarkKind::None;
}

const char *benchmarkName(BenchmarkKind kind) {
  switch (kind) {
  case BenchmarkKind::DecoderLoop:
    return "decoder-loop";
  case BenchmarkKind::RepMovsb:
    return "rep-movsb";
  case BenchmarkKind::RepMovsd:
    return "rep-movsd";
  case BenchmarkKind::RepStosd:
    return "rep-stosd";
  case BenchmarkKind::None:
    break;
  }
  return "none";
}

// Scripted keyboard input for unattended runs (--keys=...).
// Spec: comma separated "<seconds>:<KEY>" entries, e.g. "2:ENTER,5.5:ESC".
// Each key is pressed at the given time after start (wall-clock seconds,
// or virtual seconds with --deterministic) and released 100 ms later,
// through the 8042 (port 60h + IRQ1) and the BIOS type-ahead buffer.
class ScriptedKeys {
public:
  bool parse(const std::string &spec, std::string &error) {
    std::stringstream ss(spec);
    std::string item;
    while (std::getline(ss, item, ',')) {
      if (item.empty())
        continue;
      auto colon = item.find(':');
      if (colon == std::string::npos) {
        error = "missing ':' in '" + item + "'";
        return false;
      }
      Event ev{};
      try {
        ev.time = std::stod(item.substr(0, colon));
      } catch (...) {
        error = "bad time in '" + item + "'";
        return false;
      }
      if (!lookup(item.substr(colon + 1), ev)) {
        error = "unknown key '" + item.substr(colon + 1) + "'";
        return false;
      }
      m_events.push_back(ev);
    }
    return true;
  }

  void service(double t, fador::hw::KeyboardController &kbd) {
    for (auto &ev : m_events) {
      if (!ev.pressed && t >= ev.time) {
        ev.pressed = true;
        if (ev.extended)
          kbd.pushMakeKeyExtended(ev.ascii, ev.scancode);
        else
          kbd.pushMakeKey(ev.ascii, ev.scancode);
      }
      if (ev.pressed && !ev.released && t >= ev.time + 0.1) {
        ev.released = true;
        if (ev.extended)
          kbd.pushBreakKeyExtended(ev.scancode);
        else
          kbd.pushBreakKey(ev.scancode);
      }
    }
  }

private:
  struct Event {
    double time;
    uint8_t scancode;
    uint8_t ascii;
    bool extended;
    bool pressed;
    bool released;
  };

  static bool lookup(std::string name, Event &ev) {
    for (auto &c : name)
      c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    struct Named {
      const char *name;
      uint8_t scancode;
      uint8_t ascii;
      bool extended;
    };
    static const Named kNamed[] = {
        {"ESC", 0x01, 27, false},   {"ENTER", 0x1C, 13, false},
        {"SPACE", 0x39, ' ', false}, {"TAB", 0x0F, 9, false},
        {"BKSP", 0x0E, 8, false},   {"UP", 0x48, 0, true},
        {"DOWN", 0x50, 0, true},    {"LEFT", 0x4B, 0, true},
        {"RIGHT", 0x4D, 0, true},   {"F1", 0x3B, 0, false},
        {"F2", 0x3C, 0, false},     {"F3", 0x3D, 0, false},
        {"F4", 0x3E, 0, false},     {"F5", 0x3F, 0, false},
        {"F6", 0x40, 0, false},     {"F7", 0x41, 0, false},
        {"F8", 0x42, 0, false},     {"F9", 0x43, 0, false},
        {"F10", 0x44, 0, false},    {"LCTRL", 0x1D, 0, false},
        {"LALT", 0x38, 0, false},   {"LSHIFT", 0x2A, 0, false},
        {"MINUS", 0x0C, '-', false}, {"EQUALS", 0x0D, '=', false},
    };
    for (const auto &n : kNamed) {
      if (name == n.name) {
        ev.scancode = n.scancode;
        ev.ascii = n.ascii;
        ev.extended = n.extended;
        return true;
      }
    }
    if (name.size() == 1) {
      static const char *kRows[] = {"1234567890", "QWERTYUIOP", "ASDFGHJKL",
                                    "ZXCVBNM"};
      static const uint8_t kRowStart[] = {0x02, 0x10, 0x1E, 0x2C};
      for (int r = 0; r < 4; ++r) {
        const char *pos = std::strchr(kRows[r], name[0]);
        if (pos) {
          ev.scancode = static_cast<uint8_t>(kRowStart[r] + (pos - kRows[r]));
          ev.ascii = static_cast<uint8_t>(
              std::isalpha(static_cast<unsigned char>(name[0]))
                  ? std::tolower(static_cast<unsigned char>(name[0]))
                  : name[0]);
          ev.extended = false;
          return true;
        }
      }
    }
    return false;
  }

  std::vector<Event> m_events;
};

std::optional<uint32_t> parseLinearHexAddress(const std::string &value) {
  try {
    size_t parsed = 0;
    uint64_t result = std::stoull(value, &parsed, 16);
    if (parsed != value.size() || result > 0xFFFFFFFFull) {
      return std::nullopt;
    }
    return static_cast<uint32_t>(result);
  } catch (...) {
    return std::nullopt;
  }
}

void printHelp(const char *programName) {
  // clang-format off
  std::printf(
    "Usage: %s [OPTIONS] [program.com|exe|directory|archive.zip] [program-args...]\n"
    "\n"
    "Fador's DOS Emulator — a multiplatform MS-DOS 6.22 emulator written in modern C++20.\n"
    "\n"
    "Launch modes:\n"
    "  No arguments           Start the DOS 6.22 command prompt (Z:\\>).\n"
    "  <directory>            Mount directory as C: and start the command prompt at C:\\>.\n"
    "  <archive.zip>          Extract ZIP archive to temporary C: and launch its program.\n"
    "  <program.com|exe>      Load and run the program directly.\n"
    "\n"
    "Options:\n"
    "  --help, -h, -?            Print this help message and exit.\n"
    "  --mount=L:/path           Mount host directory or .zip archive at drive letter L (repeatable).\n"
    "  --cdrom=L:/image.iso      Mount CD-ROM image (.iso/.cue/.bin/.img) at drive letter L (repeatable).\n"
    "  --himem                   Enable HIMEM/XMS extended memory support.\n"
    "  --cpu=MODEL               Emulated CPU: %s\n"
    "                            (default 486dx2).  Sets CPUID, EFLAGS/CR0 bits, the\n"
    "                            available instructions and the guest cycle costs.\n"
    "  --no-fpu                  No x87 coprocessor (a 386 or 486SX without a 387): x87\n"
    "                            instructions do nothing, CR0.ET / CPUID / INT 11h say so.\n"
    "  --dpmi=off                No DPMI host (INT 2Fh AX=1687h fails): extenders use\n"
    "                            VCPI or raw mode.\n"
    "  --vcpi=off                No VCPI server (INT 67h AX=DE00h fails).\n"
    "  --env=NAME=VALUE          Add or replace a DOS environment variable (repeatable).\n"
    "                            Defaults: BLASTER=A220 I5 D1 H5 P330 T6, CAUSEWAY=DPMI.\n"
    "  --headless                No window, terminal output or host input; for scripted\n"
    "                            runs.  Exit status 124 when --max-seconds expires.\n"
    "  --max-seconds=S           Stop after S seconds (wall clock; virtual time with\n"
    "                            --deterministic).\n"
    "  --screenshot=FILE.png     Save the screen (text or graphics) as PNG at exit, or at\n"
    "  --screenshot-at=S         S seconds after start (virtual with --deterministic).\n"
    "  --audio-wav=FILE.wav      Record the sound output (AdLib, Sound Blaster, GUS, MIDI).\n"
    "  --gus[=PORT][,irq=N][,dma=N][,mem=KB]\n"
    "                            Gravis Ultrasound (GF1) at PORT (hex, default 240),\n"
    "                            IRQ 11, DMA 3, 1024 KB DRAM (256/512/768); also sets\n"
    "                            ULTRASND=240,3,3,11,11 and ULTRADIR=C:\\ULTRASND.\n"
    "  --keys=T:KEY,...          Type KEY at T seconds (letters, digits, ESC, ENTER, SPACE,\n"
    "                            TAB, BKSP, arrows, F1-F10, LCTRL, LALT, LSHIFT, ...);\n"
    "                            virtual seconds with --deterministic.\n"
    "  --sdl                     Force the SDL graphical frontend (requires SDL2 build).\n"
    "  --no-sdl                  Force headless/text-mode execution even in SDL builds.\n"
    "  --throttle-ips=N          Run at a fixed N instructions per second of emulated time.\n"
    "  --throttle-machine=name   Run at a fixed machine speed: the preset selects the CPU\n"
    "                            model (unless --cpu is given) and its clock:\n"
    "                            %s.\n"
    "                            Default: as fast as the host allows, timers in real time.\n"
    "  --deterministic           Emulated time = guest clock cycles / CPU clock (the\n"
    "                            --cpu model's clock unless --throttle-* is given), never\n"
    "                            the host clock: runs are reproducible and not paced.\n"
    "  --start-time=YYYY-MM-DDTHH:MM:SS\n"
    "                            Date and time the emulated RTC/DOS clock starts at\n"
    "                            (default: host local time; 1994-06-01T12:00:00 with\n"
    "                            --deterministic).\n"
    "  --stop-after=N            Stop execution after N instruction cycles and dump state.\n"
    "  --stop-after-debugger     When --stop-after triggers, enter the interactive debugger.\n"
    "  --dump-on-exit            Dump CPU state when the program terminates.\n"
    "  --no-fast-path            Run every instruction through the legacy decoder (no\n"
    "                            predecoded instruction cache); for debugging the CPU.\n"
    "  --instruction-trace=N     Print a trace line per executed instruction for the next N.\n"
    "  --instruction-trace-from=ADDR[#N]\n"
    "                            Delay --instruction-trace until CS:EIP reaches this hex\n"
    "                            linear address (the N-th time it is reached with #N).\n"
    "  --io-trace=LO[-HI]        Log port reads and writes in this hex range to stderr.\n"
    "  --instruction-trace-step  Pause between each instruction trace line (ENTER to step,\n"
    "                            sN to skip N, q to quit stepping, d for debugger).\n"
    "  --trace=<cats>            Enable trace-level logging for categories: cpu, video, dos\n"
    "                            (comma-separated).\n"
    "  --debug=<cats>            Enable debug logging for categories: cpu, video, dos.\n"
    "  --exec=\"asm\"              Assemble and execute instructions without loading a program.\n"
    "                            Semicolons separate multiple instructions.\n"
    "  --bench=name              Run in-process benchmark: decoder-loop, rep-movsb, rep-movsd,\n"
    "                            rep-stosd (instruction loops) or render-13h, render-modex,\n"
    "                            render-planar, render-vbe, audio-mix, mem-fill (host-SIMD\n"
    "                            kernels, timed at every SIMD level the host supports).\n"
    "  --bench-steps=N           Number of measured instruction steps for --bench.\n"
    "  --bench-warmup=N          Number of warmup steps before --bench timing.\n"
    "  --host-simd=LEVEL         Host SIMD kernels for rendering, audio and memory fills:\n"
    "                            auto (default), avx2, sse4, sse2, neon or scalar.  Also\n"
    "                            settable with the FADOR_HOST_SIMD environment variable.\n"
    "  --com1=DEV .. --com4=DEV  Serial port (16550A UART at 3F8h IRQ4, 2F8h IRQ3, 3E8h,\n"
    "                            2E8h): none (port, nothing attached; COM1/COM2 default),\n"
    "                            off (no port; COM3/COM4 default), mouse (Microsoft serial\n"
    "                            mouse), file:PATH (write output to a file),\n"
    "                            tcp:HOST:PORT or listen:PORT (null modem over TCP).\n"
    "  --lpt1=DEV .. --lpt3=DEV  Parallel port (378h IRQ7, 278h, 3BCh): none (LPT1\n"
    "                            default), off (LPT2/LPT3 default), file:PATH (printer\n"
    "                            output to a file), covox (Covox Speech Thing DAC) or\n"
    "                            disney (Disney Sound Source).\n"
    "\n"
    "Emulator flags must appear before the program path. Everything after the program\n"
    "path is forwarded verbatim to the DOS program.\n"
    "Use '--' to explicitly separate emulator flags from the program path and arguments.\n"
    "\n"
    "DOS shell commands (when in command prompt):\n"
    "  DIR, CD, MD, RD, COPY, DEL, REN, TYPE, TREE, MOVE, XCOPY\n"
    "  ECHO, SET, PATH, PROMPT, CLS, VER, VOL, DATE, TIME\n"
    "  MEM, HELP, MOUNT, UNMOUNT, BREAK, VERIFY, EXIT\n"
    "\n"
    "Examples:\n"
    "  %s                         Start DOS prompt at Z:\\>\n"
    "  %s /path/to/games          Mount /path/to/games as C:, start at C:\\>\n"
    "  %s --mount=D:/data game.com\n"
    "  %s --himem --throttle-machine=486dx2-66 doom.exe\n"
    "  %s --exec=\"MOV AX, 4C00h; INT 21h\"\n"
    "  %s --no-sdl --bench=decoder-loop --bench-steps=5000000\n",
    programName,
    std::string(fador::cpu::cpuModelNames()).c_str(),
    fador::hw::machinePresetList().c_str(),
    programName, programName, programName, programName, programName, programName);
  // clang-format on
}

} // namespace

int main(int argc, char *argv[]) {
  // Check for --help before any system initialization
  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--help" || arg == "-h" || arg == "-?") {
      printHelp(argv[0]);
      return 0;
    }
  }

  try {
#ifdef _WIN32
    // Suppress MSVC debug runtime dialogs ("Abort/Retry/Ignore",
    // "Debug Assertion Failed", etc.) so crashes exit cleanly
    // without requiring manual user intervention.
    _CrtSetReportMode(_CRT_WARN, _CRTDBG_MODE_DEBUG);
    _CrtSetReportMode(_CRT_ERROR, _CRTDBG_MODE_DEBUG);
    _CrtSetReportMode(_CRT_ASSERT, _CRTDBG_MODE_DEBUG);
    _set_abort_behavior(0, _WRITE_ABORT_MSG | _CALL_REPORTFAULT);
    _set_error_mode(_OUT_TO_STDERR);
    // Set console output to UTF-8 so CP437 characters render correctly
    SetConsoleOutputCP(CP_UTF8);
    SetConsoleCP(CP_UTF8);
#endif

    fador::utils::currentLevel = fador::utils::LogLevel::Info;
    LOG_INFO("Fador's DOS Emulator Starting...");

    fador::memory::MemoryBus memory;
    fador::hw::IOBus iobus;
    fador::hw::PIC8259 pic(true);   // Master PIC (ports 0x20-0x21)
    fador::hw::PIC8259 picSlave(false); // Slave PIC (ports 0xA0-0xA1)
    fador::hw::KeyboardController kbd;
    fador::hw::Joystick joystick;
    fador::hw::PIT8254 pit;
    fador::hw::VGAController vga(memory);
    fador::hw::DMA8237 dma;
    fador::hw::CMOSRTC cmos;
    // --cpu and --throttle-machine (which selects a CPU model) are needed
    // before the CPU is constructed (its reset state is model-specific);
    // the main option loop below parses the rest.
    fador::hw::SpeedOptions speedOptions;
    for (int i = 1; i < argc; ++i) {
      const std::string arg = argv[i];
      if (arg.rfind("--", 0) != 0 || arg == "--")
        break; // program path reached
      if (arg.rfind("--cpu=", 0) == 0) {
        const auto id = fador::cpu::parseCpuModel(arg.substr(6));
        if (!id) {
          LOG_ERROR("Unknown CPU model: ", arg.substr(6), ". Use one of ",
                    fador::cpu::cpuModelNames());
          return 1;
        }
        speedOptions.cpu = *id;
      } else if (arg.rfind("--throttle-machine=", 0) == 0) {
        speedOptions.preset = fador::hw::findMachinePreset(arg.substr(19));
        if (!speedOptions.preset) {
          LOG_ERROR("Unknown throttle machine preset: ", arg.substr(19),
                    ". Use one of ", fador::hw::machinePresetList());
          return 1;
        }
      }
    }
    fador::cpu::CPU cpu(fador::hw::planSpeed(speedOptions).model);
    LOG_INFO("CPU model: ", cpu.model().description);

    // The single emulated time base.  Virtual time advances with the guest
    // clock cycles the CPU model charges per instruction (CPU::cycles());
    // every timed device reads it.  MachineTime applies the speed and
    // pacing once the command line is parsed, and implements HLT and the
    // BIOS/DOS idle waits.
    fador::hw::VirtualClock clock;
    fador::hw::MachineTime machineTime(clock, cpu);
    pit.setTimeSource(clock.timeSource());
    cmos.setTimeSource(clock.timeSource());
    vga.setTimeSource(clock.timeSource()); // 3DAh retrace, cursor and text blink

    // Connect VGA plane memory to the memory bus
    memory.setVGA(&vga);

    fador::hw::audio::AudioBackend audio;
    // The host audio device is opened after the command line is parsed
    // (headless runs never open one).

    fador::hw::audio::AdLib adlib(44100.0f);
    fador::hw::audio::SoundBlaster sb(memory, dma, 44100.0f);
    fador::hw::audio::MidiDevice midi(0x330);
    fador::hw::audio::MidiSynth midiSynth(adlib, 44100.0f);

    midi.setEventCallback(
        [&midiSynth](uint8_t channel, uint8_t statusType, uint8_t data1,
                     uint8_t data2) {
          midiSynth.handleEvent(channel, statusType, data1, data2);
        });

    // Register devices with IOBus
    iobus.registerDevice(0x20, 0x21, &pic);
    iobus.registerDevice(0xA0, 0xA1, &picSlave);
    iobus.registerDevice(0x40, 0x43, &pit);
    iobus.registerDevice(0x60, 0x60, &kbd);
    iobus.registerDevice(0x61, 0x61, &kbd); // PPI port B: speaker, refresh
    iobus.registerDevice(0x64, 0x64, &kbd);
    iobus.registerDevice(0x201, 0x201, &joystick);
    iobus.registerDevice(0x3B4, 0x3B5, &vga); // CRTC (monochrome addressing)
    iobus.registerDevice(0x3BA, 0x3BA, &vga); // input status 1 / feature control
    iobus.registerDevice(0x3C0, 0x3CF, &vga);
    iobus.registerDevice(0x3D0, 0x3DF, &vga);
    iobus.registerDevice(0x00, 0x0F, &dma);
    iobus.registerDevice(0x70, 0x71, &cmos);
    iobus.registerDevice(0xC0, 0xDF, &dma);
    iobus.registerDevice(0x81, 0x8F, &dma);
    iobus.registerDevice(0x220, 0x22F, &sb);
    iobus.registerDevice(0x330, 0x331, &midi);
    iobus.registerDevice(0x388, 0x38B, &adlib);

    kbd.setMemoryBus(&memory);
    fador::hw::DriveManager driveManager;
    fador::hw::DOS dos(cpu, memory);
    fador::hw::BIOS bios(cpu, memory, kbd, pit, pic);
    bios.setVGA(&vga);
    bios.setSlavePIC(&picSlave);

    joystick.setCPU(&cpu);

    bios.initialize();
    dos.initialize();

    {
      auto toolsPath = std::filesystem::path(argv[0]).parent_path() / "tools";
      std::error_code ec;
      if (!std::filesystem::exists(toolsPath, ec))
        std::filesystem::create_directories(toolsPath, ec);
      driveManager.mount('Z', toolsPath.string(), "FADOR_UTIL");
    }
    dos.setDriveManager(&driveManager);

    // Set up CMOS memory sizing after BIOS/DOS init
    cmos.setBaseMemoryKB(640);
    // Extended memory (above 1 MiB): total physical - 1 MiB in KB
    constexpr uint32_t kExtendedKB =
        (fador::memory::MemoryBus::XMS_LIMIT - 0x100000u) >> 10;
    cmos.setExtendedMemoryKB(kExtendedKB);

    // Wire CMOS RTC to slave PIC for IRQ8 delivery
    cmos.setPIC(&picSlave);
    bios.setCMOS(&cmos);

    // Wire up HIMEM (XMS) driver: BIOS handles detection (INT 2Fh) and dispatch
    // (INT E0h)
    if (auto *himem = dos.getHIMEM()) {
      himem->setMemoryBus(&memory);
      bios.setHIMEM(himem);
    }

    // DPMI host: provides protected-mode services for DOS extenders (DOS/4GW)
    fador::hw::DPMI dpmi(cpu, memory);
    dpmi.setDOS(&dos);
    dpmi.setBIOS(&bios);
    if (auto * himem = dos.getHIMEM())
      dpmi.setHIMEM(himem);
    dos.setDPMI(&dpmi);

    fador::cpu::InstructionDecoder decoder(cpu, memory, iobus, bios, dos);

    dpmi.setSegReloadCallback(
        [&decoder](uint8_t seg, uint16_t sel) {
          decoder.loadSegment(static_cast<fador::cpu::SegRegIndex>(seg), sel);
        });

    sb.setFM(&adlib);
    fador::hw::audio::PcSpeaker pcSpeaker(44100.0f);
    pcSpeaker.setTimeSource(clock.timeSource());
    // Port 61h: bit 0 gates PIT channel 2, bit 1 enables the speaker; the
    // refresh toggle (bit 4) and OUT2 (bit 5) come from the PIT.
    kbd.setPort61Callback([&pcSpeaker, &pit](uint8_t v) {
      pit.setGate(2, (v & 1) != 0);
      pcSpeaker.port61Written(v);
    });
    kbd.setPort61StatusSource([&pit]() { return pit.port61Status(); });
    pit.setChannel2Callback([&pcSpeaker](uint16_t reload, uint8_t mode) {
      pcSpeaker.channel2Programmed(reload, mode);
    });
    // x87 FERR# (unmasked FPU exception with CR0.NE=0) -> IRQ13
    cpu.setFPUErrorCallback([&picSlave]() { picSlave.raiseIRQ(5); });
    sb.setIRQCallback([&pic, &picSlave, &sb]() {
      const int line = sb.getIRQ();
      if (line < 8)
        pic.raiseIRQ(static_cast<uint8_t>(line));
      else
        picSlave.raiseIRQ(static_cast<uint8_t>(line - 8));
    });

    fador::hw::ProgramLoader loader(cpu, memory, dos.getHIMEM());
    fador::ui::TerminalRenderer renderer(memory);
    // Emulated frames: the VGA captures them scanline by scanline (palette
    // and register changes land on the line the beam was on) and publishes
    // each when its display ends; frontends and screenshots use the latest.
    fador::ui::FrameRenderer video(memory, vga);
    video.setBIOS(&bios);
    vga.setRasterSink(&video);
    // Vertical retrace interrupt: the ISA IRQ2 line is IRQ9 on an AT
    // (INT 71h, which the BIOS redirects to INT 0Ah).
    vga.setIrqCallback([&picSlave]() { picSlave.raiseIRQ(1); });
    fador::ui::Debugger debugger(cpu, memory, decoder);

    auto dispatchPendingHardwareInterrupt = [&]() {
      if (cpu.hardwareInterruptsEnabled()) {
        int pending = pic.getPendingInterrupt();
        if (pending == -1) {
          pending = picSlave.getPendingInterrupt();
        }
        if (pending != -1) {
          if (pending >= 0x70) {
            picSlave.acknowledgeInterrupt();
          } else {
            pic.acknowledgeInterrupt();
          }
          decoder.injectHardwareInterrupt(static_cast<uint8_t>(pending));
          return true;
        }
      }
      return false;
    };

    // ── Install crash handlers ─────────────────────────────────────────
    // Capture raw pointers to emulator core for the crash dump callback.
    // We must not capture references in the lambda — the crash handler may
    // fire during stack unwinding when references would be dangling.  Using
    // pointers from the stack frame above main()'s try block is safe because
    // the crash handler runs before the frame is destroyed.
    fador::utils::setCrashDumpCallback([&]() {
      // Emergency crash dump — use fprintf to stderr directly because
      // the logger may not be safe inside a signal/SEH handler.
      std::fprintf(stderr, "\n=== EMULATOR CRASH DUMP ===\n");

      // CPU registers
      std::fprintf(stderr, "EAX=%08X EBX=%08X ECX=%08X EDX=%08X\n",
                   cpu.getReg32(fador::cpu::EAX), cpu.getReg32(fador::cpu::EBX),
                   cpu.getReg32(fador::cpu::ECX), cpu.getReg32(fador::cpu::EDX));
      std::fprintf(stderr, "ESI=%08X EDI=%08X EBP=%08X ESP=%08X\n",
                   cpu.getReg32(fador::cpu::ESI), cpu.getReg32(fador::cpu::EDI),
                   cpu.getReg32(fador::cpu::EBP), cpu.getReg32(fador::cpu::ESP));
      std::fprintf(stderr, "CS=%04X DS=%04X ES=%04X SS=%04X EIP=%08X\n",
                   cpu.getSegReg(fador::cpu::CS), cpu.getSegReg(fador::cpu::DS),
                   cpu.getSegReg(fador::cpu::ES), cpu.getSegReg(fador::cpu::SS),
                   cpu.getEIP());
      std::fprintf(stderr, "EFLAGS=%08X CR0=%08X\n",
                   cpu.getEFLAGS(), cpu.getCR(0));

      // Disassembly around EIP
      uint32_t eip = cpu.getEIP();
      uint32_t csBase = cpu.getSegBase(fador::cpu::CS);
      uint32_t phys = csBase + eip;
      std::fprintf(stderr, "\n--- Disassembly around CS:EIP (phys=0x%08X) ---\n", phys);
      for (int i = -5; i <= 10; i++) {
        uint32_t addr = phys + static_cast<uint32_t>(i);
        uint8_t byte = 0;
        try { byte = memory.read8(addr); } catch (...) { byte = 0x00; }
        std::fprintf(stderr, "%s 0x%08X: %02X\n", (i == 0 ? "-->" : "   "), addr, byte);
      }

      // Stack dump
      uint16_t ss = cpu.getSegReg(fador::cpu::SS);
      uint32_t ssBase = cpu.getSegBase(fador::cpu::SS);
      uint32_t sp = cpu.getReg32(fador::cpu::ESP);
      uint32_t stackPhys = ssBase + (sp & 0xFFFF);
      std::fprintf(stderr, "\n--- Stack (SS:SP = %04X:%04X) ---\n", ss, (unsigned)(sp & 0xFFFF));
      for (int i = 0; i < 32; i++) {
        uint32_t addr = stackPhys + static_cast<uint32_t>(i);
        try {
          std::fprintf(stderr, "%02X ", memory.read8(addr));
        } catch (...) {
          std::fprintf(stderr, "?? ");
        }
        if ((i & 15) == 15) std::fprintf(stderr, "\n");
      }
      std::fprintf(stderr, "\n");

      // VRAM dump (text mode)
      std::fprintf(stderr, "\n--- Text-mode VRAM ---\n");
      for (int row = 0; row < 25; row++) {
        std::fprintf(stderr, "%02d: ", row);
        for (int col = 0; col < 80; col++) {
          try {
            uint8_t ch = memory.read8(0xB8000 + (row * 80 + col) * 2);
            std::fprintf(stderr, "%c", (ch >= 0x20 && ch < 0x7F) ? (char)ch : '.');
          } catch (...) {
            std::fprintf(stderr, "?");
          }
        }
        std::fprintf(stderr, "\n");
      }
      std::fprintf(stderr, "=== END CRASH DUMP ===\n\n");
      std::fflush(stderr);
    });

    fador::utils::installCrashHandler([](const std::string &desc) {
      // The description is already written to stderr by the crash handler;
      // here we could add additional logging if the logger is safe.
      // We don't use LOG_ERROR because it may allocate memory or throw.
      std::fprintf(stderr, "Crash reason: %s\n", desc.c_str());
    });
    // ── End crash handler installation ───────────────────────────────────

    // Report where out-of-range physical accesses come from (rate-limited)
    memory.setOutOfBoundsHandler([&cpu, &debugger](uint32_t addr, bool write) {
      static int reported = 0;
      if (reported >= 8)
        return;
      ++reported;
      LOG_WARN("Out-of-range ", write ? "write" : "read", " 0x", std::hex,
               addr, " at CS:EIP=", cpu.getSegReg(fador::cpu::CS), ":",
               cpu.getInstructionStartEIP(), " EAX=", cpu.getReg32(fador::cpu::EAX),
               " EBX=", cpu.getReg32(fador::cpu::EBX), " ECX=",
               cpu.getReg32(fador::cpu::ECX), " EDX=", cpu.getReg32(fador::cpu::EDX),
               " ESI=", cpu.getReg32(fador::cpu::ESI), " EDI=",
               cpu.getReg32(fador::cpu::EDI), " EBP=", cpu.getReg32(fador::cpu::EBP),
               " ESP=", cpu.getReg32(fador::cpu::ESP),
               " CSbase=", cpu.getSegBase(fador::cpu::CS), " DSbase=",
               cpu.getSegBase(fador::cpu::DS), " ESbase=",
               cpu.getSegBase(fador::cpu::ES), " SSbase=",
               cpu.getSegBase(fador::cpu::SS), " ES=", cpu.getSegReg(fador::cpu::ES));
      (void)debugger;
    });

    LOG_INFO("System initialized successfully.");

    // Parse command line:
    //   fadors_emu [--himem] [--debug=<cats>] [--throttle-ips=N]
    //   [--throttle-machine=name]
    //   <program.com|exe>
    //   [program-args...]
    // Emulator flags (--himem, --debug=) are consumed only before the program
    // path. Everything after the program path is forwarded verbatim to the DOS
    // program, so program arguments that look like flags (e.g. -? /h) are
    // passed through unchanged.  In shells that glob-expand bare '?' (zsh, bash
    // with failglob), quote such arguments: fadors_emu prog.exe '-?' or
    // fadors_emu prog.exe -- -?
    bool useHimem = false;
    bool dumpOnExit = false;
    uint64_t stopAfterCycles = 0; // 0 = disabled
    bool stopAfterDebugger = false;
    uint64_t instructionTraceCount = 0;
    bool instructionTraceStep = false;
    uint64_t instructionTraceSkipSteps = 0;
    std::optional<uint32_t> instructionTraceFrom;
    uint64_t instructionTraceSkipHits = 0;
    bool deterministic = false;     // --deterministic: virtual time only
    std::optional<fador::hw::CivilDateTime> startTime; // --start-time
    std::string execAsm;          // --exec="asm instructions"
    std::string benchmarkArg;
    std::string hostSimdArg;      // --host-simd=auto|avx2|sse4|sse2|scalar
    BenchmarkKind benchmark = BenchmarkKind::None;
    uint64_t benchmarkSteps = 5000000;
    uint64_t benchmarkWarmup = 500000;
#ifdef HAVE_SDL2
    bool useSDL = true; // Default to SDL when available
#else
    bool useSDL = false;
#endif
    bool headless = false;          // --headless: no rendering, no host input
    double maxSeconds = 0.0;        // --max-seconds=N wall-clock limit
    std::string screenshotPath;     // --screenshot=FILE.png at exit
    std::string audioWavPath;       // --audio-wav=FILE.wav: record output
    double screenshotAt = -1.0;     // --screenshot-at=T: capture at T s instead
    ScriptedKeys scriptedKeys;      // --keys=T:KEY,...
    // --comN= / --lptN=: what is attached to the serial / parallel ports
    std::array<std::string, 4> comSpecs{"none", "none", "off", "off"};
    std::array<std::string, 3> lptSpecs{"none", "off", "off"};
    std::optional<fador::hw::audio::GusConfig> gusConfig; // --gus[=...]
    struct MountEntry { char letter; std::string hostPath; std::string label; bool cdrom = false; };
    std::vector<MountEntry> cliMounts;
    std::string path;
    std::string args;
    for (int i = 1; i < argc; ++i) {
      std::string arg = argv[i];

      if (!path.empty()) {
        // Program path already found – everything else belongs to the program.
        // '--' after the path is a conventional shell separator; discard it.
        if (arg == "--")
          continue;
        if (!args.empty())
          args += ' ';
        args += arg;
        continue;
      }

      // Emulator-specific flags (must appear before the program path).
      if (arg == "--himem") {
        useHimem = true;
      } else if (arg.find("--cpu=") == 0) {
        // Applied before the CPU was constructed.
      } else if (arg == "--dump-on-exit") {
        dumpOnExit = true;
      } else if (arg == "--no-fast-path") {
        decoder.setFastPathEnabled(false);
      } else if (arg == "--no-fpu") {
        cpu.setFpuPresent(false);
        memory.write16(0x410, memory.read16(0x410) & ~0x0002u);
      } else if (arg == "--dpmi=off" || arg == "--dpmi=on") {
        dos.setDPMI(arg == "--dpmi=on" ? &dpmi : nullptr);
      } else if (arg == "--vcpi=off" || arg == "--vcpi=on") {
        bios.vcpi().setEnabled(arg == "--vcpi=on");
      } else if (arg.find("--throttle-ips=") == 0) {
        const uint64_t ips = std::stoull(arg.substr(15));
        if (ips == 0) {
          LOG_ERROR("--throttle-ips needs a positive rate");
          return 1;
        }
        speedOptions.throttleIps = ips;
      } else if (arg.find("--throttle-machine=") == 0) {
        // Parsed before the CPU was constructed.
      } else if (arg.find("--start-time=") == 0) {
        startTime = fador::hw::parseCivilDateTime(arg.substr(13));
        if (!startTime) {
          LOG_ERROR("--start-time needs YYYY-MM-DDTHH:MM:SS (1980-2099), got ",
                    arg.substr(13));
          return 1;
        }
      } else if (arg == "--deterministic") {
        deterministic = true;
      } else if (arg.find("--stop-after=") == 0) {
        stopAfterCycles = std::stoull(arg.substr(13));
      } else if (arg == "--stop-after-debugger") {
        stopAfterDebugger = true;
      } else if (arg.find("--instruction-trace=") == 0) {
        instructionTraceCount = std::stoull(arg.substr(20));
      } else if (arg.find("--instruction-trace-from=") == 0) {
        std::string spec = arg.substr(25);
        if (auto hash = spec.find('#'); hash != std::string::npos) {
          try {
            const uint64_t hit = std::stoull(spec.substr(hash + 1));
            instructionTraceSkipHits = hit > 0 ? hit - 1 : 0;
          } catch (...) {
            LOG_ERROR("Invalid hit count in --instruction-trace-from: ", spec);
            return 1;
          }
          spec.resize(hash);
        }
        auto parsedAddress = parseLinearHexAddress(spec);
        if (!parsedAddress) {
          LOG_ERROR("Invalid --instruction-trace-from address: ",
                    arg.substr(25), ". Use a hex linear address.");
          return 1;
        }
        instructionTraceFrom = *parsedAddress;
      } else if (arg == "--instruction-trace-step") {
        instructionTraceStep = true;
      } else if (arg.find("--exec=") == 0) {
        execAsm = arg.substr(7);
      } else if (arg.find("--bench=") == 0) {
        benchmarkArg = arg.substr(8);
        benchmark = parseBenchmarkKind(benchmarkArg);
      } else if (arg.find("--host-simd=") == 0) {
        hostSimdArg = arg.substr(12);
      } else if (arg.find("--bench-steps=") == 0) {
        benchmarkSteps = std::stoull(arg.substr(14));
      } else if (arg.find("--bench-warmup=") == 0) {
        benchmarkWarmup = std::stoull(arg.substr(15));
      } else if (arg == "--sdl") {
#ifdef HAVE_SDL2
        useSDL = true;
#else
        LOG_WARN("SDL2 not available, ignoring --sdl flag");
#endif
      } else if (arg == "--no-sdl") {
        useSDL = false;
      } else if (arg == "--headless") {
        headless = true;
        useSDL = false;
      } else if (arg.find("--max-seconds=") == 0) {
        maxSeconds = std::stod(arg.substr(14));
      } else if (arg.find("--audio-wav=") == 0) {
        audioWavPath = arg.substr(12);
      } else if (arg.find("--screenshot=") == 0) {
        screenshotPath = arg.substr(13);
      } else if (arg.find("--screenshot-at=") == 0) {
        screenshotAt = std::stod(arg.substr(16));
      } else if (arg.find("--keys=") == 0) {
        std::string error;
        if (!scriptedKeys.parse(arg.substr(7), error)) {
          LOG_ERROR("Invalid --keys specification: ", error);
          return 1;
        }
      } else if (arg.find("--io-trace=") == 0) {
        const std::string spec = arg.substr(11);
        char *end = nullptr;
        const unsigned long first = std::strtoul(spec.c_str(), &end, 16);
        unsigned long last = first;
        bool ok = end != spec.c_str();
        if (ok && *end == '-') {
          const char *hi = end + 1;
          last = std::strtoul(hi, &end, 16);
          ok = end != hi;
        }
        if (!ok || *end != '\0' || first > last || last > 0xFFFF) {
          LOG_ERROR("Invalid --io-trace= range. Use --io-trace=220-22F (hex)");
          return 1;
        }
        iobus.setTraceRange(static_cast<uint16_t>(first), static_cast<uint16_t>(last));
        iobus.setTraceContext([&cpu]() {
          char buf[32];
          std::snprintf(buf, sizeof buf, "at %04X:%08X",
                        cpu.getSegReg(fador::cpu::CS), cpu.getEIP());
          return std::string(buf);
        });
      } else if (arg.size() >= 7 && arg.rfind("--com", 0) == 0 &&
                 arg[5] >= '1' && arg[5] <= '4' && arg[6] == '=') {
        comSpecs[arg[5] - '1'] = arg.substr(7);
      } else if (arg.size() >= 7 && arg.rfind("--lpt", 0) == 0 &&
                 arg[5] >= '1' && arg[5] <= '3' && arg[6] == '=') {
        lptSpecs[arg[5] - '1'] = arg.substr(7);
      } else if (arg == "--gus" || arg.rfind("--gus=", 0) == 0) {
        fador::hw::audio::GusConfig config;
        std::string error;
        if (arg.size() > 6 &&
            !fador::hw::audio::parseGusSpec(arg.substr(6), config, error)) {
          LOG_ERROR("--gus: ", error,
                    ". Use --gus[=PORT][,irq=N][,dma=N][,mem=KB]");
          return 1;
        }
        gusConfig = config;
      } else if (arg.find("--env=") == 0) {
        const std::string spec = arg.substr(6);
        const auto eq = spec.find('=');
        if (eq == std::string::npos || eq == 0) {
          LOG_ERROR("Invalid --env= syntax. Use --env=NAME=VALUE");
          return 1;
        }
        fador::hw::ProgramLoader::setEnvironmentVariable(spec.substr(0, eq),
                                                         spec.substr(eq + 1));
      } else if (arg.find("--mount=") == 0) {
        std::string mountSpec = arg.substr(8);
        if (mountSpec.size() >= 3 && std::isalpha(static_cast<unsigned char>(mountSpec[0])) && mountSpec[1] == ':') {
          char letter = static_cast<char>(std::toupper(static_cast<unsigned char>(mountSpec[0])));
          std::string hostPath = mountSpec.substr(2);
          cliMounts.push_back({letter, hostPath, "", false});
        } else {
          LOG_ERROR("Invalid --mount= syntax. Use --mount=C:/path");
          return 1;
        }
      } else if (arg.find("--cdrom=") == 0) {
        std::string spec = arg.substr(8);
        if (spec.size() >= 3 && std::isalpha(static_cast<unsigned char>(spec[0])) && spec[1] == ':') {
          char letter = static_cast<char>(std::toupper(static_cast<unsigned char>(spec[0])));
          std::string imagePath = spec.substr(2);
          if (!imagePath.empty() && imagePath[0] == '=')
            imagePath = imagePath.substr(1);
          cliMounts.push_back({letter, imagePath, "", true});
        } else {
          LOG_ERROR("Invalid --cdrom= syntax. Use --cdrom=D:/path/to/image.iso");
          return 1;
        }
      } else if (arg.find("--trace=") == 0) {
        fador::utils::currentLevel = fador::utils::LogLevel::Trace;
        const std::string cats = arg.substr(8);
        if (cats.find("cpu") != std::string::npos)
          fador::utils::enabledCategories |= fador::utils::CAT_CPU;
        if (cats.find("video") != std::string::npos)
          fador::utils::enabledCategories |= fador::utils::CAT_VIDEO;
          memory.setWriteTracing(true); // log VRAM / IVT writes
        if (cats.find("dos") != std::string::npos)
          fador::utils::enabledCategories |= fador::utils::CAT_DOS;
      } else if (arg.find("--debug=") == 0) {
        if (fador::utils::currentLevel > fador::utils::LogLevel::Debug)
          fador::utils::currentLevel = fador::utils::LogLevel::Debug;
        const std::string cats = arg.substr(8);
        if (cats.find("cpu") != std::string::npos)
          fador::utils::enabledCategories |= fador::utils::CAT_CPU;
        if (cats.find("video") != std::string::npos)
          fador::utils::enabledCategories |= fador::utils::CAT_VIDEO;
          memory.setWriteTracing(true); // log VRAM / IVT writes
        if (cats.find("dos") != std::string::npos)
          fador::utils::enabledCategories |= fador::utils::CAT_DOS;
      } else if (arg == "--") {
        if (i + 1 < argc) {
          path = argv[++i];
        }
      } else {
        path = arg;
      }
    }

    if (!fador::utils::simd::initHostSimd(hostSimdArg.c_str())) {
      LOG_ERROR("Unknown --host-simd level: ", hostSimdArg,
                ". Use auto, avx2, sse4, sse2, neon or scalar");
      return 1;
    }
    if (fador::ui::isHostSimdBenchmark(benchmarkArg))
      return fador::ui::runHostSimdBenchmark(benchmarkArg) ? 0 : 1;

    if (!headless)
      audio.init(44100, 2, 1024);

    // Serial and parallel ports.  COM1/COM3 share IRQ 4 and COM2/COM4 IRQ 3
    // on real machines; only COM1/COM2 (and LPT1 on IRQ 7) are wired here.
    static constexpr std::array<uint16_t, 4> kComBase{0x3F8, 0x2F8, 0x3E8, 0x2E8};
    static constexpr std::array<int, 4> kComIrq{4, 3, -1, -1};
    static constexpr std::array<uint16_t, 3> kLptBase{0x378, 0x278, 0x3BC};
    static constexpr std::array<int, 3> kLptIrq{7, -1, -1};
    std::vector<std::unique_ptr<fador::hw::Uart16550>> uarts;
    std::vector<std::unique_ptr<fador::hw::ParallelPort>> lpts;
    {
      std::array<uint16_t, 4> comAddr{};
      std::array<uint16_t, 3> lptAddr{};
      const fador::hw::SerialMouse::InputSource mouseInput = [&bios]() {
        const auto &ms = bios.mouseState();
        return fador::hw::SerialMouse::Input{
            ms.hostMotionX, ms.hostMotionY,
            static_cast<uint8_t>(ms.buttons & 3)};
      };
      for (size_t i = 0; i < comSpecs.size(); ++i) {
        bool present = false;
        std::unique_ptr<fador::hw::SerialBackend> backend;
        std::string error;
        if (!fador::hw::makeSerialBackend(comSpecs[i], mouseInput, present,
                                          backend, error)) {
          LOG_ERROR("--com", i + 1, "=", comSpecs[i], ": ", error);
          return 1;
        }
        if (!present)
          continue;
        auto uart = std::make_unique<fador::hw::Uart16550>(kComBase[i]);
        uart->setTimeSource(clock.timeSource());
        if (kComIrq[i] >= 0)
          uart->setIrqCallback([&pic, irq = static_cast<uint8_t>(kComIrq[i])] {
            pic.raiseIRQ(irq);
          });
        if (backend)
          LOG_INFO("COM", i + 1, " (", std::hex, kComBase[i], std::dec,
                   "h): ", backend->describe());
        uart->setBackend(std::move(backend));
        iobus.registerDevice(kComBase[i], kComBase[i] + 7, uart.get());
        comAddr[i] = kComBase[i];
        uarts.push_back(std::move(uart));
      }
      for (size_t i = 0; i < lptSpecs.size(); ++i) {
        bool present = false;
        std::unique_ptr<fador::hw::ParallelDevice> device;
        std::string error;
        if (!fador::hw::makeParallelDevice(lptSpecs[i], present, device,
                                           error)) {
          LOG_ERROR("--lpt", i + 1, "=", lptSpecs[i], ": ", error);
          return 1;
        }
        if (!present)
          continue;
        auto lpt = std::make_unique<fador::hw::ParallelPort>(kLptBase[i]);
        lpt->setTimeSource(clock.timeSource());
        if (kLptIrq[i] >= 0)
          lpt->setIrqCallback([&pic, irq = static_cast<uint8_t>(kLptIrq[i])] {
            pic.raiseIRQ(irq);
          });
        if (device)
          LOG_INFO("LPT", i + 1, " (", std::hex, kLptBase[i], std::dec,
                   "h): ", device->describe());
        lpt->setDevice(std::move(device));
        iobus.registerDevice(kLptBase[i], kLptBase[i] + 2, lpt.get());
        lptAddr[i] = kLptBase[i];
        lpts.push_back(std::move(lpt));
      }
      bios.setIOBus(&iobus);
      bios.setPortTimeSource(clock.timeSource());
      bios.setPortAddresses(comAddr, lptAddr);
    }

    // Gravis Ultrasound (--gus): ports 2X0-2XF and 3X0-3X7.
    std::unique_ptr<fador::hw::audio::GravisUltrasound> gus;
    if (gusConfig) {
      gus = std::make_unique<fador::hw::audio::GravisUltrasound>(
          memory, dma, *gusConfig, 44100.0f);
      gus->setTimeSource(clock.timeSource());
      gus->setIrqCallback([&pic, &picSlave](int line) {
        if (line < 8)
          pic.raiseIRQ(static_cast<uint8_t>(line));
        else
          picSlave.raiseIRQ(static_cast<uint8_t>(line - 8));
      });
      const uint16_t base = gusConfig->basePort;
      if (base == 0x220)
        LOG_WARN("--gus=220 takes the Sound Blaster's ports 220h-22Fh");
      iobus.registerDevice(base, base + 0x0F, gus.get());
      // 3X0/3X1 (the GUS MIDI port) stay with the MPU-401 at 330h.
      iobus.registerDevice(base == 0x230 ? base + 0x102 : base + 0x100,
                           base + 0x107, gus.get());
      auto hasEnv = [](const char *name) {
        for (const auto &entry : fador::hw::ProgramLoader::defaultEnvironment())
          if (entry.first == name)
            return true;
        return false;
      };
      if (!hasEnv("ULTRASND")) // --env=ULTRASND=... wins
        fador::hw::ProgramLoader::setEnvironmentVariable(
            "ULTRASND", fador::hw::audio::ultrasndVariable(*gusConfig));
      if (!hasEnv("ULTRADIR"))
        fador::hw::ProgramLoader::setEnvironmentVariable("ULTRADIR",
                                                         "C:\\ULTRASND");
    }

    speedOptions.deterministic = deterministic;
    const fador::hw::SpeedPlan speedPlan = fador::hw::planSpeed(speedOptions);
    for (const std::string &warning : speedPlan.warnings)
      LOG_WARN(warning);
    machineTime.configure(speedPlan);

    // The guest's wall clock: the RTC holds the date and time (the host's
    // at construction), the BIOS tick count and the DOS date start from it
    // and then follow emulated time only.
    if (startTime || deterministic)
      cmos.setDateTime(startTime.value_or(fador::hw::kDeterministicStartTime));
    bios.loadTickCountFromRTC();
    {
      const fador::hw::CivilDateTime rtcNow = cmos.dateTime();
      dos.setDate(rtcNow.year, rtcNow.month, rtcNow.day);
    }
    dos.setRTC(&cmos);

    // Apply CLI mounts
    for (auto& m : cliMounts) {
      const bool ok = m.cdrom ? driveManager.mountCdrom(m.letter, m.hostPath, m.label)
                              : driveManager.mount(m.letter, m.hostPath, m.label);
      if (!ok) {
        LOG_ERROR("Failed to mount ", m.letter, ": to ", m.hostPath);
        return 1;
      }
    }

    bool mountedArchiveShell = false;
    std::string startupBatCommand;
    if (isZipArchiveFile(path)) {
      std::string absZip = std::filesystem::absolute(path).string();
      if (!driveManager.mount('C', absZip)) {
        LOG_ERROR("Failed to mount ZIP archive as C: ", path);
        return 1;
      }
      driveManager.setCurrentDrive('C');
      const std::string mountRootStr = driveManager.getHostPath('C');
      const std::string zipStem = pathToUtf8String(std::filesystem::path(absZip).stem());
      const std::string selected =
          selectZipLaunchTarget(driveManager, mountRootStr, zipStem, args);
      if (selected.empty()) {
        mountedArchiveShell = true;
      } else {
        const std::filesystem::path selPath(selected);
        const std::string selExt = toUpperAscii(pathToUtf8String(selPath.extension()));
        if (selExt == ".BAT") {
          startupBatCommand = pathToUtf8String(selPath.filename());
          if (!args.empty()) {
            startupBatCommand += " " + args;
          }
          mountedArchiveShell = true;
        } else {
          path = selected;
        }
      }
    } else if (!path.empty() && std::filesystem::is_directory(path)) {
      // Path is a directory — mount as C: and start the shell
      std::string absDir = std::filesystem::absolute(path).string();
      if (!driveManager.mount('C', absDir, "MS-DOS_622")) {
        LOG_ERROR("Failed to mount directory as C:");
        return 1;
      }
      driveManager.setCurrentDrive('C');
    }

    if (instructionTraceFrom && instructionTraceCount == 0) {
      LOG_ERROR("--instruction-trace-from requires --instruction-trace=N");
      return 1;
    }

    fador::ui::Debugger::TraceRequest instructionTrace{
        instructionTraceCount, instructionTraceFrom, instructionTraceSkipHits};

    auto emitInstructionTrace = [&]() {
      if (debugger.emitTraceIfRequested(instructionTrace)) {
        if (instructionTraceStep) {
          if (instructionTraceSkipSteps > 0) {
            --instructionTraceSkipSteps;
            return;
          }
          std::cout << " [Step: ENTER to continue, sN to step N, q to stop stepping, d for debugger] " << std::flush;
          std::string input;
          if (!std::getline(std::cin, input)) {
            instructionTraceStep = false;
            return;
          }
          if (input == "q") {
            instructionTraceStep = false;
          } else if (input == "d") {
            debugger.run();
          } else if (!input.empty() && input[0] == 's') {
            try {
              uint64_t count = std::stoull(input.substr(1));
              if (count > 1) {
                instructionTraceSkipSteps = count - 1;
              }
            } catch (...) {
            }
          }
        }
      }
    };

    auto dumpTextVRAM = [&memory]() {
      for (int row = 0; row < 25; ++row) {
        std::string vramText;
        for (int col = 0; col < 80; ++col) {
          uint8_t ch = memory.read8(0xB8000 + (row * 80 + col) * 2);
          vramText +=
              (ch >= 0x20 && ch < 0x7F) ? static_cast<char>(ch) : '.';
        }
        while (!vramText.empty() && vramText.back() == '.')
          vramText.pop_back();
        if (!vramText.empty())
          LOG_INFO("VRAM[", row, "]: ", vramText);
      }
    };

    auto handleStopAfter = [&](uint64_t instrCount,
                               bool emitInstructionTrace) -> bool {
      LOG_INFO("Stopped after ", instrCount, " cycles (--stop-after)");

      if (stopAfterDebugger) {
        debugger.dumpState();
        dumpTextVRAM();
        stopAfterCycles = 0;
        LOG_INFO("Entering debugger (--stop-after-debugger)");
        return debugger.run();
      }

      if (emitInstructionTrace) {
        std::cout << "=== Instruction trace at stop point ===\n";
        debugger.traceInstructions(50);
      }

      debugger.dumpState();
      dumpTextVRAM();
      return false;
    };

    // No program, directory mounted as C:, or ZIP archive without a primary executable, starts the DOS shell.
    const bool shellMode =
        path.empty() || std::filesystem::is_directory(path) || mountedArchiveShell;

    if (!shellMode) {
      bool loaded = false;
      // Mounts C: (unless --mount did) and sets the current directory, which
      // the program's environment refers to.
      dos.setProgramDir(path);
      loader.setProgramDosDir(driveManager.getFullDosCwd());
      if (path.find(".com") != std::string::npos ||
          path.find(".COM") != std::string::npos) {
        loaded = loader.loadCOM(path, dos.getPSPSegment(), dos, args);
      } else {
        loaded = loader.loadEXE(path, dos.getPSPSegment(), dos, args, useHimem);
      }
      if (!loaded) {
        LOG_ERROR("Failed to load program: ", path);
        return 1;
      }
      decoder.syncSegments();
      if (clock.pacing() != fador::hw::VirtualClock::Pacing::Auto) {
        if (speedPlan.countInstructions)
          LOG_INFO("Emulated time: ",
                   fador::hw::VirtualClock::pacingName(clock.pacing()), ", ",
                   speedPlan.label);
        else
          LOG_INFO("Emulated time: ",
                   fador::hw::VirtualClock::pacingName(clock.pacing()), ", ",
                   speedPlan.label, " (", cpu.model().description,
                   " cycle table at ", clock.cpuHz() / 1e6, " MHz)");
      }
    } else if (path.empty() && (!benchmarkArg.empty() || !execAsm.empty())) {
      auto runExecLoop = [&](uint64_t steps) {
        for (uint64_t i = 0; i < steps; ++i) {
          emitInstructionTrace();
          decoder.step();
          cpu.addCycles(1);
          if (dos.isTerminated())
            return false;
        }
        return true;
      };

      auto syncSegmentsAndCaches = [&]() {
        decoder.syncSegments();
      };

      if (!benchmarkArg.empty()) {
        if (benchmark == BenchmarkKind::None) {
          LOG_ERROR("Unknown benchmark: ", benchmarkArg,
                    ". Use --bench=decoder-loop, rep-movsb, rep-movsd, "
                    "rep-stosd, ", fador::ui::hostSimdBenchmarkList());
          return 1;
        }

        uint32_t origin = (cpu.getSegReg(fador::cpu::CS) << 4) + cpu.getEIP();
        std::function<void()> prepareBenchmark = [&]() {};

        switch (benchmark) {
        case BenchmarkKind::DecoderLoop: {
          static constexpr uint8_t kDecoderLoop[] = {0x90, 0xEB, 0xFD};
          for (size_t i = 0; i < sizeof(kDecoderLoop); ++i) {
            memory.write8(origin + static_cast<uint32_t>(i), kDecoderLoop[i]);
          }
          prepareBenchmark = [&]() {
            cpu.setEIP(origin - cpu.getSegBase(fador::cpu::CS));
            syncSegmentsAndCaches();
          };
          break;
        }
        case BenchmarkKind::RepMovsb: {
          static constexpr uint8_t kRepMovsbLoop[] = {
              0xBE, 0x00, 0x02, // MOV SI,0200h
              0xBF, 0x00, 0x04, // MOV DI,0400h
              0xB9, 0x40, 0x00, // MOV CX,0040h
              0xF3, 0xA4,       // REP MOVSB
              0xEB, 0xF3        // JMP short loop start
          };
          constexpr uint16_t kBenchSeg = 0x1000;
          constexpr uint32_t kBenchBase = static_cast<uint32_t>(kBenchSeg) << 4;
          for (size_t i = 0; i < sizeof(kRepMovsbLoop); ++i) {
            memory.write8(origin + static_cast<uint32_t>(i), kRepMovsbLoop[i]);
          }
          for (uint16_t i = 0; i < 64; ++i) {
            memory.write8(kBenchBase + 0x0200u + i,
                          static_cast<uint8_t>((i * 17u + 3u) & 0xFF));
          }
          prepareBenchmark = [&]() {
            cpu.loadSegment(fador::cpu::DS, kBenchSeg);
            cpu.loadSegment(fador::cpu::ES, kBenchSeg);
            cpu.setEIP(origin - cpu.getSegBase(fador::cpu::CS));
            syncSegmentsAndCaches();
          };
          break;
        }
        case BenchmarkKind::RepMovsd: {
          static constexpr uint8_t kRepMovsdLoop[] = {
              0xBE, 0x00, 0x02, // MOV SI,0200h
              0xBF, 0x00, 0x04, // MOV DI,0400h
              0xB9, 0x40, 0x00, // MOV CX,0040h
              0xF3, 0x66, 0xA5, // REP MOVSD
              0xEB, 0xF2        // JMP short loop start
          };
          constexpr uint16_t kBenchSeg = 0x1000;
          constexpr uint32_t kBenchBase = static_cast<uint32_t>(kBenchSeg) << 4;
          for (size_t i = 0; i < sizeof(kRepMovsdLoop); ++i) {
            memory.write8(origin + static_cast<uint32_t>(i), kRepMovsdLoop[i]);
          }
          for (uint16_t i = 0; i < 64; ++i) {
            memory.write32(kBenchBase + 0x0200u + static_cast<uint32_t>(i) * 4u,
                           0x01020304u * (static_cast<uint32_t>(i) + 1u));
          }
          prepareBenchmark = [&]() {
            cpu.loadSegment(fador::cpu::DS, kBenchSeg);
            cpu.loadSegment(fador::cpu::ES, kBenchSeg);
            cpu.setEIP(origin - cpu.getSegBase(fador::cpu::CS));
            syncSegmentsAndCaches();
          };
          break;
        }
        case BenchmarkKind::RepStosd: {
          // memset()-style fill: 4096 dwords per REP STOSD (one batch).
          // (EAX is set by prepareBenchmark: the loop must fit in 16 bytes)
          static constexpr uint8_t kRepStosdLoop[] = {
              0xBF, 0x00, 0x00, // MOV DI,0000h
              0xB9, 0x00, 0x10, // MOV CX,1000h
              0xF3, 0x66, 0xAB, // REP STOSD
              0xEB, 0xF5        // JMP short loop start
          };
          constexpr uint16_t kBenchSeg = 0x2000;
          for (size_t i = 0; i < sizeof(kRepStosdLoop); ++i) {
            memory.write8(origin + static_cast<uint32_t>(i), kRepStosdLoop[i]);
          }
          prepareBenchmark = [&]() {
            cpu.loadSegment(fador::cpu::ES, kBenchSeg);
            cpu.setReg32(fador::cpu::EAX, 0x12345678u);
            cpu.setEIP(origin - cpu.getSegBase(fador::cpu::CS));
            syncSegmentsAndCaches();
          };
          break;
        }
        case BenchmarkKind::None:
          break;
        }

        LOG_INFO("Running benchmark ", benchmarkName(benchmark),
                 " warmup_steps=", benchmarkWarmup,
                 " measured_steps=", benchmarkSteps);

        prepareBenchmark();
        if (benchmarkWarmup > 0 && !runExecLoop(benchmarkWarmup)) {
          LOG_ERROR("Benchmark warmup terminated unexpectedly");
          return 1;
        }

        prepareBenchmark();
        auto start = std::chrono::steady_clock::now();
        if (!runExecLoop(benchmarkSteps)) {
          LOG_ERROR("Benchmark terminated unexpectedly during measured run");
          return 1;
        }
        auto end = std::chrono::steady_clock::now();

        double elapsedMs =
            std::chrono::duration<double, std::milli>(end - start).count();
        double elapsedSec = elapsedMs / 1000.0;
        double stepsPerSec =
            elapsedSec > 0.0 ? static_cast<double>(benchmarkSteps) / elapsedSec
                              : 0.0;
        double nsPerStep =
            benchmarkSteps > 0 ? (elapsedSec * 1'000'000'000.0) /
                                     static_cast<double>(benchmarkSteps)
                               : 0.0;

        LOG_INFO("Benchmark result name=", benchmarkName(benchmark),
                 " elapsed_ms=", elapsedMs,
                 " ns_per_step=", nsPerStep,
                 " steps_per_sec=", stepsPerSec,
                 " mips=", (stepsPerSec / 1'000'000.0));
        return 0;
      }

      // --exec mode: assemble, write to CS:IP, run, dump state
      if (!execAsm.empty()) {
        // Replace semicolons with newlines for multi-statement support
        std::string asmText = execAsm;
        for (auto &ch : asmText) {
          if (ch == ';')
            ch = '\n';
        }

        uint32_t origin = (cpu.getSegReg(fador::cpu::CS) << 4) + cpu.getEIP();
        fador::cpu::Assembler asmbler;
        auto result = asmbler.assembleBlock(asmText, origin);
        if (!result.error.empty()) {
          LOG_ERROR("Assembly error: ", result.error);
          return 1;
        }
        for (size_t i = 0; i < result.bytes.size(); ++i) {
          memory.write8(origin + static_cast<uint32_t>(i), result.bytes[i]);
        }
        LOG_INFO("Assembled ", result.bytes.size(), " bytes at ", std::hex,
                 origin);
        syncSegmentsAndCaches();

        uint64_t maxCycles = stopAfterCycles > 0 ? stopAfterCycles : 100000;
        runExecLoop(maxCycles);
        debugger.dumpState();
        return 0;
      }
    }

    // ── Frontend selection ──────────────────────────────────────────
    // Exactly one frontend is active: SDL window, ANSI terminal, or
    // headless (no output, no host input; used for automated runs).
    std::unique_ptr<fador::ui::InputManager> terminalInput;
#ifdef HAVE_SDL2
    std::unique_ptr<fador::ui::SDLRenderer> sdlRenderer;
#endif
    std::function<void()> pollHostInput = []() {};
    std::function<void(bool)> renderFrame = [](bool) {};
    std::function<bool()> quitRequested = []() { return false; };

    if (headless) {
      // nothing to wire up
    }
#ifdef HAVE_SDL2
    else if (useSDL) {
      sdlRenderer = std::make_unique<fador::ui::SDLRenderer>(memory, kbd, vga);
      sdlRenderer->setBIOS(bios);
      pollHostInput = [&]() { sdlRenderer->pollInput(); };
      // Present the latest completed emulated frame and ask for the next
      // one (frames are only rasterised when the window wants one).
      renderFrame = [&](bool) {
        sdlRenderer->present(video.completedFrame());
        vga.requestFrame();
      };
      quitRequested = [&]() { return sdlRenderer->shouldQuit(); };
    }
#endif
    else {
      renderer.clearScreen();
      terminalInput = std::make_unique<fador::ui::InputManager>(kbd);
      terminalInput->setBIOS(bios);
      pollHostInput = [&]() { terminalInput->pollInput(); };
      renderFrame = [&](bool force) { renderer.render(force); };
    }

    const auto runStart = std::chrono::steady_clock::now();
    clock.start();
    const uint64_t virtualStart = clock.nowNs();
    // Seconds since start for --max-seconds, --keys and --screenshot-at:
    // virtual time with --deterministic, the wall clock otherwise.
    auto runSeconds = [&](std::chrono::steady_clock::time_point now) {
      if (deterministic)
        return static_cast<double>(clock.nowNs() - virtualStart) * 1e-9;
      return std::chrono::duration<double>(now - runStart).count();
    };
    auto lastRender = runStart;
    // Screenshots are emulated frames: capture every frame while one is
    // wanted (without the text cursor, so they are reproducible).
    uint64_t screenshotSerial = 0;
    if (!screenshotPath.empty()) {
      video.setCaptureCursor(false);
      vga.setContinuousCapture(true);
    }
    auto saveFrame = [](const fador::ui::FrameRenderer::Frame &f,
                        const std::string &path) {
      return fador::ui::writePNG(path, f.width, f.height, f.pixels);
    };
    std::vector<float> audioBuf;
    fador::hw::audio::WavWriter wav;
    if (!audioWavPath.empty() && !wav.open(audioWavPath, 44100, 2))
      LOG_WARN("Could not open ", audioWavPath, " for --audio-wav");

    // Timers, keyboard, RTC and the Sound Blaster's DMA run on virtual time
    // and raise their IRQs here.  Cheap when nothing is due: the clock sync
    // reads the host clock once (not at all with --deterministic) and the
    // PIT/RTC compare against their next interrupt time.
    uint64_t lastHardwareNs = clock.nowNs();
    auto serviceHardware = [&]() {
      clock.sync();
      const uint64_t now = clock.nowNs();
      sb.advance(static_cast<double>((std::min<uint64_t>)(
                     now - lastHardwareNs, 100'000'000)) * 1e-9);
      lastHardwareNs = now;
      pit.advanceTo(now);
      while (pit.checkPendingIRQ0())
        pic.raiseIRQ(0);
      if (kbd.checkPendingIRQ())
        pic.raiseIRQ(1);
      cmos.advanceTo(now);
      while (cmos.checkPendingIRQ8())
        picSlave.raiseIRQ(0);
      vga.advanceTo(now); // frame completion, retrace interrupt
      for (auto &uart : uarts)
        uart->advanceTo(now); // raises IRQ 3/4 through its callback
      for (auto &lpt : lpts)
        lpt->advanceTo(now);
      if (gus)
        gus->advanceTo(now); // voice, timer and DMA IRQs
    };
    auto maybeRender = [&](std::chrono::steady_clock::time_point now) {
      vga.advanceTo(clock.nowNs()); // also at the shell prompt
      if (now - lastRender > std::chrono::milliseconds(33)) {
        renderFrame(false);
        lastRender = now;
      }
    };
    // Generate audio for the virtual time that has passed (44100 frames
    // per virtual second), so the OPL timers, the Sound Blaster and the
    // recording follow emulated time.  The host queue only steers: in auto
    // pacing a queue that runs low or high nudges the virtual clock, an
    // underrun is padded with silence and an overflow (a guest running
    // faster than real time) is dropped.
    constexpr uint64_t kSampleRate = 44100;
    constexpr uint32_t kFrameBytes = 2 * sizeof(float);
    constexpr uint64_t kTargetQueueFrames = 3072; // ~70 ms of latency
    auto framesAt = [](uint64_t ns) {
      return ns / 1'000'000'000 * kSampleRate +
             (ns % 1'000'000'000) * kSampleRate / 1'000'000'000;
    };
    uint64_t audioFramesDone = framesAt(clock.nowNs());
    auto serviceAudio = [&]() {
      const uint64_t due = framesAt(clock.nowNs());
      if (due < audioFramesDone + 128) // generate in chunks of >= ~3 ms
        return;
      constexpr uint64_t kMaxChunk = kSampleRate / 4;
      if (due - audioFramesDone > kMaxChunk)
        audioFramesDone = due - kMaxChunk; // after a stall: skip ahead
      const auto frames = static_cast<size_t>(due - audioFramesDone);
      audioFramesDone = due;
      audioBuf.assign(frames * 2, 0.0f);
      adlib.generateSamples(audioBuf.data(), frames);
      sb.generateSamples(audioBuf.data(), frames);
      pcSpeaker.generateSamples(audioBuf.data(), frames);
      if (gus)
        gus->generateSamples(audioBuf.data(), frames);
      for (auto &lpt : lpts) // Covox / Disney Sound Source
        lpt->generateSamples(audioBuf.data(), frames);
      adlib.updateTimers(static_cast<double>(frames) / kSampleRate);
      if (wav.isOpen())
        wav.write(audioBuf.data(), frames * 2);
      if (!audio.isActive())
        return;
      const uint64_t queued = audio.getQueuedAudioSize() / kFrameBytes;
      if (queued < kTargetQueueFrames / 4) {
        // Underrun (start-up or a host stall): restore the latency cushion.
        std::vector<float> silence((kTargetQueueFrames - queued) * 2, 0.0f);
        audio.queueSamples(silence.data(), silence.size());
      } else if (queued > 4 * kTargetQueueFrames) {
        return; // the guest is ahead of the audio device: drop
      }
      audio.queueSamples(audioBuf.data(), frames * 2);
      if (clock.pacing() == fador::hw::VirtualClock::Pacing::Auto) {
        // Keep the queue near its target by running the virtual clock a
        // little slower or faster than the host clock (at most ~1%).
        const int64_t error = static_cast<int64_t>(queued) -
                              static_cast<int64_t>(kTargetQueueFrames);
        const int64_t step = static_cast<int64_t>(frames) * 1'000'000'000 /
                             static_cast<int64_t>(kSampleRate) / 100;
        if (error > static_cast<int64_t>(kTargetQueueFrames / 2))
          clock.adjustSkew(-step);
        else if (error < -static_cast<int64_t>(kTargetQueueFrames / 2))
          clock.adjustSkew(step);
      }
    };

    bios.setInputPollCallback(pollHostInput);
    // A guest waiting in BIOS/DOS input retires one instruction per retry
    // (the host sleeps 1 ms each time), so the instruction-count based
    // periodic services would only run every ~16 s: run them from here too.
    std::function<void()> servicePeriodicWhileIdle = [] {};
    unsigned idleCalls = 0;
    auto guestIdle = [&]() {
      cpu.addCycles(8);
      machineTime.guestIdle(); // waits for the next event if only waiting
      serviceHardware();
      dispatchPendingHardwareInterrupt();
      if ((++idleCalls & 7) == 0)
        servicePeriodicWhileIdle();
    };
    bios.setIdleCallback([&]() {
      guestIdle();
      maybeRender(std::chrono::steady_clock::now());
    });
    dos.setKeyboard(kbd);
    dos.setInputPollCallback(pollHostInput);
    dos.setIdleCallback(guestIdle);

    cpu.setEFLAGS(cpu.getEFLAGS() | fador::cpu::FLAG_INTERRUPT);

    // Main execution loop.  Instructions run in batches; host services
    // (IRQ delivery, input, audio, rendering) run between batches.  The
    // per-instruction slow path is only used while tracing or when a
    // --stop-after limit falls inside the batch.
    constexpr uint64_t kBatch = 1024;
    uint64_t instrCount = 0;
    int exitStatus = 0;
    bool running = true; // false once the emulator should exit
    const bool perInstructionSlowPath = instructionTraceCount > 0;
    // Host services that also run while the shell waits for input.
    auto servicePeriodic = [&](std::chrono::steady_clock::time_point now) {
      pollHostInput();
      {
        // Serial mouse, sockets and printer files (host I/O).
        const uint64_t nowNs = clock.nowNs();
        for (auto &uart : uarts)
          uart->pollBackend(nowNs);
        for (auto &lpt : lpts)
          lpt->poll(nowNs);
      }
      const double elapsed = runSeconds(now);
      scriptedKeys.service(elapsed, kbd);
      if (screenshotAt >= 0 && !screenshotPath.empty()) {
        // The first emulated frame completed after the time is reached
        if (elapsed < screenshotAt) {
          screenshotSerial = video.completedFrame().serial;
        } else if (video.completedFrame().serial > screenshotSerial) {
          if (saveFrame(video.completedFrame(), screenshotPath))
            LOG_INFO("Screenshot written to ", screenshotPath, " (",
                     video.completedFrame().width, "x",
                     video.completedFrame().height, ") at ", screenshotAt,
                     " s");
          screenshotAt = -1.0;
          screenshotPath.clear(); // do not overwrite it at exit
        }
      }
      serviceAudio();
      maybeRender(now);
      if (quitRequested()) {
        LOG_INFO("Window closed by user after ", instrCount, " instructions");
        running = false;
      }
      if (maxSeconds > 0 && elapsed > maxSeconds) {
        LOG_INFO("Time limit of ", maxSeconds, " s reached after ",
                 instrCount, " instructions");
        exitStatus = 124;
        running = false;
      }
    };
    servicePeriodicWhileIdle = [&]() {
      servicePeriodic(std::chrono::steady_clock::now());
    };
    // HLT and the idle waits stop at the next device event.
    machineTime.addWakeSource([&pit] { return pit.nextIrq0Ns(); });
    machineTime.addWakeSource([&cmos] { return cmos.nextIrqNs(); });
    // Vertical retrace interrupt; never due unless the guest enabled it
    machineTime.addWakeSource([&vga] { return vga.nextIrqNs(); });
    machineTime.addWakeSource([&sb, &lastHardwareNs] {
      const double s = sb.secondsUntilIrq();
      return s < 0.0 ? fador::hw::MachineTime::kNever
                     : lastHardwareNs + static_cast<uint64_t>(s * 1e9);
    });
    if (gus)
      machineTime.addWakeSource([g = gus.get()] { return g->nextEventNs(); });
    for (const auto &uart : uarts) // character timing, FIFO timeouts
      machineTime.addWakeSource(
          [u = uart.get()] { return u->nextEventNs(); });
    bios.setDelayCallback([&](uint64_t ns) { machineTime.delay(ns); });
    machineTime.setHooks({serviceHardware, dispatchPendingHardwareInterrupt,
                          [&]() {
                            servicePeriodic(std::chrono::steady_clock::now());
                            return running;
                          }});
    // Runs the loaded program until it terminates (true) or the emulator
    // has to stop (false).
    auto runGuest = [&]() -> bool {
    while (running) {
      uint64_t batch = kBatch;
      if (stopAfterCycles > 0 && stopAfterCycles - instrCount < batch)
        batch = stopAfterCycles > instrCount ? stopAfterCycles - instrCount : 1;

      uint64_t executed = 0;
      if (perInstructionSlowPath) {
        for (; executed < batch; ++executed) {
          emitInstructionTrace();
          decoder.step();
          cpu.addCycles(1);
          if (dos.isTerminated()) {
            ++executed;
            break;
          }
        }
      } else {
        executed = decoder.run(batch);
      }
      instrCount += executed;

      if (dos.isTerminated()) {
        LOG_INFO("Program terminated normally with exit code ",
                 (int)dos.getExitCode(), " after ", instrCount,
                 " instructions");
        exitStatus = dos.getExitCode();
        return true;
      }

      if (stopAfterCycles > 0 && instrCount >= stopAfterCycles) {
        if (!handleStopAfter(instrCount, true)) {
          running = false;
          break;
        }
        continue;
      }

      // ── Between batches: host services ──
      serviceHardware();
      dispatchPendingHardwareInterrupt();
      if (bios.shouldReset()) {
        LOG_INFO("Reset requested — terminating");
        running = false;
        break;
      }

      if ((instrCount & 0x3FFF) < kBatch) {
        servicePeriodic(std::chrono::steady_clock::now());
      }
    }
    return false;
    };

    if (shellMode) {
      // DOS 6.22 COMMAND.COM equivalent.  Programs it starts run through
      // runGuest(), like a program given on the command line.
      fador::hw::DosShell shell(dos, driveManager, loader, cpu, decoder, kbd,
                                bios, memory);
      shell.setInputPollCallback(pollHostInput);
      // At the prompt no guest code runs, so no interrupts are delivered
      // (an IRQ dispatched now would stay in service, unacknowledged).
      shell.setIdleCallback([&]() {
        clock.idle(); // time passes at the prompt (scripted keys, RTC)
        clock.sync();
        // The BIOS tick count (the DOS clock) keeps running at the prompt.
        pit.advanceTo(clock.nowNs());
        while (pit.checkPendingIRQ0())
          bios.advanceTickCount();
        servicePeriodic(std::chrono::steady_clock::now());
      });
      shell.setStopCheck([&]() { return !running; });
      shell.setProgramRunner(runGuest);
      if (!startupBatCommand.empty()) {
        shell.executeCommand(startupBatCommand);
      }
      if (shell.isRunning()) {
        shell.run();
      }
    } else {
      runGuest();
    }


    {
      auto elapsed = std::chrono::duration<double>(
                         std::chrono::steady_clock::now() - runStart)
                         .count();
      LOG_INFO("Executed ", instrCount, " instructions in ", elapsed,
               " s (", elapsed > 0 ? instrCount / elapsed / 1e6 : 0.0,
               " MIPS)");
      LOG_INFO("Emulated time ",
               static_cast<double>(clock.nowNs() - virtualStart) * 1e-9,
               " s (", fador::hw::VirtualClock::pacingName(clock.pacing()),
               " pacing, ", std::dec, cpu.cycles(), " guest cycles, ",
               static_cast<double>(machineTime.haltedNs()) * 1e-9,
               " s halted in ", machineTime.halts(), " HLTs, BIOS tick count ",
               memory.read32(0x46C), ")");
      LOG_INFO("Guest CPU: ", cpu.model().cliName, ", ", cpu.cycles(),
               " cycles");
    }

    if (wav.isOpen()) {
      LOG_INFO("Audio written to ", audioWavPath, ": ",
               wav.framesWritten() / 44100.0, " s, peak ", wav.peak());
      wav.close();
    }

    // Final frame: screenshot / text dump for automated runs
    {
      fador::ui::FrameRenderer frame(memory, vga);
      frame.setBIOS(&bios);
      if (!screenshotPath.empty()) {
        // A program that is still running is shown by the last emulated
        // frame it produced (mid-frame effects included); after it exited,
        // the screen is what it left behind.
        bool ok;
        int w, h;
        if (!dos.isTerminated() && video.completedFrame().serial > 0) {
          const auto &f = video.completedFrame();
          ok = saveFrame(f, screenshotPath);
          w = f.width;
          h = f.height;
        } else {
          ok = frame.render(false) && frame.savePNG(screenshotPath);
          w = frame.width();
          h = frame.height();
        }
        if (ok)
          LOG_INFO("Screenshot written to ", screenshotPath, " (", w, "x", h,
                   ")");
        else
          LOG_WARN("Could not write screenshot to ", screenshotPath);
      }
      std::string text = frame.textScreen();
      if (headless) {
        if (!text.empty())
          std::fwrite(text.data(), 1, text.size(), stdout);
      } else if (!text.empty()) {
        std::string line;
        int row = 0;
        for (char ch : text) {
          if (ch == '\n') {
            if (!line.empty())
              LOG_INFO("VRAM[", row, "]: ", line);
            line.clear();
            ++row;
          } else {
            line += ch;
          }
        }
      }
    }
    if (dumpOnExit)
      debugger.dumpState();
    renderFrame(true);
    return headless ? exitStatus : 0;

  } catch (const std::exception &e) {
    LOG_ERROR("Fatal system error: ", e.what());
    return 1;
  }

  return 0;
}
