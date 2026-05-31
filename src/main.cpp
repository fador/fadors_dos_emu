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
#include "hw/ProgramLoader.hpp"
#include "hw/DriveManager.hpp"
#include "hw/DosShell.hpp"
#include "hw/VGAController.hpp"
#include "hw/audio/AdLib.hpp"
#include "hw/audio/AudioBackend.hpp"
#include "hw/audio/MidiDevice.hpp"
#include "hw/audio/MidiSynth.hpp"
#include "hw/audio/SoundBlaster.hpp"
#include "ui/Debugger.hpp"
#include "ui/InputManager.hpp"
#include "ui/TerminalRenderer.hpp"
#include "utils/CrashHandler.hpp"
#include "utils/Logger.hpp"
#include <array>
#include <iostream>
#ifdef _WIN32
#include <crtdbg.h>
#include <cstdlib>
#endif
#ifdef HAVE_SDL2
#include "ui/SDLRenderer.hpp"
#endif

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <optional>
#include <thread>

namespace {

enum class BenchmarkKind {
  None,
  DecoderLoop,
  RepMovsb,
  RepMovsd,
};

BenchmarkKind parseBenchmarkKind(const std::string &name) {
  if (name == "decoder-loop")
    return BenchmarkKind::DecoderLoop;
  if (name == "rep-movsb")
    return BenchmarkKind::RepMovsb;
  if (name == "rep-movsd")
    return BenchmarkKind::RepMovsd;
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
  case BenchmarkKind::None:
    break;
  }
  return "none";
}

class InstructionThrottle {
public:
  explicit InstructionThrottle(uint64_t targetInstructionsPerSecond)
      : m_targetInstructionsPerSecond(targetInstructionsPerSecond) {}

  bool enabled() const { return m_targetInstructionsPerSecond != 0; }

  void reset() {
    if (!enabled()) {
      return;
    }
    m_start = std::chrono::steady_clock::now();
    m_started = true;
  }

  void pace(uint64_t retiredInstructions) {
    if (!m_started || !enabled()) {
      return;
    }

    using Seconds = std::chrono::duration<double>;
    auto targetElapsed = Seconds(static_cast<double>(retiredInstructions) /
                                 static_cast<double>(m_targetInstructionsPerSecond));
    auto targetTime =
        m_start + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                      targetElapsed);
    auto now = std::chrono::steady_clock::now();
    if (targetTime > now) {
      std::this_thread::sleep_until(targetTime);
    }
  }

private:
  uint64_t m_targetInstructionsPerSecond{0};
  std::chrono::steady_clock::time_point m_start{};
  bool m_started{false};
};

struct ThrottleMachinePreset {
  const char *name;
  uint64_t instructionsPerSecond;
};

constexpr std::array<ThrottleMachinePreset, 5> kThrottleMachinePresets{{
    {"8088", 330000},
    {"286-12", 1200000},
    {"386dx-33", 4000000},
    {"486dx2-66", 8000000},
    {"pentium-90", 20000000},
}};

uint64_t throttleMachineInstructionsPerSecond(const std::string &name) {
  if (name == "xt" || name == "pc-xt")
    return 330000;
  if (name == "286" || name == "at" || name == "at-12")
    return 1200000;
  if (name == "386" || name == "386dx")
    return 4000000;
  if (name == "486" || name == "486dx2")
    return 8000000;
  if (name == "pentium" || name == "p90")
    return 20000000;

  for (const auto &preset : kThrottleMachinePresets) {
    if (name == preset.name) {
      return preset.instructionsPerSecond;
    }
  }

  return 0;
}

const char *throttleMachinePresetList() {
  return "8088, 286-12, 386dx-33, 486dx2-66, pentium-90";
}

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
    "Usage: %s [OPTIONS] [program.com|exe|directory] [program-args...]\n"
    "\n"
    "Fador's DOS Emulator — a multiplatform MS-DOS 6.22 emulator written in modern C++20.\n"
    "\n"
    "Launch modes:\n"
    "  No arguments           Start the DOS 6.22 command prompt (Z:\\>).\n"
    "  <directory>            Mount directory as C: and start the command prompt at C:\\>.\n"
    "  <program.com|exe>      Load and run the program directly.\n"
    "\n"
    "Options:\n"
    "  --help, -h, -?            Print this help message and exit.\n"
    "  --mount=L:/path           Mount host directory at drive letter L (repeatable).\n"
    "  --himem                   Enable HIMEM/XMS extended memory support.\n"
    "  --sdl                     Force the SDL graphical frontend (requires SDL2 build).\n"
    "  --no-sdl                  Force headless/text-mode execution even in SDL builds.\n"
    "  --throttle-ips=N          Pace guest execution to roughly N retired instructions/sec.\n"
    "  --throttle-machine=name   Apply a named speed preset: %s.\n"
    "  --stop-after=N            Stop execution after N instruction cycles and dump state.\n"
    "  --stop-after-debugger     When --stop-after triggers, enter the interactive debugger.\n"
    "  --dump-on-exit            Dump CPU state when the program terminates.\n"
    "  --instruction-trace=N     Print a trace line per executed instruction for the next N.\n"
    "  --instruction-trace-from=ADDR\n"
    "                            Delay --instruction-trace until CS:EIP reaches this hex\n"
    "                            linear address.\n"
    "  --instruction-trace-step  Pause between each instruction trace line (ENTER to step,\n"
    "                            sN to skip N, q to quit stepping, d for debugger).\n"
    "  --trace=<cats>            Enable trace-level logging for categories: cpu, video, dos\n"
    "                            (comma-separated).\n"
    "  --debug=<cats>            Enable debug logging for categories: cpu, video, dos.\n"
    "  --exec=\"asm\"              Assemble and execute instructions without loading a program.\n"
    "                            Semicolons separate multiple instructions.\n"
    "  --bench=name              Run in-process benchmark (decoder-loop, rep-movsb, rep-movsd).\n"
    "  --bench-steps=N           Number of measured instruction steps for --bench.\n"
    "  --bench-warmup=N          Number of warmup steps before --bench timing.\n"
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
    throttleMachinePresetList(),
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
    fador::cpu::CPU cpu;

    // Connect VGA plane memory to the memory bus
    memory.setVGA(&vga);

    fador::hw::audio::AudioBackend audio;
    audio.init(44100, 2, 1024);

    fador::hw::audio::AdLib adlib(44100.0f, &cpu.getCyclesRef());
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
    iobus.registerDevice(0x64, 0x64, &kbd);
    iobus.registerDevice(0x201, 0x201, &joystick);
    iobus.registerDevice(0x3C0, 0x3CF, &vga);
    iobus.registerDevice(0x3D0, 0x3DF, &vga);
    iobus.registerDevice(0x00, 0x0F, &dma);
    iobus.registerDevice(0x70, 0x71, &cmos);
    iobus.registerDevice(0xC0, 0xDF, &dma);
    iobus.registerDevice(0x81, 0x8F, &dma);
    iobus.registerDevice(0x220, 0x22F, &sb);
    iobus.registerDevice(0x330, 0x331, &midi);
    iobus.registerDevice(0x388, 0x389, &adlib);

    kbd.setMemoryBus(&memory);
    fador::hw::DOS dos(cpu, memory);
    fador::hw::BIOS bios(cpu, memory, kbd, pit, pic);
    bios.setVGA(&vga);
    bios.setSlavePIC(&picSlave);

    joystick.setCPU(&cpu);

    bios.initialize();
    dos.initialize();

    fador::hw::DriveManager driveManager;
    driveManager.mount('Z', (std::filesystem::path(argv[0]).parent_path() / "tools").string(), "FADOR_UTIL");
    dos.setDriveManager(&driveManager);

    // Set up CMOS memory sizing after BIOS/DOS init
    cmos.setBaseMemoryKB(640);
    // Extended memory (above 1 MiB): total physical - 1 MiB in KB
    constexpr uint32_t kExtendedKB =
        (fador::memory::MemoryBus::MEMORY_SIZE - 0x100000u) >> 10;
    cmos.setExtendedMemoryKB(kExtendedKB);

    // Wire CMOS RTC to slave PIC for IRQ8 delivery
    cmos.setPIC(&picSlave);

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

    sb.setIRQCallback([&pic]() { pic.raiseIRQ(5); });

    fador::hw::ProgramLoader loader(cpu, memory, dos.getHIMEM());
    fador::ui::TerminalRenderer renderer(memory);
    fador::ui::Debugger debugger(cpu, memory, decoder);

    auto dispatchPendingHardwareInterrupt = [&](bool retireShadow) {
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
        }
      }
      if (retireShadow) {
        cpu.advanceInterruptShadow();
      }
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
    uint64_t throttleInstructionsPerSecond = 0; // 0 = disabled
    std::string throttleLabel;
    std::string execAsm;          // --exec="asm instructions"
    std::string benchmarkArg;
    BenchmarkKind benchmark = BenchmarkKind::None;
    uint64_t benchmarkSteps = 5000000;
    uint64_t benchmarkWarmup = 500000;
#ifdef HAVE_SDL2
    bool useSDL = true; // Default to SDL when available
#else
    bool useSDL = false;
#endif
    struct MountEntry { char letter; std::string hostPath; std::string label; };
    std::vector<MountEntry> cliMounts;
    std::string path;
    std::string args;
    for (int i = 1; i < argc; ++i) {
      std::string arg = argv[i];

    // Apply CLI mounts
    for (auto& m : cliMounts) {
      if (!driveManager.mount(m.letter, m.hostPath, m.label)) {
        LOG_ERROR("Failed to mount ", m.letter, ": to ", m.hostPath);
        return 1;
      }
    }

    if (!path.empty() && std::filesystem::is_directory(path)) {
      // Path is a directory — mount as C: and start the shell
      std::string absDir = std::filesystem::absolute(path).string();
      if (!driveManager.mount('C', absDir, "MS-DOS_622")) {
        LOG_ERROR("Failed to mount directory as C:");
        return 1;
      }
      driveManager.setCurrentDrive('C');
      fador::hw::DosShell shell(dos, driveManager, loader, cpu, decoder, kbd, bios, memory);
      shell.run();
      return 0;
    }

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
      } else if (arg == "--dump-on-exit") {
        dumpOnExit = true;
      } else if (arg.find("--throttle-ips=") == 0) {
        throttleInstructionsPerSecond = std::stoull(arg.substr(15));
        throttleLabel = arg.substr(15);
      } else if (arg.find("--throttle-machine=") == 0) {
        std::string presetName = arg.substr(19);
        throttleInstructionsPerSecond =
            throttleMachineInstructionsPerSecond(presetName);
        if (throttleInstructionsPerSecond == 0) {
          LOG_ERROR("Unknown throttle machine preset: ", presetName,
                    ". Use one of ", throttleMachinePresetList());
          return 1;
        }
        throttleLabel = presetName;
      } else if (arg.find("--stop-after=") == 0) {
        stopAfterCycles = std::stoull(arg.substr(13));
      } else if (arg == "--stop-after-debugger") {
        stopAfterDebugger = true;
      } else if (arg.find("--instruction-trace=") == 0) {
        instructionTraceCount = std::stoull(arg.substr(20));
      } else if (arg.find("--instruction-trace-from=") == 0) {
        auto parsedAddress = parseLinearHexAddress(arg.substr(25));
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
      } else if (arg.find("--mount=") == 0) {
        std::string mountSpec = arg.substr(8);
        if (mountSpec.size() >= 3 && std::isalpha(mountSpec[0]) && mountSpec[1] == ':') {
          char letter = static_cast<char>(std::toupper(mountSpec[0]));
          std::string hostPath = mountSpec.substr(2);
          cliMounts.push_back({letter, hostPath, ""});
        } else {
          LOG_ERROR("Invalid --mount= syntax. Use --mount=C:/path");
          return 1;
        }
      } else if (arg.find("--trace=") == 0) {
        fador::utils::currentLevel = fador::utils::LogLevel::Trace;
        const std::string cats = arg.substr(8);
        if (cats.find("cpu") != std::string::npos)
          fador::utils::enabledCategories |= fador::utils::CAT_CPU;
        if (cats.find("video") != std::string::npos)
          fador::utils::enabledCategories |= fador::utils::CAT_VIDEO;
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
        if (cats.find("dos") != std::string::npos)
          fador::utils::enabledCategories |= fador::utils::CAT_DOS;
      } else if (arg == "--") {
        // Explicit end-of-emulator-flags sentinel; next arg is the program
        // path.
        if (i + 1 < argc) {
          path = argv[++i];
        }
      } else {
        // First non-flag argument is the program path.
        path = arg;
      }
    }

    if (instructionTraceFrom && instructionTraceCount == 0) {
      LOG_ERROR("--instruction-trace-from requires --instruction-trace=N");
      return 1;
    }

    fador::ui::Debugger::TraceRequest instructionTrace{instructionTraceCount,
                                                       instructionTraceFrom};

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

    if (!path.empty()) {
      bool loaded = false;
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
      dos.setProgramDir(path);
      decoder.syncSegments();
      if (throttleInstructionsPerSecond > 0) {
        if (!throttleLabel.empty()) {
          LOG_INFO("Execution throttle enabled: ", throttleLabel, " -> ",
                   std::dec, throttleInstructionsPerSecond,
                   " instructions/sec");
        } else {
          LOG_INFO("Execution throttle enabled at ", std::dec,
                   throttleInstructionsPerSecond,
                   " instructions/sec");
        }
      }
    } else {
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
                    ". Use --bench=decoder-loop, --bench=rep-movsb, or "
                    "--bench=rep-movsd");
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

      LOG_WARN("No program specified.");
      printHelp(argv[0]);
      LOG_WARN("\nStarting interactive debugger with no loaded program.\n");

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

      // Start shell by default if no program (DOS 6.22 COMMAND.COM equivalent)
      {
        fador::hw::DosShell shell(dos, driveManager, loader, cpu, decoder, kbd, bios, memory);
        shell.run();
      }
      return 0;
    }

#ifdef HAVE_SDL2
    if (useSDL) {
      // ── SDL2 graphical window path ──────────────────────────
  InstructionThrottle throttle(throttleInstructionsPerSecond);
  throttle.reset();

      fador::ui::SDLRenderer sdlRenderer(memory, kbd, vga);
      sdlRenderer.setBIOS(bios);

      bios.setInputPollCallback([&sdlRenderer]() { sdlRenderer.pollInput(); });
      bios.setIdleCallback([&sdlRenderer, &pit, &cpu, &decoder, &kbd, &pic,
                &picSlave, &cmos,
                &dispatchPendingHardwareInterrupt]() {
        cpu.addCycles(8);
        pit.update();
        while (pit.checkPendingIRQ0()) {
          pic.raiseIRQ(0);
        }
        if (kbd.checkPendingIRQ()) {
          pic.raiseIRQ(1);
        }
        cmos.advanceTime();
        while (cmos.checkPendingIRQ8()) {
          picSlave.raiseIRQ(0);
        }
        dispatchPendingHardwareInterrupt(false);
        static auto lr = std::chrono::steady_clock::now();
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - lr)
                .count() > 33) {
          sdlRenderer.render();
          lr = now;
        }
      });
      dos.setKeyboard(kbd);
      dos.setInputPollCallback([&sdlRenderer]() { sdlRenderer.pollInput(); });
      dos.setIdleCallback([&pit, &cpu, &decoder, &kbd, &pic,
               &picSlave, &cmos,
               &dispatchPendingHardwareInterrupt]() {
        cpu.addCycles(8);
        pit.update();
        while (pit.checkPendingIRQ0()) { pic.raiseIRQ(0); }
        if (kbd.checkPendingIRQ()) { pic.raiseIRQ(1); }
        cmos.advanceTime();
        while (cmos.checkPendingIRQ8()) { picSlave.raiseIRQ(0); }
        dispatchPendingHardwareInterrupt(false);
      });

      cpu.setEFLAGS(cpu.getEFLAGS() | fador::cpu::FLAG_INTERRUPT);

      bool running = true;
      uint64_t instrCount = 0;
      std::vector<float> audioBuf;

      while (running) {
        emitInstructionTrace();

        // Save prev state to catch EIP corruption
        uint16_t prevCS = cpu.getSegReg(fador::cpu::CS);
        uint32_t prevEIP = cpu.getEIP();
        uint32_t prevLinear = cpu.getSegBase(fador::cpu::CS) + prevEIP;

        decoder.step();
        cpu.addCycles(1);
        instrCount++;

        // Periodic EIP sample for progress tracking
        if ((instrCount & 0xFFFFFF) == 0) {
          uint16_t cs_s = cpu.getSegReg(fador::cpu::CS);
          uint32_t eip_s = cpu.getEIP();
          LOG_INFO("SAMPLE @", instrCount, ": CS=", std::hex, cs_s, " EIP=", eip_s,
                   " flags=", cpu.getEFLAGS());
        }

        // TEMP DIAG: catch EIP jumping to low-memory data area
        // Trigger when: PM + (app CS with data EIP) or (corrupted CS >= 0x1000)
        {
          uint16_t cs_s = cpu.getSegReg(fador::cpu::CS);
          uint32_t eip_s = cpu.getEIP();
          uint32_t linear = cpu.getSegBase(fador::cpu::CS) + eip_s;
          bool datatrip = (cpu.getCR(0) & 1) && cs_s >= 0x100 &&
                          eip_s >= 0x3000 && eip_s < 0x5000;
          bool cstrip = (cpu.getCR(0) & 1) && cs_s >= 0x1000;
          if (datatrip || cstrip) {
            LOG_ERROR("EIP CORRUPTION: CS=", std::hex, cs_s,
                      " EIP=", eip_s, " linear=", linear,
                      " instrCount=", std::dec, instrCount);
            LOG_ERROR("  prevCS=", std::hex, prevCS,
                      " prevEIP=", prevEIP, " prevLinear=", prevLinear);
            // Disassemble the instruction at prevCS:prevEIP that caused the jump
            LOG_ERROR("  Jump from prevCS:prevEIP (last instr before corruption)");
            uint32_t prevPhys = prevLinear;
            LOG_ERROR("  Opcode bytes at prev: ",
                      std::hex, (int)memory.read8(prevPhys), " ",
                      (int)memory.read8(prevPhys+1), " ",
                      (int)memory.read8(prevPhys+2), " ",
                      (int)memory.read8(prevPhys+3), " ",
                      (int)memory.read8(prevPhys+4), " ",
                      (int)memory.read8(prevPhys+5));
            LOG_ERROR("EIP IN DATA AREA: CS=", std::hex, cs_s,
                      " EIP=", eip_s, " linear=", linear,
                      " instrCount=", std::dec, instrCount);
            LOG_ERROR("  EAX=", std::hex, cpu.getReg32(fador::cpu::EAX),
                      " EBX=", cpu.getReg32(fador::cpu::EBX),
                      " ECX=", cpu.getReg32(fador::cpu::ECX),
                      " EDX=", cpu.getReg32(fador::cpu::EDX));
            LOG_ERROR("  ESP=", cpu.getReg32(fador::cpu::ESP),
                      " EBP=", cpu.getReg32(fador::cpu::EBP),
                      " ESI=", cpu.getReg32(fador::cpu::ESI),
                      " EDI=", cpu.getReg32(fador::cpu::EDI));
            LOG_ERROR("  SS=", cpu.getSegReg(fador::cpu::SS),
                      " DS=", cpu.getSegReg(fador::cpu::DS),
                      " ES=", cpu.getSegReg(fador::cpu::ES),
                      " FS=", cpu.getSegReg(fador::cpu::FS),
                      " GS=", cpu.getSegReg(fador::cpu::GS));
            LOG_ERROR("  EFLAGS=", cpu.getEFLAGS(),
                      " CR0=", cpu.getCR(0));
            // Dump top of stack
            uint32_t ssB = cpu.getSegBase(fador::cpu::SS);
            uint32_t sp = cpu.is32BitStack()
                              ? cpu.getReg32(fador::cpu::ESP)
                              : static_cast<uint32_t>(cpu.getReg16(fador::cpu::SP));
            LOG_ERROR("  Stack top (SS:SP=", std::hex, cpu.getSegReg(fador::cpu::SS),
                      ":", sp, "):");
            for (int si = 0; si < 32; si += 4) {
              uint32_t v = memory.read32(ssB + sp + si);
              LOG_ERROR("    +", std::dec, si, ": 0x", std::hex, v);
            }
            running = false;
            break;
          }
        }

        if (stopAfterCycles > 0 && instrCount >= stopAfterCycles) {
          if (!handleStopAfter(instrCount, true)) {
            running = false;
            break;
          }
          continue;
        }

        if (dos.isTerminated()) {
          LOG_INFO("Program terminated normally with exit code ",
                   (int)dos.getExitCode(), " after ", instrCount,
                   " instructions");
          {
            for (int row = 0; row < 25; ++row) {
              std::string vramText;
              for (int col = 0; col < 80; ++col) {
                uint8_t ch = memory.read8(0xB8000 + (row * 80 + col) * 2);
                vramText += (ch >= 0x20 && ch < 0x7F) ? static_cast<char>(ch) : ' ';
              }
              while (!vramText.empty() && vramText.back() == ' ')
                vramText.pop_back();
              if (!vramText.empty())
                LOG_INFO("VRAM[", row, "]: ", vramText);
            }
          }
          if (dumpOnExit) {
            debugger.dumpState();
          }
          running = false;
          break;
        }

        if (sdlRenderer.shouldQuit()) {
          LOG_INFO("Window closed by user after ", instrCount, " instructions");
          running = false;
          break;
        }

        if ((instrCount & 0x3FF) == 0) {
          sdlRenderer.pollInput();
          pit.update();

          while (pit.checkPendingIRQ0()) {
            pic.raiseIRQ(0);
          }
          if (kbd.checkPendingIRQ()) {
            pic.raiseIRQ(1);
          }
          cmos.advanceTime();
          while (cmos.checkPendingIRQ8()) {
            picSlave.raiseIRQ(0);
          }

          dispatchPendingHardwareInterrupt(true);

          // Handle Ctrl+Alt+Del reset
          if (bios.shouldReset()) {
            LOG_INFO("Reset requested — terminating");
            running = false;
            break;
          }

          // Audio generation
          uint32_t queuedAudio = audio.getQueuedAudioSize();
          const uint32_t targetQueueBytes = 32768; // 4096 samples
          const uint32_t thresholdBytes = 16384;   // 2048 samples
          if (queuedAudio < thresholdBytes) {
            size_t samplesNeeded =
                (targetQueueBytes - queuedAudio) / 8; // 2 channels * 4 bytes
            if (samplesNeeded > 0) {
              audioBuf.assign(samplesNeeded * 2, 0.0f);
              adlib.generateSamples(audioBuf.data(), samplesNeeded);
              sb.generateSamples(audioBuf.data(), samplesNeeded);
              audio.queueSamples(audioBuf.data(), samplesNeeded * 2);
            }
          }

          static auto lastRender = std::chrono::steady_clock::now();
          auto now = std::chrono::steady_clock::now();

          if (std::chrono::duration_cast<std::chrono::milliseconds>(now -
                                                                    lastRender)
                  .count() > 33) {
            sdlRenderer.render();
            lastRender = now;
          }

          throttle.pace(instrCount);
        }
      }

      sdlRenderer.render(true);
    } else
#endif
    {
      // ── Terminal rendering path ─────────────────────────────
  InstructionThrottle throttle(throttleInstructionsPerSecond);
  throttle.reset();

      renderer.clearScreen();

      fador::ui::InputManager input(kbd);
      input.setBIOS(bios);
      bios.setInputPollCallback([&input]() { input.pollInput(); });
      bios.setIdleCallback([&renderer, &pit, &cpu, &decoder, &kbd, &pic,
                &picSlave, &cmos,
                &dispatchPendingHardwareInterrupt]() {
        pit.update();
        while (pit.checkPendingIRQ0()) {
          pic.raiseIRQ(0);
        }
        if (kbd.checkPendingIRQ()) {
          pic.raiseIRQ(1);
        }
        cmos.advanceTime();
        while (cmos.checkPendingIRQ8()) {
          picSlave.raiseIRQ(0);
        }
        dispatchPendingHardwareInterrupt(false);
        static auto lr = std::chrono::steady_clock::now();
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - lr)
                .count() > 33) {
          renderer.render();
          lr = now;
        }
      });
      dos.setKeyboard(kbd);
      dos.setInputPollCallback([&input]() { input.pollInput(); });
      dos.setIdleCallback([&pit, &cpu, &decoder, &kbd, &pic,
               &picSlave, &cmos,
               &dispatchPendingHardwareInterrupt]() {
        pit.update();
        while (pit.checkPendingIRQ0()) { pic.raiseIRQ(0); }
        if (kbd.checkPendingIRQ()) { pic.raiseIRQ(1); }
        cmos.advanceTime();
        while (cmos.checkPendingIRQ8()) { picSlave.raiseIRQ(0); }
        dispatchPendingHardwareInterrupt(false);
      });

      cpu.setEFLAGS(cpu.getEFLAGS() | fador::cpu::FLAG_INTERRUPT);

      bool running = true;
      uint64_t instrCount = 0;
      std::vector<float> audioBuf;

      while (running) {
        emitInstructionTrace();
        decoder.step();
        cpu.addCycles(1);
        instrCount++;

        if ((instrCount & 0x3FF) == 0) {
          pit.update();

          while (pit.checkPendingIRQ0()) {
            pic.raiseIRQ(0);
          }
          if (kbd.checkPendingIRQ()) {
            pic.raiseIRQ(1);
          }
          cmos.advanceTime();
          while (cmos.checkPendingIRQ8()) {
            picSlave.raiseIRQ(0);
          }

          dispatchPendingHardwareInterrupt(true);

          // Handle Ctrl+Alt+Del reset
          if (bios.shouldReset()) {
            LOG_INFO("Reset requested — terminating");
            running = false;
            break;
          }
        }

        if (stopAfterCycles > 0 && instrCount >= stopAfterCycles) {
          if (!handleStopAfter(instrCount, true)) {
            running = false;
            break;
          }
          continue;
        }

        if (dos.isTerminated()) {
          LOG_INFO("Program terminated normally with exit code ",
                   (int)dos.getExitCode(), " after ", instrCount,
                   " instructions");
          // Dump text-mode VGA VRAM to see any error messages
          {
            for (int row = 0; row < 25; ++row) {
              std::string vramText;
              for (int col = 0; col < 80; ++col) {
                uint8_t ch = memory.read8(0xB8000 + (row * 80 + col) * 2);
                vramText += (ch >= 0x20 && ch < 0x7F) ? static_cast<char>(ch) : ' ';
              }
              while (!vramText.empty() && vramText.back() == ' ')
                vramText.pop_back();
              if (!vramText.empty())
                LOG_INFO("VRAM[", row, "]: ", vramText);
            }
          }
          if (dumpOnExit) {
            debugger.dumpState();
          }
          running = false;
          break;
        }

        if ((instrCount & 0x3FF) == 0) {
          input.pollInput();
          pit.update();

          while (pit.checkPendingIRQ0()) {
            pic.raiseIRQ(0);
          }
          if (kbd.checkPendingIRQ()) {
            pic.raiseIRQ(1);
          }
          dispatchPendingHardwareInterrupt(true);

          static auto lastRender = std::chrono::steady_clock::now();
          auto now = std::chrono::steady_clock::now();

          // Audio generation
          uint32_t queuedAudio = audio.getQueuedAudioSize();
          const uint32_t targetQueueBytes = 32768; // 4096 samples
          const uint32_t thresholdBytes = 16384;   // 2048 samples
          if (queuedAudio < thresholdBytes) {
            size_t samplesNeeded =
                (targetQueueBytes - queuedAudio) / 8; // 2 channels * 4 bytes
            if (samplesNeeded > 0) {
              audioBuf.assign(samplesNeeded * 2, 0.0f);
              adlib.generateSamples(audioBuf.data(), samplesNeeded);
              sb.generateSamples(audioBuf.data(), samplesNeeded);
              audio.queueSamples(audioBuf.data(), samplesNeeded * 2);
              adlib.updateTimers((double)samplesNeeded / 44100.0);
            }
          }

          if (std::chrono::duration_cast<std::chrono::milliseconds>(now -
                                                                    lastRender)
                  .count() > 33) {
            renderer.render();
            lastRender = now;
          }

          throttle.pace(instrCount);
        }
      }

      renderer.render(true);
    }

  } catch (const std::exception &e) {
    LOG_ERROR("Fatal system error: ", e.what());
    return 1;
  }

  return 0;
}
