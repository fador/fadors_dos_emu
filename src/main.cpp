#include "cpu/Assembler.hpp"
#include "cpu/InstructionDecoder.hpp"
#include "hw/BIOS.hpp"
#include "hw/DMA8237.hpp"
#include "hw/DOS.hpp"
#include "hw/DPMI.hpp"
#include "hw/IOBus.hpp"
#include "hw/Joystick.hpp"
#include "hw/KeyboardController.hpp"
#include "hw/PIC8259.hpp"
#include "hw/PIT8254.hpp"
#include "hw/ProgramLoader.hpp"
#include "hw/VGAController.hpp"
#include "hw/audio/AdLib.hpp"
#include "hw/audio/AudioBackend.hpp"
#include "hw/audio/SoundBlaster.hpp"
#include "ui/Debugger.hpp"
#include "ui/InputManager.hpp"
#include "ui/TerminalRenderer.hpp"
#include "utils/CrashHandler.hpp"
#include "utils/Logger.hpp"
#include <iostream>
#ifdef HAVE_SDL2
#include "ui/SDLRenderer.hpp"
#endif

#include <chrono>
#include <cstdint>
#include <functional>
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

} // namespace

int main(int argc, char *argv[]) {
  try {
    fador::utils::currentLevel = fador::utils::LogLevel::Info;
    LOG_INFO("Fador's DOS Emulator Starting...");

    fador::memory::MemoryBus memory;
    fador::hw::IOBus iobus;
    fador::hw::PIC8259 pic(true); // Master
    fador::hw::KeyboardController kbd;
    fador::hw::Joystick joystick;
    fador::hw::PIT8254 pit;
    fador::hw::VGAController vga(memory);
    fador::hw::DMA8237 dma;
    fador::cpu::CPU cpu;

    // Connect VGA plane memory to the memory bus
    memory.setVGA(&vga);

    fador::hw::audio::AudioBackend audio;
    audio.init(44100, 2, 1024);

    fador::hw::audio::AdLib adlib(44100.0f, &cpu.getCyclesRef());
    fador::hw::audio::SoundBlaster sb(memory, dma, 44100.0f);

    // Register devices with IOBus
    iobus.registerDevice(0x20, 0x21, &pic);
    iobus.registerDevice(0x40, 0x43, &pit);
    iobus.registerDevice(0x60, 0x60, &kbd);
    iobus.registerDevice(0x64, 0x64, &kbd);
    iobus.registerDevice(0x201, 0x201, &joystick);
    iobus.registerDevice(0x3C0, 0x3CF, &vga);
    iobus.registerDevice(0x3D0, 0x3DF, &vga);
    iobus.registerDevice(0x00, 0x0F, &dma);
    iobus.registerDevice(0x81, 0x8F, &dma);
    iobus.registerDevice(0x220, 0x22F, &sb);
    iobus.registerDevice(0x388, 0x389, &adlib);

    kbd.setMemoryBus(&memory);
    fador::hw::DOS dos(cpu, memory);
    fador::hw::BIOS bios(cpu, memory, kbd, pit, pic);

    joystick.setCPU(&cpu);

    bios.initialize();
    dos.initialize();

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
    if (auto *himem = dos.getHIMEM())
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
    //   fadors_emu [--himem] [--debug=<cats>] <program.com|exe>
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
      } else if (arg == "--dump-on-exit") {
        dumpOnExit = true;
      } else if (arg.find("--stop-after=") == 0) {
        stopAfterCycles = std::stoull(arg.substr(13));
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

    if (!path.empty()) {
      bool loaded = false;
      if (path.find(".com") != std::string::npos ||
          path.find(".COM") != std::string::npos) {
        loaded = loader.loadCOM(path, dos.getPSPSegment(), args);
      } else {
        loaded = loader.loadEXE(path, dos.getPSPSegment(), dos, args, useHimem);
      }
      if (!loaded) {
        LOG_ERROR("Failed to load program: ", path);
        return 1;
      }
      dos.setProgramDir(path);
      decoder.syncSegments();
    } else {
      auto runExecLoop = [&](uint64_t steps) {
        for (uint64_t i = 0; i < steps; ++i) {
          decoder.step();
          cpu.addCycles(4);
          pit.addCycles(4);
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

      LOG_WARN(
          "No program specified. Use: fadors_emu [--himem] [--sdl|--no-sdl] "
          "[--debug=cpu,video,dos] [--stop-after=N] [--dump-on-exit] "
          "[--bench=decoder-loop|rep-movsb|rep-movsd] [--bench-steps=N] "
          "[--bench-warmup=N] [--exec=\"asm\"] <program.com|exe> "
          "[program-args...]");

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

      // Start debugger by default if no program
      debugger.run();
      return 0;
    }

#ifdef HAVE_SDL2
    if (useSDL) {
      // ── SDL2 graphical window path ──────────────────────────
      fador::ui::SDLRenderer sdlRenderer(memory, kbd, vga);
      sdlRenderer.setBIOS(bios);

      bios.setInputPollCallback([&sdlRenderer]() { sdlRenderer.pollInput(); });
      bios.setIdleCallback([&sdlRenderer, &pit, &cpu, &decoder, &kbd, &pic]() {
        cpu.addCycles(64);
        pit.addCycles(64);
        if (pit.checkPendingIRQ0()) {
          pic.raiseIRQ(0);
        }
        if (kbd.checkPendingIRQ()) {
          pic.raiseIRQ(1);
        }
        if (cpu.getEFLAGS() & fador::cpu::FLAG_INTERRUPT) {
          int pending = pic.getPendingInterrupt();
          if (pending != -1) {
            pic.acknowledgeInterrupt();
            decoder.injectHardwareInterrupt(static_cast<uint8_t>(pending));
          }
        }
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
      dos.setIdleCallback([&pit, &cpu, &decoder, &kbd, &pic]() {
        cpu.addCycles(64);
        pit.addCycles(64);
        if (pit.checkPendingIRQ0()) { pic.raiseIRQ(0); }
        if (kbd.checkPendingIRQ()) { pic.raiseIRQ(1); }
        if (cpu.getEFLAGS() & fador::cpu::FLAG_INTERRUPT) {
          int pending = pic.getPendingInterrupt();
          if (pending != -1) {
            pic.acknowledgeInterrupt();
            decoder.injectHardwareInterrupt(static_cast<uint8_t>(pending));
          }
        }
      });

      cpu.setEFLAGS(cpu.getEFLAGS() | fador::cpu::FLAG_INTERRUPT);

      bool running = true;
      uint64_t instrCount = 0;
      std::vector<float> audioBuf;

      while (running) {
        decoder.step();
        cpu.addCycles(4);
        pit.addCycles(4);
        instrCount++;

        // Periodic EIP sample for progress tracking
        if ((instrCount & 0xFFFFFF) == 0) {
          uint16_t cs_s = cpu.getSegReg(fador::cpu::CS);
          uint32_t eip_s = cpu.getEIP();
          LOG_INFO("SAMPLE @", instrCount, ": CS=", std::hex, cs_s, " EIP=", eip_s,
                   " flags=", cpu.getEFLAGS());
        }

        if (stopAfterCycles > 0 && instrCount >= stopAfterCycles) {
          // Trace next 50 instructions for diagnostic
          LOG_INFO("=== Instruction trace at stop point ===");
          for (int t = 0; t < 50; ++t) {
            uint32_t eip = cpu.getEIP();
            uint16_t cs_val = cpu.getSegReg(fador::cpu::CS);
            uint32_t csBase = cpu.getSegBase(fador::cpu::CS);
            uint32_t phys = csBase + eip;
            char buf[256];
            snprintf(buf, sizeof(buf),
              "TRACE %02d: CS=%04X EIP=%08X phys=%08X bytes=%02X %02X %02X %02X %02X %02X EAX=%08X",
              t, cs_val, eip, phys,
              memory.read8(phys), memory.read8(phys+1), memory.read8(phys+2),
              memory.read8(phys+3), memory.read8(phys+4), memory.read8(phys+5),
              cpu.getReg32(fador::cpu::EAX));
            LOG_INFO(buf);
            decoder.step();
            cpu.addCycles(4);
            pit.addCycles(4);
          }
          LOG_INFO("Stopped after ", instrCount, " cycles (--stop-after)");
          debugger.dumpState();
          for (int row = 0; row < 25; ++row) {
            std::string vramText;
            for (int col = 0; col < 80; ++col) {
              uint8_t ch = memory.read8(0xB8000 + (row * 80 + col) * 2);
              vramText += (ch >= 0x20 && ch < 0x7F) ? static_cast<char>(ch) : '.';
            }
            while (!vramText.empty() && vramText.back() == '.')
              vramText.pop_back();
            if (!vramText.empty())
              LOG_INFO("VRAM[", row, "]: ", vramText);
          }
          running = false;
          break;
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

          if (pit.checkPendingIRQ0()) {
            pic.raiseIRQ(0);
          }
          if (kbd.checkPendingIRQ()) {
            pic.raiseIRQ(1);
          }

          if (cpu.getEFLAGS() & fador::cpu::FLAG_INTERRUPT) {
            int pending = pic.getPendingInterrupt();
            if (pending != -1) {
              pic.acknowledgeInterrupt();
              decoder.injectHardwareInterrupt(static_cast<uint8_t>(pending));
            }
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
              adlib.updateTimers((double)samplesNeeded / 44100.0);
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
        }
      }

      sdlRenderer.render(true);
    } else
#endif
    {
      // ── Terminal rendering path ─────────────────────────────
      renderer.clearScreen();

      fador::ui::InputManager input(kbd);
      input.setBIOS(bios);
      bios.setInputPollCallback([&input]() { input.pollInput(); });
      bios.setIdleCallback([&renderer, &pit, &cpu, &decoder, &kbd, &pic]() {
        pit.addCycles(64);
        if (pit.checkPendingIRQ0()) {
          pic.raiseIRQ(0);
        }
        if (kbd.checkPendingIRQ()) {
          pic.raiseIRQ(1);
        }
        if (cpu.getEFLAGS() & fador::cpu::FLAG_INTERRUPT) {
          int pending = pic.getPendingInterrupt();
          if (pending != -1) {
            pic.acknowledgeInterrupt();
            decoder.injectHardwareInterrupt(static_cast<uint8_t>(pending));
          }
        }
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
      dos.setIdleCallback([&pit, &cpu, &decoder, &kbd, &pic]() {
        pit.addCycles(64);
        if (pit.checkPendingIRQ0()) { pic.raiseIRQ(0); }
        if (kbd.checkPendingIRQ()) { pic.raiseIRQ(1); }
        if (cpu.getEFLAGS() & fador::cpu::FLAG_INTERRUPT) {
          int pending = pic.getPendingInterrupt();
          if (pending != -1) {
            pic.acknowledgeInterrupt();
            decoder.injectHardwareInterrupt(static_cast<uint8_t>(pending));
          }
        }
      });

      cpu.setEFLAGS(cpu.getEFLAGS() | fador::cpu::FLAG_INTERRUPT);

      bool running = true;
      uint64_t instrCount = 0;
      std::vector<float> audioBuf;

      while (running) {
        decoder.step();
        cpu.addCycles(4);
        pit.addCycles(4);
        instrCount++;

        if ((instrCount & 0x3FF) == 0) {
          pit.update();

          if (pit.checkPendingIRQ0()) {
            pic.raiseIRQ(0);
          }
          if (kbd.checkPendingIRQ()) {
            pic.raiseIRQ(1);
          }

          if (cpu.getEFLAGS() & fador::cpu::FLAG_INTERRUPT) {
            int pending = pic.getPendingInterrupt();
            if (pending != -1) {
              pic.acknowledgeInterrupt();
              decoder.injectHardwareInterrupt(static_cast<uint8_t>(pending));
            }
          }
        }

        if (stopAfterCycles > 0 && instrCount >= stopAfterCycles) {
          LOG_INFO("Stopped after ", instrCount, " cycles (--stop-after)");
          debugger.dumpState();
          for (int row = 0; row < 25; ++row) {
            std::string vramText;
            for (int col = 0; col < 80; ++col) {
              uint8_t ch = memory.read8(0xB8000 + (row * 80 + col) * 2);
              vramText += (ch >= 0x20 && ch < 0x7F) ? static_cast<char>(ch) : '.';
            }
            while (!vramText.empty() && vramText.back() == '.')
              vramText.pop_back();
            if (!vramText.empty())
              LOG_INFO("VRAM[", row, "]: ", vramText);
          }
          running = false;
          break;
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

          if (pit.checkPendingIRQ0()) {
            pic.raiseIRQ(0);
          }
          if (kbd.checkPendingIRQ()) {
            pic.raiseIRQ(1);
          }
          if (cpu.getEFLAGS() & fador::cpu::FLAG_INTERRUPT) {
            int pending = pic.getPendingInterrupt();
            if (pending != -1) {
              pic.acknowledgeInterrupt();
              decoder.injectHardwareInterrupt(static_cast<uint8_t>(pending));
            }
          }

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
