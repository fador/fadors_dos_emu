#include <iostream>
#include "utils/Logger.hpp"
#include "cpu/InstructionDecoder.hpp"
#include "hw/IOBus.hpp"
#include "hw/BIOS.hpp"
#include "hw/DOS.hpp"
#include "hw/KeyboardController.hpp"
#include "hw/PIC8259.hpp"
#include "hw/PIT8254.hpp"
#include "hw/ProgramLoader.hpp"
#include "hw/VGAController.hpp"
#include "ui/TerminalRenderer.hpp"
#include "ui/InputManager.hpp"
#include "ui/Debugger.hpp"
#include "cpu/Assembler.hpp"

#include <chrono>
#include <thread>

int main(int argc, char* argv[]) {
    try {
        fador::utils::currentLevel = fador::utils::LogLevel::Info;
        LOG_INFO("Fador's DOS Emulator Starting...");

        fador::memory::MemoryBus memory;
        fador::hw::IOBus iobus;
        fador::hw::PIC8259 pic(true); // Master
        fador::hw::KeyboardController kbd;
        fador::hw::PIT8254 pit;
        fador::hw::VGAController vga(memory);

        // Register devices with IOBus
        iobus.registerDevice(0x20, 0x21, &pic);
        iobus.registerDevice(0x40, 0x43, &pit);
        iobus.registerDevice(0x60, 0x60, &kbd);
        iobus.registerDevice(0x64, 0x64, &kbd);
        iobus.registerDevice(0x3C0, 0x3CF, &vga);
        iobus.registerDevice(0x3D0, 0x3DF, &vga);
        kbd.setMemoryBus(&memory);
        fador::cpu::CPU cpu;
        fador::hw::DOS dos(cpu, memory);
        fador::hw::BIOS bios(cpu, memory, kbd, pit);
        
        bios.initialize();
        dos.initialize();
        
        fador::cpu::InstructionDecoder decoder(cpu, memory, iobus, bios, dos);
        fador::hw::ProgramLoader loader(cpu, memory);
        fador::ui::TerminalRenderer renderer(memory);
        fador::ui::Debugger debugger(cpu, memory, decoder);

        LOG_INFO("System initialized successfully.");

        // Parse command line:
        //   fadors_emu [--himem] [--debug=<cats>] <program.com|exe> [program-args...]
        // Emulator flags (--himem, --debug=) are consumed only before the program path.
        // Everything after the program path is forwarded verbatim to the DOS program,
        // so program arguments that look like flags (e.g. -? /h) are passed through
        // unchanged.  In shells that glob-expand bare '?' (zsh, bash with failglob),
        // quote such arguments: fadors_emu prog.exe '-?' or fadors_emu prog.exe -- -?
        bool useHimem = false;
        bool dumpOnExit = false;
        uint64_t stopAfterCycles = 0; // 0 = disabled
        std::string execAsm;  // --exec="asm instructions"
        std::string path;
        std::string args;
        for (int i = 1; i < argc; ++i) {
            std::string arg = argv[i];

            if (!path.empty()) {
                // Program path already found – everything else belongs to the program.
                // '--' after the path is a conventional shell separator; discard it.
                if (arg == "--") continue;
                if (!args.empty()) args += ' ';
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
            } else if (arg.find("--debug=") == 0) {
                fador::utils::currentLevel = fador::utils::LogLevel::Debug;
                const std::string cats = arg.substr(8);
                if (cats.find("cpu")   != std::string::npos) fador::utils::enabledCategories |= fador::utils::CAT_CPU;
                if (cats.find("video") != std::string::npos) fador::utils::enabledCategories |= fador::utils::CAT_VIDEO;
                if (cats.find("dos")   != std::string::npos) fador::utils::enabledCategories |= fador::utils::CAT_DOS;
            } else if (arg == "--") {
                // Explicit end-of-emulator-flags sentinel; next arg is the program path.
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
            if (path.find(".com") != std::string::npos || path.find(".COM") != std::string::npos) {
                loaded = loader.loadCOM(path, 0x1000, args);
            } else {
                loaded = loader.loadEXE(path, 0x1000, dos, args, useHimem);
            }
            if (!loaded) {
                LOG_ERROR("Failed to load program: ", path);
                return 1;
            }
            dos.setProgramDir(path);
        } else {
            LOG_WARN("No program specified. Use: fadors_emu [--himem] [--debug=cpu,video,dos] [--stop-after=N] [--dump-on-exit] [--exec=\"asm\"] <program.com|exe> [program-args...]");
            
            // --exec mode: assemble, write to CS:IP, run, dump state
            if (!execAsm.empty()) {
                // Replace semicolons with newlines for multi-statement support
                std::string asmText = execAsm;
                for (auto& ch : asmText) { if (ch == ';') ch = '\n'; }
                
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
                LOG_INFO("Assembled ", result.bytes.size(), " bytes at ", std::hex, origin);
                
                uint64_t maxCycles = stopAfterCycles > 0 ? stopAfterCycles : 100000;
                for (uint64_t i = 0; i < maxCycles; ++i) {
                    decoder.step();
                    if (dos.isTerminated()) break;
                }
                debugger.dumpState();
                return 0;
            }
            
            // Start debugger by default if no program
            debugger.run();
            return 0;
        }

        renderer.clearScreen();

        fador::ui::InputManager input(kbd);
        input.setBIOS(bios);
        bios.setInputPollCallback([&input]() { input.pollInput(); });
        bios.setIdleCallback([&renderer, &pit, &cpu, &decoder, &kbd]() {
            pit.update();
            if ((cpu.getEFLAGS() & fador::cpu::FLAG_INTERRUPT) && pit.checkPendingIRQ0()) {
                decoder.injectHardwareInterrupt(0x08);
            }
            if ((cpu.getEFLAGS() & fador::cpu::FLAG_INTERRUPT) && kbd.checkPendingIRQ()) {
                decoder.injectHardwareInterrupt(0x09);
            }
            static auto lr = std::chrono::steady_clock::now();
            auto now = std::chrono::steady_clock::now();
            if (std::chrono::duration_cast<std::chrono::milliseconds>(now - lr).count() > 33) {
                renderer.render();
                lr = now;
            }
        });
        dos.setKeyboard(kbd);
        dos.setInputPollCallback([&input]() { input.pollInput(); });

        // Enable hardware interrupts — real BIOS does STI before running programs
        cpu.setEFLAGS(cpu.getEFLAGS() | fador::cpu::FLAG_INTERRUPT);

        bool running = true;
        uint64_t instrCount = 0;

        while (running) {
            decoder.step();
            instrCount++;

            if (stopAfterCycles > 0 && instrCount >= stopAfterCycles) {
                LOG_INFO("Stopped after ", instrCount, " cycles (--stop-after)");
                debugger.dumpState();
                running = false;
                break;
            }

            if (dos.isTerminated()) {
                LOG_INFO("Program terminated normally with exit code ", (int)dos.getExitCode(),
                         " after ", instrCount, " instructions");
                if (dumpOnExit) {
                    debugger.dumpState();
                }
                running = false;
                break;
            }

            // Throttle expensive operations: poll input every ~1024 instructions,
            // render at ~30 FPS.  select() syscall in pollInput() is the
            // main bottleneck if called every instruction.
            if ((instrCount & 0x3FF) == 0) {
                input.pollInput();

                pit.update();

                // Service PIT channel 0 (IRQ0 → INT 8 → timer tick)
                if ((cpu.getEFLAGS() & fador::cpu::FLAG_INTERRUPT) && pit.checkPendingIRQ0()) {
                    decoder.injectHardwareInterrupt(0x08);
                }

                // Service keyboard IRQ1 → INT 9
                if ((cpu.getEFLAGS() & fador::cpu::FLAG_INTERRUPT) && kbd.checkPendingIRQ()) {
                    decoder.injectHardwareInterrupt(0x09);
                }

                static auto lastRender = std::chrono::steady_clock::now();
                auto now = std::chrono::steady_clock::now();
                if (std::chrono::duration_cast<std::chrono::milliseconds>(now - lastRender).count() > 33) {
                    renderer.render();
                    lastRender = now;
                }
            }

            // In a real implementation we'd check for termination signals
            // For now, let's keep running or check for specific HALT if implemented
        }

        // Final render to show any output before exit
        renderer.render(true);

    } catch (const std::exception& e) {
        LOG_ERROR("Fatal system error: ", e.what());
        return 1;
    }

    return 0;
}
