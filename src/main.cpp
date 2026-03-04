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
            LOG_WARN("No program specified. Use: fadors_emu [--himem] [--debug=cpu,video,dos] <program.com|exe> [program-args...]");
            // Start debugger by default if no program
            debugger.run();
            return 0;
        }

        renderer.clearScreen();

        fador::ui::InputManager input(kbd);
        bool running = true;
        uint32_t instrCount = 0;

        while (running) {
            decoder.step();
            instrCount++;

            if (dos.isTerminated()) {
                LOG_INFO("Program terminated normally with exit code ", (int)dos.getExitCode());
                running = false;
                break;
            }

            // Throttle expensive operations: poll input every ~1024 instructions,
            // render at ~30 FPS.  select() syscall in pollInput() is the
            // main bottleneck if called every instruction.
            if ((instrCount & 0x3FF) == 0) {
                input.pollInput();
                pit.update();

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
