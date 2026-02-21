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

        // Register devices with IOBus
        iobus.registerDevice(0x20, 0x21, &pic);
        iobus.registerDevice(0x40, 0x43, &pit);
        iobus.registerDevice(0x60, 0x60, &kbd);
        iobus.registerDevice(0x64, 0x64, &kbd);
        fador::cpu::CPU cpu;
        fador::hw::DOS dos(cpu, memory);
        fador::hw::BIOS bios(cpu, memory, kbd, pit);
        
        bios.initialize();
        dos.initialize();
        
        fador::cpu::InstructionDecoder decoder(cpu, memory, iobus, bios, dos);
        fador::hw::ProgramLoader loader(cpu, memory);
        fador::ui::TerminalRenderer renderer(memory);
        fador::ui::InputManager input(kbd);
        fador::ui::Debugger debugger(cpu, memory, decoder);

        LOG_INFO("System initialized successfully.");

        // Load program if provided
        if (argc > 1) {
            std::string path = argv[1];
            bool loaded = false;
            if (path.find(".com") != std::string::npos || path.find(".COM") != std::string::npos) {
                loaded = loader.loadCOM(path, 0x1000);
            } else {
                loaded = loader.loadEXE(path, 0x1000);
            }
            if (!loaded) {
                LOG_ERROR("Failed to load program: ", path);
                return 1;
            }
        } else {
            LOG_WARN("No program specified. Use: fadors_emu <program.com|exe>");
            // Start debugger by default if no program
            debugger.run();
            return 0;
        }

        renderer.clearScreen();

        bool running = true;
        auto lastRender = std::chrono::steady_clock::now();

        while (running) {
            // Check for input
            input.pollInput();

            // Execute instruction
            decoder.step();

            if (dos.isTerminated()) {
                LOG_INFO("Program terminated normally with exit code ", (int)dos.getExitCode());
                running = false;
                break;
            }

            // Handle PIT ticks (approximate performance)
            pit.update(); 

            // Render at ~30 FPS
            auto now = std::chrono::steady_clock::now();
            if (std::chrono::duration_cast<std::chrono::milliseconds>(now - lastRender).count() > 33) {
                renderer.render();
                lastRender = now;
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
