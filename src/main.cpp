#include <iostream>
#include "utils/Logger.hpp"
#include "memory/MemoryBus.hpp"
#include "cpu/CPU.hpp"

int main(int argc, char** argv) {
    (void)argc;
    (void)argv;

    fador::utils::currentLevel = fador::utils::LogLevel::Debug;
    LOG_INFO("Fador's DOS Emulator starting...");

    try {
        fador::memory::MemoryBus memory;
        fador::cpu::CPU cpu;
        LOG_INFO("System initialized successfully.");

        memory.write16(0x1000, 0xABCD);
        uint16_t val = memory.read16(0x1000);
        LOG_DEBUG("Memory Test: read16(0x1000) = 0x", std::hex, val);

        cpu.setReg16(fador::cpu::AX, 0x1234);
        LOG_DEBUG("CPU Test: AX = 0x", std::hex, cpu.getReg16(fador::cpu::AX));

    } catch (const std::exception& e) {
        LOG_ERROR("Fatal system error: ", e.what());
        return 1;
    }

    return 0;
}
