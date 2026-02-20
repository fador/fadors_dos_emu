#include <iostream>
#include "utils/Logger.hpp"
#include "memory/MemoryBus.hpp"

int main(int argc, char** argv) {
    (void)argc;
    (void)argv;

    fador::utils::currentLevel = fador::utils::LogLevel::Debug;
    LOG_INFO("Fador's DOS Emulator starting...");

    try {
        fador::memory::MemoryBus memory;
        LOG_INFO("System initialized successfully.");

        memory.write16(0x1000, 0xABCD);
        uint16_t val = memory.read16(0x1000);
        LOG_DEBUG("Memory Test: read16(0x1000) = 0x", std::hex, val);
    } catch (const std::exception& e) {
        LOG_ERROR("Fatal system error: ", e.what());
        return 1;
    }

    return 0;
}
