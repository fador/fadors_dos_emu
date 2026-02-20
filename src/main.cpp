#include <iostream>
#include "utils/Logger.hpp"
#include "cpu/InstructionDecoder.hpp"

int main(int argc, char** argv) {
    (void)argc;
    (void)argv;

    fador::utils::currentLevel = fador::utils::LogLevel::Debug;
    LOG_INFO("Fador's DOS Emulator starting...");

    try {
        fador::memory::MemoryBus memory;
        fador::cpu::CPU cpu;
        fador::cpu::InstructionDecoder decoder(cpu, memory);
        LOG_INFO("System initialized successfully.");

        // Setup environment to jump into address 0x100 (Common for COM files)
        cpu.setSegReg(fador::cpu::CS, 0);
        cpu.setEIP(0x100);

        // Write a sequence of basic instructions: 0x90 (NOP), 0x90 (NOP), 0xCC (INT3)
        memory.write8(0x100, 0x90);
        memory.write8(0x101, 0x90);
        memory.write8(0x102, 0xCC);

        LOG_INFO("Executing 3 instructions...");
        for (int i=0; i<3; i++) {
            decoder.step();
            LOG_DEBUG("EIP after step: 0x", std::hex, cpu.getEIP());
        }
        
    } catch (const std::exception& e) {
        LOG_ERROR("Fatal system error: ", e.what());
        return 1;
    }

    return 0;
}
