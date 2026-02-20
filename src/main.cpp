#include <iostream>
#include "utils/Logger.hpp"
#include "cpu/InstructionDecoder.hpp"
#include "hw/IOBus.hpp"

int main(int argc, char** argv) {
    (void)argc;
    (void)argv;

    fador::utils::currentLevel = fador::utils::LogLevel::Debug;
    LOG_INFO("Fador's DOS Emulator starting...");

    try {
        fador::memory::MemoryBus memory;
        fador::hw::IOBus iobus;
        fador::cpu::CPU cpu;
        fador::cpu::InstructionDecoder decoder(cpu, memory, iobus);
        LOG_INFO("System initialized successfully.");

        // Setup environment to jump into address 0x100 (Common for COM files)
        cpu.setSegReg(fador::cpu::CS, 0);
        cpu.setSegReg(fador::cpu::SS, 0);
        cpu.setReg16(fador::cpu::SP, 0xFFF0); // Setup a basic 16-bit stack
        cpu.setEIP(0x100);

        // Write a sequence of instructions (16-bit code):
        // 0x100: E8 03 00 (CALL 0x106, rel16 = +3)
        // 0x103: EB 03    (JMP 0x108, rel8 = +3)
        // 0x105: 90       (NOP - padding)
        // 0x106: C3       (RET)
        // 0x107: 90       (NOP - padding)
        // 0x108: CC       (INT3 - Halt)
        
        memory.write8(0x100, 0xE8); memory.write16(0x101, 0x0003); // CALL +3
        memory.write8(0x103, 0xEB); memory.write8(0x104, 0x03);    // JMP +3
        memory.write8(0x105, 0x90);
        memory.write8(0x106, 0xC3);                                // RET
        memory.write8(0x107, 0x90);
        memory.write8(0x108, 0xCC);                                // INT3

        LOG_INFO("Executing 4 instructions...");
        for (int i=0; i<4; i++) {
            decoder.step();
            LOG_DEBUG("EIP after step: 0x", std::hex, cpu.getEIP());
        }
        
    } catch (const std::exception& e) {
        LOG_ERROR("Fatal system error: ", e.what());
        return 1;
    }

    return 0;
}
