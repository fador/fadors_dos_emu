#include "Debugger.hpp"
#include <iostream>
#include <sstream>
#include <iomanip>

namespace fador::ui {

Debugger::Debugger(cpu::CPU& cpu, memory::MemoryBus& memory, cpu::InstructionDecoder& decoder)
    : m_cpu(cpu), m_memory(memory), m_decoder(decoder) {
}

bool Debugger::run() {
    std::string line;
    std::cout << "\n[Fador Debugger] > " << std::flush;
    
    while (std::getline(std::cin, line)) {
        if (line.empty()) {
            std::cout << "[Fador Debugger] > ";
            continue;
        }

        auto args = split(line, ' ');
        std::string cmd = args[0];

        if (cmd == "q") return false;
        if (cmd == "c") return true; // Continue execution
        if (cmd == "s") {
            m_decoder.step();
            printRegisters();
        } else if (cmd == "r") {
            printRegisters();
        } else if (cmd == "d") {
            uint32_t addr = (args.size() > 1) ? std::stoul(args[1], nullptr, 16) : (m_cpu.getSegReg(cpu::DS) << 4);
            dumpMemory(addr, 128);
        } else if (cmd == "h" || cmd == "?") {
            std::cout << "Commands:\n"
                      << "  s          - Single step\n"
                      << "  c          - Continue execution\n"
                      << "  r          - Print registers\n"
                      << "  d [addr]   - Dump memory (hex)\n"
                      << "  q          - Quit emulator\n";
        } else {
            std::cout << "Unknown command: " << cmd << "\n";
        }

        std::cout << "\n[Fador Debugger] > " << std::flush;
    }
    return true;
}

void Debugger::printRegisters() {
    std::cout << std::hex << std::setfill('0')
              << "EAX: " << std::setw(8) << m_cpu.getReg32(cpu::EAX) << " "
              << "EBX: " << std::setw(8) << m_cpu.getReg32(cpu::EBX) << " "
              << "ECX: " << std::setw(8) << m_cpu.getReg32(cpu::ECX) << " "
              << "EDX: " << std::setw(8) << m_cpu.getReg32(cpu::EDX) << "\n"
              << "ESI: " << std::setw(8) << m_cpu.getReg32(cpu::ESI) << " "
              << "EDI: " << std::setw(8) << m_cpu.getReg32(cpu::EDI) << " "
              << "EBP: " << std::setw(8) << m_cpu.getReg32(cpu::EBP) << " "
              << "ESP: " << std::setw(8) << m_cpu.getReg32(cpu::ESP) << "\n"
              << "CS: " << std::setw(4) << m_cpu.getSegReg(cpu::CS) << " "
              << "DS: " << std::setw(4) << m_cpu.getSegReg(cpu::DS) << " "
              << "ES: " << std::setw(4) << m_cpu.getSegReg(cpu::ES) << " "
              << "SS: " << std::setw(4) << m_cpu.getSegReg(cpu::SS) << " "
              << "EIP: " << std::setw(8) << m_cpu.getEIP() << "\n"
              << "EFLAGS: " << std::setw(8) << m_cpu.getEFLAGS() << " ("
              << ((m_cpu.getEFLAGS() & cpu::FLAG_ZERO) ? "Z" : "-")
              << ((m_cpu.getEFLAGS() & cpu::FLAG_CARRY) ? "C" : "-")
              << ((m_cpu.getEFLAGS() & cpu::FLAG_SIGN) ? "S" : "-") << ")\n";
}

void Debugger::dumpMemory(uint32_t address, uint32_t count) {
    for (uint32_t i = 0; i < count; i += 16) {
        std::cout << std::hex << std::setw(8) << std::setfill('0') << (address + i) << ": ";
        for (uint32_t j = 0; j < 16; ++j) {
            std::cout << std::setw(2) << (int)m_memory.read8(address + i + j) << " ";
        }
        std::cout << " | ";
        for (uint32_t j = 0; j < 16; ++j) {
            char c = (char)m_memory.read8(address + i + j);
            std::cout << (isprint(c) ? c : '.');
        }
        std::cout << "\n";
    }
}

std::vector<std::string> Debugger::split(const std::string& s, char delimiter) {
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream tokenStream(s);
    while (std::getline(tokenStream, token, delimiter)) {
        if (!token.empty()) tokens.push_back(token);
    }
    return tokens;
}

} // namespace fador::ui
