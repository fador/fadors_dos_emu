#include "Debugger.hpp"
#include <iostream>
#include <sstream>
#include <iomanip>
#ifndef _WIN32
#include <termios.h>
#include <unistd.h>
#endif

namespace fador::ui {

// RAII guard: if the terminal is in raw mode, temporarily restore canonical
// (cooked) mode so std::getline / echo work.  Restores on destruction.
struct CookedModeGuard {
#ifndef _WIN32
    struct termios saved{};
    bool active = false;
    CookedModeGuard() {
        if (tcgetattr(STDIN_FILENO, &saved) == 0 && !(saved.c_lflag & ICANON)) {
            struct termios cooked = saved;
            cooked.c_iflag |= ICRNL;
            cooked.c_lflag |= ICANON | ECHO;
            cooked.c_cc[VMIN]  = 1;
            cooked.c_cc[VTIME] = 0;
            tcsetattr(STDIN_FILENO, TCSAFLUSH, &cooked);
            active = true;
        }
    }
    ~CookedModeGuard() {
        if (active) tcsetattr(STDIN_FILENO, TCSAFLUSH, &saved);
    }
#endif
};

Debugger::Debugger(cpu::CPU& cpu, memory::MemoryBus& memory, cpu::InstructionDecoder& decoder)
    : m_cpu(cpu), m_memory(memory), m_decoder(decoder), m_disasm(memory) {
}

bool Debugger::run() {
    CookedModeGuard cookedGuard;
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
        } else if (cmd == "u") {
            uint32_t addr = (args.size() > 1) ? std::stoul(args[1], nullptr, 16)
                            : ((m_cpu.getSegReg(cpu::CS) << 4) + m_cpu.getEIP());
            uint32_t cnt = (args.size() > 2) ? std::stoul(args[2]) : 16;
            disassemble(addr, cnt);
        } else if (cmd == "h" || cmd == "?") {
            std::cout << "Commands:\n"
                      << "  s          - Single step\n"
                      << "  c          - Continue execution\n"
                      << "  r          - Print registers\n"
                      << "  d [addr]   - Dump memory (hex)\n"
                      << "  u [addr] [n] - Disassemble n instructions\n"
                      << "  a [addr]   - Assemble (enter instructions, blank line to exit)\n"
                      << "  w addr bytes - Write hex bytes to memory\n"
                      << "  q          - Quit emulator\n";
        } else if (cmd == "a") {
            uint32_t addr = (args.size() > 1) ? std::stoul(args[1], nullptr, 16)
                            : ((m_cpu.getSegReg(cpu::CS) << 4) + m_cpu.getEIP());
            assembleMode(addr);
        } else if (cmd == "w") {
            if (args.size() >= 3) {
                uint32_t addr = std::stoul(args[1], nullptr, 16);
                for (size_t idx = 2; idx < args.size(); ++idx) {
                    m_memory.write8(addr++, static_cast<uint8_t>(std::stoul(args[idx], nullptr, 16)));
                }
                std::cout << "Wrote " << (args.size() - 2) << " byte(s)\n";
            } else {
                std::cout << "Usage: w <addr> <byte1> [byte2] ...\n";
            }
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

void Debugger::disassemble(uint32_t address, uint32_t count) {
    auto instrs = m_disasm.disassembleRange(address, count);
    for (const auto& instr : instrs) {
        std::cout << std::hex << std::setw(8) << std::setfill('0') << instr.address
                  << "  " << std::left << std::setw(24) << std::setfill(' ') << instr.hexBytes
                  << instr.mnemonic << std::right << "\n";
    }
}

void Debugger::dumpState(uint32_t contextLines) {
    std::cout << "\n=== CPU State Dump ==="  << std::endl;
    printRegisters();
    std::cout << "\n--- Stack (SS:SP, 32 bytes) ---" << std::endl;
    uint32_t stackAddr = (m_cpu.getSegReg(cpu::SS) << 4) + (m_cpu.getReg16(cpu::SP));
    dumpMemory(stackAddr, 32);
    std::cout << "\n--- Disassembly around CS:EIP ---" << std::endl;
    uint32_t linearIP = (m_cpu.getSegReg(cpu::CS) << 4) + m_cpu.getEIP();
    auto instrs = m_disasm.disassembleAround(linearIP, contextLines, contextLines + 1);
    for (const auto& instr : instrs) {
        const char* marker = (instr.address == linearIP) ? "-->" : "   ";
        std::cout << marker << " " << std::hex << std::setw(8) << std::setfill('0')
                  << instr.address << "  "
                  << std::left << std::setw(24) << std::setfill(' ') << instr.hexBytes
                  << instr.mnemonic << std::right << "\n";
    }
    std::cout << "=== End Dump ===\n" << std::endl;
}

void Debugger::assembleMode(uint32_t startAddr) {
    CookedModeGuard cookedGuard;
    uint32_t addr = startAddr;
    std::cout << "Entering assembly mode (blank line or '.' to exit)\n";
    std::string asmLine;
    while (true) {
        std::cout << std::hex << std::setw(8) << std::setfill('0') << addr << "  " << std::flush;
        if (!std::getline(std::cin, asmLine) || asmLine.empty() || asmLine == ".") break;
        uint32_t written = assembleAndWrite(asmLine, addr);
        if (written > 0) {
            // Show what was assembled
            auto instrs = m_disasm.disassembleRange(addr, 1);
            for (const auto& instr : instrs) {
                std::cout << "          " << std::left << std::setw(24) << std::setfill(' ')
                          << instr.hexBytes << instr.mnemonic << std::right << "\n";
            }
            addr += written;
        }
    }
}

uint32_t Debugger::assembleAndWrite(const std::string& asmLine, uint32_t address) {
    auto result = m_asm.assembleLine(asmLine, address);
    if (!result.error.empty()) {
        std::cout << "Error: " << result.error << "\n";
        return 0;
    }
    for (size_t i = 0; i < result.bytes.size(); ++i) {
        m_memory.write8(address + static_cast<uint32_t>(i), result.bytes[i]);
    }
    return static_cast<uint32_t>(result.bytes.size());
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
