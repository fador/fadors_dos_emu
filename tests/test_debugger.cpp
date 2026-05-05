#include "test_framework.hpp"
#include "cpu/CPU.hpp"
#include "cpu/InstructionDecoder.hpp"
#include "hw/BIOS.hpp"
#include "hw/DOS.hpp"
#include "hw/IOBus.hpp"
#include "hw/KeyboardController.hpp"
#include "hw/PIC8259.hpp"
#include "hw/PIT8254.hpp"
#include "memory/MemoryBus.hpp"
#include "ui/Debugger.hpp"
#include <sstream>
#include <vector>

using namespace fador;

namespace {

struct DebuggerTestEnv {
    cpu::CPU cpu;
    memory::MemoryBus mem;
    hw::IOBus iobus;
    hw::KeyboardController kbd;
    hw::PIT8254 pit;
    hw::PIC8259 pic{true};
    hw::DOS dos;
    hw::BIOS bios;
    cpu::InstructionDecoder decoder;
    ui::Debugger debugger;

    static constexpr uint16_t CODE_SEG = 0x1000;
    static constexpr uint16_t CODE_OFF = 0x0100;
    static constexpr uint32_t CODE_LINEAR = (static_cast<uint32_t>(CODE_SEG) << 4) + CODE_OFF;

    DebuggerTestEnv()
        : dos(cpu, mem),
          bios(cpu, mem, kbd, pit, pic),
          decoder(cpu, mem, iobus, bios, dos),
          debugger(cpu, mem, decoder) {
        bios.initialize();
        dos.initialize();
        dos.setPSPSegment(CODE_SEG);
        decoder.loadSegment(cpu::CS, CODE_SEG);
        decoder.loadSegment(cpu::DS, CODE_SEG);
        decoder.loadSegment(cpu::ES, CODE_SEG);
        decoder.loadSegment(cpu::SS, 0x0000);
        cpu.setReg16(cpu::SP, 0xFFFE);
        cpu.setEIP(CODE_OFF);
    }

    void writeBytes(const std::vector<uint8_t>& bytes) {
        for (size_t i = 0; i < bytes.size(); ++i) {
            mem.write8(CODE_LINEAR + static_cast<uint32_t>(i), bytes[i]);
        }
    }
};

} // namespace

TEST_CASE("Debugger - Current trace line shows bytes mnemonic and registers",
          "[Debugger]") {
    DebuggerTestEnv env;
    env.writeBytes({0xB8, 0x34, 0x12, 0x04, 0x05});

    const auto line = env.debugger.currentTraceLine();

    REQUIRE(line.find("CS:EIP=1000:00000100") != std::string::npos);
    REQUIRE(line.find("B8 34 12") != std::string::npos);
    REQUIRE(line.find("MOV AX, 1234h") != std::string::npos);
    REQUIRE(line.find("EAX=00000000") != std::string::npos);
}

TEST_CASE("Debugger - Trace stepping can start now or at a selected address",
          "[Debugger]") {
    DebuggerTestEnv env;
    env.writeBytes({0xB8, 0x34, 0x12, 0x04, 0x05, 0x90});

    SECTION("traceInstructions steps and prints one line per instruction") {
        std::ostringstream out;
        env.debugger.traceInstructions(2, out);
        const auto text = out.str();

        REQUIRE(text.find("MOV AX, 1234h") != std::string::npos);
        REQUIRE(text.find("ADD AL, 05h") != std::string::npos);
        REQUIRE(text.find("EAX=00001234") != std::string::npos);
    }

    SECTION("emitTraceIfRequested waits until the selected linear address") {
        ui::Debugger::TraceRequest request{1, DebuggerTestEnv::CODE_LINEAR + 3};
        std::ostringstream out;

        REQUIRE(!env.debugger.emitTraceIfRequested(request, out));
        REQUIRE(out.str().empty());

        env.decoder.step();

        REQUIRE(env.debugger.emitTraceIfRequested(request, out));
        REQUIRE(request.remaining == 0);
        REQUIRE(out.str().find("ADD AL, 05h") != std::string::npos);
        REQUIRE(out.str().find("EAX=00001234") != std::string::npos);
    }
}