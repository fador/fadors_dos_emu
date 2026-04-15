#include "cpu/CPU.hpp"
#include "cpu/InstructionDecoder.hpp"
#include "hw/BIOS.hpp"
#include "hw/DOS.hpp"
#include "hw/PIC8259.hpp"
#include "hw/PIT8254.hpp"
#include "hw/KeyboardController.hpp"
#include "memory/MemoryBus.hpp"
#include "hw/IOBus.hpp"
#include <iostream>

using namespace fador;

int main() {
    cpu::CPU cpu;
    memory::MemoryBus mem;
    hw::IOBus iobus;
    hw::KeyboardController kbd;
    hw::PIT8254 pit;
    hw::DOS dos(cpu, mem);
    dos.setKeyboard(kbd);
    hw::PIC8259 pic(true);
    hw::BIOS bios(cpu, mem, kbd, pit, pic);
    bios.initialize();
    dos.initialize();
    cpu::InstructionDecoder decoder(cpu, mem, iobus, bios, dos);

    cpu.setSegReg(cpu::CS, 0x1000);
    cpu.setSegBase(cpu::CS, 0x10000);
    cpu.setEIP(0x0100);

    // Write 0F FF 16 (HLE trap for INT 16h)
    mem.write8(0x10100, 0x0F);
    mem.write8(0x10101, 0xFF);
    mem.write8(0x10102, 0x16);

    // AH=00h (blocking read)
    cpu.setReg8(cpu::AH, 0x00);

    // No key in keyboard buffer
    decoder.step();

    std::cout << "EIP after first step (no key): 0x" << std::hex << cpu.getEIP() << std::endl;

    // Now push a key
    kbd.pushKey('A', 0x1E);

    decoder.step();

    std::cout << "EIP after second step (with key): 0x" << std::hex << cpu.getEIP() << std::endl;
    return 0;
}
