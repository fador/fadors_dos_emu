#include "test_framework.hpp"
#include "hw/DOS.hpp"
#include "memory/MemoryBus.hpp"
#include "cpu/CPU.hpp"
#include <chrono>

using namespace fador;

TEST_CASE("DOS File Write Performance - Memory Access", "[perf]") {
    cpu::CPU cpu;
    memory::MemoryBus mem;
    hw::DOS dos(cpu, mem);

    uint32_t bufAddr = 0x10000;
    uint16_t bytesToWrite = 65000;

    // baseline implementation loop
    auto start = std::chrono::high_resolution_clock::now();
    for (int it = 0; it < 10000; ++it) {
        std::vector<char> buf(bytesToWrite);
        for (uint16_t i = 0; i < bytesToWrite; ++i) {
            buf[i] = mem.read8(bufAddr + i);
        }
        // just to keep the compiler from optimizing it out
        if (buf[0] == 'X') std::cout << "foo";
    }
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> duration = end - start;
    std::cout << "Baseline loop took: " << duration.count() << " ms" << std::endl;

    // optimized implementation loop
    auto start2 = std::chrono::high_resolution_clock::now();
    for (int it = 0; it < 10000; ++it) {
        // directAccess
        uint8_t* ptr = mem.directAccess(bufAddr);
        // stream.write(reinterpret_cast<char*>(ptr), bytesToWrite); // what would actually happen

        // mock to prevent optimization
        if (ptr && ptr[0] == 'X') std::cout << "foo";
    }
    auto end2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> duration2 = end2 - start2;
    std::cout << "Optimized loop took: " << duration2.count() << " ms" << std::endl;
}
