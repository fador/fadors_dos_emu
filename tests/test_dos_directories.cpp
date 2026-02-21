#include <catch2/catch_test_macros.hpp>
#include "hw/DOS.hpp"
#include "cpu/CPU.hpp"
#include "memory/MemoryBus.hpp"
#include <filesystem>

using namespace fador::hw;
using namespace fador::cpu;
using namespace fador::memory;
namespace fs = std::filesystem;

TEST_CASE("DOS: Directory Operations", "[DOS][Directory]") {
    CPU cpu;
    MemoryBus memory;
    DOS dos(cpu, memory);

    std::string testDir = "test_dos_dirs";
    if (fs::exists(testDir)) fs::remove_all(testDir);
    fs::create_directory(testDir);
    
    // Set host working directory for the test scope
    auto originalDir = fs::current_path();
    fs::current_path(testDir);

    SECTION("MKDIR, CHDIR, GETCWD, RMDIR") {
        // MKDIR "newdir"
        std::string dirName = "newdir";
        for (size_t i = 0; i < dirName.length(); ++i) memory.write8(0x2000 + i, (uint8_t)dirName[i]);
        memory.write8(0x2000 + dirName.length(), 0);

        cpu.setReg8(AH, 0x39);
        cpu.setSegReg(DS, 0x0000);
        cpu.setReg16(DX, 0x2000);
        dos.handleInterrupt(0x21);
        REQUIRE(!(cpu.getEFLAGS() & FLAG_CARRY));
        REQUIRE(fs::exists("newdir"));
        REQUIRE(fs::is_directory("newdir"));

        // CHDIR "newdir"
        cpu.setReg8(AH, 0x3B);
        cpu.setReg16(DX, 0x2000);
        dos.handleInterrupt(0x21);
        REQUIRE(!(cpu.getEFLAGS() & FLAG_CARRY));

        // GETCWD
        cpu.setReg8(AH, 0x47);
        cpu.setReg16(SI, 0x3000);
        dos.handleInterrupt(0x21);
        REQUIRE(!(cpu.getEFLAGS() & FLAG_CARRY));
        
        std::string result;
        for (int i = 0; i < 255; ++i) {
            char c = (char)memory.read8(0x3000 + i);
            if (c == 0) break;
            result += c;
        }
        REQUIRE(result == "newdir");

        // RMDIR "newdir"
        // First go back to root
        memory.write8(0x2000, '.');
        memory.write8(0x2001, 0);
        cpu.setReg8(AH, 0x3B);
        cpu.setReg16(DX, 0x2000);
        dos.handleInterrupt(0x21);

        dirName = "newdir";
        for (size_t i = 0; i < dirName.length(); ++i) memory.write8(0x2000 + i, (uint8_t)dirName[i]);
        memory.write8(0x2000 + dirName.length(), 0);

        cpu.setReg8(AH, 0x3A);
        cpu.setReg16(DX, 0x2000);
        dos.handleInterrupt(0x21);
        REQUIRE(!(cpu.getEFLAGS() & FLAG_CARRY));
        REQUIRE(!fs::exists("newdir"));
    }

    // Cleanup
    fs::current_path(originalDir);
    fs::remove_all(testDir);
}
