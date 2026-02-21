#include "test_framework.hpp"
#include "hw/DOS.hpp"
#include "cpu/CPU.hpp"
#include "memory/MemoryBus.hpp"
#include <filesystem>
#include <fstream>

using namespace fador::hw;
using namespace fador::cpu;
using namespace fador::memory;
namespace fs = std::filesystem;

TEST_CASE("DOS: Directory Search", "[DOS][Search]") {
    CPU cpu;
    MemoryBus memory;
    DOS dos(cpu, memory);

    std::string testDir = "test_dos_search";
    if (fs::exists(testDir)) fs::remove_all(testDir);
    fs::create_directory(testDir);
    
    // Create some files
    {
        std::ofstream f1(testDir + "/file1.txt");
        std::ofstream f2(testDir + "/file2.com");
        std::ofstream f3(testDir + "/file3.bin");
    }

    auto originalDir = fs::current_path();
    fs::current_path(testDir);

    SECTION("FindFirst and FindNext") {
        // Set DTA to 0x1000:0x0000
        cpu.setSegReg(DS, 0x1000);
        cpu.setReg16(DX, 0x0000);
        cpu.setReg8(AH, 0x1A);
        dos.handleInterrupt(0x21);

        // FindFirst "*.com" (actually our HLE currently matches everything)
        std::string pattern = "*.*";
        for (size_t i = 0; i < pattern.length(); ++i) memory.write8(0x2000 + i, (uint8_t)pattern[i]);
        memory.write8(0x2000 + pattern.length(), 0);

        cpu.setReg8(AH, 0x4E);
        cpu.setSegReg(DS, 0x0000);
        cpu.setReg16(DX, 0x2000);
        cpu.setReg8(CL, 0x00); // Normal files
        dos.handleInterrupt(0x21);

        REQUIRE(!(cpu.getEFLAGS() & FLAG_CARRY));
        
        std::string firstMatch;
        for (int i = 0; i < 13; ++i) {
            char c = (char)memory.read8((0x1000 << 4) + 0x1E + i);
            if (c == 0) break;
            firstMatch += c;
        }
        REQUIRE(!firstMatch.empty());

        // FindNext
        cpu.setReg8(AH, 0x4F);
        dos.handleInterrupt(0x21);
        REQUIRE(!(cpu.getEFLAGS() & FLAG_CARRY));

        std::string secondMatch;
        for (int i = 0; i < 13; ++i) {
            char c = (char)memory.read8((0x1000 << 4) + 0x1E + i);
            if (c == 0) break;
            secondMatch += c;
        }
        REQUIRE(!secondMatch.empty());
        REQUIRE(secondMatch != firstMatch);

        // FindNext again
        cpu.setReg8(AH, 0x4F);
        dos.handleInterrupt(0x21);
        REQUIRE(!(cpu.getEFLAGS() & FLAG_CARRY));

        // FindNext (should fail now or after one more)
        cpu.setReg8(AH, 0x4F);
        dos.handleInterrupt(0x21);
        // We have 3 files, so 4th search should fail
        REQUIRE((cpu.getEFLAGS() & FLAG_CARRY));
    }

    fs::current_path(originalDir);
    fs::remove_all(testDir);
}
