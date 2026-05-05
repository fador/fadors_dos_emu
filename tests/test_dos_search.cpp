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

    SECTION("FindFirst honors non-zero DTA offset") {
        constexpr uint16_t dtaSegment = 0x0843;
        constexpr uint16_t dtaOffset = 0x8430;
        constexpr uint32_t dtaAddr = (static_cast<uint32_t>(dtaSegment) << 4) +
                                     dtaOffset;

        cpu.setSegReg(DS, dtaSegment);
        cpu.setReg16(DX, dtaOffset);
        cpu.setReg8(AH, 0x1A);
        dos.handleInterrupt(0x21);

        std::string pattern = "ARROW.*";
        for (size_t i = 0; i < pattern.length(); ++i) {
            memory.write8(0x2000 + static_cast<uint32_t>(i),
                          static_cast<uint8_t>(pattern[i]));
        }
        memory.write8(0x2000 + static_cast<uint32_t>(pattern.length()), 0);

        cpu.setReg8(AH, 0x4E);
        cpu.setSegReg(DS, 0x0000);
        cpu.setReg16(DX, 0x2000);
        cpu.setReg8(CL, 0x00);
        dos.handleInterrupt(0x21);

        REQUIRE(!(cpu.getEFLAGS() & FLAG_CARRY));

        std::string match;
        for (int i = 0; i < 13; ++i) {
            char c = static_cast<char>(memory.read8(dtaAddr + 0x1E + i));
            if (c == 0) break;
            match += c;
        }

        REQUIRE(match == "ARROW.SHP");
        REQUIRE(memory.read8(dtaAddr + 0x15) == 0x20);
        REQUIRE(memory.read32(dtaAddr + 0x1A) > 0);
    }

    fs::current_path(originalDir);
    fs::remove_all(testDir);
}

TEST_CASE("DOS: Path Traversal Prevention via Find First/Next", "[DOS][Search][Security]") {
    CPU cpu;
    MemoryBus memory;
    DOS dos(cpu, memory);

    std::string testDir = fs::weakly_canonical(fs::absolute("test_dos_search")).string();
    if (fs::exists(testDir)) fs::remove_all(testDir);
    fs::create_directory(testDir);

    // Set program dir which also sets m_hostRootDir internally
    dos.setProgramDir(testDir + "/fake_prog.exe");

    auto originalDir = fs::current_path();
    fs::current_path(testDir);

    SECTION("Find First via traversal attempt is blocked") {
        std::string attackPath = "../../../../../etc/passwd";
        for (size_t i = 0; i < attackPath.length(); ++i) memory.write8(0x2000 + i, attackPath[i]);
        memory.write8(0x2000 + attackPath.length(), 0);

        // Set DTA
        cpu.setReg8(AH, 0x1A);
        cpu.setSegReg(DS, 0x1000);
        cpu.setReg16(DX, 0x0000);
        dos.handleInterrupt(0x21);

        // Find First
        cpu.setReg8(AH, 0x4E);
        cpu.setSegReg(DS, 0x0000);
        cpu.setReg16(DX, 0x2000);
        cpu.setReg8(CL, 0); // Normal files

        dos.handleInterrupt(0x21);

        // This should fail to find "/etc/passwd" because it got clamped to testDir
        // Note: Unless testDir has a file literally called passwd or something weird, it should set carry
        // or return safe data. Either way, it shouldn't successfully read /etc/passwd attributes.
        REQUIRE((cpu.getEFLAGS() & FLAG_CARRY) != 0);
        REQUIRE(cpu.getReg16(AX) == 0x02); // File not found or 0x03 Path not found
    }

    fs::current_path(originalDir);
    fs::remove_all(testDir);
}
