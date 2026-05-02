#include "test_framework.hpp"
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

TEST_CASE("DOS: Path Traversal Prevention via CHDIR", "[DOS][Directory][Security]") {
    CPU cpu;
    MemoryBus memory;
    DOS dos(cpu, memory);

    std::string testDir = fs::weakly_canonical(fs::absolute("test_dos_dirs")).string();
    if (fs::exists(testDir)) fs::remove_all(testDir);
    fs::create_directory(testDir);

    // Set program dir which also sets m_hostRootDir internally
    dos.setProgramDir(testDir + "/fake_prog.exe");

    auto originalDir = fs::current_path();
    fs::current_path(testDir);

    SECTION("CHDIR traversal attempt via relative paths is blocked") {
        std::string attackPath = "../../../../../etc/passwd";
        for (size_t i = 0; i < attackPath.length(); ++i) memory.write8(0x2000 + i, attackPath[i]);
        memory.write8(0x2000 + attackPath.length(), 0);

        cpu.setReg8(AH, 0x3B);
        cpu.setSegReg(DS, 0x0000);
        cpu.setReg16(DX, 0x2000);

        dos.handleInterrupt(0x21);

        // Either CHDIR fails (carry set) or it clamps to root.
        // With our patch, resolvePath clamps to root, so it exists as a directory and succeeds,
        // keeping the current dir safely within root.

        // Check current DOS path
        cpu.setReg8(AH, 0x47);
        cpu.setReg16(SI, 0x3000);
        dos.handleInterrupt(0x21);

        std::string result;
        for (int i = 0; i < 255; ++i) {
            char c = (char)memory.read8(0x3000 + i);
            if (c == 0) break;
            result += c;
        }

        // It shouldn't contain etc or passwd. Should be root (empty in DOS terms or clamped to safe dir)
        REQUIRE(result.find("etc") == std::string::npos);
        REQUIRE(result.find("passwd") == std::string::npos);
    }

    fs::current_path(originalDir);
    fs::remove_all(testDir);
}
