#include "test_framework.hpp"
#include "hw/DriveManager.hpp"
#include "crc32.hpp"
#include "zip_reader.hpp"

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <span>
#include <string>
#include <vector>

using namespace fador::hw;

namespace {

std::string makeTempDir(const std::string& name) {
    auto base = std::filesystem::temp_directory_path() / "fador_test" / name;
    std::filesystem::create_directories(base);
    return base.string();
}

void removeTempDir(const std::string& path) {
    std::error_code ec;
    std::filesystem::remove_all(path, ec);
}

struct TempDirGuard {
    std::string path;
    explicit TempDirGuard(const std::string& name) : path(makeTempDir(name)) {}
    ~TempDirGuard() { removeTempDir(path); }
};

} // namespace

TEST_CASE("DriveManager: Mount and Unmount", "[drive_manager]") {
    TempDirGuard guardC("mount_c");
    DriveManager dm;

    SECTION("Mount returns true and host path is stored") {
        REQUIRE(dm.mount('C', guardC.path));
        REQUIRE(dm.isMounted('C'));
        REQUIRE(!dm.getHostPath('C').empty());
    }

    SECTION("Mounted count increases") {
        int before = dm.getMountedCount();
        dm.mount('C', guardC.path);
        REQUIRE(dm.getMountedCount() == before + 1);
    }

    SECTION("Unmount clears mount") {
        dm.mount('C', guardC.path);
        REQUIRE(dm.unmount('C'));
        REQUIRE(!dm.isMounted('C'));
        REQUIRE(dm.getHostPath('C').empty());
    }

    SECTION("Z: cannot be unmounted") {
        TempDirGuard guardZ("mount_z");
        REQUIRE(dm.mount('Z', guardZ.path));
        REQUIRE(!dm.unmount('Z'));
        REQUIRE(dm.isMounted('Z'));
    }

    SECTION("Unmount non-mounted drive returns false") {
        REQUIRE(!dm.unmount('X'));
    }
}

TEST_CASE("DriveManager: Mount Duplicate", "[drive_manager]") {
    TempDirGuard guard("dup_mount");
    DriveManager dm;

    REQUIRE(dm.mount('C', guard.path));
    REQUIRE(!dm.mount('C', guard.path));
    REQUIRE(dm.getMountedCount() == 1);
}

TEST_CASE("DriveManager: Mount Invalid Path", "[drive_manager]") {
    DriveManager dm;

    SECTION("Non-existent path fails") {
        REQUIRE(!dm.mount('C', "/nonexistent_path_abc123"));
        REQUIRE(!dm.isMounted('C'));
    }

    SECTION("Mounting same letter twice with different paths fails") {
        TempDirGuard guard1("mount_inv1");
        TempDirGuard guard2("mount_inv2");
        REQUIRE(dm.mount('D', guard1.path));
        REQUIRE(!dm.mount('D', guard2.path));
    }
}

TEST_CASE("DriveManager: Default Drive", "[drive_manager]") {
    DriveManager dm;

    SECTION("Default current drive is Z:") {
        REQUIRE(dm.getCurrentDriveIndex() == 25);
        REQUIRE(dm.getCurrentDriveLetter() == 'Z');
    }

    SECTION("Change to C:") {
        dm.setCurrentDrive('C');
        REQUIRE(dm.getCurrentDriveIndex() == 2);
        REQUIRE(dm.getCurrentDriveLetter() == 'C');
    }

    SECTION("Change by index") {
        dm.setCurrentDrive(static_cast<uint8_t>(3));
        REQUIRE(dm.getCurrentDriveIndex() == 3);
        REQUIRE(dm.getCurrentDriveLetter() == 'D');
    }
}

TEST_CASE("DriveManager: Per-Drive Current Directory", "[drive_manager]") {
    TempDirGuard guardC("cwd_c");
    TempDirGuard guardD("cwd_d");
    std::filesystem::create_directories(std::filesystem::path(guardC.path) / "GAMES" / "DOOM");
    std::filesystem::create_directories(std::filesystem::path(guardD.path) / "UTILS");

    DriveManager dm;
    dm.mount('C', guardC.path);
    dm.mount('D', guardD.path);

    SECTION("Each drive has independent CWD") {
        REQUIRE(dm.setCurrentDir('C', "GAMES\\DOOM"));
        REQUIRE(dm.setCurrentDir('D', "UTILS"));

        REQUIRE(dm.getCurrentDir('C') == "GAMES\\DOOM");
        REQUIRE(dm.getCurrentDir('D') == "UTILS");
    }

    SECTION("getFullDosCwd reflects current drive") {
        dm.setCurrentDrive('C');
        dm.setCurrentDir('C', "GAMES\\DOOM");
        REQUIRE(dm.getFullDosCwd() == "C:\\GAMES\\DOOM");

        dm.setCurrentDrive('D');
        dm.setCurrentDir('D', "UTILS");
        REQUIRE(dm.getFullDosCwd() == "D:\\UTILS");
    }

    SECTION("getFullDosCwd with no CWD set") {
        dm.setCurrentDrive('C');
        REQUIRE(dm.getFullDosCwd() == "C:\\");
    }

    SECTION("setCurrentDir on unmounted drive fails") {
        REQUIRE(!dm.setCurrentDir('X', "SOMEWHERE"));
    }
}

TEST_CASE("DriveManager: Path Resolution - Absolute with Drive", "[drive_manager]") {
    TempDirGuard guard("resolve_abs");
    std::filesystem::create_directories(std::filesystem::path(guard.path) / "FOO" / "BAR");

    DriveManager dm;
    dm.mount('C', guard.path);

    auto resolved = dm.resolvePath("C:\\FOO\\BAR");
    REQUIRE(!resolved.empty());

    auto expected = (std::filesystem::path(guard.path) / "FOO" / "BAR").string();
    // Compare as canonical paths to handle platform differences
    std::error_code ec;
    auto canonResolved = std::filesystem::canonical(resolved, ec);
    auto canonExpected = std::filesystem::canonical(expected, ec);
    REQUIRE(!ec);
    REQUIRE(canonResolved == canonExpected);
}

TEST_CASE("DriveManager: Path Resolution - Relative", "[drive_manager]") {
    TempDirGuard guard("resolve_rel");
    std::filesystem::create_directories(std::filesystem::path(guard.path) / "GAMES" / "DOOM");

    DriveManager dm;
    dm.mount('C', guard.path);
    dm.setCurrentDrive('C');
    dm.setCurrentDir('C', "GAMES");

    auto resolved = dm.resolvePath("DOOM");
    REQUIRE(!resolved.empty());

    auto expected = (std::filesystem::path(guard.path) / "GAMES" / "DOOM").string();
    std::error_code ec;
    auto canonResolved = std::filesystem::canonical(resolved, ec);
    auto canonExpected = std::filesystem::canonical(expected, ec);
    REQUIRE(!ec);
    REQUIRE(canonResolved == canonExpected);
}

TEST_CASE("DriveManager: Path Resolution - Parent Directory", "[drive_manager]") {
    TempDirGuard guard("resolve_parent");
    std::filesystem::create_directories(std::filesystem::path(guard.path) / "FOO");
    std::filesystem::create_directories(std::filesystem::path(guard.path) / "BAR");

    DriveManager dm;
    dm.mount('C', guard.path);

    auto resolved = dm.resolvePath("C:\\FOO\\..\\BAR");
    REQUIRE(!resolved.empty());

    auto expected = (std::filesystem::path(guard.path) / "BAR").string();
    std::error_code ec;
    auto canonResolved = std::filesystem::canonical(resolved, ec);
    auto canonExpected = std::filesystem::canonical(expected, ec);
    REQUIRE(!ec);
    REQUIRE(canonResolved == canonExpected);
}

TEST_CASE("DriveManager: Path Resolution - Unmounted Drive", "[drive_manager]") {
    DriveManager dm;

    REQUIRE(dm.resolvePath("X:\\FOO").empty());
}

TEST_CASE("DriveManager: Path Resolution - No Drive Letter", "[drive_manager]") {
    TempDirGuard guard("resolve_nodrive");
    std::filesystem::create_directories(std::filesystem::path(guard.path) / "FOO");

    DriveManager dm;
    dm.mount('C', guard.path);
    dm.setCurrentDrive('C');

    auto resolved = dm.resolvePath("\\FOO");
    REQUIRE(!resolved.empty());

    auto expected = (std::filesystem::path(guard.path) / "FOO").string();
    std::error_code ec;
    auto canonResolved = std::filesystem::canonical(resolved, ec);
    auto canonExpected = std::filesystem::canonical(expected, ec);
    REQUIRE(!ec);
    REQUIRE(canonResolved == canonExpected);
}

TEST_CASE("DriveManager: Path Resolution - Security (no escape)", "[drive_manager]") {
    TempDirGuard guard("resolve_sec");

    DriveManager dm;
    dm.mount('C', guard.path);

    SECTION("Double dots from root do not escape mount") {
        // collapsePath turns \..\..\..etc\passwd into \etc\passwd
        // which stays within the mount root
        auto resolved = dm.resolvePath("C:\\..\\..\\..\\etc\\passwd");
        if (!resolved.empty()) {
            std::filesystem::path mountRoot(guard.path);
            std::error_code ec;
            auto canon = std::filesystem::weakly_canonical(resolved, ec);
            auto rel = std::filesystem::relative(canon, mountRoot, ec);
            REQUIRE(!ec);
            REQUIRE(!rel.empty());
            REQUIRE(*rel.begin() != std::filesystem::path(".."));
        }
    }

    SECTION("Unmounted drive returns empty") {
        REQUIRE(dm.resolvePath("X:\\anything").empty());
    }
}

TEST_CASE("DriveManager: Volume Label", "[drive_manager]") {
    TempDirGuard guard("vol_label");
    DriveManager dm;
    dm.mount('C', guard.path);

    SECTION("Default volume label") {
        REQUIRE(dm.getVolumeLabel('C') == "MS-DOS_622");
    }

    SECTION("Custom volume label on mount") {
        dm.mount('D', guard.path, "MYDISK");
        REQUIRE(dm.getVolumeLabel('D') == "MYDISK");
    }

    SECTION("Set and get volume label") {
        dm.setVolumeLabel('C', "NEWLABEL");
        REQUIRE(dm.getVolumeLabel('C') == "NEWLABEL");
    }
}

TEST_CASE("DriveManager: Normalize DOS Path", "[drive_manager]") {
    SECTION("Forward slashes to backslashes") {
        REQUIRE(DriveManager::normalizeDosPath("foo/bar/baz") == "FOO\\BAR\\BAZ");
    }

    SECTION("Lowercase to uppercase") {
        REQUIRE(DriveManager::normalizeDosPath("hello") == "HELLO");
    }

    SECTION("Mixed case and slashes") {
        REQUIRE(DriveManager::normalizeDosPath("Games/Doom/DOOM.EXE") == "GAMES\\DOOM\\DOOM.EXE");
    }

    SECTION("Already normalized") {
        REQUIRE(DriveManager::normalizeDosPath("GAMES\\DOOM") == "GAMES\\DOOM");
    }

    SECTION("Empty string") {
        REQUIRE(DriveManager::normalizeDosPath("").empty());
    }
}

TEST_CASE("DriveManager: setCurrentDir Validation", "[drive_manager]") {
    TempDirGuard guard("cwd_valid");
    std::filesystem::create_directories(std::filesystem::path(guard.path) / "GAMES");

    DriveManager dm;
    dm.mount('C', guard.path);

    SECTION("Normalizes to uppercase backslashes") {
        REQUIRE(dm.setCurrentDir('C', "games"));
        REQUIRE(dm.getCurrentDir('C') == "GAMES");
    }

    SECTION("Leading backslash is stripped") {
        REQUIRE(dm.setCurrentDir('C', "\\GAMES"));
        REQUIRE(dm.getCurrentDir('C') == "GAMES");
    }

    SECTION("Setting CWD to root clears it") {
        dm.setCurrentDir('C', "GAMES");
        REQUIRE(dm.setCurrentDir('C', "\\"));
        REQUIRE(dm.getCurrentDir('C').empty());
    }

    SECTION("Setting CWD to empty string clears it") {
        dm.setCurrentDir('C', "GAMES");
        REQUIRE(dm.setCurrentDir('C', ""));
        REQUIRE(dm.getCurrentDir('C').empty());
    }

    SECTION("Non-existent directory fails") {
        REQUIRE(!dm.setCurrentDir('C', "NOSUCHDIR"));
    }
}

namespace {

void appendLe16(std::vector<uint8_t>& buf, uint16_t v) {
    buf.push_back(static_cast<uint8_t>(v & 0xFFu));
    buf.push_back(static_cast<uint8_t>((v >> 8) & 0xFFu));
}

void appendLe32(std::vector<uint8_t>& buf, uint32_t v) {
    buf.push_back(static_cast<uint8_t>(v & 0xFFu));
    buf.push_back(static_cast<uint8_t>((v >> 8) & 0xFFu));
    buf.push_back(static_cast<uint8_t>((v >> 16) & 0xFFu));
    buf.push_back(static_cast<uint8_t>((v >> 24) & 0xFFu));
}

void writeSingleStoreZip(const std::filesystem::path& zipPath,
                         const std::string& entryName,
                         const std::string& payload,
                         uint32_t crc) {
    std::vector<uint8_t> out;
    const uint32_t localOffset = 0;
    const uint16_t nameLen = static_cast<uint16_t>(entryName.size());
    const uint32_t dataSize = static_cast<uint32_t>(payload.size());

    // Local file header (30 bytes + name + data)
    appendLe32(out, 0x04034b50u);
    appendLe16(out, 20);
    appendLe16(out, 0);
    appendLe16(out, 0);
    appendLe16(out, 0);
    appendLe16(out, 0);
    appendLe32(out, crc);
    appendLe32(out, dataSize);
    appendLe32(out, dataSize);
    appendLe16(out, nameLen);
    appendLe16(out, 0);
    out.insert(out.end(), entryName.begin(), entryName.end());
    out.insert(out.end(), payload.begin(), payload.end());

    const uint32_t cdOffset = static_cast<uint32_t>(out.size());
    // Central directory header (46 bytes + name)
    appendLe32(out, 0x02014b50u);
    appendLe16(out, 20);
    appendLe16(out, 20);
    appendLe16(out, 0);
    appendLe16(out, 0);
    appendLe16(out, 0);
    appendLe16(out, 0);
    appendLe32(out, crc);
    appendLe32(out, dataSize);
    appendLe32(out, dataSize);
    appendLe16(out, nameLen);
    appendLe16(out, 0);
    appendLe16(out, 0);
    appendLe16(out, 0);
    appendLe16(out, 0);
    appendLe32(out, 0);
    appendLe32(out, localOffset);
    out.insert(out.end(), entryName.begin(), entryName.end());

    const uint32_t cdSize = static_cast<uint32_t>(out.size()) - cdOffset;
    // EOCD (22 bytes)
    appendLe32(out, 0x06054b50u);
    appendLe16(out, 0);
    appendLe16(out, 0);
    appendLe16(out, 1);
    appendLe16(out, 1);
    appendLe32(out, cdSize);
    appendLe32(out, cdOffset);
    appendLe16(out, 0);

    std::ofstream ofs(zipPath, std::ios::binary | std::ios::trunc);
    ofs.write(reinterpret_cast<const char*>(out.data()),
              static_cast<std::streamsize>(out.size()));
}

std::filesystem::path locateUseCaseZip(const std::string& filename) {
    namespace fs = std::filesystem;
    const std::vector<fs::path> prefixes = {
        fs::path("use-cases"),
        fs::path("..") / "use-cases",
        fs::path("..") / ".." / "use-cases",
    };
    std::error_code ec;
    for (const auto& dir : prefixes) {
        fs::path candidate = dir / filename;
        if (fs::is_regular_file(candidate, ec) && !ec) {
            return candidate;
        }
    }
    return {};
}

} // namespace

TEST_CASE("fzip: extract_to_directory validation and security", "[drive_manager]") {
    TempDirGuard guard("fzip_sec");
    const std::filesystem::path base(guard.path);
    const std::string payload = "HELLO";
    const uint32_t validCrc = fzip::crc32(std::span<const std::byte>(
        reinterpret_cast<const std::byte*>(payload.data()), payload.size()));

    SECTION("Valid Store entry extracts and passes CRC") {
        const std::filesystem::path zipPath = base / "good.zip";
        const std::filesystem::path outDir = base / "out_good";
        writeSingleStoreZip(zipPath, "sub/hello.txt", payload, validCrc);
        REQUIRE(fzip::extract_to_directory(zipPath.string(), outDir));
        REQUIRE(std::filesystem::is_regular_file(outDir / "sub" / "hello.txt"));
    }

    SECTION("Corrupted CRC fails extraction") {
        const std::filesystem::path zipPath = base / "bad_crc.zip";
        const std::filesystem::path outDir = base / "out_bad_crc";
        writeSingleStoreZip(zipPath, "hello.txt", payload, validCrc ^ 1u);
        REQUIRE(!fzip::extract_to_directory(zipPath.string(), outDir));
    }

    SECTION("Path traversal entry fails extraction") {
        const std::filesystem::path zipPath = base / "traversal.zip";
        const std::filesystem::path outDir = base / "out_traversal";
        writeSingleStoreZip(zipPath, "../escape.txt", payload, validCrc);
        REQUIRE(!fzip::extract_to_directory(zipPath.string(), outDir));
        REQUIRE(!std::filesystem::exists(base / "escape.txt"));
    }
}

TEST_CASE("DriveManager: Mount and unmount ZIP archives", "[drive_manager]") {
    const auto wingsZip = locateUseCaseZip("wings140.zip");
    const auto scorchZip = locateUseCaseZip("scorch15.zip");
    const auto cannonZip = locateUseCaseZip("Cannon Fodder (1993).zip");

    if (!wingsZip.empty()) {
        std::string extractedHostPath;
        {
            DriveManager dm;
            REQUIRE(dm.mount('C', wingsZip.string()));
            REQUIRE(dm.isMounted('C'));
            extractedHostPath = dm.getHostPath('C');
            REQUIRE(std::filesystem::is_directory(extractedHostPath));
            REQUIRE(std::filesystem::is_regular_file(dm.resolvePath("C:\\WINGS.EXE")));
            REQUIRE(dm.unmount('C'));
            REQUIRE(!dm.isMounted('C'));
            REQUIRE(!std::filesystem::exists(extractedHostPath));
        }
    }

    if (!wingsZip.empty()) {
        std::string extractedOnDrop;
        {
            DriveManager dm;
            REQUIRE(dm.mount('C', wingsZip.string()));
            extractedOnDrop = dm.getHostPath('C');
            REQUIRE(std::filesystem::exists(extractedOnDrop));
        }
        REQUIRE(!std::filesystem::exists(extractedOnDrop));
    }

    if (!scorchZip.empty()) {
        DriveManager dm;
        REQUIRE(dm.mount('D', scorchZip.string()));
        REQUIRE(std::filesystem::is_regular_file(dm.resolvePath("D:\\SCORCH.EXE")));
        REQUIRE(dm.unmount('D'));
    }

    if (!cannonZip.empty()) {
        DriveManager dm;
        REQUIRE(dm.mount('E', cannonZip.string()));
        const std::string hostPath = dm.getHostPath('E');
        REQUIRE(std::filesystem::is_regular_file(dm.resolvePath("E:\\run.bat")));
        REQUIRE(dm.unmount('E'));
        REQUIRE(!std::filesystem::exists(hostPath));
    }
}
