#include "test_framework.hpp"
#include "hw/DriveManager.hpp"

#include <filesystem>
#include <string>

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
            REQUIRE(!rel.native().starts_with(L".."));
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
