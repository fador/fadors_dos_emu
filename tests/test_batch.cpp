#include "test_framework.hpp"
#include "hw/BatchInterpreter.hpp"
#include "hw/DosShell.hpp"
#include "hw/DOS.hpp"
#include "hw/DriveManager.hpp"
#include "hw/ProgramLoader.hpp"
#include "hw/BIOS.hpp"
#include "hw/KeyboardController.hpp"
#include "hw/PIT8254.hpp"
#include "hw/PIC8259.hpp"
#include "hw/IOBus.hpp"
#include "cpu/CPU.hpp"
#include "cpu/InstructionDecoder.hpp"
#include "memory/MemoryBus.hpp"

#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

using namespace fador;

namespace {

namespace fs = std::filesystem;

struct BatchTestEnv {
    cpu::CPU cpu;
    memory::MemoryBus mem;
    hw::IOBus iobus;
    hw::KeyboardController kbd;
    hw::PIT8254 pit;
    hw::PIC8259 pic{true};
    hw::BIOS bios;
    hw::DOS dos;
    cpu::InstructionDecoder decoder;
    hw::ProgramLoader loader;
    hw::DriveManager dm;
    hw::DosShell shell;
    hw::BatchInterpreter batch;

    BatchTestEnv()
        : bios(cpu, mem, kbd, pit, pic)
        , dos(cpu, mem)
        , decoder(cpu, mem, iobus, bios, dos)
        , loader(cpu, mem, dos.getHIMEM())
        , shell(dos, dm, loader, cpu, decoder, kbd, bios, mem)
        , batch(shell)
    {
        bios.initialize();
        dos.initialize();
    }
};

fs::path makeTempDir(const std::string& name) {
    auto base = fs::temp_directory_path() / "fador_test" / name;
    fs::create_directories(base);
    return base;
}

void removeTempDir(const fs::path& path) {
    std::error_code ec;
    fs::remove_all(path, ec);
}

struct TempDirGuard {
    fs::path path;
    explicit TempDirGuard(const std::string& name) : path(makeTempDir(name)) {}
    ~TempDirGuard() { removeTempDir(path); }
};

fs::path writeBat(const fs::path& dir, const std::string& name, const std::string& content) {
    auto p = dir / name;
    std::ofstream ofs(p);
    ofs << content;
    ofs.close();
    return p;
}

} // namespace

TEST_CASE("BatchInterpreter: Empty BAT file", "[batch]") {
    BatchTestEnv env;
    TempDirGuard guard("batch_empty");
    auto batPath = writeBat(guard.path, "empty.bat", "");

    int rc = env.batch.execute(batPath.string());
    REQUIRE(rc == 0);
}

TEST_CASE("BatchInterpreter: REM comments only", "[batch]") {
    BatchTestEnv env;
    TempDirGuard guard("batch_rem");
    auto batPath = writeBat(guard.path, "rem.bat",
        "REM This is a comment\r\n"
        "REM Another comment\r\n"
    );

    int rc = env.batch.execute(batPath.string());
    REQUIRE(rc == 0);
}

TEST_CASE("BatchInterpreter: ECHO via shell", "[batch]") {
    BatchTestEnv env;
    TempDirGuard guard("batch_echo");
    auto batPath = writeBat(guard.path, "echo.bat",
        "ECHO hello world\r\n"
    );

    int rc = env.batch.execute(batPath.string());
    REQUIRE(rc == 0);
}

TEST_CASE("BatchInterpreter: SET environment variable", "[batch]") {
    BatchTestEnv env;
    TempDirGuard guard("batch_set");
    auto batPath = writeBat(guard.path, "setvar.bat",
        "SET MYVAR=hello\r\n"
        "SET MYNUM=42\r\n"
    );

    int rc = env.batch.execute(batPath.string());
    REQUIRE(rc == 0);
    REQUIRE(env.shell.getEnvVar("MYVAR") == "hello");
    REQUIRE(env.shell.getEnvVar("MYNUM") == "42");
}

TEST_CASE("BatchInterpreter: Environment variable expansion", "[batch]") {
    BatchTestEnv env;
    TempDirGuard guard("batch_envexp");
    env.shell.setEnvVar("GREETING", "world");
    auto batPath = writeBat(guard.path, "envexp.bat",
        "SET COPY=%GREETING%\r\n"
    );

    int rc = env.batch.execute(batPath.string());
    REQUIRE(rc == 0);
    REQUIRE(env.shell.getEnvVar("COPY") == "world");
}

TEST_CASE("BatchInterpreter: Parameter substitution %0 %1 %2", "[batch]") {
    BatchTestEnv env;
    TempDirGuard guard("batch_params");
    auto batPath = writeBat(guard.path, "params.bat",
        "SET P0=%0\r\n"
        "SET P1=%1\r\n"
        "SET P2=%2\r\n"
    );

    std::vector<std::string> args = {"hello", "world"};
    int rc = env.batch.execute(batPath.string(), args);
    REQUIRE(rc == 0);
    REQUIRE(env.shell.getEnvVar("P0") == "PARAMS");
    REQUIRE(env.shell.getEnvVar("P1") == "hello");
    REQUIRE(env.shell.getEnvVar("P2") == "world");
}

TEST_CASE("BatchInterpreter: Missing parameters expand to empty", "[batch]") {
    BatchTestEnv env;
    TempDirGuard guard("batch_missparm");
    auto batPath = writeBat(guard.path, "missparm.bat",
        "SET P1=%1\r\n"
        "SET P9=%9\r\n"
    );

    std::vector<std::string> args = {"onlyfirst"};
    int rc = env.batch.execute(batPath.string(), args);
    REQUIRE(rc == 0);
    REQUIRE(env.shell.getEnvVar("P1") == "onlyfirst");
    REQUIRE(env.shell.getEnvVar("P9") == "");
}

TEST_CASE("BatchInterpreter: ECHO ON/OFF toggle", "[batch]") {
    BatchTestEnv env;
    TempDirGuard guard("batch_echo_toggle");
    auto batPath = writeBat(guard.path, "echotog.bat",
        "ECHO OFF\r\n"
        "SET WAS_ECHO_OFF=1\r\n"
        "ECHO ON\r\n"
        "SET WAS_ECHO_ON=1\r\n"
    );

    env.shell.setEchoOn(true);
    int rc = env.batch.execute(batPath.string());
    REQUIRE(rc == 0);
    REQUIRE(env.shell.getEnvVar("WAS_ECHO_OFF") == "1");
    REQUIRE(env.shell.getEnvVar("WAS_ECHO_ON") == "1");
    REQUIRE(env.shell.isEchoOn() == true);
}

TEST_CASE("BatchInterpreter: GOTO and labels", "[batch]") {
    BatchTestEnv env;
    TempDirGuard guard("batch_goto");
    auto batPath = writeBat(guard.path, "goto.bat",
        "GOTO skip\r\n"
        "SET SHOULD_NOT_RUN=1\r\n"
        ":skip\r\n"
        "SET DID_RUN=1\r\n"
    );

    int rc = env.batch.execute(batPath.string());
    REQUIRE(rc == 0);
    REQUIRE(env.shell.getEnvVar("SHOULD_NOT_RUN") == "");
    REQUIRE(env.shell.getEnvVar("DID_RUN") == "1");
}

TEST_CASE("BatchInterpreter: GOTO label not found", "[batch]") {
    BatchTestEnv env;
    TempDirGuard guard("batch_goto_bad");
    auto batPath = writeBat(guard.path, "gotobad.bat",
        "GOTO nonexistent\r\n"
    );

    int rc = env.batch.execute(batPath.string());
    REQUIRE(rc != 0);
}

TEST_CASE("BatchInterpreter: IF string comparison true", "[batch]") {
    BatchTestEnv env;
    TempDirGuard guard("batch_if_str");
    env.shell.setEnvVar("MYVAR", "hello");
    auto batPath = writeBat(guard.path, "ifstr.bat",
        "IF %MYVAR%==hello SET MATCHED=1\r\n"
    );

    int rc = env.batch.execute(batPath.string());
    REQUIRE(rc == 0);
    REQUIRE(env.shell.getEnvVar("MATCHED") == "1");
}

TEST_CASE("BatchInterpreter: IF string comparison false", "[batch]") {
    BatchTestEnv env;
    TempDirGuard guard("batch_if_str_false");
    env.shell.setEnvVar("MYVAR", "goodbye");
    auto batPath = writeBat(guard.path, "ifstrf.bat",
        "IF %MYVAR%==hello SET MATCHED=1\r\n"
        "SET DONE=1\r\n"
    );

    int rc = env.batch.execute(batPath.string());
    REQUIRE(rc == 0);
    REQUIRE(env.shell.getEnvVar("MATCHED") == "");
    REQUIRE(env.shell.getEnvVar("DONE") == "1");
}

TEST_CASE("BatchInterpreter: IF NOT string comparison", "[batch]") {
    BatchTestEnv env;
    TempDirGuard guard("batch_if_not");
    env.shell.setEnvVar("MYVAR", "world");
    auto batPath = writeBat(guard.path, "ifnot.bat",
        "IF NOT %MYVAR%==hello SET NOT_MATCHED=1\r\n"
    );

    int rc = env.batch.execute(batPath.string());
    REQUIRE(rc == 0);
    REQUIRE(env.shell.getEnvVar("NOT_MATCHED") == "1");
}

TEST_CASE("BatchInterpreter: IF EXIST with real file", "[batch]") {
    BatchTestEnv env;
    TempDirGuard guard("batch_if_exist");
    auto testFile = guard.path / "testfile.txt";
    {
        std::ofstream ofs(testFile);
        ofs << "data";
    }
    auto batPath = writeBat(guard.path, "ifexist.bat",
        "IF EXIST \"" + testFile.string() + "\" SET FILE_FOUND=1\r\n"
    );

    int rc = env.batch.execute(batPath.string());
    REQUIRE(rc == 0);
    REQUIRE(env.shell.getEnvVar("FILE_FOUND") == "1");
}

TEST_CASE("BatchInterpreter: IF NOT EXIST with missing file", "[batch]") {
    BatchTestEnv env;
    TempDirGuard guard("batch_if_not_exist");
    auto batPath = writeBat(guard.path, "ifnexist.bat",
        "IF NOT EXIST \"C:\\nonexistent_file_xyz_123.tmp\" SET MISSING=1\r\n"
    );

    int rc = env.batch.execute(batPath.string());
    REQUIRE(rc == 0);
    REQUIRE(env.shell.getEnvVar("MISSING") == "1");
}

TEST_CASE("BatchInterpreter: IF ERRORLEVEL", "[batch]") {
    BatchTestEnv env;
    TempDirGuard guard("batch_if_errlvl");
    env.shell.setErrorLevel(5);
    auto batPath = writeBat(guard.path, "iferrlvl.bat",
        "IF ERRORLEVEL 3 SET HIGH_ENOUGH=1\r\n"
        "IF ERRORLEVEL 10 SET TOO_HIGH=1\r\n"
    );

    int rc = env.batch.execute(batPath.string());
    REQUIRE(rc == 0);
    REQUIRE(env.shell.getEnvVar("HIGH_ENOUGH") == "1");
    REQUIRE(env.shell.getEnvVar("TOO_HIGH") == "");
}

TEST_CASE("BatchInterpreter: IF NOT ERRORLEVEL", "[batch]") {
    BatchTestEnv env;
    TempDirGuard guard("batch_if_not_errlvl");
    env.shell.setErrorLevel(2);
    auto batPath = writeBat(guard.path, "ifnoterr.bat",
        "IF NOT ERRORLEVEL 5 SET BELOW=1\r\n"
    );

    int rc = env.batch.execute(batPath.string());
    REQUIRE(rc == 0);
    REQUIRE(env.shell.getEnvVar("BELOW") == "1");
}

TEST_CASE("BatchInterpreter: SHIFT moves parameters", "[batch]") {
    BatchTestEnv env;
    TempDirGuard guard("batch_shift");
    auto batPath = writeBat(guard.path, "shift.bat",
        "SET S1=%1\r\n"
        "SHIFT\r\n"
        "SET S2=%1\r\n"
        "SHIFT\r\n"
        "SET S3=%1\r\n"
    );

    std::vector<std::string> args = {"aaa", "bbb", "ccc"};
    int rc = env.batch.execute(batPath.string(), args);
    REQUIRE(rc == 0);
    REQUIRE(env.shell.getEnvVar("S1") == "aaa");
    REQUIRE(env.shell.getEnvVar("S2") == "bbb");
    REQUIRE(env.shell.getEnvVar("S3") == "ccc");
}

TEST_CASE("BatchInterpreter: EXIT /B with code", "[batch]") {
    BatchTestEnv env;
    TempDirGuard guard("batch_exit_b");
    auto batPath = writeBat(guard.path, "exitb.bat",
        "SET BEFORE=1\r\n"
        "EXIT /B 42\r\n"
        "SET AFTER=1\r\n"
    );

    int rc = env.batch.execute(batPath.string());
    REQUIRE(rc == 42);
    REQUIRE(env.shell.getEnvVar("BEFORE") == "1");
    REQUIRE(env.shell.getEnvVar("AFTER") == "");
}

TEST_CASE("BatchInterpreter: EXIT /B 0", "[batch]") {
    BatchTestEnv env;
    TempDirGuard guard("batch_exit_b0");
    auto batPath = writeBat(guard.path, "exitb0.bat",
        "EXIT /B 0\r\n"
    );

    int rc = env.batch.execute(batPath.string());
    REQUIRE(rc == 0);
}

TEST_CASE("BatchInterpreter: EXIT without /B", "[batch]") {
    BatchTestEnv env;
    TempDirGuard guard("batch_exit_nob");
    auto batPath = writeBat(guard.path, "exitnb.bat",
        "SET BEFORE=1\r\n"
        "EXIT 7\r\n"
        "SET AFTER=1\r\n"
    );

    int rc = env.batch.execute(batPath.string());
    REQUIRE(rc == 7);
    REQUIRE(env.shell.getEnvVar("BEFORE") == "1");
    REQUIRE(env.shell.getEnvVar("AFTER") == "");
}

TEST_CASE("BatchInterpreter: Nested CALL to another BAT", "[batch]") {
    BatchTestEnv env;
    TempDirGuard guard("batch_call");

    auto childBat = writeBat(guard.path, "child.bat",
        "SET CHILD_RAN=1\r\n"
        "EXIT /B 99\r\n"
    );

    auto parentBat = writeBat(guard.path, "parent.bat",
        "SET BEFORE_CALL=1\r\n"
        "CALL " + childBat.string() + "\r\n"
        "SET AFTER_CALL=1\r\n"
    );

    int rc = env.batch.execute(parentBat.string());
    REQUIRE(rc == 0);
    REQUIRE(env.shell.getEnvVar("CHILD_RAN") == "1");
    REQUIRE(env.shell.getEnvVar("BEFORE_CALL") == "1");
    REQUIRE(env.shell.getEnvVar("AFTER_CALL") == "1");
}

TEST_CASE("BatchInterpreter: Multiple GOTO with loop", "[batch]") {
    BatchTestEnv env;
    TempDirGuard guard("batch_goto_loop");
    auto batPath = writeBat(guard.path, "loop.bat",
        "SET DONE=no\r\n"
        "GOTO check\r\n"
        ":skip\r\n"
        "SET DONE=skipped\r\n"
        "GOTO end\r\n"
        ":check\r\n"
        "SET DONE=yes\r\n"
        ":end\r\n"
    );

    int rc = env.batch.execute(batPath.string());
    REQUIRE(rc == 0);
    REQUIRE(env.shell.getEnvVar("DONE") == "yes");
}

TEST_CASE("BatchInterpreter: FOR loop with items", "[batch]") {
    BatchTestEnv env;
    TempDirGuard guard("batch_for");
    auto batPath = writeBat(guard.path, "for.bat",
        "FOR %%F IN (alpha beta gamma) DO SET LAST=%%F\r\n"
    );

    int rc = env.batch.execute(batPath.string());
    REQUIRE(rc == 0);
    REQUIRE(env.shell.getEnvVar("LAST") == "gamma");
}

TEST_CASE("BatchInterpreter: FOR single item", "[batch]") {
    BatchTestEnv env;
    TempDirGuard guard("batch_for_single");
    auto batPath = writeBat(guard.path, "forsingle.bat",
        "FOR %%X IN (only) DO SET SINGLE=%%X\r\n"
    );

    int rc = env.batch.execute(batPath.string());
    REQUIRE(rc == 0);
    REQUIRE(env.shell.getEnvVar("SINGLE") == "only");
}

TEST_CASE("BatchInterpreter: Command execution through shell", "[batch]") {
    BatchTestEnv env;
    TempDirGuard guard("batch_cmd");
    auto batPath = writeBat(guard.path, "cmd.bat",
        "VER\r\n"
    );

    int rc = env.batch.execute(batPath.string());
    REQUIRE(rc == 0);
}

TEST_CASE("BatchInterpreter: Error level propagation", "[batch]") {
    BatchTestEnv env;
    TempDirGuard guard("batch_errlvl_prop");
    auto batPath = writeBat(guard.path, "errlvl.bat",
        "SET LEVEL=1\r\n"
        "EXIT /B 1\r\n"
    );

    int rc = env.batch.execute(batPath.string());
    REQUIRE(rc == 1);
    REQUIRE(env.shell.getEnvVar("LEVEL") == "1");
}

TEST_CASE("BatchInterpreter: Multiple commands sequence", "[batch]") {
    BatchTestEnv env;
    TempDirGuard guard("batch_multi");
    auto batPath = writeBat(guard.path, "multi.bat",
        "SET A=1\r\n"
        "SET B=2\r\n"
        "SET C=3\r\n"
        "SET D=%A%_%B%_%C%\r\n"
    );

    int rc = env.batch.execute(batPath.string());
    REQUIRE(rc == 0);
    REQUIRE(env.shell.getEnvVar("A") == "1");
    REQUIRE(env.shell.getEnvVar("B") == "2");
    REQUIRE(env.shell.getEnvVar("C") == "3");
    REQUIRE(env.shell.getEnvVar("D") == "1_2_3");
}

TEST_CASE("BatchInterpreter: Literal percent with %%", "[batch]") {
    BatchTestEnv env;
    TempDirGuard guard("batch_literal_pct");
    auto batPath = writeBat(guard.path, "pct.bat",
        "SET PCT=50%%\r\n"
    );

    int rc = env.batch.execute(batPath.string());
    REQUIRE(rc == 0);
    REQUIRE(env.shell.getEnvVar("PCT") == "50%");
}

TEST_CASE("BatchInterpreter: GOTO to label defined later", "[batch]") {
    BatchTestEnv env;
    TempDirGuard guard("batch_goto_forward");
    auto batPath = writeBat(guard.path, "gotorwd.bat",
        "GOTO end\r\n"
        "SET SKIPPED=1\r\n"
        ":end\r\n"
        "SET REACHED=1\r\n"
    );

    int rc = env.batch.execute(batPath.string());
    REQUIRE(rc == 0);
    REQUIRE(env.shell.getEnvVar("SKIPPED") == "");
    REQUIRE(env.shell.getEnvVar("REACHED") == "1");
}

TEST_CASE("BatchInterpreter: CASE insensitive label matching", "[batch]") {
    BatchTestEnv env;
    TempDirGuard guard("batch_label_case");
    auto batPath = writeBat(guard.path, "lblcase.bat",
        "GOTO MyLabel\r\n"
        "SET SKIPPED=1\r\n"
        ":mylabel\r\n"
        "SET FOUND=1\r\n"
    );

    int rc = env.batch.execute(batPath.string());
    REQUIRE(rc == 0);
    REQUIRE(env.shell.getEnvVar("SKIPPED") == "");
    REQUIRE(env.shell.getEnvVar("FOUND") == "1");
}

TEST_CASE("BatchInterpreter: CALL with args passed through", "[batch]") {
    BatchTestEnv env;
    TempDirGuard guard("batch_call_args");

    auto childBat = writeBat(guard.path, "childargs.bat",
        "SET C_ARG1=%1\r\n"
        "SET C_ARG2=%2\r\n"
    );

    auto parentBat = writeBat(guard.path, "parentargs.bat",
        "CALL " + childBat.string() + " hello world\r\n"
    );

    int rc = env.batch.execute(parentBat.string());
    REQUIRE(rc == 0);
    REQUIRE(env.shell.getEnvVar("C_ARG1") == "hello");
    REQUIRE(env.shell.getEnvVar("C_ARG2") == "world");
}

TEST_CASE("BatchInterpreter: SET with spaces in value", "[batch]") {
    BatchTestEnv env;
    TempDirGuard guard("batch_set_spaces");
    auto batPath = writeBat(guard.path, "setspc.bat",
        "SET MYNAME=John Doe\r\n"
    );

    int rc = env.batch.execute(batPath.string());
    REQUIRE(rc == 0);
    REQUIRE(env.shell.getEnvVar("MYNAME") == "John Doe");
}

TEST_CASE("BatchInterpreter: IF EXIST skips command when false", "[batch]") {
    BatchTestEnv env;
    TempDirGuard guard("batch_if_exist_skip");
    auto batPath = writeBat(guard.path, "ifexskip.bat",
        "IF EXIST \"C:\\no_such_file_abc_xyz.tmp\" SET SHOULD_SKIP=1\r\n"
        "SET CONTINUED=1\r\n"
    );

    int rc = env.batch.execute(batPath.string());
    REQUIRE(rc == 0);
    REQUIRE(env.shell.getEnvVar("SHOULD_SKIP") == "");
    REQUIRE(env.shell.getEnvVar("CONTINUED") == "1");
}

TEST_CASE("BatchInterpreter: Re-execution resets state", "[batch]") {
    BatchTestEnv env;
    TempDirGuard guard("batch_rerun");
    auto batPath = writeBat(guard.path, "rerun.bat",
        "SET RUN_COUNT=%RUN_COUNT%_X\r\n"
    );

    env.shell.setEnvVar("RUN_COUNT", "");
    env.batch.execute(batPath.string());
    std::string first = env.shell.getEnvVar("RUN_COUNT");

    env.batch.execute(batPath.string());
    std::string second = env.shell.getEnvVar("RUN_COUNT");

    REQUIRE(first == "_X");
    REQUIRE(second == "_X_X");
}

TEST_CASE("BatchInterpreter: Nested CALL depth", "[batch]") {
    BatchTestEnv env;
    TempDirGuard guard("batch_nested_call");

    auto level3 = writeBat(guard.path, "level3.bat",
        "SET DEPTH=3\r\n"
        "EXIT /B 3\r\n"
    );

    auto level2 = writeBat(guard.path, "level2.bat",
        "SET DEPTH=2\r\n"
        "CALL " + level3.string() + "\r\n"
    );

    auto level1 = writeBat(guard.path, "level1.bat",
        "SET DEPTH=1\r\n"
        "CALL " + level2.string() + "\r\n"
    );

    int rc = env.batch.execute(level1.string());
    REQUIRE(rc == 3);
    REQUIRE(env.shell.getEnvVar("DEPTH") == "3");
}
