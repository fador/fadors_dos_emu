#pragma once
#include <string>
#include <vector>
#include <functional>
#include <map>

namespace fador::cpu { class CPU; class InstructionDecoder; }
namespace fador::memory { class MemoryBus; }

namespace fador::hw {

class DOS;
class DriveManager;
class ProgramLoader;
class KeyboardController;
class BIOS;

class DosShell {
public:
    DosShell(DOS& dos, DriveManager& driveManager, ProgramLoader& loader,
             fador::cpu::CPU& cpu, fador::cpu::InstructionDecoder& decoder,
             KeyboardController& kbd, BIOS& bios, fador::memory::MemoryBus& memory);
    ~DosShell() = default;

    void run();
    int executeCommand(const std::string& line);

    std::string getEnvVar(const std::string& name) const;
    void setEnvVar(const std::string& name, const std::string& value);
    const std::map<std::string, std::string>& getEnvVars() const;

    std::string getPromptTemplate() const;
    void setPromptTemplate(const std::string& tmpl);

    void setInputPollCallback(std::function<void()> cb) { m_pollInput = std::move(cb); }
    void setIdleCallback(std::function<void()> cb) { m_idleCallback = std::move(cb); }

    int getErrorLevel() const { return m_errorLevel; }
    void setErrorLevel(int level) { m_errorLevel = level; }

    bool isEchoOn() const { return m_echoOn; }
    void setEchoOn(bool on) { m_echoOn = on; }

    void printChar(char c);
    void printString(const std::string& s);
    void printLine(const std::string& s = "");

    KeyboardController& getKeyboard() { return m_kbd; }

private:
    DOS& m_dos;
    DriveManager& m_driveManager;
    ProgramLoader& m_loader;
    fador::cpu::CPU& m_cpu;
    fador::cpu::InstructionDecoder& m_decoder;
    KeyboardController& m_kbd;
    BIOS& m_bios;
    fador::memory::MemoryBus& m_memory;

    std::map<std::string, std::string> m_envVars;
    std::string m_promptTemplate = "$P$G";
    int m_errorLevel = 0;
    bool m_echoOn = true;
    bool m_running = true;
    std::function<void()> m_pollInput;
    std::function<void()> m_idleCallback;

    std::string readLine();
    std::string buildPrompt();

    struct ParsedCommand {
        std::string command;
        std::vector<std::string> args;
        std::string rawArgs;
    };
    ParsedCommand parseCommandLine(const std::string& line);

    int cmdDir(const ParsedCommand& cmd);
    int cmdCd(const ParsedCommand& cmd);
    int cmdMd(const ParsedCommand& cmd);
    int cmdRd(const ParsedCommand& cmd);
    int cmdCopy(const ParsedCommand& cmd);
    int cmdDel(const ParsedCommand& cmd);
    int cmdRen(const ParsedCommand& cmd);
    int cmdType(const ParsedCommand& cmd);
    int cmdEcho(const ParsedCommand& cmd);
    int cmdSet(const ParsedCommand& cmd);
    int cmdPath(const ParsedCommand& cmd);
    int cmdPrompt(const ParsedCommand& cmd);
    int cmdCls(const ParsedCommand& cmd);
    int cmdVer(const ParsedCommand& cmd);
    int cmdVol(const ParsedCommand& cmd);
    int cmdDate(const ParsedCommand& cmd);
    int cmdTime(const ParsedCommand& cmd);
    int cmdMem(const ParsedCommand& cmd);
    int cmdHelp(const ParsedCommand& cmd);
    int cmdMount(const ParsedCommand& cmd);
    int cmdUnmount(const ParsedCommand& cmd);
    int cmdExit(const ParsedCommand& cmd);
    int cmdBreak(const ParsedCommand& cmd);
    int cmdVerify(const ParsedCommand& cmd);
    int cmdTree(const ParsedCommand& cmd);
    int cmdMove(const ParsedCommand& cmd);

    int executeExternal(const ParsedCommand& cmd);
    std::string searchPath(const std::string& name);
    bool matchWildcard(const std::string& pattern, const std::string& text);

    using CmdHandler = int (DosShell::*)(const ParsedCommand&);
    std::map<std::string, CmdHandler> m_commands;

    void registerCommands();
};

} // namespace fador::hw
