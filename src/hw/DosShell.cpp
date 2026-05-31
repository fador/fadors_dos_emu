#include "DosShell.hpp"
#include "BIOS.hpp"
#include "DOS.hpp"
#include "DriveManager.hpp"
#include "KeyboardController.hpp"
#include "ProgramLoader.hpp"
#include "../cpu/CPU.hpp"
#include "../cpu/InstructionDecoder.hpp"
#include "../memory/MemoryBus.hpp"
#include "../utils/Logger.hpp"
#include <algorithm>
#include <chrono>
#include <cctype>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <thread>

namespace fs = std::filesystem;

namespace fador::hw {

DosShell::DosShell(DOS& dos, DriveManager& driveManager, ProgramLoader& loader,
                   fador::cpu::CPU& cpu, fador::cpu::InstructionDecoder& decoder,
                   KeyboardController& kbd, BIOS& bios, fador::memory::MemoryBus& memory)
    : m_dos(dos), m_driveManager(driveManager), m_loader(loader),
      m_cpu(cpu), m_decoder(decoder), m_kbd(kbd), m_bios(bios), m_memory(memory)
{
    m_envVars["PATH"] = "C:\\";
    m_envVars["COMSPEC"] = "C:\\COMMAND.COM";
    m_envVars["PROMPT"] = "$P$G";
    registerCommands();
}

void DosShell::registerCommands()
{
    m_commands["DIR"] = &DosShell::cmdDir;
    m_commands["CD"] = &DosShell::cmdCd;
    m_commands["CHDIR"] = &DosShell::cmdCd;
    m_commands["MD"] = &DosShell::cmdMd;
    m_commands["MKDIR"] = &DosShell::cmdMd;
    m_commands["RD"] = &DosShell::cmdRd;
    m_commands["RMDIR"] = &DosShell::cmdRd;
    m_commands["COPY"] = &DosShell::cmdCopy;
    m_commands["DEL"] = &DosShell::cmdDel;
    m_commands["ERASE"] = &DosShell::cmdDel;
    m_commands["REN"] = &DosShell::cmdRen;
    m_commands["RENAME"] = &DosShell::cmdRen;
    m_commands["TYPE"] = &DosShell::cmdType;
    m_commands["ECHO"] = &DosShell::cmdEcho;
    m_commands["SET"] = &DosShell::cmdSet;
    m_commands["PATH"] = &DosShell::cmdPath;
    m_commands["PROMPT"] = &DosShell::cmdPrompt;
    m_commands["CLS"] = &DosShell::cmdCls;
    m_commands["VER"] = &DosShell::cmdVer;
    m_commands["VOL"] = &DosShell::cmdVol;
    m_commands["DATE"] = &DosShell::cmdDate;
    m_commands["TIME"] = &DosShell::cmdTime;
    m_commands["MEM"] = &DosShell::cmdMem;
    m_commands["HELP"] = &DosShell::cmdHelp;
    m_commands["MOUNT"] = &DosShell::cmdMount;
    m_commands["UNMOUNT"] = &DosShell::cmdUnmount;
    m_commands["EXIT"] = &DosShell::cmdExit;
    m_commands["BREAK"] = &DosShell::cmdBreak;
    m_commands["VERIFY"] = &DosShell::cmdVerify;
    m_commands["TREE"] = &DosShell::cmdTree;
    m_commands["MOVE"] = &DosShell::cmdMove;
}

void DosShell::run()
{
    m_running = true;
    while (m_running) {
        printString(buildPrompt());
        std::string line = readLine();
        if (line.empty()) continue;
        executeCommand(line);
    }
}

int DosShell::executeCommand(const std::string& line)
{
    auto trimmed = line;
    while (!trimmed.empty() && (trimmed.back() == ' ' || trimmed.back() == '\t'))
        trimmed.pop_back();
    while (!trimmed.empty() && (trimmed.front() == ' ' || trimmed.front() == '\t'))
        trimmed.erase(trimmed.begin());
    if (trimmed.empty()) return 0;

    auto upper = trimmed;
    std::transform(upper.begin(), upper.end(), upper.begin(),
                   [](unsigned char c) { return static_cast<char>(std::toupper(c)); });

    if (upper == "ECHO ON") { m_echoOn = true; return 0; }
    if (upper == "ECHO OFF") { m_echoOn = false; return 0; }

    auto cmd = parseCommandLine(trimmed);
    auto upperCmd = cmd.command;
    std::transform(upperCmd.begin(), upperCmd.end(), upperCmd.begin(),
                   [](unsigned char c) { return static_cast<char>(std::toupper(c)); });

    auto it = m_commands.find(upperCmd);
    if (it != m_commands.end()) {
        int rc = (this->*(it->second))(cmd);
        m_errorLevel = rc;
        return rc;
    }

    int rc = executeExternal(cmd);
    m_errorLevel = rc;
    return rc;
}

std::string DosShell::getEnvVar(const std::string& name) const
{
    auto upper = name;
    std::transform(upper.begin(), upper.end(), upper.begin(),
                   [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
    auto it = m_envVars.find(upper);
    return (it != m_envVars.end()) ? it->second : "";
}

void DosShell::setEnvVar(const std::string& name, const std::string& value)
{
    auto upper = name;
    std::transform(upper.begin(), upper.end(), upper.begin(),
                   [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
    if (value.empty())
        m_envVars.erase(upper);
    else
        m_envVars[upper] = value;
}

const std::map<std::string, std::string>& DosShell::getEnvVars() const
{
    return m_envVars;
}

std::string DosShell::getPromptTemplate() const { return m_promptTemplate; }
void DosShell::setPromptTemplate(const std::string& tmpl) { m_promptTemplate = tmpl; }

void DosShell::printChar(char c)
{
    m_dos.writeCharToVRAM(static_cast<uint8_t>(c));
}

void DosShell::printString(const std::string& s)
{
    for (char c : s) printChar(c);
}

void DosShell::printLine(const std::string& s)
{
    printString(s);
    printChar('\r');
    printChar('\n');
}

std::string DosShell::readLine()
{
    std::string line;
    while (true) {
        if (m_pollInput) m_pollInput();
        while (!m_kbd.hasKey()) {
            if (m_pollInput) m_pollInput();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        auto [ascii, scancode] = m_kbd.popKey();
        if (ascii == 0x0D) {
            printChar('\r');
            printChar('\n');
            break;
        }
        if (ascii == 0x08) {
            if (!line.empty()) {
                line.pop_back();
                printChar('\b');
                printChar(' ');
                printChar('\b');
            }
            continue;
        }
        if (ascii >= 0x20) {
            line += static_cast<char>(ascii);
            printChar(static_cast<char>(ascii));
        }
    }
    return line;
}

std::string DosShell::buildPrompt()
{
    std::string result;
    auto now = std::time(nullptr);
    auto* lt = std::localtime(&now);

    for (size_t i = 0; i < m_promptTemplate.size(); ++i) {
        if (m_promptTemplate[i] == '$' && i + 1 < m_promptTemplate.size()) {
            ++i;
            char code = static_cast<char>(std::toupper(static_cast<unsigned char>(m_promptTemplate[i])));
            switch (code) {
            case 'P': result += m_driveManager.getFullDosCwd(); break;
            case 'G': result += '>'; break;
            case 'N': result += m_driveManager.getCurrentDriveLetter(); break;
            case 'D': {
                char buf[32];
                std::snprintf(buf, sizeof(buf), "%02d-%02d-%04d",
                              lt->tm_mon + 1, lt->tm_mday, lt->tm_year + 1900);
                result += buf;
                break;
            }
            case 'T': {
                char buf[32];
                std::snprintf(buf, sizeof(buf), "%02d:%02d:%02d",
                              lt->tm_hour, lt->tm_min, lt->tm_sec);
                result += buf;
                break;
            }
            case '_': result += '\r'; result += '\n'; break;
            case '$': result += '$'; break;
            case 'E': result += '\x1B'; break;
            case 'H': result += '\b'; break;
            case 'L': result += '<'; break;
            case 'B': result += '|'; break;
            case 'Q': result += '='; break;
            case 'V': result += "6.22"; break;
            case 'A': result += '&'; break;
            case 'C': result += '('; break;
            case 'F': result += ')'; break;
            default: result += '$'; result += m_promptTemplate[i]; break;
            }
        } else {
            result += m_promptTemplate[i];
        }
    }
    return result;
}

DosShell::ParsedCommand DosShell::parseCommandLine(const std::string& line)
{
    ParsedCommand cmd;
    size_t pos = 0;
    while (pos < line.size() && line[pos] == ' ') ++pos;
    size_t start = pos;
    while (pos < line.size() && line[pos] != ' ') ++pos;
    cmd.command = line.substr(start, pos - start);
    while (pos < line.size() && line[pos] == ' ') ++pos;
    cmd.rawArgs = (pos < line.size()) ? line.substr(pos) : "";

    std::string arg;
    bool inQuote = false;
    for (size_t i = pos; i < line.size(); ++i) {
        char c = line[i];
        if (c == '"') {
            inQuote = !inQuote;
            continue;
        }
        if (c == ' ' && !inQuote) {
            if (!arg.empty()) {
                cmd.args.push_back(arg);
                arg.clear();
            }
            continue;
        }
        arg += c;
    }
    if (!arg.empty()) cmd.args.push_back(arg);
    return cmd;
}

bool DosShell::matchWildcard(const std::string& pattern, const std::string& text)
{
    size_t pi = 0, ti = 0;
    size_t starP = std::string::npos, starT = 0;
    while (ti < text.size()) {
        if (pi < pattern.size() && (pattern[pi] == '?' ||
            std::toupper(static_cast<unsigned char>(pattern[pi])) ==
            std::toupper(static_cast<unsigned char>(text[ti])))) {
            ++pi; ++ti;
        } else if (pi < pattern.size() && pattern[pi] == '*') {
            starP = pi++;
            starT = ti;
        } else if (starP != std::string::npos) {
            pi = starP + 1;
            ti = ++starT;
        } else {
            return false;
        }
    }
    while (pi < pattern.size() && pattern[pi] == '*') ++pi;
    return pi == pattern.size();
}

static std::string formatDosFilename(const std::string& name)
{
    auto dot = name.find('.');
    std::string base = (dot != std::string::npos) ? name.substr(0, dot) : name;
    std::string ext = (dot != std::string::npos) ? name.substr(dot + 1) : "";
    if (base.size() > 8) base = base.substr(0, 8);
    if (ext.size() > 3) ext = ext.substr(0, 3);
    std::string result = base;
    result.append(8 - base.size(), ' ');
    result += ' ';
    result += ext;
    result.append(3 - ext.size(), ' ');
    return result;
}

static std::string formatSize(uintmax_t size)
{
    std::ostringstream ss;
    ss << size;
    std::string s = ss.str();
    std::string out;
    int count = 0;
    for (int i = static_cast<int>(s.size()) - 1; i >= 0; --i) {
        if (count > 0 && count % 3 == 0) out = ',' + out;
        out = s[i] + out;
        ++count;
    }
    return out;
}

static std::string getFileDate(const fs::path& p)
{
    std::error_code ec;
    auto ftime = fs::last_write_time(p, ec);
    if (ec) return "01-01-2026";
    auto tt = std::chrono::system_clock::to_time_t(
        std::chrono::clock_cast<std::chrono::system_clock>(ftime));
    auto* lt = std::localtime(&tt);
    char buf[16];
    std::snprintf(buf, sizeof(buf), "%02d-%02d-%02d",
                  lt->tm_mon + 1, lt->tm_mday, (lt->tm_year + 1900) % 100);
    return buf;
}

static std::string getFileTime(const fs::path& p)
{
    std::error_code ec;
    auto ftime = fs::last_write_time(p, ec);
    if (ec) return "12:00a";
    auto tt = std::chrono::system_clock::to_time_t(
        std::chrono::clock_cast<std::chrono::system_clock>(ftime));
    auto* lt = std::localtime(&tt);
    char buf[16];
    int hour = lt->tm_hour;
    char ampm = 'a';
    if (hour >= 12) { ampm = 'p'; if (hour > 12) hour -= 12; }
    if (hour == 0) hour = 12;
    std::snprintf(buf, sizeof(buf), "%2d:%02d%c", hour, lt->tm_min, ampm);
    return buf;
}

int DosShell::cmdDir(const ParsedCommand& cmd)
{
    std::string pattern = "*.*";
    bool wide = false, paged = false, recursive = false;
    for (auto& arg : cmd.args) {
        std::string upper = arg;
        std::transform(upper.begin(), upper.end(), upper.begin(),
                       [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
        if (upper == "/W") wide = true;
        else if (upper == "/P") paged = true;
        else if (upper == "/S") recursive = true;
        else if (upper[0] != '/') pattern = arg;
    }

    std::string hostDir = m_driveManager.resolvePath(pattern);
    fs::path hostPath(hostDir);
    fs::path searchDir;
    std::string searchPattern;

    if (fs::is_directory(hostPath)) {
        searchDir = hostPath;
        searchPattern = "*.*";
    } else {
        searchDir = hostPath.parent_path();
        searchPattern = hostPath.filename().string();
    }

    if (!fs::exists(searchDir)) {
        printLine("File not found");
        return 1;
    }

    auto doDirListing = [&](const fs::path& dir, const std::string& pat, const std::string& label) -> int {
        int fileCount = 0, dirCount = 0;
        uintmax_t totalSize = 0;
        std::vector<fs::directory_entry> entries;
        std::error_code ec;
        for (auto& entry : fs::directory_iterator(dir, fs::directory_options::skip_permission_denied, ec)) {
            auto fn = entry.path().filename().string();
            if (fn == "." || fn == "..") continue;
            std::string upperFn = fn;
            std::transform(upperFn.begin(), upperFn.end(), upperFn.begin(),
                           [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
            std::string upperPat = pat;
            std::transform(upperPat.begin(), upperPat.end(), upperPat.begin(),
                           [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
            if (!matchWildcard(upperPat, upperFn)) continue;
            entries.push_back(entry);
        }

        if (!label.empty()) {
            printLine();
            printLine(" Directory of " + label);
            printLine();
        }

        if (wide) {
            int col = 0;
            for (auto& e : entries) {
                auto fn = e.path().filename().string();
                bool isDir = e.is_directory();
                if (isDir) {
                    printString("[" + fn + "]");
                    std::string pad;
                    int w = static_cast<int>(fn.size()) + 2;
                    for (int i = w; i < 16; ++i) pad += ' ';
                    printString(pad);
                } else {
                    printString(formatDosFilename(fn));
                    printString("  ");
                }
                dirCount += isDir ? 1 : 0;
                fileCount += isDir ? 0 : 1;
                totalSize += isDir ? 0 : e.file_size();
                if (++col >= 5) { printLine(); col = 0; }
            }
            if (col > 0) printLine();
        } else {
            for (auto& e : entries) {
                auto fn = e.path().filename().string();
                bool isDir = e.is_directory();
                std::string sizeStr = isDir ? "<DIR>" : formatSize(e.file_size());
                std::string dateStr = getFileDate(e.path());
                std::string timeStr = getFileTime(e.path());
                std::string line = formatDosFilename(fn) + " " + sizeStr;
                while (line.size() < 38) line += ' ';
                line += dateStr + "  " + timeStr;
                printLine(line);
                dirCount += isDir ? 1 : 0;
                fileCount += isDir ? 0 : 1;
                totalSize += isDir ? 0 : e.file_size();
            }
        }
        return fileCount + dirCount;
    };

    int totalCount = 0;
    if (recursive) {
        std::function<void(const fs::path&, const std::string&)> recurse;
        recurse = [&](const fs::path& dir, const std::string& base) {
            totalCount += doDirListing(dir, searchPattern, base);
            std::error_code ec2;
            for (auto& entry : fs::directory_iterator(dir, fs::directory_options::skip_permission_denied, ec2)) {
                if (entry.is_directory()) {
                    auto subName = entry.path().filename().string();
                    if (subName == "." || subName == "..") continue;
                    std::string newBase = base.empty() ? subName : base + "\\" + subName;
                    recurse(entry.path(), newBase);
                }
            }
        };
        recurse(searchDir, m_driveManager.getFullDosCwd());
    } else {
        totalCount = doDirListing(searchDir, searchPattern, m_driveManager.getFullDosCwd());
    }

    std::ostringstream summary;
    summary << "     " << totalCount << " file(s)";
    printLine(summary.str());
    return 0;
}

int DosShell::cmdCd(const ParsedCommand& cmd)
{
    if (cmd.args.empty()) {
        printLine(m_driveManager.getFullDosCwd());
        return 0;
    }

    std::string target = cmd.args[0];
    if (target == "\\" || target == "/") {
        m_driveManager.setCurrentDir(m_driveManager.getCurrentDriveLetter(), "");
        return 0;
    }
    if (target == "..") {
        auto cwd = m_driveManager.getFullDosCwd();
        auto pos = cwd.find_last_of('\\');
        if (pos != std::string::npos && pos > 2) {
            m_driveManager.setCurrentDir(m_driveManager.getCurrentDriveLetter(), cwd.substr(0, pos));
        } else {
            m_driveManager.setCurrentDir(m_driveManager.getCurrentDriveLetter(), "");
        }
        return 0;
    }

    if (!m_driveManager.setCurrentDir(m_driveManager.getCurrentDriveLetter(), target)) {
        std::string resolved = m_driveManager.resolvePath(target);
        if (fs::is_directory(resolved)) {
            auto hostPath = m_driveManager.getHostPath(m_driveManager.getCurrentDriveLetter());
            if (!hostPath.empty()) {
                auto rel = fs::path(resolved).lexically_relative(fs::path(hostPath));
                m_driveManager.setCurrentDir(m_driveManager.getCurrentDriveLetter(), rel.string());
                return 0;
            }
        }
        printLine("Invalid directory");
        return 1;
    }
    return 0;
}

int DosShell::cmdMd(const ParsedCommand& cmd)
{
    if (cmd.args.empty()) {
        printLine("Required parameter missing");
        return 1;
    }
    std::string hostPath = m_driveManager.resolvePath(cmd.args[0]);
    std::error_code ec;
    if (!fs::create_directory(hostPath, ec) || ec) {
        printLine("Unable to create directory");
        return 1;
    }
    return 0;
}

int DosShell::cmdRd(const ParsedCommand& cmd)
{
    if (cmd.args.empty()) {
        printLine("Required parameter missing");
        return 1;
    }
    std::string hostPath = m_driveManager.resolvePath(cmd.args[0]);
    std::error_code ec;
    if (!fs::remove(hostPath, ec) || ec) {
        printLine("Invalid path, not directory, or directory not empty");
        return 1;
    }
    return 0;
}

int DosShell::cmdCopy(const ParsedCommand& cmd)
{
    if (cmd.args.size() < 2) {
        printLine("Required parameter missing");
        return 1;
    }
    std::string srcHost = m_driveManager.resolvePath(cmd.args[0]);
    std::string dstHost = m_driveManager.resolvePath(cmd.args[1]);

    if (fs::is_directory(dstHost)) {
        dstHost = (fs::path(dstHost) / fs::path(srcHost).filename()).string();
    }

    std::error_code ec;
    fs::copy_file(srcHost, dstHost, fs::copy_options::overwrite_existing, ec);
    if (ec) {
        printLine("Access denied");
        return 1;
    }
    printLine("        1 file(s) copied");
    return 0;
}

int DosShell::cmdDel(const ParsedCommand& cmd)
{
    if (cmd.args.empty()) {
        printLine("Required parameter missing");
        return 1;
    }
    std::string hostPath = m_driveManager.resolvePath(cmd.args[0]);
    fs::path p(hostPath);

    if (fs::is_directory(p)) {
        printLine("Access denied");
        return 1;
    }

    int count = 0;
    if (p.filename().string().find_first_of("*?") != std::string::npos) {
        auto parent = p.parent_path();
        auto pat = p.filename().string();
        std::error_code ec;
        for (auto& entry : fs::directory_iterator(parent, fs::directory_options::skip_permission_denied, ec)) {
            auto fn = entry.path().filename().string();
            if (matchWildcard(pat, fn)) {
                fs::remove(entry.path());
                ++count;
            }
        }
    } else {
        if (fs::exists(p)) {
            fs::remove(p);
            count = 1;
        }
    }
    if (count == 0) {
        printLine("File not found");
        return 1;
    }
    return 0;
}

int DosShell::cmdRen(const ParsedCommand& cmd)
{
    if (cmd.args.size() < 2) {
        printLine("Required parameter missing");
        return 1;
    }
    std::string srcHost = m_driveManager.resolvePath(cmd.args[0]);
    std::string dstHost = (fs::path(srcHost).parent_path() / cmd.args[1]).string();
    std::error_code ec;
    fs::rename(srcHost, dstHost, ec);
    if (ec) {
        printLine("Access denied");
        return 1;
    }
    return 0;
}

int DosShell::cmdType(const ParsedCommand& cmd)
{
    if (cmd.args.empty()) {
        printLine("Required parameter missing");
        return 1;
    }
    std::string hostPath = m_driveManager.resolvePath(cmd.args[0]);
    std::ifstream file(hostPath);
    if (!file.is_open()) {
        printLine("File not found");
        return 1;
    }
    std::string line;
    while (std::getline(file, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        printLine(line);
    }
    return 0;
}

int DosShell::cmdEcho(const ParsedCommand& cmd)
{
    if (cmd.rawArgs.empty()) {
        printLine(m_echoOn ? "ECHO is on" : "ECHO is off");
        return 0;
    }
    std::string upper = cmd.rawArgs;
    std::transform(upper.begin(), upper.end(), upper.begin(),
                   [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
    if (upper == "ON") { m_echoOn = true; return 0; }
    if (upper == "OFF") { m_echoOn = false; return 0; }
    if (upper == ".") { printLine(); return 0; }
    printLine(cmd.rawArgs);
    return 0;
}

int DosShell::cmdSet(const ParsedCommand& cmd)
{
    if (cmd.args.empty()) {
        for (auto& [k, v] : m_envVars) {
            printLine(k + "=" + v);
        }
        return 0;
    }
    auto raw = cmd.rawArgs;
    auto eq = raw.find('=');
    if (eq == std::string::npos) {
        auto upper = raw;
        std::transform(upper.begin(), upper.end(), upper.begin(),
                       [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
        auto it = m_envVars.find(upper);
        if (it != m_envVars.end()) {
            printLine(upper + "=" + it->second);
        } else {
            printLine("Environment variable " + raw + " not defined");
        }
        return 0;
    }
    std::string name = raw.substr(0, eq);
    std::string value = raw.substr(eq + 1);
    while (!name.empty() && name.back() == ' ') name.pop_back();
    while (!value.empty() && value.front() == ' ') value.erase(value.begin());
    setEnvVar(name, value);
    return 0;
}

int DosShell::cmdPath(const ParsedCommand& cmd)
{
    if (cmd.args.empty()) {
        printLine("PATH=" + getEnvVar("PATH"));
        return 0;
    }
    setEnvVar("PATH", cmd.rawArgs);
    return 0;
}

int DosShell::cmdPrompt(const ParsedCommand& cmd)
{
    if (cmd.args.empty()) {
        setEnvVar("PROMPT", "$P$G");
        m_promptTemplate = "$P$G";
    } else {
        setEnvVar("PROMPT", cmd.rawArgs);
        m_promptTemplate = cmd.rawArgs;
    }
    return 0;
}

int DosShell::cmdCls(const ParsedCommand&)
{
    uint16_t cols = m_memory.read16(0x44A);
    uint8_t maxRow = m_memory.read8(0x484);
    if (cols == 0) cols = 80;
    if (maxRow == 0) maxRow = 24;

    for (uint8_t r = 0; r <= maxRow; ++r) {
        for (uint16_t c = 0; c < cols; ++c) {
            uint32_t off = (r * cols + c) * 2;
            m_memory.write8(0xB8000 + off, ' ');
            m_memory.write8(0xB8000 + off + 1, 0x07);
        }
    }
    m_memory.write8(0x450, 0);
    m_memory.write8(0x451, 0);
    return 0;
}

int DosShell::cmdVer(const ParsedCommand&)
{
    printLine();
    printLine("Fador's DOS Emulator [Version 6.22]");
    printLine();
    return 0;
}

int DosShell::cmdVol(const ParsedCommand&)
{
    char drive = m_driveManager.getCurrentDriveLetter();
    auto label = m_driveManager.getVolumeLabel(drive);
    if (label.empty()) label = "no label";
    std::string line = " Volume in drive ";
    line += drive;
    line += " is ";
    line += label;
    printLine(line);
    return 0;
}

int DosShell::cmdDate(const ParsedCommand&)
{
    auto now = std::time(nullptr);
    auto* lt = std::localtime(&now);
    const char* days[] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};
    char buf[64];
    std::snprintf(buf, sizeof(buf), "Current date is %s %02d-%02d-%04d",
                  days[lt->tm_wday], lt->tm_mon + 1, lt->tm_mday, lt->tm_year + 1900);
    printLine(buf);
    return 0;
}

int DosShell::cmdTime(const ParsedCommand&)
{
    auto now = std::time(nullptr);
    auto* lt = std::localtime(&now);
    char buf[64];
    std::snprintf(buf, sizeof(buf), "Current time is %02d:%02d:%02d.%02d",
                  lt->tm_hour, lt->tm_min, lt->tm_sec, 0);
    printLine(buf);
    return 0;
}

int DosShell::cmdMem(const ParsedCommand&)
{
    printLine();
    printLine("Memory Type        Total  =     Used  +     Free");
    printLine("----------------  -------     -------     -------");

    uint16_t current = 0x0800;
    uint32_t totalConventional = 0;
    uint32_t usedConventional = 0;
    uint32_t freeConventional = 0;

    while (true) {
        auto mcb = m_dos.readMCB(current);
        uint32_t sizeBytes = static_cast<uint32_t>(mcb.size) * 16;
        totalConventional += sizeBytes;
        if (mcb.owner != 0) {
            usedConventional += sizeBytes;
        } else {
            freeConventional += sizeBytes;
        }
        if (mcb.type == 'Z') break;
        uint32_t next = static_cast<uint32_t>(current) + mcb.size + 1;
        if (next > 0xFFFF || next > 0x9FFF) break;
        current = static_cast<uint16_t>(next);
    }

    constexpr uint32_t kTotalConventional = 640 * 1024;
    constexpr uint32_t kTotalExtended = 15 * 1024 * 1024;

    auto fmtLine = [](const std::string& label, uint32_t total, uint32_t used, uint32_t free) -> std::string {
        char buf[128];
        std::snprintf(buf, sizeof(buf), "%-17s %7uK      %7uK      %7uK",
                      label.c_str(), total / 1024, used / 1024, free / 1024);
        return buf;
    };

    printLine(fmtLine("Conventional", kTotalConventional, usedConventional, freeConventional));
    printLine(fmtLine("Extended (XMS)", kTotalExtended, 0, kTotalExtended));
    printLine("----------------  -------     -------     -------");
    printLine(fmtLine("Total memory", kTotalConventional + kTotalExtended,
                      usedConventional, freeConventional + kTotalExtended));
    printLine();
    return 0;
}

int DosShell::cmdHelp(const ParsedCommand&)
{
    printLine("For more information on a specific command, type HELP command-name");
    printLine();
    struct HelpEntry { const char* name; const char* desc; };
    HelpEntry entries[] = {
        {"CD",       "Displays the name of or changes the current directory."},
        {"CLS",      "Clears the screen."},
        {"COPY",     "Copies one or more files to another location."},
        {"DATE",     "Displays or sets the date."},
        {"DEL",      "Deletes one or more files."},
        {"DIR",      "Displays a list of files and subdirectories in a directory."},
        {"ECHO",     "Displays messages, or turns command echoing on or off."},
        {"EXIT",     "Quits the command interpreter."},
        {"HELP",     "Provides Help information for DOS commands."},
        {"MD",       "Creates a directory."},
        {"MEM",      "Displays the amount of used and free memory."},
        {"MOUNT",    "Mounts a host directory as a DOS drive."},
        {"MOVE",     "Moves files and renames files and directories."},
        {"PATH",     "Displays or sets a search path for executable files."},
        {"PROMPT",   "Changes the command prompt."},
        {"RD",       "Removes a directory."},
        {"REN",      "Renames a file or files."},
        {"SET",      "Displays, sets, or removes environment variables."},
        {"TIME",     "Displays or sets the system time."},
        {"TREE",     "Graphically displays the directory structure of a drive."},
        {"TYPE",     "Displays the contents of a text file."},
        {"UNMOUNT",  "Unmounts a previously mounted drive."},
        {"VER",      "Displays the DOS version."},
        {"VERIFY",   "Tells DOS whether to verify that files are written correctly."},
        {"VOL",      "Displays a disk volume label and serial number."},
    };
    for (auto& e : entries) {
        std::string line = "  ";
        line += e.name;
        line.append(12 - std::min<size_t>(strlen(e.name), 12), ' ');
        line += e.desc;
        printLine(line);
    }
    return 0;
}

int DosShell::cmdMount(const ParsedCommand& cmd)
{
    if (cmd.args.empty()) {
        for (int i = 0; i < 26; ++i) {
            char letter = static_cast<char>('A' + i);
            if (m_driveManager.isMounted(letter)) {
                std::string line;
                line += letter;
                line += ": => ";
                line += m_driveManager.getHostPath(letter);
                printLine(line);
            }
        }
        return 0;
    }
    if (cmd.args.size() < 2) {
        printLine("Usage: MOUNT <drive> <host_path>");
        return 1;
    }
    char drive = static_cast<char>(std::toupper(static_cast<unsigned char>(cmd.args[0][0])));
    std::string hostPath = cmd.args[1];

    if (!fs::is_directory(hostPath)) {
        printLine("Host path does not exist or is not a directory");
        return 1;
    }
    if (m_driveManager.mount(drive, hostPath)) {
        std::string msg;
        msg += drive;
        msg += ": mounted to ";
        msg += hostPath;
        printLine(msg);
        return 0;
    }
    printLine("Mount failed");
    return 1;
}

int DosShell::cmdUnmount(const ParsedCommand& cmd)
{
    if (cmd.args.empty()) {
        printLine("Usage: UNMOUNT <drive>");
        return 1;
    }
    char drive = static_cast<char>(std::toupper(static_cast<unsigned char>(cmd.args[0][0])));
    if (m_driveManager.unmount(drive)) {
        printLine("Drive unmounted");
        return 0;
    }
    printLine("Drive not mounted");
    return 1;
}

int DosShell::cmdExit(const ParsedCommand&)
{
    m_running = false;
    return 0;
}

int DosShell::cmdBreak(const ParsedCommand&)
{
    return 0;
}

int DosShell::cmdVerify(const ParsedCommand&)
{
    return 0;
}

int DosShell::cmdTree(const ParsedCommand& cmd)
{
    std::string startPath;
    if (cmd.args.empty() || cmd.args[0][0] == '/') {
        startPath = m_driveManager.getFullDosCwd();
    } else {
        startPath = cmd.args[0];
    }

    std::string hostStart = m_driveManager.resolvePath(startPath);
    if (!fs::is_directory(hostStart)) {
        printLine("Invalid path");
        return 1;
    }

    std::function<void(const fs::path&, const std::string&)> printTree;
    printTree = [&](const fs::path& dir, const std::string& prefix) {
        std::vector<fs::directory_entry> dirs;
        std::error_code ec;
        for (auto& entry : fs::directory_iterator(dir, fs::directory_options::skip_permission_denied, ec)) {
            if (entry.is_directory()) {
                auto fn = entry.path().filename().string();
                if (fn != "." && fn != "..") dirs.push_back(entry);
            }
        }
        for (size_t i = 0; i < dirs.size(); ++i) {
            bool last = (i == dirs.size() - 1);
            auto connector = last ? "+---" : "|---";
            auto fn = dirs[i].path().filename().string();
            printLine(prefix + connector + fn);
            printTree(dirs[i].path(), prefix + (last ? "    " : "|   "));
        }
    };

    printLine(hostStart);
    printTree(fs::path(hostStart), "");
    return 0;
}

int DosShell::cmdMove(const ParsedCommand& cmd)
{
    if (cmd.args.size() < 2) {
        printLine("Required parameter missing");
        return 1;
    }
    std::string srcHost = m_driveManager.resolvePath(cmd.args[0]);
    std::string dstHost = m_driveManager.resolvePath(cmd.args[1]);

    if (fs::is_directory(dstHost)) {
        dstHost = (fs::path(dstHost) / fs::path(srcHost).filename()).string();
    }

    std::error_code ec;
    fs::rename(srcHost, dstHost, ec);
    if (ec) {
        printLine("Access denied");
        return 1;
    }
    printLine("        1 file(s) moved");
    return 0;
}

int DosShell::executeExternal(const ParsedCommand& cmd)
{
    std::string name = cmd.command;
    std::string upperName = name;
    std::transform(upperName.begin(), upperName.end(), upperName.begin(),
                   [](unsigned char c) { return static_cast<char>(std::toupper(c)); });

    auto hasExt = [](const std::string& n) {
        auto dot = n.rfind('.');
        if (dot == std::string::npos) return false;
        std::string ext = n.substr(dot);
        std::transform(ext.begin(), ext.end(), ext.begin(),
                       [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
        return ext == ".COM" || ext == ".EXE" || ext == ".BAT";
    };

    auto tryLoad = [&](const std::string& path) -> std::string {
        if (hasExt(path)) {
            std::string resolved = m_driveManager.resolvePath(path);
            if (fs::exists(resolved)) return resolved;
            return "";
        }
        static const char* exts[] = {".COM", ".EXE", ".BAT"};
        for (auto ext : exts) {
            std::string candidate = path + ext;
            std::string resolved = m_driveManager.resolvePath(candidate);
            if (fs::exists(resolved)) return resolved;
        }
        return "";
    };

    std::string found = tryLoad(name);
    if (found.empty()) {
        found = searchPath(name);
    }

    if (found.empty()) {
        printLine("Bad command or file name");
        return 1;
    }

    std::string lowerFound = found;
    std::transform(lowerFound.begin(), lowerFound.end(), lowerFound.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (lowerFound.size() >= 4 && lowerFound.substr(lowerFound.size() - 4) == ".bat") {
        printLine("Batch files not yet supported");
        return 1;
    }

    uint16_t childSeg = m_dos.allocateMemory(0xFFFF, 0xFFFF);
    if (childSeg == 0) {
        printLine("Insufficient memory");
        return 1;
    }

    bool isCom = false;
    {
        std::string lowerPath = found;
        std::transform(lowerPath.begin(), lowerPath.end(), lowerPath.begin(),
                       [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        isCom = lowerPath.size() >= 4 && lowerPath.substr(lowerPath.size() - 4) == ".com";
    }

    std::string args;
    for (size_t i = 0; i < cmd.args.size(); ++i) {
        if (i > 0) args += ' ';
        args += cmd.args[i];
    }

    uint16_t parentPSP = m_dos.getPSPSegment();
    bool loaded = false;
    if (isCom) {
        loaded = m_loader.loadCOM(found, childSeg, m_dos, args);
    } else {
        loaded = m_loader.loadEXE(found, childSeg, m_dos, args);
    }

    if (!loaded) {
        m_dos.freeMemory(childSeg);
        printLine("Program load error");
        return 1;
    }

    m_dos.setProgramDir(found);
    m_decoder.syncSegments();
    m_dos.resetTerminated();

    while (!m_dos.isTerminated()) {
        m_decoder.step();
        m_cpu.addCycles(1);
    }

    int exitCode = static_cast<int>(m_dos.getExitCode());

    m_dos.setPSPSegment(parentPSP);
    m_dos.freeMemory(childSeg);
    m_dos.resetTerminated();
    m_dos.clearExecTriggered();

    return exitCode;
}

std::string DosShell::searchPath(const std::string& name)
{
    auto hasExt = [](const std::string& n) {
        auto dot = n.rfind('.');
        if (dot == std::string::npos) return false;
        std::string ext = n.substr(dot);
        std::transform(ext.begin(), ext.end(), ext.begin(),
                       [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
        return ext == ".COM" || ext == ".EXE" || ext == ".BAT";
    };

    std::string pathEnv = getEnvVar("PATH");
    if (pathEnv.empty()) return "";

    std::vector<std::string> dirs;
    std::istringstream ss(pathEnv);
    std::string token;
    while (std::getline(ss, token, ';')) {
        if (!token.empty()) dirs.push_back(token);
    }

    static const char* exts[] = {".COM", ".EXE", ".BAT"};
    for (auto& dir : dirs) {
        fs::path dirPath = fs::path(dir);
        if (dirPath.is_relative()) {
            std::string hostRoot = m_driveManager.getHostPath(m_driveManager.getCurrentDriveLetter());
            if (!hostRoot.empty()) dirPath = fs::path(hostRoot) / dirPath;
        }
        if (hasExt(name)) {
            fs::path full = dirPath / name;
            if (fs::exists(full)) return full.string();
        } else {
            for (auto ext : exts) {
                fs::path full = dirPath / (name + ext);
                if (fs::exists(full)) return full.string();
            }
        }
    }
    return "";
}

} // namespace fador::hw
