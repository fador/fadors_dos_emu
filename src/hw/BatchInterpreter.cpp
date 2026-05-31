#include "BatchInterpreter.hpp"
#include "DosShell.hpp"
#include "DriveManager.hpp"
#include "../utils/Logger.hpp"
#include <algorithm>
#include <fstream>
#include <filesystem>
#include <sstream>
#include <cctype>

namespace fs = std::filesystem;

namespace fador::hw {

BatchInterpreter::BatchInterpreter(DosShell& shell)
    : m_shell(shell)
{
}

std::string BatchInterpreter::trim(const std::string& s)
{
    size_t start = 0;
    while (start < s.size() && (s[start] == ' ' || s[start] == '\t' || s[start] == '\r' || s[start] == '\n'))
        ++start;
    size_t end = s.size();
    while (end > start && (s[end - 1] == ' ' || s[end - 1] == '\t' || s[end - 1] == '\r' || s[end - 1] == '\n'))
        --end;
    return s.substr(start, end - start);
}

std::string BatchInterpreter::toUpper(const std::string& s)
{
    std::string result = s;
    std::transform(result.begin(), result.end(), result.begin(),
                   [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
    return result;
}

std::vector<std::string> BatchInterpreter::readBatFile(const std::string& path)
{
    std::vector<std::string> lines;
    std::ifstream file(path);
    if (!file.is_open()) {
        LOG_ERROR("BatchInterpreter: cannot open %s", path.c_str());
        return lines;
    }
    std::string line;
    while (std::getline(file, line)) {
        if (!line.empty() && line.back() == '\r')
            line.pop_back();
        lines.push_back(line);
    }
    return lines;
}

std::string BatchInterpreter::getParam(int index) const
{
    if (index == 0)
        return m_batName;
    int actual = index - 1 + m_shiftOffset;
    if (actual < 0 || actual >= static_cast<int>(m_params.size()))
        return "";
    return m_params[actual];
}

std::string BatchInterpreter::expandVariables(const std::string& line)
{
    std::string result;
    result.reserve(line.size());

    for (size_t i = 0; i < line.size(); ++i) {
        if (line[i] == '%') {
            // %% -> literal %
            if (i + 1 < line.size() && line[i + 1] == '%') {
                result += '%';
                ++i;
                continue;
            }
            // %0-%9 -> parameter
            if (i + 1 < line.size() && std::isdigit(static_cast<unsigned char>(line[i + 1]))) {
                int paramIdx = line[i + 1] - '0';
                result += getParam(paramIdx);
                ++i;
                continue;
            }
            // %VAR% -> environment variable
            auto endPos = line.find('%', i + 1);
            if (endPos != std::string::npos) {
                std::string varName = line.substr(i + 1, endPos - i - 1);
                if (!varName.empty()) {
                    std::string upperName = toUpper(varName);
                    if (upperName == "ERRORLEVEL") {
                        result += std::to_string(m_shell.getErrorLevel());
                    } else {
                        // Check local vars first, then shell env
                        auto it = m_vars.find(upperName);
                        if (it != m_vars.end()) {
                            result += it->second;
                        } else {
                            result += m_shell.getEnvVar(upperName);
                        }
                    }
                    i = endPos;
                    continue;
                }
            }
            result += '%';
        } else {
            result += line[i];
        }
    }
    return result;
}

int BatchInterpreter::findLabel(const std::string& label)
{
    std::string upperLabel = toUpper(trim(label));
    for (size_t i = 0; i < m_lines.size(); ++i) {
        std::string line = trim(m_lines[i]);
        if (!line.empty() && line[0] == ':') {
            std::string lbl = toUpper(trim(line.substr(1)));
            if (lbl == upperLabel) {
                return static_cast<int>(i);
            }
        }
    }
    return -1;
}

int BatchInterpreter::execute(const std::string& batFilePath, const std::vector<std::string>& args)
{
    m_params = args;
    m_lines = readBatFile(batFilePath);
    m_vars.clear();
    m_errorLevel = 0;
    m_echoOn = true;
    m_exitBatch = false;
    m_shiftOffset = 0;
    m_gotoTarget = SIZE_MAX;
    m_callStack.clear();

    // Extract batch file name for %0
    fs::path p(batFilePath);
    m_batName = toUpper(p.stem().string());

    return executeFrom(0);
}

int BatchInterpreter::executeFrom(size_t startLine)
{
    int lastRc = 0;
    for (size_t pc = startLine; pc < m_lines.size() && !m_exitBatch; ++pc) {
        if (m_gotoTarget != SIZE_MAX) {
            pc = m_gotoTarget;
            m_gotoTarget = SIZE_MAX;
            if (pc >= m_lines.size()) break;
        }

        std::string rawLine = m_lines[pc];
        std::string trimmed = trim(rawLine);
        if (trimmed.empty()) continue;

        // Skip labels
        if (trimmed[0] == ':') continue;

        lastRc = executeLine(trimmed);
    }
    return lastRc;
}

int BatchInterpreter::executeLine(const std::string& line)
{
    std::string expanded = expandVariables(line);
    std::string trimmed = trim(expanded);
    if (trimmed.empty()) return 0;

    int exitCode = 0;
    if (handleControlStatement(trimmed, exitCode))
        return exitCode;

    // Delegate to shell for regular commands
    int rc = m_shell.executeCommand(expanded);
    m_errorLevel = rc;
    return rc;
}

bool BatchInterpreter::handleControlStatement(const std::string& line, int& exitCode)
{
    std::string trimmed = trim(line);
    std::string upper = toUpper(trimmed);

    // REM
    if (upper.substr(0, 4) == "REM " || upper == "REM") {
        exitCode = 0;
        return true;
    }

    // ECHO ON/OFF
    if (upper == "ECHO ON") {
        m_echoOn = true;
        m_shell.setEchoOn(true);
        exitCode = 0;
        return true;
    }
    if (upper == "ECHO OFF") {
        m_echoOn = false;
        m_shell.setEchoOn(false);
        exitCode = 0;
        return true;
    }
    // ECHO.
    if (upper == "ECHO.") {
        m_shell.executeCommand("ECHO.");
        exitCode = 0;
        return true;
    }
    // ECHO text
    if (upper.substr(0, 5) == "ECHO ") {
        std::string rest = trim(trimmed.substr(5));
        m_shell.executeCommand("ECHO " + rest);
        exitCode = 0;
        return true;
    }

    // SET
    if (upper == "SET") {
        m_shell.executeCommand("SET");
        exitCode = 0;
        return true;
    }
    if (upper.substr(0, 4) == "SET ") {
        std::string rest = trim(trimmed.substr(4));
        auto eq = rest.find('=');
        if (eq != std::string::npos) {
            std::string name = trim(rest.substr(0, eq));
            std::string value = trim(rest.substr(eq + 1));
            std::string upperName = toUpper(name);
            m_vars[upperName] = value;
            m_shell.setEnvVar(upperName, value);
        } else {
            m_shell.executeCommand("SET " + rest);
        }
        exitCode = 0;
        return true;
    }

    // IF
    if (upper.substr(0, 3) == "IF ") {
        std::string rest = trim(trimmed.substr(3));
        std::string upperRest = toUpper(rest);
        bool negated = false;
        if (upperRest.substr(0, 4) == "NOT ") {
            negated = true;
            rest = trim(rest.substr(4));
            upperRest = toUpper(rest);
        }

        // IF EXIST
        if (upperRest.substr(0, 6) == "EXIST ") {
            std::string afterExist = trim(rest.substr(6));
            std::string filename;
            std::string cmd;
            if (!afterExist.empty() && afterExist[0] == '"') {
                auto closeQ = afterExist.find('"', 1);
                if (closeQ != std::string::npos) {
                    filename = afterExist.substr(1, closeQ - 1);
                    cmd = trim(afterExist.substr(closeQ + 1));
                }
            } else {
                auto sp = afterExist.find(' ');
                if (sp != std::string::npos) {
                    filename = afterExist.substr(0, sp);
                    cmd = trim(afterExist.substr(sp + 1));
                } else {
                    filename = afterExist;
                }
            }
            bool exists = fs::exists(filename);
            if (negated) exists = !exists;
            if (exists && !cmd.empty()) {
                exitCode = executeLine(cmd);
            } else {
                exitCode = 0;
            }
            return true;
        }

        // IF ERRORLEVEL
        if (upperRest.substr(0, 10) == "ERRORLEVEL") {
            std::string after = trim(rest.substr(10));
            auto spacePos = after.find(' ');
            if (spacePos != std::string::npos) {
                std::string numStr = after.substr(0, spacePos);
                int threshold = 0;
                try { threshold = std::stoi(numStr); } catch (...) { exitCode = 0; return true; }
                bool cond = m_shell.getErrorLevel() >= threshold;
                if (negated) cond = !cond;
                if (cond) {
                    std::string cmd = trim(after.substr(spacePos + 1));
                    exitCode = executeLine(cmd);
                } else {
                    exitCode = 0;
                }
                return true;
            }
            exitCode = 0;
            return true;
        }

        // IF string==string
        auto eqPos = rest.find("==");
        if (eqPos != std::string::npos) {
            std::string left = trim(rest.substr(0, eqPos));
            std::string afterEq = trim(rest.substr(eqPos + 2));
            std::string right;
            std::string cmd;
            if (!afterEq.empty() && afterEq[0] == '"') {
                auto closeQ = afterEq.find('"', 1);
                if (closeQ != std::string::npos) {
                    right = afterEq.substr(1, closeQ - 1);
                    cmd = trim(afterEq.substr(closeQ + 1));
                } else {
                    right = afterEq.substr(1);
                }
            } else {
                auto sp = afterEq.find(' ');
                if (sp != std::string::npos) {
                    right = afterEq.substr(0, sp);
                    cmd = trim(afterEq.substr(sp + 1));
                } else {
                    right = afterEq;
                }
            }
            bool cond = (toUpper(left) == toUpper(right));
            if (negated) cond = !cond;
            if (cond && !cmd.empty()) {
                exitCode = executeLine(cmd);
            } else {
                exitCode = 0;
            }
            return true;
        }

        exitCode = 0;
        return true;
    }

    // GOTO
    if (upper.substr(0, 5) == "GOTO ") {
        std::string label = trim(trimmed.substr(5));
        if (!label.empty() && label[0] == ':')
            label = label.substr(1);
        int target = findLabel(label);
        if (target >= 0) {
            m_gotoTarget = static_cast<size_t>(target);
            exitCode = 0;
        } else {
            LOG_ERROR("BatchInterpreter: label %s not found", label.c_str());
            exitCode = 1;
        }
        return true;
    }

    // SHIFT
    if (upper == "SHIFT") {
        ++m_shiftOffset;
        exitCode = 0;
        return true;
    }

    // EXIT /B code
    if (upper.size() >= 7 && upper.substr(0, 5) == "EXIT " && upper[5] == '/') {
        std::string afterB = trim(trimmed.substr(6));
        if (!afterB.empty() && (afterB[0] == 'B' || afterB[0] == 'b')) {
            std::string codeStr = trim(afterB.substr(1));
            int code = 0;
            if (!codeStr.empty()) {
                try { code = std::stoi(codeStr); } catch (...) { code = 0; }
            }
            m_exitBatch = true;
            exitCode = code;
            return true;
        }
    }
    if (upper == "EXIT") {
        m_exitBatch = true;
        exitCode = 0;
        return true;
    }
    if (upper.substr(0, 5) == "EXIT ") {
        std::string after = trim(trimmed.substr(5));
        int code = 0;
        try { code = std::stoi(after); } catch (...) { code = 0; }
        m_exitBatch = true;
        exitCode = code;
        return true;
    }

    // CALL
    if (upper.substr(0, 5) == "CALL ") {
        std::string rest = trim(trimmed.substr(5));
        std::string upperRest = toUpper(rest);

        // Parse bat path and arguments
        std::string batPath;
        std::vector<std::string> callArgs;
        auto spacePos = rest.find(' ');
        if (spacePos != std::string::npos) {
            batPath = rest.substr(0, spacePos);
            std::string argStr = trim(rest.substr(spacePos + 1));
            std::istringstream ss(argStr);
            std::string tok;
            while (ss >> tok) callArgs.push_back(tok);
        } else {
            batPath = rest;
        }

        std::string upperBat = toUpper(batPath);
        if (upperBat.size() >= 4 && upperBat.substr(upperBat.size() - 4) == ".BAT") {
            BatchInterpreter sub(m_shell);
            int rc = sub.execute(batPath, callArgs);
            m_errorLevel = rc;
            exitCode = rc;
        } else {
            int rc = m_shell.executeCommand(rest);
            m_errorLevel = rc;
            exitCode = rc;
        }
        return true;
    }

    // FOR
    if (upper.substr(0, 4) == "FOR ") {
        std::string rest = trim(trimmed.substr(4));
        std::string upperRest = toUpper(rest);
        if (rest.size() < 4 || rest[0] != '%' || rest[1] == '%') {
            exitCode = 1;
            return true;
        }
        char varChar = rest[1];
        rest = trim(rest.substr(2));
        upperRest = toUpper(rest);

        // Find " IN " - may be at start or after whitespace
        auto inPos = upperRest.find(" IN ");
        if (inPos == std::string::npos) {
            if (upperRest.substr(0, 3) == "IN ")
                inPos = 0;
            else { exitCode = 1; return true; }
        }

        rest = trim(rest.substr(inPos + (inPos == 0 ? 3 : 4)));
        upperRest = toUpper(rest);

        if (rest.empty() || rest[0] != '(') { exitCode = 1; return true; }
        auto closeParen = rest.find(')');
        if (closeParen == std::string::npos) { exitCode = 1; return true; }

        std::string setStr = trim(rest.substr(1, closeParen - 1));
        rest = trim(rest.substr(closeParen + 1));
        upperRest = toUpper(rest);

        auto doPos = upperRest.find("DO ");
        if (doPos == std::string::npos) { exitCode = 1; return true; }
        std::string cmdTemplate = trim(rest.substr(doPos + 3));

        std::vector<std::string> items;
        std::istringstream ss(setStr);
        std::string item;
        while (ss >> item) {
            items.push_back(item);
        }

        int lastRc = 0;
        for (const auto& it : items) {
            std::string cmd = cmdTemplate;
            std::string searchStr = "%" + std::string(1, varChar);
            size_t pos = 0;
            while ((pos = cmd.find(searchStr, pos)) != std::string::npos) {
                cmd.replace(pos, searchStr.size(), it);
                pos += it.size();
            }
            lastRc = executeLine(cmd);
            if (m_exitBatch) break;
        }
        exitCode = lastRc;
        return true;
    }

    return false;
}

} // namespace fador::hw
