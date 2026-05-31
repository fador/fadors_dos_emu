#pragma once
#include <string>
#include <vector>
#include <map>
#include <functional>

namespace fador::hw {

class DosShell;

class BatchInterpreter {
public:
    explicit BatchInterpreter(DosShell& shell);
    ~BatchInterpreter() = default;

    int execute(const std::string& batFilePath, const std::vector<std::string>& args = {});

private:
    DosShell& m_shell;

    std::string m_batName;
    std::vector<std::string> m_lines;
    std::vector<std::string> m_params;
    std::map<std::string, std::string> m_vars;
    int m_errorLevel = 0;
    bool m_echoOn = true;
    bool m_exitBatch = false;
    int m_shiftOffset = 0;
    size_t m_gotoTarget = SIZE_MAX;

    struct CallFrame {
        size_t returnLine;
        std::vector<std::string> savedParams;
        int savedShiftOffset;
        size_t savedGotoTarget;
    };
    std::vector<CallFrame> m_callStack;

    std::vector<std::string> readBatFile(const std::string& path);
    int executeFrom(size_t startLine);
    int executeLine(const std::string& line);
    std::string expandVariables(const std::string& line);
    int findLabel(const std::string& label);
    bool handleControlStatement(const std::string& line, int& exitCode);
    std::string getParam(int index) const;
    static std::string trim(const std::string& s);
    static std::string toUpper(const std::string& s);
};

} // namespace fador::hw
