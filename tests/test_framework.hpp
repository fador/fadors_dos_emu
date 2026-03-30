#pragma once
#include <iostream>
#include <cstdlib>
#include <vector>
#include <string>
#include <stdexcept>
#include <functional>
#include <set>

namespace fador::test {

struct TestCase {
    const char* name;
    void (*func)();
};

inline std::vector<TestCase>& get_tests() {
    static std::vector<TestCase> tests;
    return tests;
}

struct RegisterTest {
    RegisterTest(const char* name, void (*func)()) {
        get_tests().push_back({name, func});
    }
};

static std::set<std::string> g_executed_sections;
static std::vector<std::string> g_stack;
static bool g_entered_new;

inline bool enter_section(const char* name) {
    std::string path;
    for (auto& s : g_stack) path += s + "/";
    path += name;

    if (g_executed_sections.count(path)) return false;
    if (g_entered_new) return false;

    g_stack.push_back(name);
    g_entered_new = true;
    return true;
}

struct SectionScope {
    bool run;
    SectionScope(const char* name) : run(enter_section(name)) {}
    ~SectionScope() { if (run) g_stack.pop_back(); }
    operator bool() const { return run; }
};

inline int run_all_tests() {
    const char* envFilter = std::getenv("TEST_FILTER");
    std::string testFilter = envFilter ? std::string(envFilter) : std::string();
    int passed = 0;
    int failed = 0;
    auto& tests = get_tests();
    
    std::cout << "===============================================================" << std::endl;
    std::cout << "Running " << tests.size() << " test cases..." << std::endl;
    std::cout << "===============================================================" << std::endl;

    for (auto& test : tests) {
        if (!testFilter.empty()) {
            if (std::string(test.name).find(testFilter) == std::string::npos)
                continue;
        }
        std::cout << "[ RUN      ] " << test.name << std::endl;
        std::set<std::string> local_executed;
        bool te_failed = false;
        while (true) {
            g_stack.clear();
            g_entered_new = false;
            g_executed_sections = local_executed;
            try {
                test.func();
            } catch (const std::exception& e) {
                std::cerr << "[  FAILED  ] " << e.what() << std::endl;
                te_failed = true;
                break;
            } catch (...) {
                std::cerr << "[  FAILED  ] Unknown exception" << std::endl;
                te_failed = true;
                break;
            }

            if (!g_entered_new) break;
            
            std::string path;
            for (auto& s : g_stack) path += s + "/";
            if (!path.empty()) path.pop_back();
            local_executed.insert(path);
        }
        
        if (!te_failed) {
            std::cout << "[       OK ] " << test.name << std::endl;
            passed++;
        } else {
            failed++;
        }
    }

    std::cout << "===============================================================" << std::endl;
    std::cout << "TEST SUMMARY" << std::endl;
    std::cout << "  Passed: " << passed << std::endl;
    std::cout << "  Failed: " << failed << std::endl;
    std::cout << "===============================================================" << std::endl;

    return failed == 0 ? 0 : 1;
}

} // namespace fador::test

#define TEST_FRAMEWORK_CONCAT(a, b) a ## b
#define TEST_FRAMEWORK_CONCAT_EXPANDED(a, b) TEST_FRAMEWORK_CONCAT(a, b)

#define TEST_CASE(name, tags) \
    static void TEST_FRAMEWORK_CONCAT_EXPANDED(test_func_, __LINE__)(); \
    static fador::test::RegisterTest TEST_FRAMEWORK_CONCAT_EXPANDED(reg_, __LINE__)(name, TEST_FRAMEWORK_CONCAT_EXPANDED(test_func_, __LINE__)); \
    static void TEST_FRAMEWORK_CONCAT_EXPANDED(test_func_, __LINE__)()

#define SECTION(name) \
    if (fador::test::SectionScope _scope{name})

#define REQUIRE(cond) \
    if (!(cond)) { \
        throw std::runtime_error("REQUIRE failed: " #cond " at " __FILE__ ":" + std::to_string(__LINE__)); \
    }

#define CHECK(cond) \
    if (!(cond)) { \
        std::cerr << "    CHECK failed: " << #cond << " at " << __FILE__ << ":" << __LINE__ << std::endl; \
    }
