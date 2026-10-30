#include "test_framework.hpp"
#include "utils/CrashHandler.hpp"
#include <iostream>

int main() {
    fador::utils::installCrashHandler([](const std::string &msg) {
        std::cerr << "CRASH: " << msg << std::endl;
    });
    return fador::test::run_all_tests();
}
