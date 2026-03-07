cmake_minimum_required(VERSION 3.20)
project(FadorsDOSEmulator VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF) # Ensure strict standard C++

# Define the source directory
set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)


# Gather all source files EXCEPT main.cpp, and ensure HIMEM.cpp is included
file(GLOB_RECURSE SRC_FILES "${SRC_DIR}/*.cpp")
list(REMOVE_ITEM SRC_FILES "${SRC_DIR}/main.cpp")
# Ensure HIMEM.cpp is included (in case GLOB_RECURSE misses nested folders or symlinks)
list(APPEND SRC_FILES "${SRC_DIR}/memory/himem/HIMEM.cpp")

# Define an internal static library for the emulator core
add_library(fadors_core STATIC ${SRC_FILES})
target_include_directories(fadors_core PUBLIC ${SRC_DIR})

# Define the main emulator executable
add_executable(fadors_emu ${SRC_DIR}/main.cpp)
target_link_libraries(fadors_emu PRIVATE fadors_core)

# Compiler warnings
if(MSVC)
    target_compile_options(fadors_emu PRIVATE /W4)
    target_compile_options(fadors_core PRIVATE /W4)
else()
    target_compile_options(fadors_emu PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(fadors_core PRIVATE -Wall -Wextra -Wpedantic)
endif()

# -----------------
# Testing Subsystem
# -----------------
enable_testing()


set(TEST_FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_main.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_cpu_behavior.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_instruction_decoder.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_interrupts.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_hw_devices.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_bios.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_dos.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_dos_memory.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_dos_directories.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_dos_drive.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_dos_search.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_addressing_modes.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_assembler.cpp"
)

add_executable(fadors_tests ${TEST_FILES})
target_link_libraries(fadors_tests PRIVATE fadors_core)
target_include_directories(fadors_tests PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/tests")

add_test(NAME fadors_tests COMMAND fadors_tests)
