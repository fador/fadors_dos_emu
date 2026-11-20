cmake_minimum_required(VERSION 3.20)
project(FadorsDOSEmulator VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF) # Ensure strict standard C++

option(FADOR_ENABLE_DEBUG_DIAGNOSTICS
    "Enable temporary investigation diagnostics"
    OFF)
option(FADOR_SOFT_X87
    "Use the portable software x87 back end even where the host has a native x87"
    OFF)

# Optional SDL2 graphics backend
find_package(SDL2 QUIET)

if(WIN32 AND NOT SDL2_FOUND)
    message(STATUS "SDL2 not found — attempting to download and build SDL2 from source")
    include(FetchContent)
    
    # Disable SDL2 test applications and installation to speed up the subproject build
    set(SDL_TEST OFF CACHE BOOL "" FORCE)
    set(SDL_TESTS OFF CACHE BOOL "" FORCE)
    set(SDL2_DISABLE_INSTALL ON CACHE BOOL "" FORCE)

    FetchContent_Declare(
        SDL2
        GIT_REPOSITORY https://github.com/libsdl-org/SDL.git
        GIT_TAG release-2.26.5
    )
    FetchContent_MakeAvailable(SDL2)

    # FetchContent doesn't set find_package variables natively.
    # We check for the exported targets and manually flag it as found.
    if(TARGET SDL2::SDL2 OR TARGET SDL2)
        set(SDL2_FOUND TRUE)
    endif()
endif()

if(SDL2_FOUND)
    message(STATUS "SDL2 found/built — graphical window backend enabled")
    # You can now link safely: target_link_libraries(your_target PRIVATE SDL2::SDL2)
else()
    message(STATUS "SDL2 not found — using terminal-only rendering")
endif()

# Define the source directory
set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)


# Gather all source files EXCEPT main.cpp, and ensure HIMEM.cpp is included
file(GLOB_RECURSE SRC_FILES "${SRC_DIR}/*.cpp")
list(REMOVE_ITEM SRC_FILES "${SRC_DIR}/main.cpp")
# Exclude SDL renderer when SDL2 is not available
if(NOT SDL2_FOUND)
    list(REMOVE_ITEM SRC_FILES "${SRC_DIR}/ui/SDLRenderer.cpp")
endif()
# Ensure HIMEM.cpp is included (in case GLOB_RECURSE misses nested folders or symlinks)
list(APPEND SRC_FILES "${SRC_DIR}/memory/himem/HIMEM.cpp")

# Third-party: ymfm (BSD-3-Clause) for the OPL2/OPL3 FM synthesizer
set(YMFM_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/ymfm)
add_library(ymfm STATIC
    ${YMFM_DIR}/ymfm_opl.cpp
    ${YMFM_DIR}/ymfm_adpcm.cpp
    ${YMFM_DIR}/ymfm_pcm.cpp)
target_include_directories(ymfm PUBLIC ${YMFM_DIR})
set_target_properties(ymfm PROPERTIES POSITION_INDEPENDENT_CODE ON)

# Third-party: fzip (https://github.com/fador/fzip) for reading ZIP archives
set(FZIP_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/fzip)
add_library(fzip STATIC
    ${FZIP_DIR}/codec.cpp
    ${FZIP_DIR}/crc32.cpp
    ${FZIP_DIR}/deflate.cpp
    ${FZIP_DIR}/file_type.cpp
    ${FZIP_DIR}/xxhash.cpp
    ${FZIP_DIR}/zip_reader.cpp
    ${FZIP_DIR}/zstd_codec.cpp
    ${FZIP_DIR}/zstd_compress.cpp
    ${FZIP_DIR}/zstd_decompress.cpp
    ${FZIP_DIR}/zstd_fse.cpp
    ${FZIP_DIR}/zstd_huffman.cpp
    ${FZIP_DIR}/zstd_sequence.cpp)
target_include_directories(fzip PUBLIC ${FZIP_DIR})
set_target_properties(fzip PROPERTIES POSITION_INDEPENDENT_CODE ON)

# Define an internal static library for the emulator core
add_library(fadors_core STATIC ${SRC_FILES}
    src/hw/CMOSRTC.hpp)
target_include_directories(fadors_core PUBLIC ${SRC_DIR})
target_link_libraries(fadors_core PUBLIC ymfm fzip)
target_compile_definitions(fadors_core PUBLIC
    FADOR_ENABLE_DEBUG_DIAGNOSTICS=$<BOOL:${FADOR_ENABLE_DEBUG_DIAGNOSTICS}>
    FADOR_X87_FORCE_SOFT=$<BOOL:${FADOR_SOFT_X87}>)

if(SDL2_FOUND)
    target_compile_definitions(fadors_core PUBLIC HAVE_SDL2=1)
    target_include_directories(fadors_core PUBLIC ${SDL2_INCLUDE_DIRS})
    target_link_libraries(fadors_core PUBLIC SDL2::SDL2)
endif()

# Define the main emulator executable
add_executable(fadors_emu ${SRC_DIR}/main.cpp
    src/utils/CrashHandler.cpp)
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
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_disassembler.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_instruction_decoder.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_interrupts.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_hw_devices.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_dma8237.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_cmos_rtc.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_timing.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_bios.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_dos.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_dos_correctness.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_dos_memory.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_dos_directories.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_dos_drive.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_drive_manager.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_dos_search.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_addressing_modes.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_assembler.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_int_asm.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_cpu.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_cpu_model.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_debugger.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_dpmi.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_vga_and_irq.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_himem.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/memory/himem/test_himem.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/memory/test_memory_bus.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/memory/test_paging.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_audio_backend.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_midi.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/hw/test_vga_controller.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/hw/test_vga_registers.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_frame_renderer.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_batch.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_cdrom.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_x87.cpp"
)

add_executable(fadors_tests ${TEST_FILES})
target_link_libraries(fadors_tests PRIVATE fadors_core)
target_include_directories(fadors_tests PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/tests")

add_test(NAME fadors_tests COMMAND fadors_tests)

# Differential CPU tester (host x86-64 CPU vs. interpreter).  Needs to run
# generated code natively, so it is only built on x86-64 POSIX hosts.
if(UNIX AND CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|AMD64|amd64")
    add_executable(cpu_fuzz tools/cpu_fuzz.cpp)
    target_link_libraries(cpu_fuzz PRIVATE fadors_core)
    add_test(NAME cpu_fuzz COMMAND cpu_fuzz 200000 1)
    # Differential x87 tester (host x87 vs. emulator); see tools/fpu_fuzz.cpp
    add_executable(fpu_fuzz tools/fpu_fuzz.cpp)
    target_link_libraries(fpu_fuzz PRIVATE fadors_core)
    target_include_directories(fpu_fuzz PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/tools)
    add_test(NAME fpu_fuzz COMMAND fpu_fuzz 20000 1)
    add_test(NAME fpu_fuzz_backend COMMAND fpu_fuzz --backend 200000 1)
endif()
