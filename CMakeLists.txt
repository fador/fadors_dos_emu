cmake_minimum_required(VERSION 3.20)
project(FadorsDOSEmulator VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF) # Ensure strict standard C++

# Define the source directory
set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)

# Define the executable
add_executable(fadors_emu
    ${SRC_DIR}/main.cpp
)

# Target include directories
target_include_directories(fadors_emu PRIVATE ${SRC_DIR})

# Compiler warnings
if(MSVC)
    target_compile_options(fadors_emu PRIVATE /W4)
else()
    target_compile_options(fadors_emu PRIVATE -Wall -Wextra -Wpedantic)
endif()
