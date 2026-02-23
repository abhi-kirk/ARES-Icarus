cmake_minimum_required(VERSION 3.10)

# Project Name
project(Icarus VERSION 1.0)

# Specify C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Generate compile_commands.json for better IDE support
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Include modules
add_subdirectory(src/control)
add_subdirectory(modules/math)

# Add the executable
add_executable(Icarus src/main.cpp)

# Link the relevant libraries to the executable
target_link_libraries(Icarus PUBLIC icarus_control)
target_include_directories(Icarus PUBLIC src)