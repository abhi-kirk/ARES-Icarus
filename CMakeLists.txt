cmake_minimum_required(VERSION 3.10)

project(Icarus VERSION 1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Dependency order: math <- config <- dynamics <- control <- guidance <- sim
add_subdirectory(modules/math)
add_subdirectory(src/config)
add_subdirectory(src/dynamics)
add_subdirectory(src/gnc/control)
add_subdirectory(src/gnc/guidance)
add_subdirectory(src/sim)

add_executable(Icarus src/main.cpp)
target_link_libraries(Icarus PRIVATE icarus_sim)
target_include_directories(Icarus PRIVATE src)
