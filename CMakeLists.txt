cmake_minimum_required(VERSION 3.10)

project(Icarus VERSION 1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(FetchContent)
FetchContent_Declare(rerun_sdk
    URL https://github.com/rerun-io/rerun/releases/latest/download/rerun_cpp_sdk.zip
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
FetchContent_Populate(rerun_sdk)

set(_arrow_cmake "${rerun_sdk_SOURCE_DIR}/download_and_build_arrow.cmake")
file(READ "${_arrow_cmake}" _arrow_cmake_content)
string(REPLACE
    "git apply --check \${MIMALLOC_PATCH} && git apply \${MIMALLOC_PATCH} || true"
    "patch --forward -p1 -i \${MIMALLOC_PATCH} || true"
    _arrow_cmake_content "${_arrow_cmake_content}")
string(REPLACE
    "set(ARROW_CXXFLAGS \"\")"
    "set(ARROW_CXXFLAGS \"-Wno-documentation\")"
    _arrow_cmake_content "${_arrow_cmake_content}")
file(WRITE "${_arrow_cmake}" "${_arrow_cmake_content}")

add_subdirectory(${rerun_sdk_SOURCE_DIR} ${rerun_sdk_BINARY_DIR})

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
