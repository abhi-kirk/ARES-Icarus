cmake_minimum_required(VERSION 3.10)

project(Icarus VERSION 1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# On by default. `make run` keeps the Rerun landing. The Fly image passes OFF
# and builds IcarusStream only.
option(ICARUS_WITH_RERUN "Build the desktop landing with Rerun" ON)

if(ICARUS_WITH_RERUN)
  include(FetchContent)
  cmake_policy(SET CMP0169 OLD)
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
endif()

# OSQP for Phase 5 LTV-MPC. Static lib only; Fly builds with Rerun OFF still link it.
include(FetchContent)
set(OSQP_BUILD_SHARED_LIB OFF CACHE BOOL "" FORCE)
set(OSQP_BUILD_DEMO_EXE OFF CACHE BOOL "" FORCE)
set(UNITTESTS OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
  osqp
  GIT_REPOSITORY https://github.com/osqp/osqp.git
  GIT_TAG v0.6.3
  GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(osqp)

# Dependency order: math <- config <- dynamics/environment <- control <- guidance <- sim
add_subdirectory(modules/math)
add_subdirectory(src/config)
add_subdirectory(src/dynamics)
add_subdirectory(src/gnc/control)
add_subdirectory(src/gnc/guidance)
add_subdirectory(src/environment)
add_subdirectory(src/sim)

if(ICARUS_WITH_RERUN)
  add_executable(Icarus src/main.cpp)
  target_link_libraries(Icarus PRIVATE icarus_sim)
  target_include_directories(Icarus PRIVATE src)
endif()

# Case on stdin, one sample per line on stdout. No log file and no Rerun.
add_executable(IcarusStream src/stream_main.cpp)
target_link_libraries(IcarusStream PRIVATE icarus_sim icarus_config)
target_include_directories(IcarusStream PRIVATE src)
