# The desktop landing logs to Rerun. The stream target only emits samples,
# so a flight image can skip that library.
if(ICARUS_WITH_RERUN)
  add_library(icarus_sim simulator.cpp logger.cpp)
  target_link_libraries(icarus_sim PUBLIC icarus_guidance icarus_dynamics environment rerun_sdk)
else()
  add_library(icarus_sim simulator.cpp)
  target_compile_definitions(icarus_sim PUBLIC ICARUS_NO_RERUN)
  target_link_libraries(icarus_sim PUBLIC icarus_guidance icarus_dynamics environment)
endif()
target_include_directories(icarus_sim PUBLIC ${CMAKE_SOURCE_DIR}/src)
