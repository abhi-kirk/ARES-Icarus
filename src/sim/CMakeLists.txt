add_library(icarus_sim simulator.cpp logger.cpp)
target_include_directories(icarus_sim PUBLIC ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(icarus_sim PUBLIC icarus_guidance icarus_dynamics)
