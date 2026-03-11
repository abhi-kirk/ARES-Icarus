add_library(icarus_dynamics rocket_1dof.cpp)
target_include_directories(icarus_dynamics PUBLIC ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(icarus_dynamics PUBLIC icarus_math)
