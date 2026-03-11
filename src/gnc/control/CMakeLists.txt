add_library(icarus_control pid_controller.cpp controller.cpp)
target_include_directories(icarus_control PUBLIC ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(icarus_control PUBLIC icarus_math)
