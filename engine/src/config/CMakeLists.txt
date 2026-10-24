add_library(icarus_config config.cpp)
target_include_directories(icarus_config PUBLIC ${CMAKE_SOURCE_DIR}/src)
target_include_directories(icarus_config PRIVATE ${CMAKE_SOURCE_DIR}/modules)
