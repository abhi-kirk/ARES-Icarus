add_library(environment wind_model.cpp landing_pad.cpp)
target_include_directories(environment PUBLIC ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(environment PUBLIC icarus_math)
