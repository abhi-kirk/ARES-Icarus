# Define the library
add_library(icarus_control
    controller.cpp
    hoverslam.cpp
    pid.cpp
    config.cpp
)

# Allow other parts of the app to see the header files
target_include_directories(icarus_control PUBLIC .)
target_include_directories(icarus_control PUBLIC ../dynamics)
target_include_directories(icarus_control PUBLIC ${CMAKE_SOURCE_DIR}/modules)

# Link against math library
target_link_libraries(icarus_control PUBLIC icarus_math)