# Define the library
add_library(icarus_math
    src/integrator.cpp
)

# Find Eigen3
find_package(Eigen3 REQUIRED)

# Allow other parts of the app to see the header file
target_include_directories(icarus_math PUBLIC include)

# Link Eigen3 to the library
target_link_libraries(icarus_math PUBLIC Eigen3::Eigen)