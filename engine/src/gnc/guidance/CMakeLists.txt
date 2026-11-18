add_library(icarus_guidance
  cascade_guidance.cpp
  horizontal_guidance.cpp
  soft_land_reference.cpp
  ltv_mpc.cpp
  scvx_guidance.cpp
)
target_include_directories(icarus_guidance PUBLIC ${CMAKE_SOURCE_DIR}/src)
# osqpstatic is the static target from OSQP 0.6.x FetchContent.
target_link_libraries(icarus_guidance PUBLIC icarus_control icarus_config osqpstatic)
target_include_directories(icarus_guidance PUBLIC ${osqp_SOURCE_DIR}/include)
