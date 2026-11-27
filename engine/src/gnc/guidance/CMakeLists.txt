add_library(icarus_guidance
  cascade_guidance.cpp
  horizontal_guidance.cpp
  soft_land_reference.cpp
  ltv_mpc.cpp
  scvx_guidance.cpp
  pitch_program.cpp
)
target_include_directories(icarus_guidance PUBLIC ${CMAKE_SOURCE_DIR}/src)
# osqpstatic is the tracking planner. qocostatic is the fuel plan's cone solver.
target_link_libraries(icarus_guidance PUBLIC icarus_control icarus_config osqpstatic qocostatic)
target_include_directories(icarus_guidance PUBLIC
    ${osqp_SOURCE_DIR}/include
    ${CMAKE_BINARY_DIR}/_deps/qoco/src/qoco/include
    ${CMAKE_BINARY_DIR}/_deps/qoco/src/qoco/lib/qdldl/include
)
if(UNIX)
    target_link_libraries(icarus_guidance PUBLIC m)
endif()
if(UNIX AND NOT APPLE)
    target_link_libraries(icarus_guidance PUBLIC dl)
endif()
