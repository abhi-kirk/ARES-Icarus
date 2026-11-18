# Drop the symbol-prefix header into the QOCO source tree and compile every
# QOCO file with it. Idempotent: a second run leaves the tree alone.
if(NOT DEFINED SRC)
    message(FATAL_ERROR "SRC must point at the QOCO source directory")
endif()

set(prefix_header "${CMAKE_CURRENT_LIST_DIR}/qoco_symbol_prefix.h")
if(NOT EXISTS "${prefix_header}")
    message(FATAL_ERROR "missing ${prefix_header}")
endif()
execute_process(COMMAND ${CMAKE_COMMAND} -E copy_if_different
    "${prefix_header}" "${SRC}/qoco_symbol_prefix.h")

file(READ "${SRC}/CMakeLists.txt" text)
if(text MATCHES "qoco_symbol_prefix.h")
    return()
endif()

set(include_flag "-include \${CMAKE_CURRENT_SOURCE_DIR}/qoco_symbol_prefix.h")
string(REPLACE
    "set(CMAKE_C_FLAGS \"-O3 -Wall\")"
    "set(CMAKE_C_FLAGS \"-O3 -Wall ${include_flag}\")"
    text "${text}")
string(REPLACE
    "set(CMAKE_C_FLAGS \"-g -Wall\")"
    "set(CMAKE_C_FLAGS \"-g -Wall ${include_flag}\")"
    text "${text}")
if(NOT text MATCHES "qoco_symbol_prefix.h")
    message(FATAL_ERROR "QOCO CMakeLists.txt no longer has the flag lines this injects into")
endif()
file(WRITE "${SRC}/CMakeLists.txt" "${text}")
