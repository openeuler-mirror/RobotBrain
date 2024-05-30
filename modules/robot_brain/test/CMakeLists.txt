cmake_minimum_required (VERSION 3.22.1)

include_directories("../include")
set(TEST_BINARY ${MODULE_NAME}_unittest)

# Use 'source orocos_toolchain/env.sh' if the command below fails:
find_package(OROCOS-RTT QUIET)
if (NOT OROCOS-RTT_FOUND)
  message (FATAL_ERROR "\nCould not find Orocos. Please use the shell command\n 'source orocos_toolchain/env.sh' and then run cmake again.")
endif()

# Defines the orocos_* cmake macros. See that file for additional
# documentation.

include( ${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake )

aux_source_directory(./ TEST_DIR_SRCS)
orocos_executable(${TEST_BINARY} ${TEST_DIR_SRCS})
# target_link_libraries(${TEST_BINARY} PRIVATE ${MODULE_NAME})
add_test(NAME ${TEST_BINARY} COMMAND ${TEST_BINARY})

# target_link_libraries(${TEST_BINARY} PRIVATE ${MODULE_NAME})

target_link_libraries(${TEST_BINARY} PUBLIC ${MODULE_NAME} gtest)
