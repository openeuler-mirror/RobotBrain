cmake_minimum_required (VERSION 3.8)

include_directories("../include")
set(TEST_BINARY ${MODULE_NAME}_unittest)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)
message(STATUS "FIND Eigen3 " ${EIGEN3_INCLUDE_DIR})

# Use 'source orocos_toolchain/env.sh' if the command below fails:
find_package(OROCOS-RTT QUIET)
if (NOT OROCOS-RTT_FOUND)
  message (FATAL_ERROR "\nCould not find Orocos. Please use the shell command\n 'source orocos_toolchain/env.sh' and then run cmake again.")
endif()

set(_PROTOBUF_LIBPROTOBUF protobuf)
set(_REFLECTION grpc++_reflection)
set(_GRPC_GRPCPP grpc++)

# Defines the orocos_* cmake macros. See that file for additional
# documentation.

include( ${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake )

aux_source_directory(./ TEST_DIR_SRCS)

add_executable(${TEST_BINARY} ${TEST_DIR_SRCS})
add_test(NAME ${TEST_BINARY} COMMAND ${TEST_BINARY})

target_link_libraries(${TEST_BINARY} gtest)
target_link_libraries(${TEST_BINARY} Eigen3::Eigen)
target_link_libraries(${TEST_BINARY} robot_exception)
target_link_libraries(${TEST_BINARY} ${MODULE_NAME})
