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
include( ${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake )
# Defines the orocos_* cmake macros. See that file for additional
# documentation.

aux_source_directory(./ TEST_DIR_SRCS)

add_executable(${TEST_BINARY} ${TEST_DIR_SRCS})
add_test(NAME ${TEST_BINARY} COMMAND ${TEST_BINARY})


target_include_directories(${TEST_BINARY} PUBLIC
                          ${PROJECT_SOURCE_DIR}/modules/core/include
                          ${PROJECT_SOURCE_DIR}/modules/components/ethercat_component/include
                          ${PROJECT_SOURCE_DIR}/modules/components/robot_component/include
                          )

target_link_libraries(${TEST_BINARY} gtest)
target_link_libraries(${TEST_BINARY} Eigen3::Eigen)
target_link_libraries(${TEST_BINARY} core)
target_link_libraries(${TEST_BINARY} ethercat_component)
target_link_libraries(${TEST_BINARY} robot_component)
target_link_libraries(${TEST_BINARY} ${MODULE_NAME})
