cmake_minimum_required (VERSION 3.22.1)

include_directories("../include")
set(TEST_BINARY ${MODULE_NAME}_unittest)

find_package (glog 0.6.0 REQUIRED)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
message(STATUS "FIND Eigen3 " ${EIGEN3_INCLUDE_DIR})

find_package(OROCOS-RTT QUIET)
if (NOT OROCOS-RTT_FOUND)
  message (FATAL_ERROR "\nCould not find Orocos. Please use the shell command\n 'source orocos_toolchain/env.sh' and then run cmake again.")
endif()

# Defines the orocos_* cmake macros. See that file for additional
# documentation.

include( ${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake )


aux_source_directory(./ TEST_DIR_SRCS)

orocos_executable(${TEST_BINARY} ${TEST_DIR_SRCS})
add_test(NAME ${TEST_BINARY} COMMAND ${TEST_BINARY})


target_include_directories(${TEST_BINARY} PUBLIC
                          ${PROJECT_SOURCE_DIR}/modules/core/include
                          ${PROJECT_SOURCE_DIR}/modules/components/ethercat_component/include
                          ${RPOJECT_SOURCE_DIR}/modules/robot_planning/include
                          ${PROJECT_SOURCE_DIR}/modules/robot_teach/include
                          )

target_link_libraries(${TEST_BINARY} gtest)
target_link_libraries(${TEST_BINARY} Eigen3::Eigen)
target_link_libraries(${TEST_BINARY} core)
target_link_libraries(${TEST_BINARY} ethercat_component)
target_link_libraries(${TEST_BINARY} robot_planning)
target_link_libraries(${TEST_BINARY} ${MODULE_NAME})
