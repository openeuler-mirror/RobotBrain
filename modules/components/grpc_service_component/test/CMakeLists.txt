cmake_minimum_required (VERSION 3.8)

include_directories("../include")
set(TEST_BINARY ${MODULE_NAME}_unittest)

# Use 'source orocos_toolchain/env.sh' if the command below fails:
find_package(OROCOS-RTT QUIET)
if (NOT OROCOS-RTT_FOUND)
  message (FATAL_ERROR "\nCould not find Orocos. Please use the shell command\n 'source orocos_toolchain/env.sh' and then run cmake again.")
endif()
include( ${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake )

find_package (glog 0.6.0 REQUIRED)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
message(STATUS "FIND Eigen3 " ${EIGEN3_INCLUDE_DIR})

find_path(OROCOS_ROBOT_BRAIN_CPP_INCLUDE_DIR NAMES orocos/robot_brain_cpp PATH_SUFFIXES include)
if (NOT OROCOS_ROBOT_BRAIN_CPP_INCLUDE_DIR)
  message(FATAL_ERROR "Could not find orocos/robot_brain_cpp")
endif()
# OROCOS_ROBOT_BRAIN_CPP_INCLUDE_DIR=${OROCOS_ROBOT_BRAIN_CPP_INCLUDE_DIR}/orocos/robot_brain_cpp
set(OROCOS_ROBOT_BRAIN_CPP_INCLUDE_DIR ${OROCOS_ROBOT_BRAIN_CPP_INCLUDE_DIR}/orocos/robot_brain_cpp)

set(_PROTOBUF_LIBPROTOBUF protobuf)
set(_REFLECTION grpc++_reflection)
set(_GRPC_GRPCPP grpc++)

aux_source_directory(./ TEST_DIR_SRCS)
# file(GLOB DIR_SRCS "../src/*.cc" "../include/*.cc")
# list(FILTER DIR_SRCS EXCLUDE REGEX ".*_main.cc$")

add_executable(${TEST_BINARY} ${TEST_DIR_SRCS})
add_test(NAME ${TEST_BINARY} COMMAND ${TEST_BINARY})

target_include_directories(${TEST_BINARY} PUBLIC
                           ${OROCOS_ROBOT_BRAIN_CPP_INCLUDE_DIR}
                           ${PROJECT_SOURCE_DIR}/modules/core/include
                           ${PROJECT_SOURCE_DIR}/modules/components/ethercat_component/include
                           ${PROJECT_SOURCE_DIR}/modules/components/robot_component/include
                           ${PROJECT_SOURCE_DIR}/modules/components/robot_service_component/include
                           ${PROJECT_SOURCE_DIR}/modules/components/trajectory_component/include
                           ${PROJECT_SOURCE_DIR}/modules/components/exception_component/include
                           ${PROJECT_SOURCE_DIR}/3rdparty/include
                          )
                        
target_link_directories(${TEST_BINARY} PUBLIC $ENV{RTT_COMPONENT_PATH}/${OROCOS_TARGET}/robot_brain_cpp/)

target_link_libraries(${TEST_BINARY} core)
target_link_libraries(${TEST_BINARY} glog::glog)
target_link_libraries(${TEST_BINARY} yaml-cpp)
target_link_libraries(${TEST_BINARY} gtest)
target_link_libraries(${TEST_BINARY} ethercat_component)
target_link_libraries(${TEST_BINARY} robot_component)
target_link_libraries(${TEST_BINARY} trajectory_component)
target_link_libraries(${TEST_BINARY} robot_service_component)
target_link_libraries(${TEST_BINARY} exception_component)
target_link_libraries(${TEST_BINARY} orocos-rtt-mqueue-${OROCOS_TARGET})
target_link_libraries(${TEST_BINARY} ${MODULE_NAME})
target_link_libraries(${TEST_BINARY} ${_REFLECTION} ${_GRPC_GRPCPP} ${_PROTOBUF_LIBPROTOBUF})
