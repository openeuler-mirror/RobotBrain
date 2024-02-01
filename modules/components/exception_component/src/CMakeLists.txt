cmake_minimum_required (VERSION 3.8)

include_directories("../include")
aux_source_directory(./ DIR_SRCS)

# Use 'source orocos_toolchain/env.sh' if the command below fails:
find_package(OROCOS-RTT QUIET)
if (NOT OROCOS-RTT_FOUND)
  message (FATAL_ERROR "\nCould not find Orocos. Please use the shell command\n 'source orocos_toolchain/env.sh' and then run cmake again.")
endif()
include( ${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake )

find_package (glog 0.6.0 REQUIRED)
find_package (Threads REQUIRED)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
message(STATUS "FIND Eigen3 " ${EIGEN3_INCLUDE_DIR})
find_package(protobuf CONFIG REQUIRED)
message(STATUS "Using protobuf ${protobuf_VERSION}")

orocos_component(${MODULE_NAME} ${DIR_SRCS})

if (${OROCOS_TARGET} MATCHES "xenomai")
  target_include_directories(${MODULE_NAME} PUBLIC
                            ${PROJECT_SOURCE_DIR}/modules/core/include
                            ${PROJECT_SOURCE_DIR}/modules/robot_exception/include
                            $ENV{XENOMAI_ROOT_DIR}/include
                            $ENV{XENOMAI_ROOT_DIR}/include/cobalt
                            )
else()
  target_include_directories(${MODULE_NAME} PUBLIC
                            ${PROJECT_SOURCE_DIR}/modules/core/include
                            ${PROJECT_SOURCE_DIR}/modules/robot_exception/include
                            )
endif()

target_link_libraries(${MODULE_NAME} ethercat pthread rt)
target_link_libraries(${MODULE_NAME} core)
target_link_libraries(${MODULE_NAME} glog::glog)
target_link_libraries(${MODULE_NAME} yaml-cpp)
target_link_libraries(${MODULE_NAME} Eigen3::Eigen)
target_link_libraries(${MODULE_NAME} ${_REFLECTION} ${_GRPC_GRPCPP} ${_PROTOBUF_LIBPROTOBUF})

# You may add multiple orocos_component statements.
#
# Additional headers:
#
# Installs in the include/orocos/ethercat_master/ directory
#
orocos_install_headers(../include/exception_component.h)


orocos_generate_package()
