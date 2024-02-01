cmake_minimum_required (VERSION 3.22.1)

include_directories("../include")
aux_source_directory(./ DIR_SRCS)

# Use 'source orocos_toolchain/env.sh' if the command below fails:
find_package(OROCOS-RTT QUIET)
if (NOT OROCOS-RTT_FOUND)
  message (FATAL_ERROR "\nCould not find Orocos. Please use the shell command\n 'source orocos_toolchain/env.sh' and then run cmake again.")
endif()
include( ${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake )

# Creates a component library libethercat_master-<target>.so
# and installs in the directory lib/orocos/ethercat_master/
#
find_package (glog 0.6.0 REQUIRED)
find_package(Threads REQUIRED)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
message(STATUS "FIND Eigen3 " ${EIGEN3_INCLUDE_DIR})
find_package(protobuf CONFIG REQUIRED)
message(STATUS "Using protobuf ${protobuf_VERSION}")

# gRPC
find_package(absl REQUIRED)
find_package(gRPC CONFIG REQUIRED)
message(STATUS "Using gRPC ${gRPC_VERSION}")

# grpc
# 此处直接将grpc部分作为一个第三方的库，引入到了库的代码中
set(_PROTOBUF_LIBPROTOBUF protobuf)
set(_REFLECTION grpc++_reflection)


if(CMAKE_CROSSCOMPILING)
  find_program(_PROTOBUF_PROTOC protoc)
else()
  set(_PROTOBUF_PROTOC $<TARGET_FILE:protobuf::protoc>)
endif()
set(_GRPC_GRPCPP grpc++)
if(CMAKE_CROSSCOMPILING)
  find_program(_GRPC_CPP_PLUGIN_EXECUTABLE grpc_cpp_plugin)
else()
  set(_GRPC_CPP_PLUGIN_EXECUTABLE $<TARGET_FILE:gRPC::grpc_cpp_plugin>)
endif()

MESSAGE(STATUS "GRPC PLUGIN:" ${_GRPC_CPP_PLUGIN_EXECUTABLE})

# Proto file
get_filename_component(hw_proto "../../../../data/protos/robot_grpc_service.proto" ABSOLUTE)
get_filename_component(hw_proto_path "${hw_proto}" PATH)

# To create a directory when CMake generates the build system
file(MAKE_DIRECTORY ${CMAKE_SOURCE_DIR}/modules/components/${MODULE_NAME}/include/proto/)

# Generated sources
set(hw_proto_srcs "${CMAKE_SOURCE_DIR}/modules/components/${MODULE_NAME}/include/proto/robot_grpc_service.pb.cc")
set(hw_proto_hdrs "${CMAKE_SOURCE_DIR}/modules/components/${MODULE_NAME}/include/proto/robot_grpc_service.pb.h")
set(hw_grpc_srcs "${CMAKE_SOURCE_DIR}/modules/components/${MODULE_NAME}/include/proto/robot_grpc_service.grpc.pb.cc")
set(hw_grpc_hdrs "${CMAKE_SOURCE_DIR}/modules/components/${MODULE_NAME}/include/proto/robot_grpc_service.grpc.pb.h")
add_custom_command(
      OUTPUT "${hw_proto_srcs}" "${hw_proto_hdrs}" "${hw_grpc_srcs}" "${hw_grpc_hdrs}"
      COMMAND ${_PROTOBUF_PROTOC}
      ARGS --grpc_out "${CMAKE_SOURCE_DIR}/modules/components/${MODULE_NAME}/include/proto/"
        --cpp_out "${CMAKE_SOURCE_DIR}/modules/components/${MODULE_NAME}/include/proto/"
        -I "${hw_proto_path}"
        --plugin=protoc-gen-grpc="${_GRPC_CPP_PLUGIN_EXECUTABLE}"
        "${hw_proto}"
      DEPENDS "${hw_proto}")
 
# hw_grpc_proto
add_library(robot_brain_grpc_proto SHARED
  ${hw_grpc_srcs}
  ${hw_grpc_hdrs}
  ${hw_proto_srcs}
  ${hw_proto_hdrs})
target_link_libraries(robot_brain_grpc_proto
  ${_REFLECTION}
  ${_GRPC_GRPCPP}
  ${_PROTOBUF_LIBPROTOBUF})

install(TARGETS robot_brain_grpc_proto
       LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})

orocos_component(${MODULE_NAME} ${DIR_SRCS})

target_include_directories(${MODULE_NAME} PUBLIC
                           ${PROJECT_SOURCE_DIR}/3rdparty/include
                           ${PROJECT_SOURCE_DIR}/modules/core/include
                           ${PROJECT_SOURCE_DIR}/modules/components/robot_service_component/include
                          )
target_link_libraries(${MODULE_NAME} yaml-cpp)
target_link_libraries(${MODULE_NAME} robot_exception)
target_link_libraries(${MODULE_NAME} glog::glog)
target_link_libraries(${MODULE_NAME} core)
target_link_libraries(${MODULE_NAME} robot_service_component)
target_link_libraries(${MODULE_NAME} Threads::Threads)
target_link_libraries(${MODULE_NAME} Eigen3::Eigen)
target_link_libraries(${MODULE_NAME} robot_brain_grpc_proto ${_REFLECTION} ${_GRPC_GRPCPP} ${_PROTOBUF_LIBPROTOBUF})

orocos_install_headers(DIRECTORY ../include/)

orocos_generate_package()
