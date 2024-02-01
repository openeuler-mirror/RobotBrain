cmake_minimum_required (VERSION 3.22.1)

include_directories("../include")
aux_source_directory(./ DIR_SRCS)

find_package(Threads REQUIRED)
find_package (glog 0.6.0 REQUIRED)
find_package(protobuf CONFIG REQUIRED)
message(STATUS "Using protobuf ${protobuf_VERSION}")

# gRPC
find_package(gRPC CONFIG REQUIRED)
message(STATUS "Using gRPC ${gRPC_VERSION}")

set(_PROTOBUF_LIBPROTOBUF protobuf)
set(_GRPC_GRPCPP grpc++)

# grpc

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

# Proto file
get_filename_component(hw_proto "../../../data/protos/robot_grpc_service.proto" ABSOLUTE)
get_filename_component(hw_proto_path "${hw_proto}" PATH)

# Generated sources
set(hw_proto_srcs "${CMAKE_SOURCE_DIR}/modules/robot_ctl/include/robot_grpc_service.pb.cc")
set(hw_proto_hdrs "${CMAKE_SOURCE_DIR}/modules/robot_ctl/include/robot_grpc_service.pb.h")
set(hw_grpc_srcs "${CMAKE_SOURCE_DIR}/modules/robot_ctl/include/robot_grpc_service.grpc.pb.cc")
set(hw_grpc_hdrs "${CMAKE_SOURCE_DIR}/modules/robot_ctl/include/robot_grpc_service.grpc.pb.h")
add_custom_command(
      OUTPUT "${hw_proto_srcs}" "${hw_proto_hdrs}" "${hw_grpc_srcs}" "${hw_grpc_hdrs}"
      COMMAND ${_PROTOBUF_PROTOC}
      ARGS --grpc_out "${CMAKE_SOURCE_DIR}/modules/robot_ctl/include/"
        --cpp_out "${CMAKE_SOURCE_DIR}/modules/robot_ctl/include/"
        -I "${hw_proto_path}"
        --plugin=protoc-gen-grpc="${_GRPC_CPP_PLUGIN_EXECUTABLE}"
        "${hw_proto}"
      DEPENDS "${hw_proto}")
 

# 将源代码添加到此项目的可执行文件。
add_executable (${ROBOT_CTL_EXE} ${DIR_SRCS} ${hw_proto_srcs} ${hw_grpc_srcs})

target_include_directories(${ROBOT_CTL_EXE} PUBLIC
                           ${PROJECT_SOURCE_DIR}/3rdparty/include
                           ${PROJECT_SOURCE_DIR}/modules/robot_ctl/include
                          )
message(STATUS "PROJECT_BINARY_DIR is" ${PROJECT_BINARY_DIR})
target_link_libraries(${ROBOT_CTL_EXE} PUBLIC glog::glog)
target_link_libraries(${ROBOT_CTL_EXE} PUBLIC Threads::Threads)
target_link_libraries(${ROBOT_CTL_EXE} PUBLIC ${_GRPC_GRPCPP} ${_PROTOBUF_LIBPROTOBUF})
