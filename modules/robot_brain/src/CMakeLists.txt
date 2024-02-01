cmake_minimum_required (VERSION 3.22.1)

include_directories("../include")

aux_source_directory(./ DIR_SRCS)

find_package (glog 0.6.0 REQUIRED)
find_package(Threads REQUIRED)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
message(STATUS "FIND Eigen3 " ${EIGEN3_INCLUDE_DIR})

# # Use 'source orocos_toolchain/env.sh' if the command below fails:
find_package(OROCOS-RTT QUIET ${RTT_HINTS})
if (NOT OROCOS-RTT_FOUND)
  message (FATAL_ERROR "\nCould not find Orocos. Please use the shell command\n 'source orocos_toolchain/env.sh' and then run cmake again.")
endif()

# # Defines the orocos_* cmake macros. See that file for additional
# # documentation.
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake )

# 将源代码添加到此项目的可执行文件。

orocos_use_package(ocl-taskbrowser)
# Creates an orocos executable HelloWorld
orocos_executable(${MODULE_NAME} ${DIR_SRCS}) # ...you may add multiple source files


target_include_directories(${MODULE_NAME} PUBLIC
                           ${PROJECT_SOURCE_DIR}/3rdparty/include
                           ${PROJECT_SOURCE_DIR}/modules/core/include
                           ${PROJECT_SOURCE_DIR}/modules/components/robot_component/include
                           ${PROJECT_SOURCE_DIR}/modules/components/ethercat_component/include
                           ${PROJECT_SOURCE_DIR}/modules/components/robot_service_component/include
                           ${PROJECT_SOURCE_DIR}/modules/components/grpc_service_component/include
                           )
target_link_libraries(${MODULE_NAME} core)
target_link_libraries(${MODULE_NAME} glog::glog)
target_link_libraries(${MODULE_NAME} Threads::Threads)
target_link_libraries(${MODULE_NAME} Eigen3::Eigen)
target_link_libraries(${MODULE_NAME} robot_component)
target_link_libraries(${MODULE_NAME} ethercat_component)
target_link_libraries(${MODULE_NAME} robot_service_component)
target_link_libraries(${MODULE_NAME} grpc_service_component)

orocos_generate_package()
