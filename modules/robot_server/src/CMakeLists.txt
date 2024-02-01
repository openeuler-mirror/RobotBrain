cmake_minimum_required (VERSION 3.22.1)

include_directories("../include")
aux_source_directory(./ DIR_SRCS)

find_package (glog 0.6.0 REQUIRED)

# 将源代码添加到此项目的可执行文件。
add_executable(${ROBOT_SERVER_EXE} ${DIR_SRCS})
find_path(OROCOS_ROBOT_BRAIN_CPP_INCLUDE_DIR NAMES orocos/robot_brain_cpp PATH_SUFFIXES include)
if (NOT OROCOS_ROBOT_BRAIN_CPP_INCLUDE_DIR)
  message(FATAL_ERROR "Could not find orocos/robot_brain_cpp")
endif()
set(OROCOS_ROBOT_BRAIN_CPP_INCLUDE_DIR ${OROCOS_ROBOT_BRAIN_CPP_INCLUDE_DIR}/orocos/robot_brain_cpp)

target_include_directories(${ROBOT_SERVER_EXE} PUBLIC  
                          ${PROJECT_SOURCE_DIR}/modules/components/ethercat_component/include
                          ${PROJECT_SOURCE_DIR}/modules/components/robot_component/include
                          ${PROJECT_SOURCE_DIR}/modules/components/trajectory_component/include
                          ${PROJECT_SOURCE_DIR}/modules/components/robot_service_component/include
                          ${PROJECT_SOURCE_DIR}/modules/components/exception_component/include
                          #${PROJECT_SOURCE_DIR}/modules/components/grpc_service_component/include
                          ${PROJECT_SOURCE_DIR}/modules/components/interlock_component/include
                       
                          ${OROCOS_ROBOT_BRAIN_CPP_INCLUDE_DIR}
                        )
#target_link_directories(${ROBOT_SERVER_EXE} PUBLIC $ENV{RTT_COMPONENT_PATH}/${OROCOS_TARGET}/robot_brain_cpp/)

target_link_libraries(${ROBOT_SERVER_EXE} glog::glog gtest)
target_link_libraries(${ROBOT_SERVER_EXE} ethercat_component)
target_link_libraries(${ROBOT_SERVER_EXE} robot_component)
target_link_libraries(${ROBOT_SERVER_EXE} exception_component)
target_link_libraries(${ROBOT_SERVER_EXE} trajectory_component)
target_link_libraries(${ROBOT_SERVER_EXE} robot_service_component)
target_link_libraries(${ROBOT_SERVER_EXE} interlock_component)
#target_link_libraries(${ROBOT_SERVER_EXE} grpc_service_component)
target_link_libraries(${ROBOT_SERVER_EXE} core)
target_link_libraries(${ROBOT_SERVER_EXE} orocos-rtt-mqueue-${OROCOS_TARGET})
#target_link_libraries(${ROBOT_SERVER_EXE} Eigen3::Eigen)
install(TARGETS ${ROBOT_SERVER_EXE} DESTINATION bin)
