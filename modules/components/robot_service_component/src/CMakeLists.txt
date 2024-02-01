cmake_minimum_required (VERSION 3.22.1)

include_directories("../include")
aux_source_directory(./ DIR_SRCS)
find_package (glog 0.6.0 REQUIRED)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

# Use 'source orocos_toolchain/env.sh' if the command below fails:
find_package(OROCOS-RTT QUIET)
if (NOT OROCOS-RTT_FOUND)
  message (FATAL_ERROR "\nCould not find Orocos. Please use the shell command\n 'source orocos_toolchain/env.sh' and then run cmake again.")
endif()
include( ${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake )

# Creates a component library libethercat_master-<target>.so
# and installs in the directory lib/orocos/ethercat_master/
#
orocos_component(${MODULE_NAME} ${DIR_SRCS})

target_include_directories(${MODULE_NAME} PUBLIC
                           ${PROJECT_SOURCE_DIR}/3rdparty/include
                           ${PROJECT_SOURCE_DIR}/modules/core/include
                           ${PROJECT_SOURCE_DIR}/modules/robot_teach/include
                           ${PROJECT_SOURCE_DIR}/modules/robot_planning/include
                          )

target_link_libraries(${MODULE_NAME} orocos-kdl robot_teach robot_planning)
target_link_libraries(${MODULE_NAME} ethercat pthread rt)
target_link_libraries(${MODULE_NAME} glog::glog)
target_link_libraries(${MODULE_NAME} core)
target_link_libraries(${MODULE_NAME} yaml-cpp)
target_link_libraries(${MODULE_NAME} Eigen3::Eigen)
target_link_libraries(${MODULE_NAME} robot_exception)


# You may add multiple orocos_component statements.
#
# Additional headers:
#
# Installs in the include/orocos/ethercat_master/ directory
#
orocos_install_headers(../include/robot_service_component.h)

orocos_generate_package()
