cmake_minimum_required (VERSION 3.22.1)

include_directories("../include")
set(TEST_BINARY ${MODULE_NAME}_unittest)
find_package (glog 0.6.0 REQUIRED)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

aux_source_directory(./ TEST_DIR_SRCS)

add_executable(${TEST_BINARY} ${TEST_DIR_SRCS})
add_test(NAME ${TEST_BINARY} COMMAND ${TEST_BINARY})

find_path(OROCOS_ROBOT_BRAIN_CPP_INCLUDE_DIR NAMES orocos/robot_brain_cpp PATH_SUFFIXES include)
if (NOT OROCOS_ROBOT_BRAIN_CPP_INCLUDE_DIR)
  message(FATAL_ERROR "Could not find orocos/robot_brain_cpp")
endif()
set(OROCOS_ROBOT_BRAIN_CPP_INCLUDE_DIR ${OROCOS_ROBOT_BRAIN_CPP_INCLUDE_DIR}/orocos/robot_brain_cpp)

target_include_directories(${TEST_BINARY} PUBLIC
                          ${PROJECT_SOURCE_DIR}/modules/core/include
                          ${OROCOS_ROBOT_BRAIN_CPP_INCLUDE_DIR}
                          ${PROJECT_SOURCE_DIR}/modules/components/ethercat_component/include
                          ${PROJECT_SOURCE_DIR}/modules/components/robot_component/include
                          ${PROJECT_SOURCE_DIR}/modules/components/robot_service_component/include
                          ${PROJECT_SOURCE_DIR}/modules/components/trajectory_component/include
                          ${PROJECT_SOURCE_DIR}/modules/components/exception_component/include
                          ${PROJECT_SOURCE_DIR}/modules/components/interlock_component/include
                          )
#target_link_directories(${TEST_BINARY} PUBLIC $ENV{RTT_COMPONENT_PATH}/${OROCOS_TARGET}/robot_brain_cpp/)
target_link_libraries(${TEST_BINARY} ${MODULE_NAME})
target_link_libraries(${TEST_BINARY} ethercat_component)
target_link_libraries(${TEST_BINARY} robot_component)
target_link_libraries(${TEST_BINARY} trajectory_component)
target_link_libraries(${TEST_BINARY} robot_service_component)
target_link_libraries(${TEST_BINARY} exception_component)
target_link_libraries(${TEST_BINARY} core)
target_link_libraries(${TEST_BINARY} robot_exception)
target_link_libraries(${TEST_BINARY} interlock_component)
# target_link_libraries(${TEST_BINARY} /usr/local/lib/orocos/gnulinux/robot_brain_cpp/types/librobot_brain_cpp-transport-mqueue-gnulinux.so)
# target_link_libraries(${TEST_BINARY} /home/robot/projects/robot_brain_cpp/out/build/typekit/librobot_brain_cpp-typekit-gnulinux.so)
# target_link_libraries(${TEST_BINARY} /home/robot/projects/robot_brain_cpp/out/build/typekit/transports/typelib/librobot_brain_cpp-transport-typelib-gnulinux.so)
# target_link_libraries(${TEST_BINARY} /home/robot/projects/robot_brain_cpp/out/build/typekit/transports/corba/librobot_brain_cpp-transport-corba-gnulinux.so)
# TARGET_LINK_LIBRARIES(${TEST_BINARY} orocos-rtt-${OROCOS_TARGET})
target_link_libraries(${TEST_BINARY} orocos-rtt-mqueue-${OROCOS_TARGET})
target_link_libraries(${TEST_BINARY} glog::glog gtest)
target_link_libraries(${TEST_BINARY} Eigen3::Eigen)
