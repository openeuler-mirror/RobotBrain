cmake_minimum_required (VERSION 3.8)
find_package (glog 0.6.0 REQUIRED)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
message(STATUS "FIND Eigen3" ${EIGEN3_INCLUDE_DIR})

include_directories("../include")
set(TEST_BINARY ${MODULE_NAME}_unittest)

aux_source_directory(./ TEST_DIR_SRCS)
add_executable(${TEST_BINARY} ${TEST_DIR_SRCS})
target_link_libraries(${TEST_BINARY} PRIVATE ${MODULE_NAME})
add_test(NAME ${TEST_BINARY} COMMAND ${TEST_BINARY})

target_link_libraries(${TEST_BINARY} PRIVATE ${MODULE_NAME} gtest yaml-cpp)
target_link_libraries(${TEST_BINARY} PRIVATE glog::glog)
target_link_libraries(${TEST_BINARY} PRIVATE Eigen3::Eigen)

