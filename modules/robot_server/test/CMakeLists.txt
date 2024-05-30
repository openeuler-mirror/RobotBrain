cmake_minimum_required (VERSION 3.22.1)
find_package (glog 0.6.0 REQUIRED)

include_directories("../include")
set(TEST_BINARY ${ROBOT_SERVER_EXE}_unittest)

aux_source_directory(./ TEST_DIR_SRCS)
orocos_executable(${TEST_BINARY} ${TEST_DIR_SRCS})
add_test(NAME ${TEST_BINARY} COMMAND ${TEST_BINARY})

target_link_libraries(${TEST_BINARY} glog::glog gtest)

