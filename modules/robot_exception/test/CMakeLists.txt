cmake_minimum_required (VERSION 3.8)

find_package (glog 0.6.0 REQUIRED)

include_directories("../include")
set(TEST_BINARY ${MODULE_NAME}_unittest)


aux_source_directory(./ TEST_DIR_SRCS)
add_executable(${TEST_BINARY} ${TEST_DIR_SRCS})
add_test(NAME ${TEST_BINARY} COMMAND ${TEST_BINARY})

target_link_libraries(${TEST_BINARY} PUBLIC ${MODULE_NAME} gtest glog::glog)