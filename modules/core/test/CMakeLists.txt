cmake_minimum_required (VERSION 3.22.1)

include_directories("../include")
set(TEST_BINARY ${MODULE_NAME}_unittest)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
find_package (glog 0.6.0 REQUIRED)

find_package (glog 0.6.0 REQUIRED)
aux_source_directory(./ TEST_DIR_SRCS)

set(Boost_NO_BOOST_CMAKE ON)
find_package(Boost 1.50 REQUIRED COMPONENTS filesystem)
message(STATUS "Boost version: ${Boost_VERSION}")

# This is needed if your Boost version is newer than your CMake version
# or if you have an old version of CMake (<3.5)
if(NOT TARGET Boost::filesystem)
    add_library(Boost::filesystem IMPORTED INTERFACE)
    set_property(TARGET Boost::filesystem PROPERTY
        INTERFACE_INCLUDE_DIRECTORIES ${Boost_INCLUDE_DIR})
    set_property(TARGET Boost::filesystem PROPERTY
        INTERFACE_LINK_LIBRARIES ${Boost_LIBRARIES})
endif()

orocos_executable(${TEST_BINARY} ${TEST_DIR_SRCS})
add_test(NAME ${TEST_BINARY} COMMAND ${TEST_BINARY})

target_link_libraries(${TEST_BINARY} yaml-cpp)
target_link_libraries(${TEST_BINARY} glog::glog)
target_link_libraries(${TEST_BINARY} ${MODULE_NAME} gtest)
target_link_libraries(${TEST_BINARY} Eigen3::Eigen)
target_link_libraries(${TEST_BINARY} Boost::filesystem)

