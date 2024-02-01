# CMakeList.txt: 顶层 CMake 项目文件，在此处执行全局配置
# 并包含子项目。
#
cmake_minimum_required (VERSION 3.22.1)
project ("robot_brain_cpp")

# git及库的版本相关内容
include(cmake/gitutils.cmake)
set(VERSION_MAJOR 0)
set(VERSION_MINOR 0)
set(VERSION_PATCH 1)
message("Version is ${VERSION_MAJOR} ${VERSION_MINOR} ${VERSION_PATCH}")
string(TIMESTAMP BUILD_TIMESTAMP "%Y-%m-%d %H:%M:%S")
message("Build timestamp is ${BUILD_TIMESTAMP}")
set(GIT_HASH "")
get_git_hash(GIT_HASH)
message("Git hash is ${GIT_HASH}")
set(GIT_BRANCH "")
get_git_branch(GIT_BRANCH)
message("Git branch is ${GIT_BRANCH}")

# 检查c++编译器标志，设置c++11支持变量
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++17" COMPILER_SUPPORTS_CXX17)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)

# 使用变量设置编译标志
if(COMPILER_SUPPORTS_CXX17)
  message(STATUS "The compiler use C++17.")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17  -fno-gnu-unique")
elseif(COMPILER_SUPPORTS_CXX11)
  message(STATUS "The compiler use C++11.")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11  -fno-gnu-unique")
elseif(COMPILER_SUPPORTS_CXX0X)
  message(STATUS "The compiler use C++0X.")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x  -fno-gnu-unique")
else()
  message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

set(CMAKE_CXX_FLAGS_DEBUG "-g -ggdb -O0 -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -DNDEBUG")
message(STATUS "======CMAKE_CXX_FLAGS: ${CMAKE_CXX_FLAGS}")
message(STATUS "======CMAKE_CXX_FLAGS_DEBUG: ${CMAKE_CXX_FLAGS_DEBUG}")
message(STATUS "======CMAKE_CXX_FLAGS_RELEASE: ${CMAKE_CXX_FLAGS_RELEASE}")

option(BUILD_TESTING "Build test cases." OFF)
# 设置测试用例开启选项
option(ROSC_ROBOT_BRAIN_BUILD_TESTING "Build test cases." ON)
if(ROSC_ROBOT_BRAIN_BUILD_TESTING)
    # 启用测试
    message(STATUS "Enable ROSC_ROBOT_BRAIN_BUILD_TESTING. We will compiling test cases.")
    enable_testing()
    # 为了测试私有方法，在测试过程中，将访问控制去除。参考https://blog.csdn.net/wlqingwei/article/details/44982093
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-access-control")
    # CMAKE_CXX_FLAGS="-Dprivate=public -Dprotected=public"
endif()

# 在编译阶段设置机器人型号
set(ROSC_ROBOT_TYPE "ZERO_6" CACHE STRING
  "Robot type chosen by the user at CMake configure time")
set_property(CACHE ROSC_ROBOT_TYPE PROPERTY STRINGS "ZERO_6" "SCARA" "panasonic_2_slave" "rokae_xb4s")
if(NOT ROSC_ROBOT_TYPE)
  message(FATAL_ERROR "ROSC_ROBOT_TYPE is not set. Please set ROSC_ROBOT_TYPE to one of the supported robot types.")
endif()

if (${ROSC_ROBOT_TYPE} STREQUAL "ZERO_6")
  set(ROBOT_TYPE_ZERO_6 ON)
elseif(${ROSC_ROBOT_TYPE} STREQUAL "SCARA")
  set(ROBOT_TYPE_SCARA ON)
elseif(${ROSC_ROBOT_TYPE} STREQUAL "panasonic_2_slave")
  set(ROBOT_TYPE_PANASONIC_2_SLAVE ON)
elseif(${ROSC_ROBOT_TYPE} STREQUAL "rokae_xb4s")
  set(ROBOT_TYPE_ROKAE_XB4S ON)
endif()

# 加入一个配置头文件，用于处理 CMake 对源码的设置
configure_file (
  "${PROJECT_SOURCE_DIR}/config.h.in"
  "${PROJECT_SOURCE_DIR}/include/robot_brain/config.h"
  )

configure_file (
  "${PROJECT_SOURCE_DIR}/version.h.in"
  "${PROJECT_SOURCE_DIR}/include/robot_brain/version.h"
)

# uninstall 增加uninstall指令 
if(NOT TARGET uninstall)
  configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake_uninstall.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
    IMMEDIATE @ONLY)

  add_custom_target(uninstall
    COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)
endif()


# Use 'source orocos_toolchain/env.sh' if the command below fails:
find_package(OROCOS-RTT QUIET)
if (NOT OROCOS-RTT_FOUND)
  message (FATAL_ERROR "\nCould not find Orocos. Please use the shell command\n 'source orocos_toolchain/env.sh' and then run cmake again.")
endif()

# Defines the orocos_* cmake macros. See that file for additional
# documentation.
include( ${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake )

include_directories(${PROJECT_SOURCE_DIR}/include)
# set(ENV{CPLUS_INCLUDE_PATH} ${PROJECT_SOURCE_DIR}/modules/config_tools/include/)
# include_directories(${PROJECT_SOURCE_DIR}/modules/config_tools/include/)
orocos_typegen_headers(${PROJECT_SOURCE_DIR}/include/robot_brain/ethercat_frame_types.hpp
                      ${PROJECT_SOURCE_DIR}/include/robot_brain/command_types.hpp
                      ) # ...you may add multiple header files

install(DIRECTORY include/robot_brain
  DESTINATION include
)

# 添加源码模块所在位置
add_subdirectory(modules)
