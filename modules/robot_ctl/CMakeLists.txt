# CMakeList.txt: robot_server 的 CMake 项目，在此处包括源代码并定义
# 项目特定的逻辑。
#
cmake_minimum_required (VERSION 3.22.1)
set(ROBOT_CTL_EXE "rctl")
message(STATUS "Module robot_ctl," ${CMAKE_CURRENT_SOURCE_DIR})

add_subdirectory(src)
if(ROSC_ROBOT_BRAIN_BUILD_TESTING)
    add_subdirectory(test)
endif()
